// Self-recovering lane jobs.
//
// A job is a short sequence of moves the firmware runs and retries on its
// own, the way an AMS retries before it bothers the printer. Every retry is
// reported as {"ev":"job","state":"recovering",...}; only the final result
// ({"ev":"job_done","ok":false,...}) is a fault the host should act on.
//
//   feed    move toward the toolhead until a switch triggers.
//           No trigger in "mm": back off backoff_mm, retry slower.
//           Trigger before "min_mm" (a snag or kink, e.g. the TurtleNeck
//           filling early in the bowden): back off, retry.
//   unload  retract until a switch clears (hub by default), then extra_mm.
//           Still blocked after "mm": push repush_mm, retract again.
//   verify  the at-extruder check (BoxTurtle ramming): push until the
//           TurtleNeck fills, pull back until it relaxes, push again; it must
//           fill again within verify_mm or the filament is not at the gears.
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h>
#include "sched.h" // DECL_TASK
#include "board.h"
#include "events.h"
#include "jobs.h"
#include "json.h"
#include "link.h"
#include "motion.h"
#include "proto.h"
#include "spool.h"

enum { JOB_NONE, JOB_FEED, JOB_UNLOAD, JOB_VERIFY };
enum {
    ST_MAIN,       // main move
    ST_BACKOFF,    // feed: backing off before a retry
    ST_REPUSH,     // unload: pushing forward before a retry
    ST_EXTRA,      // unload: extra retract after the switch cleared
    ST_RELAX,      // verify: pulling back until the buffer relaxes
    ST_SETTLE,     // verify: a little more pull back
    ST_CHECK,      // verify: second push that must fill quickly
};

struct job {
    uint8_t type, state, tries, try_n;
    int8_t until_sw, until_v, watch_sw;
    int32_t mm, min_mm, speed, accel, backoff_mm, extra_mm, verify_mm;
    int32_t relax_mm, slow_pct;
    int32_t start_pos, main_start;
    int32_t cur_speed;
    int32_t seq;                  // the host's id, echoed in job_done
    uint8_t hold;
};

static struct job jobs[NUM_LANES];

// Defaults (cfg command).
static int32_t def_tries = 2;
static int32_t def_backoff = 20000, def_repush = 10000, def_snag_back = 30000;
static int32_t def_verify = 8000, def_relax = 30000, def_settle = 2000;

static const char *
job_type_name(int t)
{
    switch (t) {
    case JOB_FEED: return "feed";
    case JOB_UNLOAD: return "unload";
    case JOB_VERIFY: return "verify";
    }
    return "none";
}

const char *
jobs_name(int lane)
{
    return job_type_name(jobs[lane].type);
}

int
jobs_busy(int lane)
{
    return jobs[lane].type != JOB_NONE;
}

int
jobs_owns_motion(int lane)
{
    return jobs[lane].type != JOB_NONE;
}

int
jobs_any_active(void)
{
    int i;
    for (i = 0; i < NUM_LANES; i++)
        if (jobs[i].type != JOB_NONE)
            return 1;
    return 0;
}

static void
job_event(int lane, const char *state, const char *why)
{
    struct job *j = &jobs[lane];
    char buf[160];
    struct jw w;
    jw_begin(&w, buf, sizeof(buf));
    jw_str(&w, "ev", "job");
    jw_int(&w, "lane", lane + 1);
    jw_str(&w, "job", job_type_name(j->type));
    jw_str(&w, "state", state);
    jw_int(&w, "try", j->try_n);
    if (why)
        jw_str(&w, "why", why);
    link_send(buf, jw_end(&w));
}

static void
job_done(int lane, int ok, const char *code, const char *msg)
{
    struct job *j = &jobs[lane];
    char buf[200];
    struct jw w;
    jw_begin(&w, buf, sizeof(buf));
    jw_str(&w, "ev", "job_done");
    jw_int(&w, "lane", lane + 1);
    jw_str(&w, "job", job_type_name(j->type));
    jw_bool(&w, "ok", ok);
    jw_int(&w, "tries", j->try_n + 1);
    jw_milli(&w, "mm", motion_position(lane) - j->start_pos);
    if (j->seq)
        jw_int(&w, "seq", j->seq);
    if (code)
        jw_str(&w, "code", code);
    if (msg)
        jw_str(&w, "msg", msg);
    int len = jw_end(&w);
    ev_keep(lane, EV_KEEP_JOB, buf, len);
    link_send(buf, len);
    if (!ok)
        ev_fault(code, lane, msg);
    j->type = JOB_NONE;
    // Moves inside a job keep the driver on; at the end it lets go unless
    // the request asked to hold.
    if (!j->hold)
        motion_enable(lane, 0);
}

void
jobs_abort(int lane, const char *code)
{
    struct job *j = &jobs[lane];
    if (j->type == JOB_NONE)
        return;
    motion_stop(lane, END_STOPPED);
    int end;
    int32_t moved;
    motion_take_done(lane, &end, &moved);
    job_done(lane, 0, code, 0);
}

void
jobs_cancel(int lane)
{
    jobs_abort(lane, "cancelled");
}

static int
move(int lane, int32_t mm, int8_t until_sw, int8_t until_v, int32_t speed)
{
    struct job *j = &jobs[lane];
    struct move_req m;
    memset(&m, 0, sizeof(m));
    m.mm = mm;
    m.speed = speed;
    m.accel = j->accel;
    m.until_sw = until_sw;
    m.until_v = until_v;
    m.watch_sw = mm > 0 ? j->watch_sw : -1; // runout check on feeds only
    m.watch_v = 1;
    m.hold = 1;
    const char *err = motion_start(lane, &m);
    if (err) {
        job_done(lane, 0, "move_refused", err);
        return 0;
    }
    return 1;
}

static const char *
no_trigger_code(int sw)
{
    if (sw == SW_HUB)
        return "no_hub";
    if (sw >= SW_LOAD1 && sw <= SW_LOAD4)
        return "no_load";
    if (sw == SW_TN_ADV)
        return "not_at_extruder";
    return "no_trigger";
}

static void
start_main(int lane)
{
    struct job *j = &jobs[lane];
    j->state = ST_MAIN;
    j->main_start = motion_position(lane);
    switch (j->type) {
    case JOB_FEED:
        // Every try gets the full distance: after a slip the position count
        // says nothing about where the filament really is.
        move(lane, j->mm, j->until_sw, j->until_v, j->cur_speed);
        break;
    case JOB_UNLOAD:
        move(lane, -j->mm, j->until_sw, j->until_v, j->cur_speed);
        break;
    case JOB_VERIFY:
        move(lane, j->mm, SW_TN_ADV, 1, j->cur_speed);
        break;
    }
}

// One more try, or the final failure.
static void
retry_or_fail(int lane, const char *why, const char *code, const char *msg)
{
    struct job *j = &jobs[lane];
    if (j->try_n >= j->tries) {
        job_done(lane, 0, code, msg);
        return;
    }
    j->try_n++;
    job_event(lane, "recovering", why);
    if (j->type == JOB_FEED) {
        if (j->slow_pct > 0 && j->slow_pct < 100)
            j->cur_speed = (int64_t)j->cur_speed * j->slow_pct / 100;
        j->state = ST_BACKOFF;
        move(lane, -j->backoff_mm, -1, 0, j->speed);
    } else if (j->type == JOB_UNLOAD) {
        j->state = ST_REPUSH;
        move(lane, def_repush, -1, 0, j->cur_speed);
    } else {
        // verify: back off and start the whole check again
        j->state = ST_BACKOFF;
        move(lane, -j->backoff_mm, -1, 0, j->speed);
    }
}

static void
on_move_end(int lane, int end, int32_t moved)
{
    struct job *j = &jobs[lane];
    if (end == END_FAULT) {
        job_done(lane, 0, "fault", "driver fault");
        return;
    }
    if (end == END_LINK) {
        job_done(lane, 0, "link_lost", 0);
        return;
    }
    if (end == END_STOPPED) {
        job_done(lane, 0, "cancelled", 0);
        return;
    }
    if (end == END_WATCH) {
        job_done(lane, 0, "runout", "prep switch opened during the feed");
        return;
    }
    switch (j->state) {
    case ST_BACKOFF:
    case ST_REPUSH:
        start_main(lane);
        return;
    case ST_MAIN:
        break;
    case ST_EXTRA:
        job_done(lane, 1, 0, 0);
        return;
    case ST_RELAX:
        if (end != END_TRIGGERED) {
            retry_or_fail(lane, "buffer_stuck", "buffer_stuck",
                          "TurtleNeck did not relax when pulled back");
            return;
        }
        j->state = ST_SETTLE;
        move(lane, -def_settle, -1, 0, j->cur_speed);
        return;
    case ST_SETTLE:
        j->state = ST_CHECK;
        move(lane, j->verify_mm + def_settle, SW_TN_ADV, 1, j->cur_speed);
        return;
    case ST_CHECK:
        if (end == END_TRIGGERED) {
            job_done(lane, 1, 0, 0);
            return;
        }
        retry_or_fail(lane, "no_refill", "not_at_extruder",
                      "TurtleNeck did not fill again on the check push");
        return;
    }

    // ST_MAIN results
    int32_t main_moved = motion_position(lane) - j->main_start;
    if (main_moved < 0)
        main_moved = -main_moved;
    if (j->type == JOB_FEED) {
        if (end == END_TRIGGERED || (end == END_DONE && j->until_sw < 0)) {
            int32_t fed = motion_position(lane) - j->start_pos;
            if (j->min_mm && fed < j->min_mm) {
                int32_t back = j->backoff_mm;
                j->backoff_mm = def_snag_back > back ? def_snag_back : back;
                retry_or_fail(lane, "early_trigger", "snag",
                              "switch triggered well before the expected"
                              " length (snag or kink)");
                j->backoff_mm = back;
                return;
            }
            job_done(lane, 1, 0, 0);
            return;
        }
        retry_or_fail(lane, "no_trigger", no_trigger_code(j->until_sw),
                      "switch did not trigger within the distance");
    } else if (j->type == JOB_UNLOAD) {
        if (end == END_TRIGGERED || (end == END_DONE && j->until_sw < 0)) {
            if (j->extra_mm) {
                j->state = ST_EXTRA;
                move(lane, -j->extra_mm, -1, 0, j->cur_speed);
            } else {
                job_done(lane, 1, 0, 0);
            }
            return;
        }
        retry_or_fail(lane, "still_blocked", j->until_sw == SW_HUB
                      ? "hub_not_clear" : "not_clear",
                      "switch did not clear on the retract");
    } else { // JOB_VERIFY
        if (end != END_TRIGGERED) {
            retry_or_fail(lane, "no_fill", "not_at_extruder",
                          "TurtleNeck never filled (filament not at the"
                          " extruder?)");
            return;
        }
        j->state = ST_RELAX;
        move(lane, -j->relax_mm, SW_TN_ADV, 0, j->cur_speed);
    }
}

/****************************************************************
 * Commands
 ****************************************************************/

void
jobs_cfg(const char *line)
{
    int32_t v;
    if (json_get_int(line, "job_tries", &v) && v >= 0 && v <= 10)
        def_tries = v;
    if (json_get_milli(line, "job_backoff_mm", &v) && v >= 0)
        def_backoff = v;
    if (json_get_milli(line, "job_snag_backoff_mm", &v) && v >= 0)
        def_snag_back = v;
    if (json_get_milli(line, "job_repush_mm", &v) && v >= 0)
        def_repush = v;
    if (json_get_milli(line, "verify_mm", &v) && v > 0)
        def_verify = v;
    if (json_get_milli(line, "verify_relax_mm", &v) && v > 0)
        def_relax = v;
    if (json_get_milli(line, "verify_settle_mm", &v) && v >= 0)
        def_settle = v;
}

// {"cmd":"job","job":"feed","lane":1,"until":"hub","mm":250,"speed":100,
//  "min_mm":0,"tries":2,"backoff_mm":20,"slow_pct":50,"watch":"prep",
//  "hold":false}
// {"cmd":"job","job":"unload","lane":1,"until":"hub","mm":300,
//  "extra_mm":35,"speed":100}
// {"cmd":"job","job":"verify","lane":1,"mm":60,"speed":20}
// Every job also takes espool_fwd, espool_rwd and espool_kick (spool.c).
static void
cmd_job(char *line)
{
    int lane;
    if (!get_lane(line, &lane))
        return;
    struct job *j = &jobs[lane];
    if (j->type != JOB_NONE || motion_is_moving(lane)) {
        reply_err(line, "lane busy");
        return;
    }
    char name[12];
    if (!json_get_str(line, "job", name, sizeof(name))) {
        reply_err(line, "no job");
        return;
    }
    struct job n;
    memset(&n, 0, sizeof(n));
    if (!strcmp(name, "feed"))
        n.type = JOB_FEED;
    else if (!strcmp(name, "unload"))
        n.type = JOB_UNLOAD;
    else if (!strcmp(name, "verify"))
        n.type = JOB_VERIFY;
    else {
        reply_err(line, "bad job");
        return;
    }
    n.speed = n.type == JOB_VERIFY ? 20000 : 50000;
    n.accel = 400000;
    n.tries = def_tries;
    n.backoff_mm = def_backoff;
    n.verify_mm = def_verify;
    n.relax_mm = def_relax;
    n.slow_pct = 50;
    n.until_v = n.type == JOB_UNLOAD ? 0 : 1;
    n.until_sw = n.type == JOB_UNLOAD ? SW_HUB : -1;
    n.watch_sw = -1;
    int32_t v;
    int hold;
    if (json_get_bool(line, "hold", &hold))
        n.hold = hold;
    if (!json_get_milli(line, "mm", &n.mm) || n.mm <= 0) {
        reply_err(line, "bad mm");
        return;
    }
    json_get_milli(line, "speed", &n.speed);
    json_get_milli(line, "accel", &n.accel);
    json_get_milli(line, "min_mm", &n.min_mm);
    json_get_milli(line, "backoff_mm", &n.backoff_mm);
    json_get_milli(line, "extra_mm", &n.extra_mm);
    json_get_milli(line, "verify_mm", &n.verify_mm);
    json_get_milli(line, "relax_mm", &n.relax_mm);
    json_get_int(line, "slow_pct", &n.slow_pct);
    json_get_int(line, "seq", &n.seq);
    if (json_get_int(line, "tries", &v) && v >= 0 && v <= 10)
        n.tries = v;
    if (json_get_int(line, "until_v", &v))
        n.until_v = !!v;
    char sw[16];
    if (json_get_str(line, "until", sw, sizeof(sw))) {
        if (!strcmp(sw, "load"))
            n.until_sw = SW_LOAD(lane);
        else if (!strcmp(sw, "prep"))
            n.until_sw = SW_PREP(lane);
        else if (!sw[0])
            n.until_sw = -1;
        else if ((n.until_sw = sw_lookup(sw)) < 0) {
            reply_err(line, "bad until");
            return;
        }
    }
    if (json_get_str(line, "watch", sw, sizeof(sw))) {
        if (!strcmp(sw, "prep"))
            n.watch_sw = SW_PREP(lane);
        else if (!strcmp(sw, "load"))
            n.watch_sw = SW_LOAD(lane);
        else if (sw[0] && (n.watch_sw = sw_lookup(sw)) < 0) {
            reply_err(line, "bad watch");
            return;
        }
    }
    if (n.type == JOB_FEED && n.until_sw < 0 && !n.min_mm) {
        // A plain distance feed has nothing to recover from.
        n.tries = 0;
    }
    if (n.speed <= 0 || n.accel <= 0) {
        reply_err(line, "bad speed");
        return;
    }
    if (n.watch_sw >= 0 && !sensor(n.watch_sw)) {
        reply_err(line, "watch switch not triggered (no filament?)");
        return;
    }
    n.cur_speed = n.speed;
    n.start_pos = motion_position(lane);
    espool_move_cfg(lane, line);
    *j = n;
    reply_ok(line);
    job_event(lane, "started", 0);
    start_main(lane);
}

int
jobs_handle_cmd(const char *cmd, char *line)
{
    if (strcmp(cmd, "job"))
        return 0;
    cmd_job(line);
    return 1;
}

void
jobs_task(void)
{
    int i;
    for (i = 0; i < NUM_LANES; i++) {
        if (jobs[i].type == JOB_NONE)
            continue;
        int end;
        int32_t moved;
        if (motion_take_done(i, &end, &moved))
            on_move_end(i, end, moved);
    }
}
DECL_TASK(jobs_task);
