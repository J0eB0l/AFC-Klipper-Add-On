// Lane commands (move, stop, enable, lane and driver settings), move-done
// events and the link-loss safety stop.
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
#include "tmc.h"

static uint32_t link_timeout_ms = 2000;
static int32_t move_seq[NUM_LANES];   // the host's id, echoed in move_done
static uint32_t lost_rx;              // link_last_rx() at the last link loss

int
sw_lookup(const char *name)
{
    int i;
    for (i = 0; i < SW_COUNT; i++)
        if (!strcmp(name, SW_NAMES[i]))
            return i;
    return -1;
}

// Reads a switch name for key into *sw; "" or a missing key gives -1.
// Lane-relative names "prep" and "load" map to the lane's own switch.
static int
get_sw(const char *line, const char *key, int lane, int8_t *sw)
{
    char name[16];
    *sw = -1;
    if (!json_get_str(line, key, name, sizeof(name)) || !name[0])
        return 1;
    int id;
    if (!strcmp(name, "prep"))
        id = SW_PREP(lane);
    else if (!strcmp(name, "load"))
        id = SW_LOAD(lane);
    else
        id = sw_lookup(name);
    if (id < 0)
        return 0;
    *sw = id;
    return 1;
}

int
parse_move(const char *line, int lane, struct move_req *m, int32_t def_speed)
{
    memset(m, 0, sizeof(*m));
    m->speed = def_speed;
    m->accel = 400000;
    m->until_v = 1;
    m->watch_v = 1;
    int32_t v;
    if (!json_get_milli(line, "mm", &m->mm)) {
        reply_err(line, "no mm");
        return 0;
    }
    json_get_milli(line, "speed", &m->speed);
    json_get_milli(line, "accel", &m->accel);
    if (!get_sw(line, "until", lane, &m->until_sw)
        || !get_sw(line, "watch", lane, &m->watch_sw)) {
        reply_err(line, "bad switch");
        return 0;
    }
    if (json_get_int(line, "until_v", &v))
        m->until_v = !!v;
    if (json_get_int(line, "watch_v", &v))
        m->watch_v = !!v;
    int b;
    if (json_get_bool(line, "hold", &b))
        m->hold = b;
    return 1;
}

// {"cmd":"move","lane":1,"mm":300,"speed":100,"accel":400,
//  "until":"hub","until_v":1,"watch":"prep","watch_v":1,"hold":false,
//  "espool_fwd":0.4,"espool_rwd":0.6,"espool_kick":true,"seq":7}
// Replies at once; {"ev":"move_done",...} follows when it ends, with the
// seq when one was given.
static void
cmd_move(char *line)
{
    int lane;
    struct move_req m;
    if (!get_lane(line, &lane) || !parse_move(line, lane, &m, 50000))
        return;
    if (jobs_busy(lane)) {
        reply_err(line, "lane busy");
        return;
    }
    if (tmc_health(lane) != TMC_OK && tmc_health(lane) != TMC_OTPW
        && tmc_health(lane) != TMC_OPEN) {
        reply_err(line, tmc_health_name(tmc_health(lane)));
        return;
    }
    espool_move_cfg(lane, line);
    int32_t seq = 0;
    json_get_int(line, "seq", &seq);
    move_seq[lane] = seq;
    const char *err = motion_start(lane, &m);
    if (err) {
        reply_err(line, err);
        return;
    }
    reply_ok(line);
}

// {"cmd":"replay","lane":1} sends the lane's last move_done and job_done
// again (the host lost them in a link drop), then replies.
static void
cmd_replay(char *line)
{
    int lane;
    if (!get_lane(line, &lane))
        return;
    ev_replay(lane);
    reply_ok(line);
}

// {"cmd":"stop"} stops every lane, {"cmd":"stop","lane":2} one lane.
static void
cmd_stop(char *line)
{
    int32_t v;
    if (json_get_int(line, "lane", &v)) {
        if (v < 1 || v > NUM_LANES) {
            reply_err(line, "bad lane");
            return;
        }
        jobs_cancel(v - 1);
        assist_stop(v - 1);
        motion_stop(v - 1, END_STOPPED);
        espool_set(v - 1, 0, 0);
    } else {
        int i;
        for (i = 0; i < NUM_LANES; i++) {
            jobs_cancel(i);
            assist_stop(i);
            espool_set(i, 0, 0);
        }
        motion_stop(-1, END_STOPPED);
    }
    reply_ok(line);
}

// {"cmd":"enable","lane":1,"on":true}
static void
cmd_enable(char *line)
{
    int lane, on;
    if (!get_lane(line, &lane))
        return;
    if (!json_get_bool(line, "on", &on)) {
        reply_err(line, "no on");
        return;
    }
    motion_enable(lane, on);
    reply_ok(line);
}

// {"cmd":"lane_cfg","lane":1,"steps_per_mm":688.172,"dir_invert":true,
//  "run_current":0.8,"hold_current":0.8,"sense_resistor":0.110,
//  "microsteps":16,"stealthchop":false,"sgthrs":0,
//  "espool_pulse_mm":150,"espool_pulse_ms":420}
static void
cmd_lane_cfg(char *line)
{
    int lane, b;
    int32_t v;
    if (!get_lane(line, &lane))
        return;
    int32_t spm = -1;
    int inv = -1;
    json_get_milli(line, "steps_per_mm", &spm);
    if (json_get_bool(line, "dir_invert", &b))
        inv = b;
    if (spm != -1 && (spm < 1000 || spm > 20000000)) {
        reply_err(line, "bad steps_per_mm");
        return;
    }
    motion_set_lane_cfg(lane, spm, inv);
    struct tmc_cfg c = *tmc_get_cfg(lane);
    int changed = 0;
    if (json_get_milli(line, "run_current", &v) && v > 0 && v <= 2000) {
        c.run_ma = v;
        changed = 1;
    }
    if (json_get_milli(line, "hold_current", &v) && v > 0 && v <= 2000) {
        c.hold_ma = v;
        changed = 1;
    }
    if (json_get_milli(line, "sense_resistor", &v) && v > 0 && v < 1000) {
        c.sense_mohm = v;
        changed = 1;
    }
    if (json_get_int(line, "microsteps", &v) && v >= 1 && v <= 256) {
        c.microsteps = v;
        changed = 1;
    }
    if (json_get_bool(line, "stealthchop", &b)) {
        c.spreadcycle = !b;
        changed = 1;
    }
    if (json_get_int(line, "sgthrs", &v) && v >= 0 && v <= 255) {
        c.sgthrs = v;
        changed = 1;
    }
    if (changed)
        tmc_reconfigure(lane, &c);
    espool_lane_cfg(lane, line);
    reply_ok(line);
}

// {"cmd":"tmc_read","lane":1,"reg":111} -> {"val":...}
static void
cmd_tmc_read(char *line)
{
    int lane;
    int32_t reg;
    uint32_t val;
    if (!get_lane(line, &lane))
        return;
    if (!json_get_int(line, "reg", &reg) || reg < 0 || reg > 0x7f) {
        reply_err(line, "bad reg");
        return;
    }
    if (tmc_read(lane, reg, &val)) {
        reply_err(line, "no answer");
        return;
    }
    struct reply r;
    reply_begin(&r, line);
    jw_bool(&r.w, "ok", 1);
    jw_int(&r.w, "reg", reg);
    // As two 16 bit halves so the value fits a signed int on the wire.
    jw_int(&r.w, "hi", val >> 16);
    jw_int(&r.w, "lo", val & 0xffff);
    reply_send(&r);
}

// Bench: {"cmd":"tmc_wtest","lane":1,"flags":0,"gap":20,"reg":0,"lo":452}
// -> IFCNT before and after n writes sent with the given tx flags and
// idle bit times before each datagram.
static void
cmd_tmc_wtest(char *line)
{
    int lane;
    int32_t flags = 0, gap = 0, n = 1, reg = 0, hi = 0, lo = 0;
    if (!get_lane(line, &lane))
        return;
    json_get_int(line, "flags", &flags);
    json_get_int(line, "gap", &gap);
    json_get_int(line, "n", &n);
    json_get_int(line, "reg", &reg);
    json_get_int(line, "hi", &hi);
    json_get_int(line, "lo", &lo);
    uint32_t before = 0, after = 0;
    int ret = tmc_wtest(lane, flags, gap, n, reg & 0x7f,
                        ((uint32_t)hi << 16) | (lo & 0xffff), &before, &after);
    struct reply r;
    reply_begin(&r, line);
    jw_bool(&r.w, "ok", ret == 0);
    jw_int(&r.w, "ret", ret);
    jw_int(&r.w, "before", before & 0xff);
    jw_int(&r.w, "after", after & 0xff);
    reply_send(&r);
}

// {"cmd":"cfg","link_timeout_ms":2000}  (0 = never stop on link loss)
static void
cmd_cfg(char *line)
{
    int32_t v;
    if (json_get_int(line, "link_timeout_ms", &v) && v >= 0)
        link_timeout_ms = v;
    jobs_cfg(line);
    assist_cfg(line);
    reply_ok(line);
}

static const struct {
    const char *name;
    void (*fn)(char *line);
} EXTRA[] = {
    { "move", cmd_move },
    { "stop", cmd_stop },
    { "enable", cmd_enable },
    { "lane_cfg", cmd_lane_cfg },
    { "tmc_read", cmd_tmc_read },
    { "tmc_wtest", cmd_tmc_wtest },
    { "cfg", cmd_cfg },
    { "replay", cmd_replay },
};

int
proto_handle_extra(const char *cmd, char *line)
{
    unsigned i;
    for (i = 0; i < sizeof(EXTRA) / sizeof(EXTRA[0]); i++) {
        if (!strcmp(cmd, EXTRA[i].name)) {
            EXTRA[i].fn(line);
            return 1;
        }
    }
    return spool_handle_cmd(cmd, line) || jobs_handle_cmd(cmd, line);
}

void
status_extra(struct jw *w)
{
    // "lanes":[{"pos":..,"v":..,"en":1,"tmc":"ok","job":"none",
    //           "espool":0,"assist":0,"follow":0}, ...]
    static char buf[560];
    struct jw a;
    int i, n = 0;
    buf[n++] = '[';
    for (i = 0; i < NUM_LANES; i++) {
        jw_begin(&a, buf + n, sizeof(buf) - n);
        jw_milli(&a, "pos", motion_position(i));
        jw_milli(&a, "v", motion_get_velocity(i));
        jw_int(&a, "en", motion_enabled(i));
        jw_str(&a, "tmc", tmc_health_name(tmc_health(i)));
        jw_str(&a, "job", jobs_name(i));
        jw_int(&a, "espool", espool_get(i));
        jw_int(&a, "assist", assist_active(i));
        jw_int(&a, "follow", assist_following(i));
        int len = jw_end(&a);
        if (!len)
            break;
        n += len - 1; // drop the newline
        if (i < NUM_LANES - 1 && n < (int)sizeof(buf) - 1)
            buf[n++] = ',';
    }
    if (n < (int)sizeof(buf) - 1)
        buf[n++] = ']';
    buf[n] = 0;
    jw_raw(w, "lanes", buf);
    jw_int(w, "link_timeout_ms", link_timeout_ms);
}

/****************************************************************
 * Move-done events and link loss
 ****************************************************************/

void
cmds_task(void)
{
    int i;
    for (i = 0; i < NUM_LANES; i++) {
        int end;
        int32_t moved;
        if (jobs_owns_motion(i))
            continue; // the job reads its own move results
        if (!motion_take_done(i, &end, &moved))
            continue;
        char buf[120];
        struct jw w;
        jw_begin(&w, buf, sizeof(buf));
        jw_str(&w, "ev", "move_done");
        jw_int(&w, "lane", i + 1);
        jw_str(&w, "end", end_name(end));
        jw_milli(&w, "mm", moved);
        if (move_seq[i])
            jw_int(&w, "seq", move_seq[i]);
        int len = jw_end(&w);
        ev_keep(i, EV_KEEP_MOVE, buf, len);
        link_send(buf, len);
    }
    // Once per loss: the espoolers can still be braking after the stop.
    if (link_timeout_ms && link_host_seen() && link_last_rx() != lost_rx
        && (motion_any_active() || spool_any_active() || jobs_any_active())
        && uptime() - link_last_rx() > link_timeout_ms) {
        lost_rx = link_last_rx();
        for (i = 0; i < NUM_LANES; i++) {
            jobs_abort(i, "link_lost");
            assist_stop(i);
            espool_set(i, 0, 0);
        }
        motion_stop(-1, END_LINK);
        ev_fault("link_lost", -1, "no word from the host; all motors stopped");
    }
}
DECL_TASK(cmds_task);
