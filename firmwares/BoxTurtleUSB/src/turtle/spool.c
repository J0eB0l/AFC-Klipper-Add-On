// Espooler (N20 respooler) control and TurtleNeck feed assist.
//
// Espoolers: FWD/RWD are hardware PWM (4 ms cycle, espool_cycle_time's default),
// EN is a plain output. Off brakes the motor briefly (both sides high) like
// AFC's break_espooler. When the host has not taken an espooler over by
// hand, it follows its lane: rewind while the lane retracts, forward while
// the lane feeds. A move or job line can carry the duty for that move
// (espool_fwd, espool_rwd, espool_kick), so the host can size it from the
// speed and spool weight the way AFC's calculate_pwm_value does; 0 keeps
// the espooler off for that move.
//
// Feed assist: the host streams the extruder position ("e" command, a few
// times a second). The lane follows that position both ways, each change
// scaled by a multiplier picked from the TurtleNeck switches exactly like
// AFC_buffer sets a synced lane stepper's rotation distance:
//   advance changed last  (TurtleNeck expanded)   -> change * mult_low
//   trailing changed last (TurtleNeck compressed) -> change * mult_high
// A change is a press or a release, so a multiplier holds until the other
// switch changes and the TurtleNeck sweeps between its switches. With no
// change seen since power up the lane feeds at mult_high, like AFC_buffer's
// enable with no state yet. While the clog check is on (fault_mm > 0) a
// pressed switch scales harder, like AFC_buffer's fault detection: advance
// mult_low * 2 / 5, trailing mult_high * 1.5. An assist start uses the
// harder one for the last switch until a switch changes, like AFC's
// enable_buffer. The lane stops when the extruder stops. With no extruder
// updates for 1 s it falls back to bang-bang: feed at fallback speed on
// trailing, stop on advance.
//
// Faults while assisting (like AFC_buffer's filament_error_sensitivity):
//   advance stays on while the extruder moves fault_mm -> "clog"
//   trailing stays on while the extruder moves fault_mm -> "not_feeding"
//   the lane's prep switch opens                        -> "runout"
//
// Follow mode ("mode":"follow") is AFC's sync_to_extruder outside a print:
// the lane tracks the extruder's position both ways (tool_stn loads, the
// tool_stn_unload retract), with no TurtleNeck multiplier and no faults.
// The espooler follows the lane like on a move.
//
// External buffer mode ("mode":"ext") is for a buffer the host reads (an
// AFC_buffer on a Klipper MCU, such as an analog PSF): the lane tracks the
// position the host sends, which the host scales by that buffer's
// multiplier, and the espooler gives print assist pulses like in tn mode.
// The host's buffer does the fault checks.
//
// The host sends each lane its own extruder position ("e" with "lane"), so
// lanes on different extruders can follow at once. Every assist start waits
// for a fresh position before it moves.
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h>
#include "board/gpio.h" // gpio_pwm_setup
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_from_us
#include "sched.h" // DECL_TASK
#include "board.h"
#include "events.h"
#include "json.h"
#include "link.h"
#include "motion.h"
#include "proto.h"
#include "spool.h"

#define PWM_FULL 32768            // MAX_PWM in Klipper's stm32 hard_pwm.c

struct espool {
    struct gpio_pwm fwd, rwd;
    struct gpio_out en;
    int32_t duty;                 // permille, signed, what is applied now
    int32_t target;               // permille after the kick
    uint32_t kick_until;          // uptime ms
    uint32_t brake_until;
    uint8_t manual;               // host set it by hand
    uint8_t braking;
    // print assist pulses
    int32_t last_pulse_pos;       // lane position (milli-mm)
    uint32_t pulse_until;
    // Per lane settings, -1 = the cfg default.
    int16_t move_fwd, move_rwd;   // follow duty for the current move/job
    uint8_t move_kick;
    int32_t pulse_mm;             // milli-mm (lane_cfg)
    int32_t pulse_ms;
};

static struct espool esp[NUM_LANES];

// Settings (cfg command).
static int32_t esp_fwd_duty = 500, esp_rwd_duty = 500;   // permille
static uint32_t esp_kick_ms = 70, esp_brake_ms = 50;
static int32_t esp_pulse_mm = 0;      // 0 = no print assist pulses
static uint32_t esp_pulse_ms = 0;
static int esp_follow = 1;            // follow lane moves when not manual
// PWM cycle (AFC cycle_time). The timers are set up on first use, so the
// host's cfg at connect picks it; a later change needs a reboot.
static uint32_t esp_cycle_us = 4000;
static uint8_t esp_ready;

static int32_t assist_high = 1150, assist_low = 850;     // permille
static int32_t assist_fallback = 30000;                  // milli-mm/s
static int32_t assist_max = 80000;                       // milli-mm/s
static int32_t assist_fault_mm = 80000;                  // milli-mm, 0 = off

/****************************************************************
 * Espooler outputs
 ****************************************************************/

static void
espool_hw_setup(void)
{
    if (esp_ready)
        return;
    esp_ready = 1;
    uint32_t cycle = timer_from_us(esp_cycle_us);
    int i;
    for (i = 0; i < NUM_LANES; i++) {
        esp[i].fwd = gpio_pwm_setup(LANE_PINS[i].mot_fwd, cycle, 0);
        esp[i].rwd = gpio_pwm_setup(LANE_PINS[i].mot_rwd, cycle, 0);
    }
}

static void
apply(int lane, int32_t duty)
{
    espool_hw_setup();
    struct espool *e = &esp[lane];
    e->duty = duty;
    uint32_t mag = duty < 0 ? -duty : duty;
    if (mag > 1000)
        mag = 1000;
    uint32_t v = mag * PWM_FULL / 1000;
    if (duty > 0) {
        gpio_pwm_write(e->rwd, 0);
        gpio_pwm_write(e->fwd, v);
        gpio_out_write(e->en, 1);
    } else if (duty < 0) {
        gpio_pwm_write(e->fwd, 0);
        gpio_pwm_write(e->rwd, v);
        gpio_out_write(e->en, 1);
    } else {
        gpio_pwm_write(e->fwd, 0);
        gpio_pwm_write(e->rwd, 0);
        gpio_out_write(e->en, 0);
    }
}

static void
espool_drive(int lane, int32_t duty, int kick)
{
    struct espool *e = &esp[lane];
    if (duty == e->target && !e->braking)
        return;
    int32_t old = e->target;
    e->target = duty;
    if (!duty) {
        if (old) {
            espool_hw_setup();
            // Brake: both sides high for a moment, then off.
            gpio_pwm_write(e->fwd, PWM_FULL);
            gpio_pwm_write(e->rwd, PWM_FULL);
            gpio_out_write(e->en, 1);
            e->duty = 0;
            e->braking = 1;
            e->brake_until = uptime() + esp_brake_ms;
        }
        return;
    }
    e->braking = 0;
    if (kick && esp_kick_ms && (old == 0 || (old > 0) != (duty > 0))) {
        apply(lane, duty > 0 ? 1000 : -1000);
        e->kick_until = uptime() + esp_kick_ms;
    } else {
        e->kick_until = 0;
        apply(lane, duty);
    }
}

void
espool_set(int lane, int32_t duty, int kick)
{
    esp[lane].manual = duty != 0;
    espool_drive(lane, duty, kick);
}

int32_t
espool_get(int lane)
{
    return esp[lane].duty;
}

/****************************************************************
 * Feed assist
 ****************************************************************/

struct assist {
    uint8_t on;
    uint8_t follow;               // ASSIST_FOLLOW / ASSIST_EXT: track the
                                  // position both ways
    uint8_t f_init;
    int32_t f_target;             // lane position to be at (milli-mm)
    int32_t e_pos;                // milli-mm
    uint32_t e_time;              // uptime ms
    int32_t e_rate;               // milli-mm/s, smoothed
    int32_t fault_start;          // extruder pos when the switch latched
    int8_t fault_sw;              // switch that is latched, -1 none
    uint8_t start_held;           // no TurtleNeck change since the start
    uint32_t tn_edges;            // sensors_tn_edges() last seen
};

static struct assist as[NUM_LANES];

// Latest extruder position per lane (milli-mm, uptime ms).
struct efeed {
    int32_t pos;
    uint32_t time;
    uint8_t have;
};
static struct efeed ef[NUM_LANES];

void
assist_start(int lane, int mode)
{
    struct assist *a = &as[lane];
    memset(a, 0, sizeof(*a));
    a->fault_sw = -1;
    a->follow = mode;
    a->start_held = 1;
    a->tn_edges = sensors_tn_edges();
    a->on = 1;
    // A position sent before this start may be old: wait for a new one.
    ef[lane].have = 0;
    esp[lane].last_pulse_pos = motion_position(lane);
}

int
assist_following(int lane)
{
    return as[lane].on ? as[lane].follow : 0;
}

#define FOLLOW_GAIN 4             // 1/s: closes a position error in ~0.25 s

// Moves the lane with the extruder position, each change scaled by mult
// (permille; 1000 = one to one).
static void
follow_update(int lane, int32_t mult)
{
    struct assist *a = &as[lane];
    struct efeed *f = &ef[lane];
    uint32_t now = uptime();
    if (!f->have)
        return;
    if (!a->f_init) {
        a->f_init = 1;
        a->f_target = motion_position(lane);
        a->e_pos = f->pos;
        a->e_time = f->time;
        a->e_rate = 0;
        return;
    }
    if (f->time != a->e_time && f->time - a->e_time > 0) {
        int32_t d = f->pos - a->e_pos;
        int32_t dt = f->time - a->e_time;
        int32_t rate = (int64_t)d * 1000 / dt;
        a->e_rate = (a->e_rate + rate) / 2;
        a->f_target += (int64_t)d * mult / 1000;
        a->e_pos = f->pos;
        a->e_time = f->time;
    }
    int fresh = now - f->time < 1000;
    int32_t rate = (int64_t)a->e_rate * mult / 1000;
    int32_t target = a->f_target;
    if (fresh)
        target += (int64_t)rate * (int32_t)(now - f->time) / 1000;
    int32_t err = target - motion_position(lane);
    int64_t v = (int64_t)err * FOLLOW_GAIN + (fresh ? rate : 0);
    if (err > -200 && err < 200 && (!fresh || !a->e_rate))
        v = 0;                    // within 0.2 mm and the extruder is still
    if (v > assist_max)
        v = assist_max;
    if (v < -assist_max)
        v = -assist_max;
    motion_set_velocity(lane, v);
}

void
assist_stop(int lane)
{
    if (!as[lane].on)
        return;
    as[lane].on = 0;
    motion_set_velocity(lane, 0);
}

int
assist_active(int lane)
{
    return as[lane].on;
}

static void
assist_fault(int lane, const char *code, const char *msg)
{
    assist_stop(lane);
    motion_stop(lane, END_FAULT);
    ev_fault(code, lane, msg);
}

// The TurtleNeck multiplier (permille), AFC_buffer's advance_callback,
// trailing_callback and enable_buffer in one: the switch that changed last
// picks mult_low or mult_high, and with the clog check on a pressed switch
// (or no change since the assist started) scales it harder.
static int32_t
tn_multiplier(struct assist *a, int advance, int trailing)
{
    uint32_t edges = sensors_tn_edges();
    if (edges != a->tn_edges) {
        a->tn_edges = edges;
        a->start_held = 0;
    }
    int fault_on = assist_fault_mm > 0;
    if (sensors_tn_last() == SW_TN_ADV) {
        if (fault_on && (advance || a->start_held))
            return assist_low * 2 / 5;
        return assist_low;
    }
    if (fault_on && (trailing || a->start_held))
        return assist_high * 3 / 2;
    return assist_high;
}

static void
assist_update(int lane)
{
    struct assist *a = &as[lane];
    if (a->follow) {
        follow_update(lane, 1000);
        return;
    }
    struct efeed *f = &ef[lane];
    uint32_t now = uptime();
    uint32_t st = sensors_state();
    int trailing = (st >> SW_TN_TRL) & 1, advance = (st >> SW_TN_ADV) & 1;

    if (!((st >> SW_PREP(lane)) & 1)) {
        assist_fault(lane, "runout", "prep switch opened while printing");
        return;
    }

    int fresh = f->have && now - f->time < 1000;
    if (fresh) {
        follow_update(lane, tn_multiplier(a, advance, trailing));
    } else {
        // Start over from where the lane is once updates come back.
        a->f_init = 0;
        int32_t v = trailing ? assist_fallback : (advance ? 0 : -1);
        if (v > assist_max)
            v = assist_max;
        if (v >= 0)
            motion_set_velocity(lane, v);
    }

    // Fault check: a switch that stays latched while the extruder moves.
    if (assist_fault_mm && fresh) {
        int8_t latched = advance ? SW_TN_ADV : (trailing ? SW_TN_TRL : -1);
        if (latched != a->fault_sw) {
            a->fault_sw = latched;
            a->fault_start = a->e_pos;
        } else if (latched >= 0 && a->e_pos - a->fault_start
                   > assist_fault_mm) {
            if (latched == SW_TN_ADV)
                assist_fault(lane, "clog", "TurtleNeck stayed full while the"
                             " extruder moved (clog or grinding)");
            else
                assist_fault(lane, "not_feeding", "TurtleNeck stayed pulled"
                             " while the extruder moved (tangle or jam)");
        }
    }
}

/****************************************************************
 * Settings and commands
 ****************************************************************/

void
assist_cfg(const char *line)
{
    int32_t v;
    int b;
    if (json_get_milli(line, "assist_high", &v) && v >= 1000 && v <= 3000)
        assist_high = v;
    if (json_get_milli(line, "assist_low", &v) && v >= 0 && v <= 1000)
        assist_low = v;
    if (json_get_milli(line, "assist_fallback_speed", &v) && v > 0)
        assist_fallback = v;
    if (json_get_milli(line, "assist_max_speed", &v) && v > 0)
        assist_max = v;
    if (json_get_milli(line, "assist_fault_mm", &v) && v >= 0)
        assist_fault_mm = v;
    if (json_get_milli(line, "espool_fwd", &v) && v >= 0 && v <= 1000)
        esp_fwd_duty = v;
    if (json_get_milli(line, "espool_rwd", &v) && v >= 0 && v <= 1000)
        esp_rwd_duty = v;
    if (json_get_int(line, "espool_kick_ms", &v) && v >= 0 && v <= 1000)
        esp_kick_ms = v;
    if (json_get_int(line, "espool_brake_ms", &v) && v >= 0 && v <= 1000)
        esp_brake_ms = v;
    if (json_get_milli(line, "espool_pulse_mm", &v) && v >= 0)
        esp_pulse_mm = v;
    if (json_get_int(line, "espool_pulse_ms", &v) && v >= 0 && v <= 5000)
        esp_pulse_ms = v;
    if (json_get_bool(line, "espool_follow", &b))
        esp_follow = b;
    if (json_get_int(line, "espool_cycle_us", &v) && v >= 100 && v <= 100000
        && (uint32_t)v != esp_cycle_us) {
        if (esp_ready)
            ev_log("espool_cycle_us changes after a reboot");
        else
            esp_cycle_us = v;
    }
}

// Reads espool_fwd, espool_rwd (0..1) and espool_kick from a move or job
// line. They hold until the next move or job on the lane; a missing key
// uses the cfg default.
void
espool_move_cfg(int lane, const char *line)
{
    struct espool *e = &esp[lane];
    int32_t v;
    int b;
    e->move_fwd = e->move_rwd = -1;
    e->move_kick = 1;
    if (json_get_milli(line, "espool_fwd", &v) && v >= 0 && v <= 1000)
        e->move_fwd = v;
    if (json_get_milli(line, "espool_rwd", &v) && v >= 0 && v <= 1000)
        e->move_rwd = v;
    if (json_get_bool(line, "espool_kick", &b))
        e->move_kick = b;
}

// Reads espool_pulse_mm and espool_pulse_ms from a lane_cfg line (AFC works
// the pulse out from each spool's weight). A negative value goes back to
// the cfg default.
void
espool_lane_cfg(int lane, const char *line)
{
    struct espool *e = &esp[lane];
    int32_t v;
    if (json_get_milli(line, "espool_pulse_mm", &v))
        e->pulse_mm = v < 0 ? -1 : v;
    if (json_get_int(line, "espool_pulse_ms", &v))
        e->pulse_ms = v < 0 ? -1 : (v > 5000 ? 5000 : v);
}

int
spool_any_active(void)
{
    int i;
    for (i = 0; i < NUM_LANES; i++)
        if (as[i].on || esp[i].duty || esp[i].braking)
            return 1;
    return 0;
}

// {"cmd":"espool","lane":1,"duty":0.5,"kick":true}  (duty -1..1, 0 = off)
static void
cmd_espool(char *line)
{
    int lane, kick = 1;
    int32_t duty;
    if (!get_lane(line, &lane))
        return;
    if (!json_get_milli(line, "duty", &duty) || duty < -1000 || duty > 1000) {
        reply_err(line, "bad duty");
        return;
    }
    json_get_bool(line, "kick", &kick);
    espool_set(lane, duty, kick);
    reply_ok(line);
}

// {"cmd":"assist","lane":1,"on":true,"mode":"follow"}  (mode "tn" default,
// "follow", "ext"). Also takes espool_fwd, espool_rwd and espool_kick for
// follow mode.
static void
cmd_assist(char *line)
{
    int lane, on;
    if (!get_lane(line, &lane))
        return;
    if (!json_get_bool(line, "on", &on)) {
        reply_err(line, "no on");
        return;
    }
    if (on) {
        char mode[8] = "tn";
        json_get_str(line, "mode", mode, sizeof(mode));
        int follow;
        if (!strcmp(mode, "tn")) {
            follow = 0;
        } else if (!strcmp(mode, "follow")) {
            follow = ASSIST_FOLLOW;
        } else if (!strcmp(mode, "ext")) {
            follow = ASSIST_EXT;
        } else {
            reply_err(line, "bad mode");
            return;
        }
        if (motion_is_moving(lane) && !assist_active(lane)) {
            reply_err(line, "lane busy");
            return;
        }
        // Switching modes keeps the lane running; the new mode takes over.
        espool_move_cfg(lane, line);
        assist_start(lane, follow);
    } else {
        assist_stop(lane);
    }
    reply_ok(line);
}

// {"cmd":"e","lane":1,"pos":12345.678} the position lane 1 follows, a few
// times a second; with no lane it goes to every lane. No reply (it is sent
// often); bad lines are ignored.
static void
cmd_e(char *line)
{
    int32_t pos, lane;
    if (!json_get_milli(line, "pos", &pos))
        return;
    uint32_t now = uptime();
    if (!now)
        now = 1;
    int first = 0, last = NUM_LANES - 1;
    if (json_get_int(line, "lane", &lane)) {
        if (lane < 1 || lane > NUM_LANES)
            return;
        first = last = lane - 1;
    }
    int i;
    for (i = first; i <= last; i++) {
        ef[i].pos = pos;
        ef[i].time = now;
        ef[i].have = 1;
    }
}

int
spool_handle_cmd(const char *cmd, char *line)
{
    if (!strcmp(cmd, "espool"))
        cmd_espool(line);
    else if (!strcmp(cmd, "assist"))
        cmd_assist(line);
    else if (!strcmp(cmd, "e"))
        cmd_e(line);
    else
        return 0;
    return 1;
}

/****************************************************************
 * Task
 ****************************************************************/

static uint32_t last_run;

void
spool_task(void)
{
    uint32_t now = uptime();
    if (now == last_run)
        return;
    last_run = now;
    int i;
    for (i = 0; i < NUM_LANES; i++) {
        struct espool *e = &esp[i];
        if (e->braking && (int32_t)(now - e->brake_until) >= 0) {
            e->braking = 0;
            apply(i, 0);
        }
        if (e->kick_until && (int32_t)(now - e->kick_until) >= 0) {
            e->kick_until = 0;
            apply(i, e->target);
        }
        if (as[i].on)
            assist_update(i);
        if (e->manual)
            continue;
        int32_t v = motion_get_velocity(i);
        if (as[i].on && as[i].follow != ASSIST_FOLLOW) {
            // Print assist pulses every espool_pulse_mm the lane feeds.
            int32_t pos = motion_position(i);
            int32_t pmm = e->pulse_mm >= 0 ? e->pulse_mm : esp_pulse_mm;
            int32_t pms = e->pulse_ms >= 0 ? e->pulse_ms
                                           : (int32_t)esp_pulse_ms;
            if (pmm && pms && pos - e->last_pulse_pos > pmm) {
                e->last_pulse_pos = pos;
                e->pulse_until = now + pms;
                espool_drive(i, 1000, 1);
            } else if (e->pulse_until
                       && (int32_t)(now - e->pulse_until) >= 0) {
                e->pulse_until = 0;
                espool_drive(i, 0, 0);
            }
            continue;
        }
        if (!esp_follow)
            continue;
        int32_t fwd = e->move_fwd >= 0 ? e->move_fwd : esp_fwd_duty;
        int32_t rwd = e->move_rwd >= 0 ? e->move_rwd : esp_rwd_duty;
        if (v < 0)
            espool_drive(i, -rwd, e->move_kick);
        else if (v > 0)
            espool_drive(i, fwd, e->move_kick);
        else
            espool_drive(i, 0, 0);
    }
}
DECL_TASK(spool_task);

void
spool_init(void)
{
    int i;
    for (i = 0; i < NUM_LANES; i++) {
        // Motors off until the PWM timers are set up on first use.
        gpio_out_setup(LANE_PINS[i].mot_fwd, 0);
        gpio_out_setup(LANE_PINS[i].mot_rwd, 0);
        esp[i].en = gpio_out_setup(LANE_PINS[i].mot_en, 0);
        esp[i].move_fwd = esp[i].move_rwd = -1;
        esp[i].move_kick = 1;
        esp[i].pulse_mm = esp[i].pulse_ms = -1;
        as[i].fault_sw = -1;
    }
}

void
spool_shutdown(void)
{
    int i;
    for (i = 0; i < NUM_LANES; i++) {
        if (esp_ready) {
            gpio_pwm_write(esp[i].fwd, 0);
            gpio_pwm_write(esp[i].rwd, 0);
        }
        gpio_out_write(esp[i].en, 0);
    }
}
DECL_SHUTDOWN(spool_shutdown);
