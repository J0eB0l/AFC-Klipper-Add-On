// Lane stepper motion for the BoxTurtle firmware.
//
// Each lane has its own step timer. Speed changes happen in the 1 ms sensor
// tick (sensors_tick_hook), which also checks the stop ("until") and keep
// ("watch") switches, so a move ends within about 1 ms plus the switch
// debounce. The drivers run in dedge mode: every toggle of STEP is a step.
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stddef.h> // offsetof
#include "autoconf.h" // CONFIG_CLOCK_FREQ
#include "compiler.h" // container_of
#include "board/gpio.h" // gpio_out_setup
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_read_time
#include "sched.h" // sched_add_timer
#include "board.h"
#include "events.h"
#include "motion.h"

#define MAX_STEP_RATE 200000   // steps/s (2000 ticks per step at 400 MHz)
#define MIN_STEP_RATE 200      // slowest step rate while moving

enum { MODE_IDLE, MODE_MOVE, MODE_VELOCITY };

struct lane {
    struct timer timer;
    struct gpio_out step, dir, en;
    int32_t spm_milli;          // steps per mm * 1000
    uint8_t dir_invert, en_invert, enabled, hold;
    volatile uint8_t mode, stepping, done, end;
    int8_t dirn;                // +1 / -1 for the running motion
    int8_t until_sw, until_v, watch_sw, watch_v;
    volatile int32_t pos;       // steps since boot, + = toward toolhead
    int32_t start_pos;
    volatile uint32_t steps_left;
    uint32_t v, vmax, accel;    // steps/s and steps/s^2
    int32_t vel_target;         // signed steps/s for MODE_VELOCITY
    int32_t vel_now;
    volatile uint32_t interval;
};

static struct lane lanes[NUM_LANES];
static struct task_wake motion_wake;

const char *
end_name(int end)
{
    switch (end) {
    case END_DONE: return "done";
    case END_TRIGGERED: return "triggered";
    case END_NO_TRIGGER: return "no_trigger";
    case END_WATCH: return "watch";
    case END_STOPPED: return "stopped";
    case END_FAULT: return "fault";
    case END_LINK: return "link_lost";
    }
    return "none";
}

static int64_t
mm_to_steps(struct lane *l, int64_t milli_mm)
{
    return milli_mm * l->spm_milli / 1000000;
}

static int32_t
steps_to_mm(struct lane *l, int64_t steps)
{
    return steps * 1000000 / l->spm_milli;
}

static void
set_enable(struct lane *l, int on)
{
    gpio_out_write(l->en, on ^ l->en_invert);
    l->enabled = on;
}

static void
set_dir(struct lane *l, int dirn)
{
    l->dirn = dirn;
    gpio_out_write(l->dir, (dirn > 0) ^ l->dir_invert);
}

static uint32_t
rate_interval(uint32_t v)
{
    if (v < MIN_STEP_RATE)
        v = MIN_STEP_RATE;
    if (v > MAX_STEP_RATE)
        v = MAX_STEP_RATE;
    return CONFIG_CLOCK_FREQ / v;
}

// Ends the running motion (irq context or irqs off).
static void
finish(struct lane *l, int end)
{
    l->stepping = 0;
    l->mode = MODE_IDLE;
    l->end = end;
    l->done = 1;
    l->vel_now = l->vel_target = 0;
    if (!l->hold)
        set_enable(l, 0);
    sched_wake_task(&motion_wake);
}

static uint_fast8_t
step_event(struct timer *t)
{
    struct lane *l = container_of(t, struct lane, timer);
    if (!l->stepping)
        return SF_DONE;
    gpio_out_toggle_noirq(l->step);
    l->pos += l->dirn;
    if (l->mode == MODE_MOVE && !--l->steps_left) {
        finish(l, l->until_sw >= 0 ? END_NO_TRIGGER : END_DONE);
        return SF_DONE;
    }
    t->waketime += l->interval;
    return SF_RESCHEDULE;
}

static void
start_stepping(struct lane *l)
{
    if (l->stepping)
        return;
    l->stepping = 1;
    l->timer.func = step_event;
    l->timer.waketime = timer_read_time() + timer_from_us(100);
    sched_add_timer(&l->timer);
}

static void
stop_stepping(struct lane *l)
{
    if (!l->stepping)
        return;
    l->stepping = 0;
    sched_del_timer(&l->timer);
}

// 1 ms tick from the sensor timer (irq context).
void
sensors_tick_hook(uint32_t state, uint32_t changed)
{
    int i;
    for (i = 0; i < NUM_LANES; i++) {
        struct lane *l = &lanes[i];
        uint32_t dv = l->accel / 1000;
        if (!dv)
            dv = 1;
        if (l->mode == MODE_MOVE) {
            if (l->until_sw >= 0
                && ((state >> l->until_sw) & 1) == (uint32_t)l->until_v) {
                stop_stepping(l);
                finish(l, END_TRIGGERED);
                continue;
            }
            if (l->watch_sw >= 0
                && ((state >> l->watch_sw) & 1) != (uint32_t)l->watch_v) {
                stop_stepping(l);
                finish(l, END_WATCH);
                continue;
            }
            uint64_t stop_dist = (uint64_t)l->v * l->v / (2 * l->accel);
            if (l->steps_left <= stop_dist) {
                l->v = l->v > dv + MIN_STEP_RATE ? l->v - dv : MIN_STEP_RATE;
            } else if (l->v < l->vmax) {
                l->v += dv;
                if (l->v > l->vmax)
                    l->v = l->vmax;
            }
            l->interval = rate_interval(l->v);
        } else if (l->mode == MODE_VELOCITY) {
            int32_t tgt = l->vel_target, cur = l->vel_now;
            if (cur < tgt)
                cur = cur + (int32_t)dv > tgt ? tgt : cur + (int32_t)dv;
            else if (cur > tgt)
                cur = cur - (int32_t)dv < tgt ? tgt : cur - (int32_t)dv;
            l->vel_now = cur;
            uint32_t mag = cur < 0 ? -cur : cur;
            if (mag < MIN_STEP_RATE) {
                stop_stepping(l);
                if (!tgt && !cur) {
                    l->mode = MODE_IDLE;
                    if (!l->hold)
                        set_enable(l, 0);
                }
                continue;
            }
            int dirn = cur > 0 ? 1 : -1;
            if (dirn != l->dirn || !l->stepping) {
                stop_stepping(l);
                set_dir(l, dirn);
            }
            l->interval = rate_interval(mag);
            start_stepping(l);
        }
    }
}

/****************************************************************
 * Task-context API
 ****************************************************************/

const char *
motion_start(int lane, const struct move_req *m)
{
    struct lane *l = &lanes[lane];
    if (l->mode != MODE_IDLE)
        return "lane busy";
    if (m->speed <= 0 || m->accel <= 0)
        return "bad speed";
    if (m->watch_sw >= 0 && sensor(m->watch_sw) != m->watch_v)
        return "watch switch not in its state (no filament?)";
    int64_t steps = mm_to_steps(l, m->mm < 0 ? -(int64_t)m->mm : m->mm);
    if (steps <= 0)
        return "too short";
    if (steps > 0x7fffffff)
        return "too long";
    uint32_t vmax = mm_to_steps(l, m->speed);
    uint32_t accel = mm_to_steps(l, m->accel);
    if (vmax > MAX_STEP_RATE)
        vmax = MAX_STEP_RATE;
    if (vmax < MIN_STEP_RATE)
        vmax = MIN_STEP_RATE;
    if (!accel)
        accel = 1;
    irq_disable();
    l->until_sw = m->until_sw;
    l->until_v = m->until_v;
    l->watch_sw = m->watch_sw;
    l->watch_v = m->watch_v;
    l->hold = m->hold;
    l->steps_left = steps;
    l->vmax = vmax;
    l->accel = accel;
    l->v = accel / 1000 > MIN_STEP_RATE ? accel / 1000 : MIN_STEP_RATE;
    if (l->v > vmax)
        l->v = vmax;
    l->interval = rate_interval(l->v);
    l->start_pos = l->pos;
    l->done = 0;
    l->end = END_NONE;
    set_enable(l, 1);
    set_dir(l, m->mm > 0 ? 1 : -1);
    l->mode = MODE_MOVE;
    start_stepping(l);
    irq_enable();
    return 0;
}

void
motion_stop(int lane, int end)
{
    int i;
    for (i = 0; i < NUM_LANES; i++) {
        if (lane >= 0 && i != lane)
            continue;
        struct lane *l = &lanes[i];
        irq_disable();
        if (l->mode != MODE_IDLE) {
            stop_stepping(l);
            int hold = l->hold;
            if (end == END_FAULT)
                l->hold = 0;
            finish(l, end);
            l->hold = hold;
        }
        irq_enable();
    }
}

void
motion_fault(int lane, const char *code)
{
    motion_stop(lane, END_FAULT);
    motion_enable(lane, 0);
}

int
motion_is_moving(int lane)
{
    return lanes[lane].mode != MODE_IDLE;
}

int
motion_any_active(void)
{
    int i;
    for (i = 0; i < NUM_LANES; i++)
        if (lanes[i].mode != MODE_IDLE)
            return 1;
    return 0;
}

int
motion_take_done(int lane, int *end, int32_t *moved)
{
    struct lane *l = &lanes[lane];
    irq_disable();
    int done = l->done;
    l->done = 0;
    int32_t steps = l->pos - l->start_pos;
    int e = l->end;
    irq_enable();
    if (!done)
        return 0;
    *end = e;
    *moved = steps_to_mm(l, steps);
    return 1;
}

void
motion_enable(int lane, int on)
{
    struct lane *l = &lanes[lane];
    irq_disable();
    if (!on && l->mode != MODE_IDLE) {
        irq_enable();
        return;
    }
    l->hold = on;
    set_enable(l, on);
    irq_enable();
}

int
motion_enabled(int lane)
{
    return lanes[lane].enabled;
}

void
motion_set_lane_cfg(int lane, int32_t spm_milli, int dir_invert)
{
    struct lane *l = &lanes[lane];
    if (spm_milli > 0)
        l->spm_milli = spm_milli;
    if (dir_invert >= 0)
        l->dir_invert = dir_invert;
}

int32_t
motion_steps_per_mm(int lane)
{
    return lanes[lane].spm_milli;
}

void
motion_set_velocity(int lane, int32_t speed)
{
    struct lane *l = &lanes[lane];
    int64_t s = mm_to_steps(l, speed < 0 ? -(int64_t)speed : speed);
    if (s > MAX_STEP_RATE)
        s = MAX_STEP_RATE;
    int32_t tgt = speed < 0 ? -s : s;
    irq_disable();
    if (l->mode == MODE_MOVE) {
        irq_enable();
        return;
    }
    if (l->mode == MODE_IDLE) {
        if (!tgt) {
            irq_enable();
            return;
        }
        if (!l->accel)
            l->accel = mm_to_steps(l, 500000);
        l->vel_now = 0;
        l->start_pos = l->pos;
        set_enable(l, 1);
        l->mode = MODE_VELOCITY;
    }
    l->vel_target = tgt;
    irq_enable();
}

int32_t
motion_get_velocity(int lane)
{
    struct lane *l = &lanes[lane];
    return steps_to_mm(l, l->mode == MODE_VELOCITY ? l->vel_now
                       : (l->mode == MODE_MOVE ? (int32_t)l->v * l->dirn : 0));
}

int32_t
motion_position(int lane)
{
    return steps_to_mm(&lanes[lane], lanes[lane].pos);
}

void
motion_init(void)
{
    int i;
    for (i = 0; i < NUM_LANES; i++) {
        struct lane *l = &lanes[i];
        // Stock BoxTurtle: microsteps 16, rotation_distance 4.65 -> 688.172 steps/mm
        l->spm_milli = 688172;
        l->dir_invert = 1;   // dir_pin: !Mx_DIR
        l->en_invert = 1;    // enable_pin: !Mx_EN
        l->step = gpio_out_setup(LANE_PINS[i].step, 0);
        l->dir = gpio_out_setup(LANE_PINS[i].dir, 0);
        l->en = gpio_out_setup(LANE_PINS[i].en, 1); // high = disabled
        l->until_sw = l->watch_sw = -1;
        l->mode = MODE_IDLE;
    }
}

void
motion_shutdown(void)
{
    int i;
    for (i = 0; i < NUM_LANES; i++) {
        lanes[i].stepping = 0;
        lanes[i].mode = MODE_IDLE;
        gpio_out_write(lanes[i].en, lanes[i].en_invert); // disable
    }
}
DECL_SHUTDOWN(motion_shutdown);
