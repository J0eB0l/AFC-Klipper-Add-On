// Switch sampling and debounce for the BoxTurtle firmware.
//
// A 1 ms timer samples every switch. A switch changes state only after it
// reads the same for debounce_ms samples in a row. Changes are sent to the
// host from task context as {"ev":"sw",...} lines.
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "board/gpio.h" // gpio_in_setup
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_read_time
#include "sched.h" // DECL_TASK
#include "board.h"
#include "events.h"

const char * const SW_NAMES[SW_COUNT] = {
    "hub", "prep1", "prep2", "prep3", "prep4",
    "load1", "load2", "load3", "load4", "tn_adv", "tn_trl",
};

static struct gpio_in sw_gpio[SW_COUNT];
static uint8_t sw_count[SW_COUNT];
static uint32_t sw_invert;
static volatile uint32_t sw_state, sw_pending, uptime_ms;
static uint8_t debounce_ms = 3;
static struct timer sensor_timer;
static struct task_wake sensor_wake;
static volatile int8_t tn_last = -1;
static volatile uint32_t tn_edges;

// Defaults match the stock AFC_Turtle_1.cfg: prep is ^! (inverted),
// load, hub and TurtleNeck are ^ (not inverted). All pins pull up.
static const uint32_t DEFAULT_INVERT = (1 << SW_PREP1) | (1 << SW_PREP2)
    | (1 << SW_PREP3) | (1 << SW_PREP4);

#define TN_MASK ((1 << SW_TN_ADV) | (1 << SW_TN_TRL))

// Like AFC_buffer's switch callbacks: a TurtleNeck switch that changes,
// pressed or released, becomes the last one.
static void
tn_changed(uint32_t changed, uint32_t state)
{
    uint32_t tn = changed & TN_MASK;
    if (!tn)
        return;
    if (tn == TN_MASK)
        tn &= state;              // both at once: the one now pressed
    tn_last = (tn & (1 << SW_TN_TRL)) ? SW_TN_TRL : SW_TN_ADV;
    tn_edges++;
}

static uint_fast8_t
sensor_event(struct timer *t)
{
    uint32_t state = sw_state, changed = 0;
    uptime_ms++;
    int i;
    for (i = 0; i < SW_COUNT; i++) {
        int raw = !!gpio_in_read(sw_gpio[i]);
        int trig = raw ^ ((sw_invert >> i) & 1);
        int cur = (state >> i) & 1;
        if (trig == cur) {
            sw_count[i] = 0;
            continue;
        }
        if (++sw_count[i] >= debounce_ms) {
            sw_count[i] = 0;
            state ^= 1 << i;
            changed |= 1 << i;
        }
    }
    sw_state = state;
    tn_changed(changed, state);
    sensors_tick_hook(state, changed);
    if (changed) {
        sw_pending |= changed;
        sched_wake_task(&sensor_wake);
    }
    t->waketime += timer_from_us(1000);
    return SF_RESCHEDULE;
}

void
sensors_init(void)
{
    uint8_t pins[SW_COUNT];
    pins[SW_HUB] = PIN_HUB;
    int i;
    for (i = 0; i < NUM_LANES; i++) {
        pins[SW_PREP(i)] = LANE_PINS[i].prep;
        pins[SW_LOAD(i)] = LANE_PINS[i].load;
    }
    pins[SW_TN_ADV] = PIN_TN_ADV;
    pins[SW_TN_TRL] = PIN_TN_TRL;
    sw_invert = DEFAULT_INVERT;
    uint32_t state = 0;
    for (i = 0; i < SW_COUNT; i++) {
        sw_gpio[i] = gpio_in_setup(pins[i], 1);
    }
    // Let the pull-ups settle before taking the first reading.
    uint32_t start = timer_read_time();
    while (timer_read_time() - start < timer_from_us(200))
        ;
    for (i = 0; i < SW_COUNT; i++) {
        int raw = !!gpio_in_read(sw_gpio[i]);
        if (raw ^ ((sw_invert >> i) & 1))
            state |= 1 << i;
    }
    sw_state = state;
    // A switch pressed at power up counts as the last one (like the host's
    // first status read).
    if (state & (1 << SW_TN_ADV))
        tn_last = SW_TN_ADV;
    else if (state & (1 << SW_TN_TRL))
        tn_last = SW_TN_TRL;
    sensor_timer.func = sensor_event;
    sensor_timer.waketime = timer_read_time() + timer_from_us(1000);
    irq_disable();
    sched_add_timer(&sensor_timer);
    irq_enable();
}

uint32_t
uptime(void)
{
    return uptime_ms;
}

uint32_t
sensors_state(void)
{
    return sw_state;
}

int
sensors_tn_last(void)
{
    return tn_last;
}

uint32_t
sensors_tn_edges(void)
{
    return tn_edges;
}

void
sensors_set_invert(int sw, int invert)
{
    if (sw < 0 || sw >= SW_COUNT)
        return;
    irq_disable();
    uint32_t bit = 1 << sw;
    uint32_t old = (sw_invert & bit) ? 1 : 0;
    if (old != !!invert) {
        sw_invert ^= bit;
        // The pin did not change, so the triggered state flips at once.
        sw_state ^= bit;
        tn_changed(bit, sw_state);
        sw_pending |= bit;
        sched_wake_task(&sensor_wake);
    }
    irq_enable();
}

int
sensors_get_invert(int sw)
{
    return (sw_invert >> sw) & 1;
}

void
sensors_set_debounce(int ms)
{
    if (ms < 1)
        ms = 1;
    if (ms > 50)
        ms = 50;
    debounce_ms = ms;
}

void
sensors_task(void)
{
    if (!sched_check_wake(&sensor_wake))
        return;
    irq_disable();
    uint32_t pending = sw_pending, state = sw_state;
    sw_pending = 0;
    irq_enable();
    int i;
    for (i = 0; i < SW_COUNT; i++)
        if (pending & (1 << i))
            ev_switch(i, (state >> i) & 1);
}
DECL_TASK(sensors_task);
