// WS2812/SK6812 LED strings on RGB1-RGB4.
//
// The bit timing follows Klipper's src/neopixel.c (Kevin O'Connor, GPLv3).
// Strips are sent from task context; a send that an interrupt disturbed is
// retried a few times.
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h>
#include "compiler.h" // DIV_ROUND_UP
#include "board/gpio.h" // gpio_out_setup
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_read_time
#include "sched.h" // DECL_TASK
#include "board.h"

struct strip {
    struct gpio_out pin;
    uint16_t count;
    uint8_t bpp;        // 3 or 4 bytes per LED
    uint8_t order[4];   // index into r,g,b,w for each byte sent
    uint8_t dirty;
    uint32_t last_send;
    uint8_t data[STRIP_MAX_LEDS * 4];
};

static struct strip strips[NUM_STRIPS];
static struct task_wake led_wake;

static uint32_t
ns_ticks(uint32_t ns)
{
    return DIV_ROUND_UP(timer_from_us(ns * 1000), 1000000);
}

static inline void
delay_from(uint32_t start, uint32_t ticks)
{
    while (timer_read_time() - start < ticks)
        ;
}

static int
send_strip(struct strip *s)
{
    uint32_t long_t = ns_ticks(800), edge_t = ns_ticks(200);
    uint32_t bit_t = ns_ticks(1250), max_t = ns_ticks(4000);
    uint32_t reset_t = ns_ticks(80000);
    while (timer_read_time() - s->last_send < reset_t)
        irq_poll();
    struct gpio_out pin = s->pin;
    uint8_t *data = s->data;
    int len = s->count * s->bpp;
    uint32_t last = timer_read_time();
    while (len--) {
        uint8_t byte = *data++;
        int bits = 8;
        while (bits--) {
            if (byte & 0x80) {
                delay_from(last, bit_t);
                irq_disable();
                uint32_t start = timer_read_time();
                gpio_out_toggle_noirq(pin);
                irq_enable();
                if (start - last >= max_t)
                    goto fail;
                last = start;
                delay_from(start, long_t);
                irq_disable();
                gpio_out_toggle_noirq(pin);
                irq_enable();
                delay_from(timer_read_time(), edge_t);
            } else {
                delay_from(last, bit_t);
                irq_disable();
                uint32_t start = timer_read_time();
                gpio_out_toggle_noirq(pin);
                delay_from(start, edge_t);
                gpio_out_toggle_noirq(pin);
                irq_enable();
                if (start - last >= max_t)
                    goto fail;
                last = start;
            }
            byte <<= 1;
        }
    }
    s->last_send = timer_read_time();
    return 0;
fail:
    gpio_out_write(pin, 0);
    s->last_send = timer_read_time();
    return -1;
}

int
leds_config(int strip, int count, const char *order)
{
    if (strip < 0 || strip >= NUM_STRIPS || count < 0
        || count > STRIP_MAX_LEDS)
        return 0;
    int bpp = strlen(order);
    if (bpp != 3 && bpp != 4)
        return 0;
    struct strip *s = &strips[strip];
    int i;
    uint8_t ord[4];
    for (i = 0; i < bpp; i++) {
        switch (order[i]) {
        case 'R': ord[i] = 0; break;
        case 'G': ord[i] = 1; break;
        case 'B': ord[i] = 2; break;
        case 'W': ord[i] = 3; break;
        default: return 0;
        }
    }
    memcpy(s->order, ord, 4);
    s->bpp = bpp;
    s->count = count;
    memset(s->data, 0, sizeof(s->data));
    s->dirty = 1;
    sched_wake_task(&led_wake);
    return 1;
}

int
leds_set(int strip, int first, int count, uint8_t r, uint8_t g,
         uint8_t b, uint8_t w)
{
    if (strip < 0 || strip >= NUM_STRIPS)
        return 0;
    struct strip *s = &strips[strip];
    if (first < 0 || count < 1 || first + count > s->count)
        return 0;
    uint8_t rgbw[4] = {r, g, b, w};
    int i, j;
    for (i = first; i < first + count; i++)
        for (j = 0; j < s->bpp; j++)
            s->data[i * s->bpp + j] = rgbw[s->order[j]];
    return 1;
}

void
leds_commit(int strip)
{
    if (strip < 0 || strip >= NUM_STRIPS)
        return;
    strips[strip].dirty = 1;
    sched_wake_task(&led_wake);
}

int
leds_count(int strip)
{
    if (strip < 0 || strip >= NUM_STRIPS)
        return 0;
    return strips[strip].count;
}

void
leds_init(void)
{
    static const uint32_t pins[NUM_STRIPS] = {
        PIN_RGB1, PIN_RGB2, PIN_RGB3, PIN_RGB4 };
    int i;
    // Every header stays dark until the host says what is wired to it.
    for (i = 0; i < NUM_STRIPS; i++) {
        strips[i].pin = gpio_out_setup(pins[i], 0);
        leds_config(i, 0, "GRB");
    }
}

void
leds_task(void)
{
    if (!sched_check_wake(&led_wake))
        return;
    int i;
    for (i = 0; i < NUM_STRIPS; i++) {
        struct strip *s = &strips[i];
        if (!s->dirty || !s->count)
            continue;
        s->dirty = 0;
        int tries;
        for (tries = 0; tries < 3; tries++)
            if (!send_strip(s))
                break;
    }
}
DECL_TASK(leds_task);
