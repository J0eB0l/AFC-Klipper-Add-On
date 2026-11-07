// Host-side simulator for the BoxTurtle firmware.
//
// Builds the real firmware sources (json, link, events, sensors, proto,
// motion, cmds, spool, jobs) against stand-ins for Klipper's scheduler and
// GPIO, and a simple physical model of a BoxTurtle: four lanes, prep and
// load switches, a hub, a bowden, a TurtleNeck in front of the extruder.
// Scenarios send JSON lines like AFC would and check the replies/events.
//
//   make -C test/sim && ./test/sim/sim
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "board/gpio.h"
#include "board/internal.h"
#include "board/misc.h"
#include "sched.h"
#include "board.h"
#include "link.h"

/****************************************************************
 * Clock and timers
 ****************************************************************/

static uint64_t now; // ticks at 400 MHz
#define MS (400000ull)

// Each read moves the clock a little, so firmware busy-waits end.
uint32_t timer_read_time(void) { now += 4; return (uint32_t)now; }

#define MAX_TIMERS 16
static struct timer *timers[MAX_TIMERS];
static int ntimers;

void
sched_add_timer(struct timer *t)
{
    int i;
    for (i = 0; i < ntimers; i++)
        if (timers[i] == t) {
            printf("BUG: timer added twice\n");
            exit(2);
        }
    timers[ntimers++] = t;
}

void
sched_del_timer(struct timer *t)
{
    int i;
    for (i = 0; i < ntimers; i++)
        if (timers[i] == t) {
            timers[i] = timers[--ntimers];
            return;
        }
}

void sched_wake_task(struct task_wake *w) { w->wake = 1; }
uint8_t
sched_check_wake(struct task_wake *w)
{
    uint8_t v = w->wake;
    w->wake = 0;
    return v;
}

struct sim_rcc sim_rcc = { RCC_RSR_PORRSTF };
uint32_t sim_uid[3] = { 0x00390041, 0x3433510a, 0x32303536 };
void NVIC_SystemReset(void) {}
void bootloader_request(void) {}
struct sim_guard_mem sim_guard;
void try_request_canboot(void) {}

/****************************************************************
 * TMC stand-in (always healthy)
 ****************************************************************/

#include "tmc.h"
#include "guard.h"
static struct tmc_cfg tcfg[NUM_LANES];
int tmc_write(int lane, uint8_t reg, uint32_t val) { return 0; }
int tmc_read(int lane, uint8_t reg, uint32_t *val) { *val = 0; return 0; }
int tmc_wtest(int lane, int flags, int gap, int n, uint8_t reg, uint32_t val, uint32_t *b,
              uint32_t *a) { *b = *a = 0; return 0; }
int tmc_setup(int lane, const struct tmc_cfg *c) { tcfg[lane] = *c; return 0; }
void tmc_default_cfg(struct tmc_cfg *c) { memset(c, 0, sizeof(*c)); }
int tmc_health(int lane) { return TMC_OK; }
const char *tmc_health_name(int code) { return code ? "tmc_bad" : "ok"; }
void tmc_init(void) {}
void tmc_reconfigure(int lane, const struct tmc_cfg *c) { tcfg[lane] = *c; }
const struct tmc_cfg *tmc_get_cfg(int lane) { return &tcfg[lane]; }

// LED stand-ins (the real leds.c busy-waits on the clock).
int leds_config(int strip, int count, const char *order) { return 1; }
int leds_set(int strip, int first, int count, uint8_t r, uint8_t g, uint8_t b,
             uint8_t w) { return 1; }
void leds_commit(int strip) {}
int leds_count(int strip) { return 0; }
void leds_init(void) {}

/****************************************************************
 * Physical model
 ****************************************************************/

#define STEPS_PER_MM 688.172
static double dist_hub = 150, load_at = 20, bowden = 1800;
static double gears;            // dist_hub + bowden
struct mlane {
    int present;                // spool loaded past prep
    double tip;                 // filament tip, mm past the prep switch
    long steps;                 // net steps the drive made
    double snag_at;             // tip cannot pass this (0 = none)
    int snag_holds;             // how many approaches the snag stops
    int snag_armed;
};
static struct mlane ml[NUM_LANES];
static int hub_dead;            // hub switch never triggers
static double buf_c;            // TurtleNeck compression (mm), tip at gears
static double e_rate;           // extruder consumption (mm/s)
static double e_cmd;            // extruder position the host reports
static int clog;                // extruder turns but nothing is consumed
static double starved_ms, full_ms;
static double slip_mm;          // drive pushed into a full TurtleNeck (grinding)
static double buf_max;          // most the TurtleNeck was compressed
static double buf_hand = -1;    // TurtleNeck held by hand here, -1 = free
static double feed_scale = 1.0; // real feed per commanded mm (calibration error)
static int pin_level[256];      // output pins
static int step_pin_lane[256];
static int pwm_val[256];

static int
lane_at_gears(int i)
{
    return ml[i].present && ml[i].tip >= gears - 1e-9;
}

static void
model_step(int lane, int dir)
{
    struct mlane *m = &ml[lane];
    m->steps += dir;
    if (!m->present)
        return;
    double d = dir / STEPS_PER_MM * feed_scale;
    if (d > 0 && m->snag_at && m->snag_holds && m->tip < m->snag_at
        && m->tip + d >= m->snag_at) {
        // Hits the snag: the drive slips.
        m->snag_armed = 1;
        return;
    }
    if (d < 0 && m->snag_armed) {
        // Backing off clears this snag approach.
        m->snag_armed = 0;
        m->snag_holds--;
    }
    if (d > 0 && m->tip >= gears) {
        // At the gears: pushing compresses the TurtleNeck, up to 18 mm.
        if (buf_c + d > 18) {
            slip_mm += d; // slips: the drive grinds the filament
            return;
        }
        buf_c += d;
        if (buf_c > buf_max)
            buf_max = buf_c;
        return;
    }
    if (d < 0 && m->tip >= gears && buf_c > -3) {
        buf_c += d;
        if (buf_c >= 0)
            return;
        // Pulled past relaxed: tip leaves the gears.
        m->tip = gears + buf_c;
        buf_c = 0;
        return;
    }
    m->tip += d;
    if (m->tip > gears)
        m->tip = gears;
}

static int
sw_value(int sw)
{
    int i;
    switch (sw) {
    case SW_HUB:
        if (hub_dead)
            return 0;
        for (i = 0; i < NUM_LANES; i++)
            if (ml[i].present && ml[i].tip >= dist_hub)
                return 1;
        return 0;
    case SW_TN_ADV:
        for (i = 0; i < NUM_LANES; i++)
            if (lane_at_gears(i))
                return buf_c >= 12;
        return 0;
    case SW_TN_TRL:
        for (i = 0; i < NUM_LANES; i++)
            if (lane_at_gears(i))
                return buf_c <= 2;
        return 0;
    }
    for (i = 0; i < NUM_LANES; i++) {
        if (sw == SW_PREP(i))
            return ml[i].present;
        if (sw == SW_LOAD(i))
            return ml[i].present && ml[i].tip >= load_at;
    }
    return 0;
}

static void
model_ms(void)
{
    // Extruder consumption with a lane at the gears.
    int i;
    for (i = 0; i < NUM_LANES; i++) {
        if (!lane_at_gears(i))
            continue;
        if (!clog && e_rate > 0) {
            buf_c -= e_rate / 1000.0;
            if (buf_c < -3) {
                buf_c = -3;
                starved_ms += 1;
            }
        }
        if (buf_c >= 12)
            full_ms += 1;
    }
    e_cmd += e_rate / 1000.0;
    if (buf_hand >= 0)
        buf_c = buf_hand;
}

/****************************************************************
 * GPIO stand-ins
 ****************************************************************/

struct gpio_out
gpio_out_setup(uint32_t pin, uint32_t val)
{
    pin_level[pin] = val;
    return (struct gpio_out){pin};
}

void
gpio_out_write(struct gpio_out g, uint32_t val)
{
    pin_level[g.pin] = !!val;
}

void
gpio_out_toggle_noirq(struct gpio_out g)
{
    pin_level[g.pin] ^= 1;
    int lane = step_pin_lane[g.pin];
    if (lane >= 0) {
        int dirpin = LANE_PINS[lane].dir;
        // dir_pin is inverted in the default config: low = toward the head
        int dir = pin_level[dirpin] ? -1 : 1;
        if (pin_level[LANE_PINS[lane].en]) {
            printf("BUG: step on lane %d with the driver disabled\n", lane + 1);
            exit(2);
        }
        model_step(lane, dir);
    }
}

struct gpio_in
gpio_in_setup(uint32_t pin, int32_t pull_up)
{
    return (struct gpio_in){pin};
}

uint8_t
gpio_in_read(struct gpio_in g)
{
    int sw;
    for (sw = 0; sw < SW_COUNT; sw++) {
        int pin;
        if (sw == SW_HUB) pin = PIN_HUB;
        else if (sw == SW_TN_ADV) pin = PIN_TN_ADV;
        else if (sw == SW_TN_TRL) pin = PIN_TN_TRL;
        else if (sw >= SW_PREP1 && sw <= SW_PREP4)
            pin = LANE_PINS[sw - SW_PREP1].prep;
        else
            pin = LANE_PINS[sw - SW_LOAD1].load;
        if (pin == (int)g.pin) {
            int v = sw_value(sw);
            // prep switches are wired inverted (^!), the rest are not
            if (sw >= SW_PREP1 && sw <= SW_PREP4)
                v = !v;
            return v;
        }
    }
    return 0;
}

struct gpio_pwm
gpio_pwm_setup(uint8_t pin, uint32_t cycle_time, uint32_t val)
{
    pwm_val[pin] = val;
    return (struct gpio_pwm){pin};
}

void
gpio_pwm_write(struct gpio_pwm g, uint32_t val)
{
    pwm_val[g.pin] = val;
}

/****************************************************************
 * Firmware glue
 ****************************************************************/

void turtle_init(void);
void sensors_task(void);
void cmds_task(void);
void spool_task(void);
void jobs_task(void);

static char out_log[1 << 20];
static int out_len;
static int verbose;

static void
drain_output(void)
{
    uint8_t buf[256];
    int n;
    while ((n = turtle_link_tx_fill(buf, sizeof(buf))) > 0) {
        if (out_len + n < (int)sizeof(out_log) - 1) {
            memcpy(out_log + out_len, buf, n);
            out_len += n;
            out_log[out_len] = 0;
        }
        if (verbose)
            fwrite(buf, 1, n, stdout);
    }
}

static void
run_tasks(void)
{
    sensors_task();
    jobs_task();
    cmds_task();
    spool_task();
    drain_output();
}

// Runs the firmware for ms of virtual time. The "host" pings every 200 ms
// and, when stream_e is set, streams the extruder position.
static uint64_t next_model_ms, next_task, next_host;
static int stream_e;
static int stream_lane;            // 0: no lane key (every lane)
static double stream_scale = 1.0;  // the host's buffer multiplier
static int stream_period = 200;    // ms between host position updates
static int stream_jitter;          // +- ms, alternating (reactor jitter)
static int jitter_sign = 1;
static int ping_host = 1;
static void send(const char *fmt, ...);
static void
run_ms(double ms)
{
    uint64_t end = now + (uint64_t)(ms * MS);
    while (now < end) {
        // earliest of: timers, next model ms, next task slot
        uint64_t t = end;
        int i, best = -1;
        for (i = 0; i < ntimers; i++) {
            int32_t d = (int32_t)(timers[i]->waketime - (uint32_t)now);
            uint64_t tt = now + (d > 0 ? d : 0);
            if (tt < t) {
                t = tt;
                best = i;
            }
        }
        if (next_model_ms < t) {
            t = next_model_ms;
            best = -2;
        }
        if (next_task < t) {
            t = next_task;
            best = -3;
        }
        if (next_host < t) {
            t = next_host;
            best = -4;
        }
        now = t;
        if (best >= 0) {
            struct timer *tm = timers[best];
            if (tm->func(tm) == SF_DONE)
                sched_del_timer(tm);
        } else if (best == -2) {
            model_ms();
            next_model_ms += MS;
        } else if (best == -3) {
            run_tasks();
            next_task += MS / 5; // tasks every 0.2 ms
        } else if (best == -4) {
            next_host += (stream_period + jitter_sign * stream_jitter) * MS;
            jitter_sign = -jitter_sign;
            if (ping_host)
                send("{\"cmd\":\"ping\"}");
            if (stream_e && stream_lane)
                send("{\"cmd\":\"e\",\"lane\":%d,\"pos\":%.3f}",
                     stream_lane, e_cmd * stream_scale);
            else if (stream_e)
                send("{\"cmd\":\"e\",\"pos\":%.3f}", e_cmd);
        }
    }
}

static void
send(const char *fmt, ...)
{
    char line[300];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(line, sizeof(line) - 1, fmt, ap);
    va_end(ap);
    if (verbose)
        printf(">> %s\n", line);
    line[n++] = '\n';
    turtle_link_rx((uint8_t *)line, n);
    drain_output();
}

// Runs until the output contains pat (after mark) or ms passes.
static int
wait_for(const char *pat, int mark, double ms)
{
    double t = 0;
    while (t < ms) {
        if (strstr(out_log + mark, pat))
            return 1;
        run_ms(50);
        t += 50;
    }
    return strstr(out_log + mark, pat) != 0;
}

static int fails, checks;
#define CHECK(c, ...) do { checks++; if (!(c)) { fails++; printf("  FAIL: "); \
    printf(__VA_ARGS__); printf("\n"); } } while (0)

static void
reset_model(void)
{
    memset(ml, 0, sizeof(ml));
    hub_dead = 0;
    buf_c = 0;
    buf_hand = -1;
    e_rate = 0;
    clog = 0;
    starved_ms = full_ms = 0;
    slip_mm = buf_max = 0;
    feed_scale = 1.0;
    stream_e = 0;
    stream_lane = 0;
    stream_scale = 1.0;
    stream_period = 200;
    stream_jitter = 0;
    ping_host = 1;
    send("{\"cmd\":\"stop\"}");
    run_ms(200);
}

static void
park_lane(int i, double tip)
{
    ml[i].present = 1;
    ml[i].tip = tip;
    run_ms(20); // let the switches debounce
}

/****************************************************************
 * Scenarios
 ****************************************************************/

static void
t_info(void)
{
    printf("info/status\n");
    int mark = out_len;
    send("{\"id\":1,\"cmd\":\"info\"}");
    CHECK(strstr(out_log + mark, "\"proto\":\"boxturtle-1\""), "info proto");
    CHECK(strstr(out_log + mark, "\"reset\":\"power_on\""), "reset cause");
    mark = out_len;
    send("{\"id\":2,\"cmd\":\"status\"}");
    CHECK(strstr(out_log + mark, "\"id\":2,\"ok\":true"), "status ok");
    mark = out_len;
    send("{\"id\":3,\"cmd\":\"nope\"}");
    CHECK(strstr(out_log + mark, "unknown cmd"), "unknown cmd");
}

static void
t_switch_events(void)
{
    printf("switch events\n");
    reset_model();
    int mark = out_len;
    ml[0].present = 1;
    run_ms(20);
    CHECK(strstr(out_log + mark, "{\"ev\":\"sw\",\"sw\":\"prep1\",\"v\":1}"),
          "prep1 insert event");
    mark = out_len;
    ml[0].present = 0;
    run_ms(20);
    CHECK(strstr(out_log + mark, "\"sw\":\"prep1\",\"v\":0"), "prep1 pull");
}

static void
t_move_until(void)
{
    printf("move until hub\n");
    reset_model();
    park_lane(0, 0);
    int mark = out_len;
    send("{\"id\":5,\"cmd\":\"move\",\"lane\":1,\"mm\":300,\"speed\":100,"
         "\"accel\":400,\"until\":\"hub\"}");
    CHECK(wait_for("move_done", mark, 10000), "move finished");
    CHECK(strstr(out_log + mark, "\"end\":\"triggered\""), "ended on hub");
    CHECK(fabs(ml[0].tip - dist_hub) < 3, "tip at hub (%.2f)", ml[0].tip);
    CHECK(pin_level[LANE_PINS[0].en] == 1, "driver disabled after the move");
}

static void
t_move_no_trigger(void)
{
    printf("move no trigger\n");
    reset_model();
    park_lane(1, 0);
    hub_dead = 1;
    int mark = out_len;
    send("{\"cmd\":\"move\",\"lane\":2,\"mm\":200,\"speed\":100,\"until\":\"hub\"}");
    CHECK(wait_for("move_done", mark, 10000), "move finished");
    CHECK(strstr(out_log + mark, "\"end\":\"no_trigger\""), "no trigger");
    CHECK(fabs(ml[1].tip - 200) < 0.5, "full distance (%.2f)", ml[1].tip);
}

static void
t_feed_job_ok(void)
{
    printf("feed job ok\n");
    reset_model();
    park_lane(0, 0);
    int mark = out_len;
    send("{\"cmd\":\"job\",\"job\":\"feed\",\"lane\":1,\"until\":\"hub\","
         "\"mm\":250,\"speed\":100,\"watch\":\"prep\"}");
    CHECK(wait_for("job_done", mark, 10000), "job finished");
    CHECK(strstr(out_log + mark, "\"job\":\"feed\",\"ok\":true"), "feed ok");
    CHECK(!strstr(out_log + mark, "recovering"), "no retries");
}

static void
t_feed_job_snag(void)
{
    printf("feed job recovers from a snag\n");
    reset_model();
    park_lane(0, 0);
    ml[0].snag_at = 80;
    ml[0].snag_holds = 1;
    int mark = out_len;
    send("{\"cmd\":\"job\",\"job\":\"feed\",\"lane\":1,\"until\":\"hub\","
         "\"mm\":250,\"speed\":100}");
    CHECK(wait_for("job_done", mark, 20000), "job finished");
    CHECK(strstr(out_log + mark, "\"state\":\"recovering\""), "retried");
    CHECK(strstr(out_log + mark, "\"job\":\"feed\",\"ok\":true"), "then ok");
    CHECK(fabs(ml[0].tip - dist_hub) < 3, "tip at hub (%.2f)", ml[0].tip);
}

static void
t_feed_job_dead_hub(void)
{
    printf("feed job gives up on a dead hub switch\n");
    reset_model();
    park_lane(2, 0);
    hub_dead = 1;
    int mark = out_len;
    send("{\"cmd\":\"job\",\"job\":\"feed\",\"lane\":3,\"until\":\"hub\","
         "\"mm\":250,\"speed\":100,\"tries\":2}");
    CHECK(wait_for("job_done", mark, 30000), "job finished");
    CHECK(strstr(out_log + mark, "\"job\":\"feed\",\"ok\":false"), "failed");
    CHECK(strstr(out_log + mark, "\"code\":\"no_hub\""), "code no_hub");
    CHECK(strstr(out_log + mark, "\"ev\":\"fault\",\"code\":\"no_hub\""),
          "fault event");
    CHECK(strstr(out_log + mark, "\"try\":2"), "two retries");
}

static void
t_feed_job_runout(void)
{
    printf("feed job stops on runout\n");
    reset_model();
    park_lane(0, 0);
    int mark = out_len;
    send("{\"cmd\":\"job\",\"job\":\"feed\",\"lane\":1,\"until\":\"hub\","
         "\"mm\":250,\"speed\":50,\"watch\":\"prep\"}");
    run_ms(500);
    ml[0].present = 0;
    CHECK(wait_for("job_done", mark, 5000), "job finished");
    CHECK(strstr(out_log + mark, "\"code\":\"runout\""), "runout");
}

static void
t_bowden_snag(void)
{
    printf("bowden feed: early buffer fill is a snag\n");
    reset_model();
    park_lane(0, dist_hub);
    // A kink 300 mm into the bowden that holds once.
    ml[0].snag_at = dist_hub + 300;
    ml[0].snag_holds = 1;
    int mark = out_len;
    // Model the kink as an early TurtleNeck fill: the snag stops the tip,
    // so the feed never fills the buffer; it must time out and retry.
    send("{\"cmd\":\"job\",\"job\":\"feed\",\"lane\":1,\"until\":\"tn_adv\","
         "\"mm\":%.0f,\"speed\":150,\"min_mm\":%.0f}", bowden + 40,
         bowden - 100);
    CHECK(wait_for("job_done", mark, 60000), "job finished");
    CHECK(strstr(out_log + mark, "recovering"), "retried");
    CHECK(strstr(out_log + mark, "\"job\":\"feed\",\"ok\":true"), "ok after retry");
    CHECK(lane_at_gears(0) && buf_c >= 11, "at gears, buffer full (%.1f)",
          buf_c);
}

static void
t_verify_ok(void)
{
    printf("verify at extruder ok\n");
    reset_model();
    park_lane(0, gears);
    buf_c = 0;
    int mark = out_len;
    send("{\"cmd\":\"job\",\"job\":\"verify\",\"lane\":1,\"mm\":60,"
         "\"speed\":20}");
    CHECK(wait_for("job_done", mark, 20000), "job finished");
    CHECK(strstr(out_log + mark, "\"job\":\"verify\",\"ok\":true"), "verified");
    CHECK(!strstr(out_log + mark, "recovering"), "no retries");
}

static void
t_verify_short(void)
{
    printf("verify fails when filament is short of the extruder\n");
    reset_model();
    park_lane(0, gears - 300);
    int mark = out_len;
    send("{\"cmd\":\"job\",\"job\":\"verify\",\"lane\":1,\"mm\":60,"
         "\"speed\":40,\"tries\":1}");
    CHECK(wait_for("job_done", mark, 30000), "job finished");
    CHECK(strstr(out_log + mark, "\"code\":\"not_at_extruder\""),
          "not_at_extruder");
}

static void
t_unload(void)
{
    printf("unload job clears the hub\n");
    reset_model();
    park_lane(1, dist_hub + 400);
    int mark = out_len;
    send("{\"cmd\":\"job\",\"job\":\"unload\",\"lane\":2,\"mm\":600,"
         "\"extra_mm\":35,\"speed\":100}");
    CHECK(wait_for("job_done", mark, 20000), "job finished");
    CHECK(strstr(out_log + mark, "\"job\":\"unload\",\"ok\":true"), "ok");
    CHECK(ml[1].tip < dist_hub - 30 && ml[1].tip > dist_hub - 40,
          "parked 35 mm before the hub (%.1f)", ml[1].tip);
    run_ms(200); // brake
    CHECK(pwm_val[LANE_PINS[1].mot_rwd] == 0, "espooler off after");
}

static void
t_espool_follows(void)
{
    printf("espooler rewinds during a retract\n");
    reset_model();
    park_lane(0, 400);
    send("{\"cmd\":\"move\",\"lane\":1,\"mm\":-200,\"speed\":50}");
    run_ms(1500);
    CHECK(pwm_val[LANE_PINS[0].mot_rwd] > 0, "rewinding (%d)",
          pwm_val[LANE_PINS[0].mot_rwd]);
    CHECK(pin_level[LANE_PINS[0].mot_en] == 1, "espool enabled");
    run_ms(4000);
    CHECK(pwm_val[LANE_PINS[0].mot_rwd] == 0
          && pwm_val[LANE_PINS[0].mot_fwd] == 0, "off after the move");
}

static void
t_espool_move_duty(void)
{
    printf("espooler duty from the move line\n");
    reset_model();
    park_lane(0, 400);
    send("{\"cmd\":\"move\",\"lane\":1,\"mm\":-200,\"speed\":50,"
         "\"espool_rwd\":0.3,\"espool_kick\":false}");
    run_ms(500);
    int want = 300 * 32768 / 1000;
    CHECK(abs(pwm_val[LANE_PINS[0].mot_rwd] - want) < 50,
          "rewind at 0.3 with no kick (%d)", pwm_val[LANE_PINS[0].mot_rwd]);
    run_ms(4000);
    send("{\"cmd\":\"move\",\"lane\":1,\"mm\":100,\"speed\":50,"
         "\"espool_fwd\":0}");
    run_ms(500);
    CHECK(pwm_val[LANE_PINS[0].mot_fwd] == 0
          && pin_level[LANE_PINS[0].mot_en] == 0, "fwd 0 keeps it off");
    run_ms(2500);
    // The next move without the keys uses the cfg default (0.5).
    send("{\"cmd\":\"move\",\"lane\":1,\"mm\":-100,\"speed\":50}");
    run_ms(500);
    want = 500 * 32768 / 1000;
    CHECK(abs(pwm_val[LANE_PINS[0].mot_rwd] - want) < 50,
          "default rewind duty (%d)", pwm_val[LANE_PINS[0].mot_rwd]);
    run_ms(2500);
}

static void
t_assist_pulses(void)
{
    printf("print assist pulses per lane\n");
    reset_model();
    park_lane(0, gears);
    buf_c = 6;
    e_rate = 10;
    stream_e = 1;
    send("{\"cmd\":\"lane_cfg\",\"lane\":1,\"espool_pulse_mm\":20,"
         "\"espool_pulse_ms\":300}");
    send("{\"cmd\":\"assist\",\"lane\":1,\"on\":true}");
    int pulses = 0, on = 0, on_ms = 0, t;
    for (t = 0; t < 10000; t += 10) {
        run_ms(10);
        int now_on = pwm_val[LANE_PINS[0].mot_fwd] > 0;
        if (now_on && !on)
            pulses++;
        on_ms += now_on ? 10 : 0;
        on = now_on;
    }
    while (on) {                      // let the last pulse finish
        run_ms(10);
        on = pwm_val[LANE_PINS[0].mot_fwd] > 0;
        on_ms += on ? 10 : 0;
    }
    // 10 mm/s for 10 s is ~100 mm: a pulse every 20 mm.
    CHECK(pulses >= 4 && pulses <= 5, "%d pulses in 100 mm", pulses);
    // Each pulse: 300 ms on, then the 50 ms brake (both sides high).
    CHECK(abs(on_ms - pulses * 350) <= pulses * 20, "%d ms on for %d pulses",
          on_ms, pulses);
    // -1 goes back to the cfg default (no pulses).
    send("{\"cmd\":\"lane_cfg\",\"lane\":1,\"espool_pulse_ms\":-1}");
    pulses = on = 0;
    for (t = 0; t < 5000; t += 10) {
        run_ms(10);
        int now_on = pwm_val[LANE_PINS[0].mot_fwd] > 0;
        if (now_on && !on)
            pulses++;
        on = now_on;
    }
    CHECK(pulses == 0, "back to the default: no pulses (%d)", pulses);
    send("{\"cmd\":\"assist\",\"lane\":1,\"on\":false}");
    run_ms(500);
}

static void
t_assist(void)
{
    printf("feed assist keeps the buffer in range for 60 s\n");
    reset_model();
    park_lane(0, gears);
    buf_c = 6;
    e_rate = 8;
    stream_e = 1;
    int mark = out_len;
    long s0 = ml[0].steps;
    send("{\"cmd\":\"assist\",\"lane\":1,\"on\":true}");
    wait_for("\"ev\":\"fault\"", mark, 20000);
    e_rate = 0;                       // travel / pause
    wait_for("\"ev\":\"fault\"", mark, 10000);
    e_rate = 15;                      // fast infill
    wait_for("\"ev\":\"fault\"", mark, 30000);
    CHECK(!strstr(out_log + mark, "\"ev\":\"fault\""), "no fault");
    double fed = (ml[0].steps - s0) / STEPS_PER_MM;
    double want = 8 * 20 + 15 * 30;
    CHECK(fabs(fed - want) < 25, "fed %.1f mm for %.0f consumed", fed, want);
    printf("    fed %.1f mm for %.0f mm consumed\n", fed, want);
    CHECK(starved_ms < 2000, "starved %.0f ms", starved_ms);
    CHECK(full_ms < 30000, "full %.0f ms", full_ms);
    printf("    starved %.0f ms, full %.0f ms over 60 s\n", starved_ms, full_ms);
    send("{\"cmd\":\"assist\",\"lane\":1,\"on\":false}");
    run_ms(500);
}

// Leaves the TurtleNeck just off advance with advance the last switch to
// change, as a ramming load does. The lane must be at the gears.
static void
tn_after_ram(void)
{
    buf_c = 12.5;
    run_ms(20);
    buf_c = buf_max = 11.5;
    run_ms(20);
}

// The lane's feed per extruder mm with the TurtleNeck held by hand. last is
// pressed and let go first (the switch that changed last), hold1 holds it
// from before the assist start and hold2 from 1 s after it.
static void
mult_run(const char *what, int last, double hold1, double hold2,
         int fault_mm, double want)
{
    reset_model();
    park_lane(0, gears);
    stream_e = 1;
    stream_period = 100;
    send("{\"cmd\":\"cfg\",\"assist_fault_mm\":%d}", fault_mm);
    buf_hand = last == SW_TN_ADV ? 13 : 1;
    run_ms(20);
    buf_hand = hold1;
    run_ms(20);
    int mark = out_len;
    send("{\"cmd\":\"assist\",\"lane\":1,\"on\":true}");
    run_ms(500);
    e_rate = 10;
    run_ms(1000);
    buf_hand = hold2;
    run_ms(1000);                      // settle after a change
    long s0 = ml[0].steps;
    double e0 = e_cmd;
    run_ms(4000);
    double got = (ml[0].steps - s0) / STEPS_PER_MM / (e_cmd - e0);
    e_rate = 0;
    run_ms(500);
    CHECK(fabs(got - want) < 0.02, "%s: x%.3f, want x%.3f", what, got, want);
    CHECK(!strstr(out_log + mark, "\"ev\":\"fault\""), "%s: no fault", what);
    printf("    %s: x%.3f\n", what, got);
    send("{\"cmd\":\"assist\",\"lane\":1,\"on\":false}");
    send("{\"cmd\":\"cfg\",\"assist_fault_mm\":80}");
    run_ms(500);
}

static void
t_assist_afc_mult(void)
{
    printf("feed assist multipliers match AFC_buffer's\n");
    // Clog check off (AFC's disable_fault_sensitivity in a load purge).
    mult_run("advance pressed", SW_TN_ADV, 13, 13, 0, 0.85);
    mult_run("trailing pressed", SW_TN_TRL, 1, 1, 0, 1.15);
    mult_run("advance let go", SW_TN_ADV, 7, 7, 0, 0.85);
    mult_run("trailing let go", SW_TN_TRL, 7, 7, 0, 1.15);
    // Clog check on: pressed scales harder, a start too until a change.
    mult_run("advance pressed, check on", SW_TN_ADV, 13, 13, 80, 0.34);
    mult_run("trailing pressed, check on", SW_TN_TRL, 1, 1, 80, 1.725);
    mult_run("advance let go after the start", SW_TN_ADV, 13, 7, 80, 0.85);
    mult_run("trailing let go after the start", SW_TN_TRL, 1, 7, 80, 1.15);
    mult_run("start after advance let go", SW_TN_ADV, 7, 7, 80, 0.34);
    mult_run("start after trailing let go", SW_TN_TRL, 7, 7, 80, 1.725);
}

// A load's purge: 10 s at 10 mm/s, then the cool pause. The ramming load
// leaves the TurtleNeck just off advance. scale is the drive's real feed
// per commanded mm.
static void
purge_run(double scale)
{
    reset_model();
    park_lane(0, gears);
    tn_after_ram();
    feed_scale = scale;
    stream_e = 1;
    stream_period = 100;               // the host streams at 10 Hz
    stream_jitter = 30;
    send("{\"cmd\":\"cfg\",\"assist_fault_mm\":0}"); // AFC: no faults in a purge
    int mark = out_len;
    send("{\"cmd\":\"assist\",\"lane\":1,\"on\":true}");
    run_ms(500);
    long s0 = ml[0].steps;
    double e0 = e_cmd;
    e_rate = 10;
    run_ms(10000);
    e_rate = 0;
    double at_stop = buf_c;
    run_ms(5000);
    double fed = (ml[0].steps - s0) / STEPS_PER_MM * scale;
    double used = e_cmd - e0;
    CHECK(slip_mm < 0.01, "no grinding into a full TurtleNeck (%.2f mm)",
          slip_mm);
    // The TurtleNeck can take up its 10 mm between the switches.
    CHECK(fabs(fed - used) < 11, "fed %.1f mm for %.1f", fed, used);
    CHECK(buf_c > 1 && buf_c < 13, "buffer between the switches at rest "
          "(%.1f)", buf_c);
    CHECK(!strstr(out_log + mark, "\"ev\":\"fault\""), "no fault");
    printf("    drive x%.2f: fed %.1f mm for %.1f, buffer %.1f at the stop, "
           "%.1f at rest, %.1f most, slipped %.2f mm\n", scale, fed, used,
           at_stop, buf_c, buf_max, slip_mm);
    send("{\"cmd\":\"assist\",\"lane\":1,\"on\":false}");
    send("{\"cmd\":\"cfg\",\"assist_fault_mm\":80}");
    run_ms(500);
}

static void
t_assist_purge(void)
{
    printf("feed assist through a load purge and its cool pause\n");
    purge_run(1.0);
    purge_run(1.05);                   // drive feeds 5 % more than it thinks
    purge_run(0.95);
    purge_run(1.10);
    purge_run(0.90);
}

// How many times out_log shows needle after mark.
static int
count_after(int mark, const char *needle)
{
    int n = 0;
    const char *p = out_log + mark;
    while ((p = strstr(p, needle))) {
        n++;
        p++;
    }
    return n;
}

// Printing for 2 minutes: the TurtleNeck sweeps between its switches.
static void
float_run(double scale)
{
    reset_model();
    park_lane(0, gears);
    tn_after_ram();
    feed_scale = scale;
    stream_e = 1;
    stream_period = 100;
    stream_jitter = 30;
    int mark = out_len;
    send("{\"cmd\":\"assist\",\"lane\":1,\"on\":true}");
    run_ms(500);
    e_rate = 6;
    double lo = buf_c;
    int start = out_len, t;
    long s0 = ml[0].steps;
    double e0 = e_cmd;
    for (t = 0; t < 120000; t += 10) {
        run_ms(10);
        if (buf_c < lo)
            lo = buf_c;
    }
    double fed = (ml[0].steps - s0) / STEPS_PER_MM * scale;
    double used = e_cmd - e0;
    // The board's own switch reports.
    int adv_trips = count_after(start, "\"sw\":\"tn_adv\",\"v\":1");
    int trl_trips = count_after(start, "\"sw\":\"tn_trl\",\"v\":1");
    e_rate = 0;
    run_ms(1000);
    CHECK(slip_mm < 0.01, "no grinding (%.2f mm)", slip_mm);
    CHECK(starved_ms == 0, "never starved (%.0f ms)", starved_ms);
    CHECK(adv_trips >= 2 && trl_trips >= 2,
          "sweeps between the switches (advance %d, trailing %d)",
          adv_trips, trl_trips);
    CHECK(buf_max < 13 && lo > 1, "stays near the switches' span "
          "(%.1f to %.1f)", lo, buf_max);
    // The TurtleNeck takes up what is between its switches.
    CHECK(fabs(fed - used) < 11, "fed %.1f mm for %.1f", fed, used);
    CHECK(!strstr(out_log + mark, "\"ev\":\"fault\""), "no fault");
    printf("    drive x%.2f: TurtleNeck %.1f to %.1f mm, advance %d, "
           "trailing %d trips in 2 min, fed %.1f mm for %.1f\n", scale, lo,
           buf_max, adv_trips, trl_trips, fed, used);
    send("{\"cmd\":\"assist\",\"lane\":1,\"on\":false}");
    run_ms(500);
}

static void
t_assist_float(void)
{
    printf("feed assist floats the TurtleNeck between its switches\n");
    float_run(1.0);
    float_run(1.05);
    float_run(0.95);
}

static void
t_assist_follow(void)
{
    printf("follow mode tracks the extruder both ways\n");
    reset_model();
    park_lane(0, gears - 100);         // in the bowden, short of the gears
    e_rate = 0;
    stream_e = 1;
    run_ms(500);
    long s0 = ml[0].steps;
    send("{\"cmd\":\"assist\",\"lane\":1,\"on\":true,\"mode\":\"follow\","
         "\"espool_rwd\":0.4,\"espool_kick\":false}");
    run_ms(500);
    e_rate = 25;                       // tool_stn load
    run_ms(1440);
    e_rate = 0;
    run_ms(1500);
    double fwd = (ml[0].steps - s0) / STEPS_PER_MM;
    CHECK(fabs(fwd - 36) < 1.5, "followed +36 mm (%.2f)", fwd);
    e_rate = -25;                      // tool_stn_unload retract
    run_ms(1500);
    int want = 400 * 32768 / 1000;
    CHECK(abs(pwm_val[LANE_PINS[0].mot_rwd] - want) < 50,
          "espooler rewinds at 0.4 (%d)", pwm_val[LANE_PINS[0].mot_rwd]);
    run_ms(1500);
    e_rate = 0;
    run_ms(1500);
    double net = (ml[0].steps - s0) / STEPS_PER_MM;
    CHECK(fabs(net - (36 - 75)) < 1.5, "then -75 mm (net %.2f)", net);
    send("{\"cmd\":\"assist\",\"lane\":1,\"on\":false}");
    run_ms(500);
    stream_e = 0;
}

static void
t_assist_ext(void)
{
    printf("external buffer mode follows the host's scaled position\n");
    reset_model();
    park_lane(1, gears);
    e_rate = 0;
    // An old position for lane 2 from before this start must not count.
    send("{\"cmd\":\"e\",\"lane\":2,\"pos\":1000}");
    send("{\"cmd\":\"lane_cfg\",\"lane\":2,\"espool_pulse_mm\":10,"
         "\"espool_pulse_ms\":200}");
    long s0 = ml[1].steps, o0 = ml[0].steps;
    send("{\"cmd\":\"assist\",\"lane\":2,\"on\":true,\"mode\":\"ext\"}");
    run_ms(300);
    stream_e = 1;
    stream_lane = 2;
    stream_scale = 0.9;               // the buffer asks for 10 % less
    run_ms(500);
    double start = (ml[1].steps - s0) / STEPS_PER_MM;
    CHECK(fabs(start) < 1.0, "no jump from the old position (%.2f)", start);
    e_rate = 10;
    int pulses = 0, on = 0, t;
    for (t = 0; t < 5000; t += 10) {
        run_ms(10);
        int now_on = pwm_val[LANE_PINS[1].mot_fwd] > 0;
        if (now_on && !on)
            pulses++;
        on = now_on;
    }
    e_rate = 0;
    run_ms(1500);
    double fed = (ml[1].steps - s0) / STEPS_PER_MM;
    CHECK(fabs(fed - 45) < 1.5, "fed 0.9 x 50 mm (%.2f)", fed);
    CHECK(pulses >= 3 && pulses <= 5, "%d print pulses in 45 mm", pulses);
    CHECK(ml[0].steps == o0, "lane 1 did not move");
    int mark = out_len;
    send("{\"cmd\":\"status\",\"id\":7}");
    CHECK(strstr(out_log + mark, "\"follow\":2") != 0, "status shows ext mode");
    send("{\"cmd\":\"assist\",\"lane\":2,\"on\":false}");
    run_ms(500);
    stream_e = 0;
}

static void
t_assist_fallback(void)
{
    printf("feed assist without extruder updates (bang-bang)\n");
    reset_model();
    park_lane(0, gears);
    buf_c = 6;
    e_rate = 8;
    stream_e = 0;
    int mark = out_len;
    send("{\"cmd\":\"assist\",\"lane\":1,\"on\":true}");
    wait_for("\"ev\":\"fault\"", mark, 30000);
    CHECK(!strstr(out_log + mark, "\"ev\":\"fault\""), "no fault");
    CHECK(starved_ms < 3000, "starved %.0f ms", starved_ms);
    printf("    starved %.0f ms, full %.0f ms over 30 s\n", starved_ms, full_ms);
    send("{\"cmd\":\"assist\",\"lane\":1,\"on\":false}");
    run_ms(500);
}

static void
t_assist_clog(void)
{
    printf("feed assist reports a clog\n");
    reset_model();
    park_lane(0, gears);
    buf_c = 6;
    e_rate = 8;
    stream_e = 1;
    clog = 1;
    int mark = out_len;
    send("{\"cmd\":\"assist\",\"lane\":1,\"on\":true}");
    CHECK(wait_for("\"code\":\"clog\"", mark, 30000), "clog fault");
}

static void
t_assist_runout(void)
{
    printf("feed assist reports a runout\n");
    reset_model();
    park_lane(0, gears);
    buf_c = 6;
    e_rate = 8;
    stream_e = 1;
    int mark = out_len;
    send("{\"cmd\":\"assist\",\"lane\":1,\"on\":true}");
    run_ms(2000);
    ml[0].present = 0;
    CHECK(wait_for("\"code\":\"runout\"", mark, 3000), "runout fault");
}

static void
t_link_loss(void)
{
    printf("link loss stops the motors\n");
    reset_model();
    park_lane(0, 0);
    int mark = out_len;
    send("{\"cmd\":\"move\",\"lane\":1,\"mm\":1000,\"speed\":20,"
         "\"seq\":41}");
    ping_host = 0;
    CHECK(wait_for("link_lost", mark, 4000), "link_lost fault");
    double tip = ml[0].tip;
    run_ms(1000);
    CHECK(fabs(ml[0].tip - tip) < 0.01, "lane stopped");
    CHECK(pin_level[LANE_PINS[0].en] == 1, "driver off");
    CHECK(count_after(mark, "\"code\":\"link_lost\"") == 1,
          "link_lost sent once (%d)", count_after(mark, "\"code\":\"link_lost\""));
    CHECK(strstr(out_log + mark, "\"ev\":\"move_done\",\"lane\":1,"
                 "\"end\":\"link_lost\"") != NULL, "move_done says link_lost");
    CHECK(strstr(out_log + mark, ",\"seq\":41}") != NULL, "move_done has the seq");
    ping_host = 1;
    // The host lost those lines: replay sends the move_done again.
    int again = out_len;
    send("{\"cmd\":\"replay\",\"lane\":1}");
    run_ms(20);
    CHECK(count_after(again, "\"end\":\"link_lost\"") == 1
          && count_after(again, ",\"seq\":41}") == 1, "replay resends it");
}

static void
t_link_loss_job(void)
{
    printf("link loss ends a job with link_lost\n");
    reset_model();
    park_lane(0, 0);
    int mark = out_len;
    send("{\"cmd\":\"job\",\"job\":\"feed\",\"lane\":1,\"until\":\"hub\","
         "\"mm\":1000,\"speed\":20,\"hold\":true,\"seq\":42}");
    ping_host = 0;
    CHECK(wait_for("\"ev\":\"job_done\"", mark, 4000), "job ended");
    CHECK(strstr(out_log + mark, "\"code\":\"link_lost\"") != NULL
          && strstr(out_log + mark, "\"seq\":42") != NULL,
          "job_done has link_lost and the seq");
    run_ms(500);
    CHECK(pin_level[LANE_PINS[0].en] == 0, "driver still holding");
    ping_host = 1;
    int again = out_len;
    send("{\"cmd\":\"replay\",\"lane\":1}");
    run_ms(20);
    CHECK(count_after(again, "\"ev\":\"job_done\"") == 1, "replay resends it");
    send("{\"cmd\":\"enable\",\"lane\":1,\"on\":false}");
    run_ms(20);
}

static void
t_two_lanes(void)
{
    printf("two lanes move at once\n");
    reset_model();
    park_lane(0, 0);
    park_lane(3, 0);
    int mark = out_len;
    send("{\"cmd\":\"move\",\"lane\":1,\"mm\":100,\"speed\":80}");
    send("{\"cmd\":\"move\",\"lane\":4,\"mm\":-50,\"speed\":40}");
    run_ms(4000);
    CHECK(fabs(ml[0].tip - 100) < 0.5, "lane 1 at 100 (%.2f)", ml[0].tip);
    CHECK(fabs(ml[3].tip + 50) < 0.5, "lane 4 at -50 (%.2f)", ml[3].tip);
    int n = 0;
    const char *p = out_log + mark;
    while ((p = strstr(p, "move_done"))) {
        n++;
        p++;
    }
    CHECK(n == 2, "two move_done (%d)", n);
}

int
main(int argc, char **argv)
{
    verbose = argc > 1 && !strcmp(argv[1], "-v");
    gears = dist_hub + bowden;
    memset(step_pin_lane, -1, sizeof(step_pin_lane));
    int i;
    for (i = 0; i < NUM_LANES; i++)
        step_pin_lane[LANE_PINS[i].step] = i;
    turtle_init();
    next_model_ms = now + MS;
    next_task = now;
    next_host = now + 200 * MS;
    t_info();
    t_switch_events();
    t_move_until();
    t_move_no_trigger();
    t_feed_job_ok();
    t_feed_job_snag();
    t_feed_job_dead_hub();
    t_feed_job_runout();
    t_bowden_snag();
    t_verify_ok();
    t_verify_short();
    t_unload();
    t_espool_follows();
    t_espool_move_duty();
    t_assist();
    t_assist_pulses();
    t_assist_afc_mult();
    t_assist_purge();
    t_assist_float();
    t_assist_follow();
    t_assist_ext();
    t_assist_fallback();
    t_assist_clog();
    t_assist_runout();
    t_link_loss();
    t_link_loss_job();
    t_two_lanes();
    printf("%d/%d checks passed\n", checks - fails, checks);
    return fails != 0;
}
