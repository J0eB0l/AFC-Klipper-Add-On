// Host command handling for the BoxTurtle firmware (protocol boxturtle-1).
//
// Every request is one JSON line with "cmd" and an optional "id". Every
// reply echoes the id and carries "ok". See PROTOCOL.md.
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h>
#include "board/internal.h" // RCC, NVIC_SystemReset
#include "board/misc.h" // bootloader_request
#include "sched.h" // DECL_INIT
#include "board.h"
#include "events.h"
#include "guard.h"
#include "json.h"
#include "link.h"
#include "motion.h"
#include "proto.h"
#include "spool.h"
#include "tmc.h"

static const char *reset_cause = "unknown";

// The reset flags add up until cleared, and firmware before this one may
// never have cleared them, so a software or watchdog flag beside the
// power-on flag means this is not a fresh power-on.
static void
read_reset_cause(void)
{
    uint32_t rsr = RCC->RSR;
    int wd = !!(rsr & (RCC_RSR_IWDG1RSTF | RCC_RSR_WWDG1RSTF));
    int por = (rsr & (RCC_RSR_PORRSTF | RCC_RSR_BORRSTF))
              && !(rsr & RCC_RSR_SFTRSTF) && !wd;
    guard_boot(por, wd);
    if (rsr & (RCC_RSR_IWDG1RSTF | RCC_RSR_WWDG1RSTF))
        reset_cause = "watchdog";
    else if (rsr & RCC_RSR_SFTRSTF)
        reset_cause = "software";
    else if (rsr & RCC_RSR_BORRSTF && !(rsr & RCC_RSR_PORRSTF))
        reset_cause = "brownout";
    else if (rsr & RCC_RSR_PORRSTF)
        reset_cause = "power_on";
    else if (rsr & RCC_RSR_LPWRRSTF)
        reset_cause = "low_power";
    else if (rsr & RCC_RSR_PINRSTF)
        reset_cause = "reset_pin";
    RCC->RSR |= RCC_RSR_RMVF;
}

static void
chip_uid(char *out)
{
    // 96-bit unique id at 0x1FF1E800 on the STM32H72x/73x.
#ifndef TURTLE_UID_ADDR
#define TURTLE_UID_ADDR 0x1FF1E800
#endif
    const uint32_t *uid = (const uint32_t *)TURTLE_UID_ADDR;
    static const char hex[] = "0123456789ABCDEF";
    int i, j, n = 0;
    for (i = 0; i < 3; i++)
        for (j = 28; j >= 0; j -= 4)
            out[n++] = hex[(uid[i] >> j) & 0xf];
    out[n] = 0;
}

/****************************************************************
 * Reply helpers
 ****************************************************************/

void
reply_begin(struct reply *r, const char *line)
{
    jw_begin(&r->w, r->buf, sizeof(r->buf));
    int32_t id;
    if (json_get_int(line, "id", &id))
        jw_int(&r->w, "id", id);
}

void
reply_send(struct reply *r)
{
    link_send(r->buf, jw_end(&r->w));
}

void
reply_ok(const char *line)
{
    struct reply r;
    reply_begin(&r, line);
    jw_bool(&r.w, "ok", 1);
    reply_send(&r);
}

void
reply_err(const char *line, const char *err)
{
    struct reply r;
    reply_begin(&r, line);
    jw_bool(&r.w, "ok", 0);
    jw_str(&r.w, "err", err);
    reply_send(&r);
}

// Reads "lane" (1-based) into 0-based; replies with an error if bad.
int
get_lane(const char *line, int *lane)
{
    int32_t v;
    if (!json_get_int(line, "lane", &v) || v < 1 || v > NUM_LANES) {
        reply_err(line, "bad lane");
        return 0;
    }
    *lane = v - 1;
    return 1;
}

static int
sw_by_name(const char *name)
{
    int i;
    for (i = 0; i < SW_COUNT; i++)
        if (!strcmp(name, SW_NAMES[i]))
            return i;
    return -1;
}

/****************************************************************
 * Commands
 ****************************************************************/

static void
cmd_info(char *line)
{
    struct reply r;
    char uid[25];
    chip_uid(uid);
    reply_begin(&r, line);
    jw_bool(&r.w, "ok", 1);
    jw_str(&r.w, "name", FW_NAME);
    jw_str(&r.w, "fw", FW_VERSION);
    jw_str(&r.w, "proto", FW_PROTO);
    jw_str(&r.w, "board", "afc-lite");
    jw_int(&r.w, "lanes", NUM_LANES);
    jw_str(&r.w, "uid", uid);
    jw_str(&r.w, "reset", reset_cause);
    jw_int(&r.w, "up", uptime());
    jw_int(&r.w, "dropped", link_dropped());
    jw_int(&r.w, "rx", link_rx_lines());
    jw_int(&r.w, "tx", link_tx_lines());
    jw_bool(&r.w, "trial", guard_boot_trial());
    jw_int(&r.w, "boot_fails", guard_boot_fails());
    reply_send(&r);
}

static void
sw_object(char *buf, int size)
{
    // {"hub":0,"prep1":1,...}
    struct jw w;
    jw_begin(&w, buf, size);
    uint32_t st = sensors_state();
    int i;
    for (i = 0; i < SW_COUNT; i++)
        jw_int(&w, SW_NAMES[i], (st >> i) & 1);
    int n = jw_end(&w);
    if (n >= 1)
        buf[n - 1] = 0; // drop the newline
}

static void
cmd_status(char *line)
{
    // Longer than other replies (every switch and four lanes), so it has its
    // own buffer: in a 512 byte reply it came out empty.
    static char buf[1024];
    char sw[220];
    sw_object(sw, sizeof(sw));
    struct jw w;
    jw_begin(&w, buf, sizeof(buf));
    int32_t id;
    if (json_get_int(line, "id", &id))
        jw_int(&w, "id", id);
    jw_bool(&w, "ok", 1);
    jw_raw(&w, "sw", sw);
    jw_int(&w, "up", uptime());
    status_extra(&w);
    link_send(buf, jw_end(&w));
}

static int
parse_hex(const char *s, uint8_t *out, int maxbytes)
{
    int n = 0;
    while (s[0] && s[1] && n < maxbytes) {
        int v = 0, k;
        for (k = 0; k < 2; k++) {
            char c = s[k];
            v <<= 4;
            if (c >= '0' && c <= '9')
                v |= c - '0';
            else if (c >= 'a' && c <= 'f')
                v |= c - 'a' + 10;
            else if (c >= 'A' && c <= 'F')
                v |= c - 'A' + 10;
            else
                return -1;
        }
        out[n++] = v;
        s += 2;
    }
    return s[0] ? -1 : n;
}

// {"cmd":"led","strip":2,"i":1,"n":8,"color":"FF0000"} (i 1-based,
// color RRGGBB or RRGGBBWW). "send":false holds the update for a later led.
static void
cmd_led(char *line)
{
    int32_t strip, first = 1, count = 1;
    char color[12];
    if (!json_get_int(line, "strip", &strip) || strip < 1
        || strip > NUM_STRIPS) {
        reply_err(line, "bad strip");
        return;
    }
    json_get_int(line, "i", &first);
    json_get_int(line, "n", &count);
    uint8_t c[4] = {0, 0, 0, 0};
    if (!json_get_str(line, "color", color, sizeof(color))
        || parse_hex(color, c, 4) < 3) {
        reply_err(line, "bad color");
        return;
    }
    if (!leds_set(strip - 1, first - 1, count, c[0], c[1], c[2], c[3])) {
        reply_err(line, "bad range");
        return;
    }
    int send = 1;
    json_get_bool(line, "send", &send);
    if (send)
        leds_commit(strip - 1);
    reply_ok(line);
}

// {"cmd":"led_cfg","strip":2,"count":64,"order":"GRB"}
static void
cmd_led_cfg(char *line)
{
    int32_t strip, count;
    char order[8];
    if (!json_get_int(line, "strip", &strip) || !json_get_int(line, "count",
                                                               &count)
        || !json_get_str(line, "order", order, sizeof(order))
        || !leds_config(strip - 1, count, order)) {
        reply_err(line, "bad led_cfg");
        return;
    }
    reply_ok(line);
}

// {"cmd":"sw_cfg","sw":"prep1","invert":true} and/or "debounce_ms":3
static void
cmd_sw_cfg(char *line)
{
    char name[16];
    int inv;
    int32_t db;
    if (json_get_str(line, "sw", name, sizeof(name))) {
        int sw = sw_by_name(name);
        if (sw < 0) {
            reply_err(line, "bad sw");
            return;
        }
        if (json_get_bool(line, "invert", &inv))
            sensors_set_invert(sw, inv);
    }
    if (json_get_int(line, "debounce_ms", &db))
        sensors_set_debounce(db);
    reply_ok(line);
}

static void
cmd_bootloader(char *line)
{
    reply_ok(line);
    bootloader_request();
}

static void
cmd_reboot(char *line)
{
    reply_ok(line);
    NVIC_SystemReset();
}

static void
cmd_ping(char *line)
{
    reply_ok(line);
}

// Bench: {"cmd":"rxlog"} -> the first bytes of the last lines received.
static void
cmd_rxlog(char *line)
{
    char buf[300];
    link_rxlog(buf, sizeof(buf));
    struct reply r;
    reply_begin(&r, line);
    jw_bool(&r.w, "ok", 1);
    jw_raw(&r.w, "lines", buf);
    reply_send(&r);
}

static const struct {
    const char *name;
    void (*fn)(char *line);
} COMMANDS[] = {
    { "info", cmd_info },
    { "ping", cmd_ping },
    { "rxlog", cmd_rxlog },
    { "status", cmd_status },
    { "led", cmd_led },
    { "led_cfg", cmd_led_cfg },
    { "sw_cfg", cmd_sw_cfg },
    { "bootloader", cmd_bootloader },
    { "reboot", cmd_reboot },
};

void
proto_handle_line(char *line)
{
    char cmd[24];
    // Skip anything before the object: the tail of a half line, or a modem
    // probe ("AT") from ModemManager. A line with no object is ignored.
    while (*line && *line != '{')
        line++;
    if (!*line)
        return;
    if (!json_get_str(line, "cmd", cmd, sizeof(cmd))) {
        // Our own event or reply coming back: a tty that echoes until the
        // host puts it in raw mode. Answering it would echo forever.
        char ev[16];
        int ok;
        if (json_get_str(line, "ev", ev, sizeof(ev))
            || json_get_bool(line, "ok", &ok))
            return;
        reply_err(line, "no cmd");
        return;
    }
    unsigned i;
    for (i = 0; i < sizeof(COMMANDS) / sizeof(COMMANDS[0]); i++) {
        if (!strcmp(cmd, COMMANDS[i].name)) {
            guard_host_ok();
            COMMANDS[i].fn(line);
            return;
        }
    }
    if (proto_handle_extra(cmd, line)) {
        guard_host_ok();
        return;
    }
    reply_err(line, "unknown cmd");
}

void
turtle_init(void)
{
    read_reset_cause();
    motion_init();
    tmc_init();
    sensors_init();
    leds_init();
    spool_init();
}
DECL_INIT(turtle_init);
