// Events sent to the host, unasked, as {"ev":...} lines.
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h>
#include "board.h"
#include "events.h"
#include "json.h"
#include "link.h"

#define KEEP_SIZE 200
static char kept[NUM_LANES][EV_KEEP_KINDS][KEEP_SIZE];
static uint8_t kept_len[NUM_LANES][EV_KEEP_KINDS];

void
ev_keep(int lane, int kind, const char *line, int len)
{
    if (lane < 0 || lane >= NUM_LANES || kind < 0 || kind >= EV_KEEP_KINDS
        || len <= 0 || len > KEEP_SIZE)
        return;
    memcpy(kept[lane][kind], line, len);
    kept_len[lane][kind] = len;
}

void
ev_replay(int lane)
{
    int k;
    for (k = 0; k < EV_KEEP_KINDS; k++)
        if (kept_len[lane][k])
            link_send(kept[lane][k], kept_len[lane][k]);
}

void
ev_switch(int sw, int state)
{
    char buf[96];
    struct jw w;
    jw_begin(&w, buf, sizeof(buf));
    jw_str(&w, "ev", "sw");
    jw_str(&w, "sw", SW_NAMES[sw]);
    jw_int(&w, "v", state);
    link_send(buf, jw_end(&w));
}

void
ev_fault(const char *code, int lane, const char *msg)
{
    char buf[200];
    struct jw w;
    jw_begin(&w, buf, sizeof(buf));
    jw_str(&w, "ev", "fault");
    jw_str(&w, "code", code);
    if (lane >= 0)
        jw_int(&w, "lane", lane + 1);
    if (msg)
        jw_str(&w, "msg", msg);
    link_send(buf, jw_end(&w));
}

void
ev_log(const char *msg)
{
    char buf[200];
    struct jw w;
    jw_begin(&w, buf, sizeof(buf));
    jw_str(&w, "ev", "log");
    jw_str(&w, "msg", msg);
    link_send(buf, jw_end(&w));
}
