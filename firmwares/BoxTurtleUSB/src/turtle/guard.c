// Boot guard: a build that can't talk to the host goes back to the
// Katapult bootloader by itself, so a board in a closed case never needs
// its BOOT or RESET buttons after a bad flash.
//
// The state lives in the last 32 bytes of SRAM4 (D3 domain). Klipper and
// Katapult leave that RAM alone, and it keeps its contents through every
// reset except a power cut.
//
// - A power-on boot trusts the build (proto.c works out power-on from the
//   reset flags).
// - A boot of a build the host has not talked to yet (just flashed), or a
//   boot after a watchdog reset, is a trial: the host must send a line with
//   a known command within GUARD_TRIAL_MS. If not, the board resets and
//   counts a failure.
// - After GUARD_MAX_FAILS failures in a row, it restarts into Katapult and
//   stays there until it is flashed again.
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "board/armcm_reset.h" // try_request_canboot
#include "board/internal.h" // NVIC_SystemReset, SCB_CleanDCache_by_Addr
#include "sched.h" // DECL_TASK
#include "board.h" // uptime, FW_VERSION
#include "guard.h"

#ifndef TURTLE_GUARD_ADDR
#define TURTLE_GUARD_ADDR 0x38003FE0 // last 32 bytes of the H723's SRAM4
#endif
#define GUARD_MAGIC 0x54525447
#define GUARD_TRIAL_MS 60000
#define GUARD_MAX_FAILS 3

struct guard {
    uint32_t magic, build, confirmed, fails, check;
};

static struct guard *const g = (struct guard *)TURTLE_GUARD_ADDR;
static int trial, boot_trial, boot_fails;

static uint32_t
guard_sum(const struct guard *s)
{
    return s->magic ^ s->build ^ s->confirmed ^ s->fails ^ 0xA5A5A5A5;
}

static void
guard_save(void)
{
    g->magic = GUARD_MAGIC;
    g->check = guard_sum(g);
    // SRAM4 is cached; a reset does not write the cache back.
    SCB_CleanDCache_by_Addr((void *)g, sizeof(*g));
}

// Changes with every compile, so a new flash always starts a trial.
static uint32_t
build_id(void)
{
    const char *s = __DATE__ " " __TIME__ " " FW_VERSION;
    uint32_t h = 2166136261u;
    while (*s) {
        h ^= (uint8_t)*s++;
        h *= 16777619u;
    }
    return h;
}

void
guard_boot(int power_on, int watchdog)
{
    uint32_t id = build_id();
    int valid = g->magic == GUARD_MAGIC && g->check == guard_sum(g);
    if (power_on) {
        g->build = id;
        g->confirmed = 1;
        g->fails = 0;
    } else if (!valid || g->build != id) {
        g->build = id;
        g->confirmed = 0;
        g->fails = 0;
    } else if (watchdog) {
        g->confirmed = 0;
        g->fails++;
    }
    boot_fails = g->fails;
    if (g->fails >= GUARD_MAX_FAILS) {
        // Start over after the next flash (or a power cycle).
        g->fails = 0;
        guard_save();
        try_request_canboot();
    }
    trial = boot_trial = !g->confirmed;
    guard_save();
}

void
guard_host_ok(void)
{
    if (!trial)
        return;
    trial = 0;
    g->confirmed = 1;
    g->fails = 0;
    guard_save();
}

int
guard_boot_trial(void)
{
    return boot_trial;
}

int
guard_boot_fails(void)
{
    return boot_fails;
}

void
guard_task(void)
{
    if (!trial || uptime() < GUARD_TRIAL_MS)
        return;
    trial = 0;
    g->fails++;
    guard_save();
    NVIC_SystemReset();
}
DECL_TASK(guard_task);
