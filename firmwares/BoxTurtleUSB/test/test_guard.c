// Tests for the boot guard (src/turtle/guard.c).
// Build: cc -Itest/sim/include -Isrc/turtle -o test/test_guard
//        test/test_guard.c src/turtle/guard.c
#include <stdio.h>
#include <string.h>
#include "board/internal.h"
#include "guard.h"

struct sim_guard_mem sim_guard;
static uint32_t now_ms;
static int resets, canboots;

uint32_t uptime(void) { return now_ms; }
void NVIC_SystemReset(void) { resets++; }
void try_request_canboot(void) { canboots++; }

static int fails;
#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); \
                                  fails++; } } while (0)

// One boot: power_on/watchdog as proto.c works them out.
static void
boot(int power_on, int watchdog)
{
    now_ms = 0;
    resets = canboots = 0;
    guard_boot(power_on, watchdog);
}

// Runs the guard task until `ms` of uptime.
static void
run_to(uint32_t ms)
{
    while (now_ms < ms && !resets) {
        now_ms += 100;
        guard_task();
    }
}

int
main(void)
{
    // First flash over firmware that never cleared the reset flags: SRAM4
    // holds junk and the flags are not a clean power-on, so it's a trial.
    memset(&sim_guard, 0x5a, sizeof(sim_guard));
    boot(0, 0);
    CHECK(guard_boot_trial());
    CHECK(guard_boot_fails() == 0);
    run_to(5000);
    guard_host_ok();
    run_to(120000);
    CHECK(resets == 0);

    // A reboot of the confirmed build is not a trial.
    boot(0, 0);
    CHECK(!guard_boot_trial());
    run_to(120000);
    CHECK(resets == 0);

    // A power-on with junk in SRAM4 trusts the build.
    memset(&sim_guard, 0xa5, sizeof(sim_guard));
    boot(1, 0);
    CHECK(!guard_boot_trial());
    run_to(120000);
    CHECK(resets == 0 && canboots == 0);

    // A new build (different id) is a trial; with no host it resets after
    // 60 s, three times, then the next boot goes to Katapult.
    sim_guard.w[1] ^= 1; // stored build id no longer matches
    sim_guard.w[4] ^= 1; // keep the check word valid
    int i;
    for (i = 0; i < 3; i++) {
        boot(0, 0);
        CHECK(guard_boot_trial());
        CHECK(guard_boot_fails() == i);
        CHECK(canboots == 0);
        run_to(59000);
        CHECK(resets == 0);
        run_to(61000);
        CHECK(resets == 1);
    }
    boot(0, 0);
    CHECK(canboots == 1);
    CHECK(guard_boot_fails() == 3);
    // Flashed again with the same build: counting starts over.
    boot(0, 0);
    CHECK(canboots == 0);
    CHECK(guard_boot_trial());
    CHECK(guard_boot_fails() == 0);
    guard_host_ok();

    // Watchdog resets: each one is a trial; three in a row -> Katapult.
    boot(0, 1);
    CHECK(guard_boot_trial());
    CHECK(guard_boot_fails() == 1);
    boot(0, 1);
    CHECK(guard_boot_fails() == 2);
    CHECK(canboots == 0);
    boot(0, 1);
    CHECK(canboots == 1);

    // A watchdog reset the host then talks to clears the count.
    boot(1, 0);
    boot(0, 1);
    CHECK(guard_boot_fails() == 1);
    guard_host_ok();
    boot(0, 1);
    CHECK(guard_boot_fails() == 1);
    CHECK(canboots == 0);
    guard_host_ok();

    // A trial that times out, then a watchdog: both count.
    sim_guard.w[1] ^= 1;
    sim_guard.w[4] ^= 1;
    boot(0, 0);
    run_to(61000);
    CHECK(resets == 1);
    boot(0, 1);
    CHECK(guard_boot_fails() == 2);

    if (fails) {
        printf("%d failed\n", fails);
        return 1;
    }
    printf("guard: all passed\n");
    return 0;
}
