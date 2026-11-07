// TMC2209 single-wire UART driver setup and monitoring.
//
// Bit-banged half duplex at 40 kbaud on each driver's own PDN_UART pin, from
// task context. The register values and the current formula follow Klipper's
// klippy/extras/tmc2209.py and tmc2130.py.
//
// The monitor task reads GSTAT and DRV_STATUS of one driver every 125 ms
// (each driver every 0.5 s), reports faults once per change and asks
// motion.c to stop the lane on a hard fault.
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "board/gpio.h" // gpio_out_setup
#include "board/irq.h" // irq_save
#include "board/misc.h" // timer_read_time
#include "sched.h" // DECL_TASK
#include "board.h"
#include "events.h"
#include "motion.h"
#include "tmc.h"

#define BAUD 40000

static uint32_t bit_ticks;
// The TMC2209 drops a datagram that starts too soon after the one before.
// On a bench AFC-Lite, 9 writes in a row land 1-3 times with 12-40 idle
// bit times between them and every time with 100 (2.5 ms), so every
// datagram waits gap_bits first.
static uint8_t gap_bits = 100;
// Bench experiments (tmc_wtest): TXF_SYNC_F5 sends sync 0xF5 like Klipper,
// TXF_NOIRQ sends with interrupts off, TXF_STOP2 sends two stop bits.
enum { TXF_SYNC_F5 = 1, TXF_NOIRQ = 2, TXF_STOP2 = 8 };
static uint8_t tx_flags;
static struct tmc_cfg cfgs[NUM_LANES];
static uint8_t health[NUM_LANES], fail_count[NUM_LANES];
static uint8_t setup_done[NUM_LANES];

static uint8_t
crc8(const uint8_t *d, int n)
{
    uint8_t crc = 0;
    int i, j;
    for (i = 0; i < n; i++) {
        uint8_t b = d[i];
        for (j = 0; j < 8; j++) {
            if ((crc >> 7) ^ (b & 1))
                crc = (crc << 1) ^ 0x07;
            else
                crc <<= 1;
            b >>= 1;
        }
    }
    return crc;
}

static void
wait_until(uint32_t t)
{
    while ((int32_t)(timer_read_time() - t) < 0)
        ;
}

static void
send_bytes(int lane, const uint8_t *d, int n)
{
    struct gpio_out pin = gpio_out_setup(LANE_PINS[lane].uart, 1);
    uint32_t t = timer_read_time() + bit_ticks * gap_bits; // idle high first
    wait_until(t);
    irqstatus_t flag = 0;
    if (tx_flags & TXF_NOIRQ)
        flag = irq_save();
    int i, b, nbits = tx_flags & TXF_STOP2 ? 11 : 10;
    for (i = 0; i < n; i++) {
        uint16_t frame = 0x600 | (d[i] << 1); // start 0, 8 data, stop 1(+1)
        for (b = 0; b < nbits; b++) {
            gpio_out_write(pin, (frame >> b) & 1);
            t += bit_ticks;
            wait_until(t);
        }
    }
    if (tx_flags & TXF_NOIRQ)
        irq_restore(flag);
}

// Receives n bytes; each byte syncs on its own start bit. Returns 0 or -1.
static int
recv_bytes(int lane, uint8_t *d, int n)
{
    struct gpio_in pin = gpio_in_setup(LANE_PINS[lane].uart, 1);
    uint32_t timeout = timer_read_time() + timer_from_us(3000);
    int i, b;
    for (i = 0; i < n; i++) {
        while (gpio_in_read(pin)) {
            if ((int32_t)(timer_read_time() - timeout) > 0)
                return -1;
        }
        uint32_t t = timer_read_time() + bit_ticks / 2;
        wait_until(t);
        if (gpio_in_read(pin))
            return -1; // glitch, not a start bit
        uint8_t v = 0;
        for (b = 0; b < 8; b++) {
            t += bit_ticks;
            wait_until(t);
            v |= (gpio_in_read(pin) ? 1 : 0) << b;
        }
        t += bit_ticks;
        wait_until(t); // stop bit
        d[i] = v;
        timeout = timer_read_time() + timer_from_us(1000);
    }
    return 0;
}

int
tmc_write(int lane, uint8_t reg, uint32_t val)
{
    uint8_t d[8] = {tx_flags & TXF_SYNC_F5 ? 0xf5 : 0x05, 0x00, reg | 0x80,
                    val >> 24, val >> 16, val >> 8, val, 0};
    d[7] = crc8(d, 7);
    send_bytes(lane, d, 8);
    gpio_in_setup(LANE_PINS[lane].uart, 1); // release the line
    return 0;
}

int
tmc_read(int lane, uint8_t reg, uint32_t *val)
{
    uint8_t q[4] = {tx_flags & TXF_SYNC_F5 ? 0xf5 : 0x05, 0x00, reg, 0};
    q[3] = crc8(q, 3);
    send_bytes(lane, q, 4);
    uint8_t r[8];
    if (recv_bytes(lane, r, 8))
        return -1;
    if (r[0] != 0x05 || r[1] != 0xff || r[2] != reg || crc8(r, 7) != r[7])
        return -1;
    *val = ((uint32_t)r[3] << 24) | (r[4] << 16) | (r[5] << 8) | r[6];
    return 0;
}

// Bench: one write with the given tx flags, IFCNT before and after.
int
tmc_wtest(int lane, int flags, int gap, int count, uint8_t reg, uint32_t val,
          uint32_t *before, uint32_t *after)
{
    uint8_t old = tx_flags, old_gap = gap_bits;
    int ret = -1;
    tx_flags = flags;
    if (gap > 0 && gap < 250)
        gap_bits = gap;
    if (!tmc_read(lane, TMC_IFCNT, before)) {
        int i;
        for (i = 0; i < (count > 0 ? count : 1); i++)
            tmc_write(lane, reg, val);
        ret = tmc_read(lane, TMC_IFCNT, after) ? -2 : 0;
    }
    tx_flags = old;
    gap_bits = old_gap;
    return ret;
}

void
tmc_default_cfg(struct tmc_cfg *c)
{
    c->run_ma = 800;
    c->hold_ma = 800;
    c->sense_mohm = 110;
    c->microsteps = 16;
    c->spreadcycle = 1;
    c->sgthrs = 0;
}

// Klipper's _calc_current_bits: cs = 32*R*I*sqrt(2)/Vref - 1, rounded,
// with R = sense + 20 mOhm, Vref 0.32 V (vsense 0) or 0.18 V (vsense 1).
static int
current_bits(uint32_t ma, uint32_t sense_mohm, int vsense)
{
    uint64_t num = 32ull * (sense_mohm + 20) * ma * 1414214ull;
    uint64_t den = (vsense ? 180ull : 320ull) * 1000000ull * 1000ull;
    int cs = (int)((num + den / 2) / den) - 1;
    if (cs < 0)
        cs = 0;
    if (cs > 31)
        cs = 31;
    return cs;
}

static int
mres_bits(int microsteps)
{
    int mres = 8;
    while (mres > 0 && (256 >> mres) != microsteps)
        mres--;
    return (256 >> mres) == microsteps ? mres : 4;
}

int
tmc_setup(int lane, const struct tmc_cfg *c)
{
    cfgs[lane] = *c;
    uint32_t before = 0, after = 0;
    if (tmc_read(lane, TMC_IFCNT, &before))
        return -1;
    int vsense = 0;
    int irun = current_bits(c->run_ma, c->sense_mohm, 0);
    if (irun < 16) {
        vsense = 1;
        irun = current_bits(c->run_ma, c->sense_mohm, 1);
    }
    int ihold = current_bits(c->hold_ma < c->run_ma ? c->hold_ma : c->run_ma,
                             c->sense_mohm, vsense);
    // pdn_disable, mstep_reg_select, multistep_filt (+ en_spreadcycle)
    uint32_t gconf = (1 << 6) | (1 << 7) | (1 << 8)
        | (c->spreadcycle ? (1 << 2) : 0);
    // toff 3, hstrt 5, hend 0, tbl 2, vsense, mres, intpol, dedge
    uint32_t chop = 3 | (5 << 4) | (0 << 7) | (2 << 15)
        | (vsense ? (1u << 17) : 0) | ((uint32_t)mres_bits(c->microsteps) << 24)
        | (1u << 28) | (1u << 29);
    uint32_t ihr = ihold | (irun << 8) | (8 << 16);
    int n = 0;
    tmc_write(lane, TMC_GCONF, gconf); n++;
    tmc_write(lane, TMC_GSTAT, 7); n++; // clear reset/drv_err/uv_cp
    tmc_write(lane, TMC_CHOPCONF, chop); n++;
    tmc_write(lane, TMC_IHOLD_IRUN, ihr); n++;
    tmc_write(lane, TMC_TPOWERDOWN, 20); n++;
    tmc_write(lane, TMC_PWMCONF, 0xC80D0E24); n++;
    tmc_write(lane, TMC_TPWMTHRS, 0); n++;
    tmc_write(lane, TMC_TCOOLTHRS, c->sgthrs ? 0xFFFFF : 0); n++;
    tmc_write(lane, TMC_SGTHRS, c->sgthrs); n++;
    if (tmc_read(lane, TMC_IFCNT, &after))
        return -1;
    if (((after - before) & 0xff) != (uint32_t)n)
        return -2;
    setup_done[lane] = 1;
    return 0;
}

int
tmc_health(int lane)
{
    return health[lane];
}

const char *
tmc_health_name(int code)
{
    switch (code) {
    case TMC_OK: return "ok";
    case TMC_NO_UART: return "tmc_uart";
    case TMC_OT: return "tmc_ot";
    case TMC_OTPW: return "tmc_otpw";
    case TMC_SHORT: return "tmc_short";
    case TMC_OPEN: return "tmc_open";
    case TMC_RESET: return "tmc_reset";
    case TMC_UV: return "tmc_uv";
    }
    return "tmc_unknown";
}

static void
set_health(int lane, int code, const char *msg)
{
    if (health[lane] == code)
        return;
    health[lane] = code;
    if (code == TMC_OK) {
        ev_fault("tmc_ok", lane, msg);
        return;
    }
    ev_fault(tmc_health_name(code), lane, msg);
    // Hard faults stop the lane; a warning (otpw, open) only reports.
    if (code != TMC_OTPW && code != TMC_OPEN)
        motion_fault(lane, tmc_health_name(code));
}

static void
check_lane(int lane)
{
    uint32_t gstat, drv;
    if (!setup_done[lane]) {
        int ret = tmc_setup(lane, &cfgs[lane]);
        if (ret == 0) {
            fail_count[lane] = 0;
            set_health(lane, TMC_OK, "driver set up");
        } else if (++fail_count[lane] >= 3) {
            set_health(lane, TMC_NO_UART, ret == -2
                       ? "driver answers but drops writes"
                       : "driver does not answer (no motor power?)");
        }
        return;
    }
    if (tmc_read(lane, TMC_GSTAT, &gstat) || tmc_read(lane, TMC_DRV_STATUS,
                                                     &drv)) {
        if (++fail_count[lane] >= 3) {
            setup_done[lane] = 0;
            set_health(lane, TMC_NO_UART, "driver stopped answering");
        }
        return;
    }
    fail_count[lane] = 0;
    if (gstat & 1) {
        // The driver lost power or reset: its registers are back to
        // defaults, so set it up again.
        setup_done[lane] = 0;
        set_health(lane, TMC_RESET, "driver reset (motor power lost?)");
        return;
    }
    if (drv & (1 << 1))
        set_health(lane, TMC_OT, "over temperature shutdown");
    else if (drv & (0xf << 2))
        set_health(lane, TMC_SHORT, (drv & (3 << 2)) ? "short to ground"
                   : "short on low side");
    else if (gstat & (1 << 2))
        set_health(lane, TMC_UV, "charge pump under voltage");
    else if ((drv & (3 << 6)) && motion_is_moving(lane))
        set_health(lane, TMC_OPEN, "open load (motor wire?)");
    else if (drv & 1)
        set_health(lane, TMC_OTPW, "over temperature warning");
    else
        set_health(lane, TMC_OK, 0);
    if (gstat & 6)
        tmc_write(lane, TMC_GSTAT, 6);
}

static uint32_t next_check;
static int check_idx;

void
tmc_init(void)
{
    bit_ticks = timer_from_us(1000000 / BAUD);
    int i;
    for (i = 0; i < NUM_LANES; i++) {
        tmc_default_cfg(&cfgs[i]);
        gpio_in_setup(LANE_PINS[i].uart, 1);
    }
    next_check = 500;
}

void
tmc_reconfigure(int lane, const struct tmc_cfg *c)
{
    cfgs[lane] = *c;
    setup_done[lane] = 0;
    next_check = uptime();
}

const struct tmc_cfg *
tmc_get_cfg(int lane)
{
    return &cfgs[lane];
}

void
tmc_task(void)
{
    uint32_t now = uptime();
    if ((int32_t)(now - next_check) < 0)
        return;
    next_check = now + 125;
    check_lane(check_idx);
    check_idx = (check_idx + 1) % NUM_LANES;
}
DECL_TASK(tmc_task);
