// TMC2209 single-wire UART driver setup and monitoring.
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#ifndef __TURTLE_TMC_H
#define __TURTLE_TMC_H

#include <stdint.h>

#define TMC_GCONF      0x00
#define TMC_GSTAT      0x01
#define TMC_IFCNT      0x02
#define TMC_IHOLD_IRUN 0x10
#define TMC_TPOWERDOWN 0x11
#define TMC_TPWMTHRS   0x13
#define TMC_TCOOLTHRS  0x14
#define TMC_SGTHRS     0x40
#define TMC_SG_RESULT  0x41
#define TMC_CHOPCONF   0x6C
#define TMC_DRV_STATUS 0x6F
#define TMC_PWMCONF    0x70

struct tmc_cfg {
    uint16_t run_ma, hold_ma;   // currents in mA (RMS)
    uint16_t sense_mohm;        // sense resistor in milliohm
    uint16_t microsteps;        // 1..256
    uint8_t spreadcycle;        // 1 = spreadCycle, 0 = stealthChop
    uint8_t sgthrs;             // StallGuard threshold (stealthChop only)
};

// Blocking register access from task context (a few ms each).
// Return 0 on success, -1 when the driver does not answer or the CRC fails.
int tmc_write(int lane, uint8_t reg, uint32_t val);
int tmc_read(int lane, uint8_t reg, uint32_t *val);
int tmc_wtest(int lane, int flags, int gap, int count, uint8_t reg, uint32_t val,
              uint32_t *before, uint32_t *after);

// Writes every register for the lane. Returns 0 when the write counter
// shows all writes landed.
int tmc_setup(int lane, const struct tmc_cfg *cfg);
// Default config (stock BoxTurtle: 0.8 A, 0.110 ohm, 16 microsteps, spreadCycle).
void tmc_default_cfg(struct tmc_cfg *cfg);

// Last health state per lane, updated by the monitor task.
enum {
    TMC_OK = 0, TMC_NO_UART, TMC_OT, TMC_OTPW, TMC_SHORT, TMC_OPEN,
    TMC_RESET, TMC_UV,
};
int tmc_health(int lane);
void tmc_init(void);
// New settings for a lane; the monitor task writes them to the driver.
void tmc_reconfigure(int lane, const struct tmc_cfg *cfg);
const struct tmc_cfg *tmc_get_cfg(int lane);
const char *tmc_health_name(int code);

#endif // tmc.h
