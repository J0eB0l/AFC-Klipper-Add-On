// AFC-Lite (BoxTurtle) pin map and shared firmware state.
//
// Pins come from AFC-Klipper-Add-On config/mcu/AFC_Lite.cfg.
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#ifndef __TURTLE_BOARD_H
#define __TURTLE_BOARD_H

#include <stdint.h>

#define TGPIO(PORT, NUM) (((PORT)-'A') * 16 + (NUM))

#define FW_NAME "AFC-Turtle"
#define FW_VERSION "0.1.0"
#define FW_PROTO "boxturtle-1"

#define NUM_LANES 4

// Drivers: STEP, DIR, EN, UART (TMC2209 single wire)
struct lane_pins {
    uint8_t step, dir, en, uart;
    uint8_t mot_rwd, mot_fwd, mot_en;
    uint8_t prep, load;
};

static const struct lane_pins LANE_PINS[NUM_LANES] = {
    { TGPIO('D', 4),  TGPIO('D', 3),  TGPIO('D', 6), TGPIO('D', 5),
      TGPIO('A', 0),  TGPIO('A', 1),  TGPIO('A', 2),
      TGPIO('C', 5),  TGPIO('E', 8) },
    { TGPIO('C', 12), TGPIO('C', 11), TGPIO('D', 1), TGPIO('D', 0),
      TGPIO('A', 6),  TGPIO('A', 7),  TGPIO('A', 5),
      TGPIO('B', 0),  TGPIO('E', 9) },
    { TGPIO('E', 2),  TGPIO('E', 3),  TGPIO('E', 0), TGPIO('E', 1),
      TGPIO('B', 14), TGPIO('B', 15), TGPIO('B', 13),
      TGPIO('B', 1),  TGPIO('E', 10) },
    { TGPIO('D', 15), TGPIO('D', 14), TGPIO('C', 7), TGPIO('C', 6),
      TGPIO('D', 12), TGPIO('D', 13), TGPIO('D', 11),
      TGPIO('B', 2),  TGPIO('E', 11) },
};

#define PIN_HUB     TGPIO('C', 4)
#define PIN_TN_ADV  TGPIO('E', 12)
#define PIN_TN_TRL  TGPIO('E', 13)
#define PIN_RGB1    TGPIO('E', 14)
#define PIN_RGB2    TGPIO('E', 15)
#define PIN_RGB3    TGPIO('B', 10)
#define PIN_RGB4    TGPIO('B', 11)

/****************************************************************
 * Switches (sensors.c)
 ****************************************************************/

// Switch ids, also the bit numbers in the state mask.
enum {
    SW_HUB = 0,
    SW_PREP1, SW_PREP2, SW_PREP3, SW_PREP4,
    SW_LOAD1, SW_LOAD2, SW_LOAD3, SW_LOAD4,
    SW_TN_ADV, SW_TN_TRL,
    SW_COUNT
};
#define SW_PREP(lane) (SW_PREP1 + (lane))
#define SW_LOAD(lane) (SW_LOAD1 + (lane))

extern const char * const SW_NAMES[SW_COUNT];

void sensors_init(void);
// Milliseconds since power up (counted by the sensor timer).
uint32_t uptime(void);
// Debounced state, bit n = switch n triggered.
uint32_t sensors_state(void);
static inline int sensor(int sw) { return (sensors_state() >> sw) & 1; }
// TurtleNeck switch that changed last, pressed or released (SW_TN_ADV,
// SW_TN_TRL, -1 none yet), and a count of its changes.
int sensors_tn_last(void);
uint32_t sensors_tn_edges(void);
// Sets whether a switch reads inverted (1 = triggered when the pin is low).
void sensors_set_invert(int sw, int invert);
int sensors_get_invert(int sw);
// Debounce time in ms (1..50).
void sensors_set_debounce(int ms);
// Called by the 1 ms sensor timer after each update (motion.c hooks in here).
void sensors_tick_hook(uint32_t state, uint32_t changed);

/****************************************************************
 * LEDs (leds.c)
 ****************************************************************/

#define NUM_STRIPS 4
#define STRIP_MAX_LEDS 96

void leds_init(void);
// order is "GRB", "GRBW", "RGB" or "RGBW". Returns 0 on bad input.
int leds_config(int strip, int count, const char *order);
// Sets count LEDs from index first to r,g,b,w (0-255 each).
int leds_set(int strip, int first, int count, uint8_t r, uint8_t g,
             uint8_t b, uint8_t w);
// Marks a strip for sending; the LED task transmits it.
void leds_commit(int strip);
int leds_count(int strip);

#endif // board.h
