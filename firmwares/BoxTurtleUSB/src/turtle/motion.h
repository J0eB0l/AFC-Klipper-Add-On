// Lane stepper motion for the BoxTurtle firmware.
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#ifndef __TURTLE_MOTION_H
#define __TURTLE_MOTION_H

#include <stdint.h>

// Why a move ended.
enum {
    END_NONE = 0,     // still running
    END_DONE,         // full distance, no stop switch asked for
    END_TRIGGERED,    // the "until" switch reached its state
    END_NO_TRIGGER,   // full distance, but the "until" switch never did
    END_WATCH,        // the "watch" switch left its state (runout, lost)
    END_STOPPED,      // host or firmware stop
    END_FAULT,        // driver fault
    END_LINK,         // link loss (the driver keeps holding)
};
const char *end_name(int end);

struct move_req {
    int32_t mm;            // milli-mm, signed (+ = toward the toolhead)
    int32_t speed;         // milli-mm/s
    int32_t accel;         // milli-mm/s^2
    int8_t until_sw;       // switch id, -1 = none
    int8_t until_v;        // state that ends the move
    int8_t watch_sw;       // switch id that must stay in watch_v, -1 = none
    int8_t watch_v;
    int8_t hold;           // keep the driver enabled after the move
};

void motion_init(void);
// Starts a move from task context. Returns 0, or an error string.
const char *motion_start(int lane, const struct move_req *m);
// Stops a lane (or all with lane = -1) at once.
void motion_stop(int lane, int end);
// Stops a lane because of a fault and reports it (task context).
void motion_fault(int lane, const char *code);
int motion_is_moving(int lane);
int motion_any_active(void);
// Last finished move: reason and distance actually moved (milli-mm).
// Returns 1 once per finished move (the "done" flag clears on read).
int motion_take_done(int lane, int *end, int32_t *moved);
void motion_enable(int lane, int on);
int motion_enabled(int lane);
// Driver step direction toward the toolhead: steps/mm (milli) and dir sense.
void motion_set_lane_cfg(int lane, int32_t steps_per_mm_milli, int dir_invert);
int32_t motion_steps_per_mm(int lane);
// Constant speed run for feed assist (milli-mm/s, 0 = stop, sign = dir).
void motion_set_velocity(int lane, int32_t speed);
int32_t motion_get_velocity(int lane);
// Total distance fed by a lane since boot (milli-mm, signed).
int32_t motion_position(int lane);

#endif // motion.h
