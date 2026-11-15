// Espooler (N20 respooler) control and TurtleNeck feed assist.
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#ifndef __TURTLE_SPOOL_H
#define __TURTLE_SPOOL_H

#include <stdint.h>

void spool_init(void);
// duty in permille, signed: > 0 forward (unwind), < 0 rewind, 0 = brake+off.
// kick = 1 runs full duty for the kick-start time first.
void espool_set(int lane, int32_t duty, int kick);
int32_t espool_get(int lane);

// Feed assist modes: 0 = TurtleNeck while printing, ASSIST_FOLLOW = track
// the extruder position both ways like AFC's sync_to_extruder, ASSIST_EXT =
// track the position the host scales from its own buffer, with print pulses.
#define ASSIST_FOLLOW 1
#define ASSIST_EXT 2
void assist_start(int lane, int mode);
// 0 (off or tn), ASSIST_FOLLOW or ASSIST_EXT
int assist_following(int lane);
void assist_stop(int lane);
int assist_active(int lane);
// Reads assist_* and espool_* keys from a cfg line.
void assist_cfg(const char *line);
// Per move espooler duty (espool_fwd/espool_rwd/espool_kick on a move or
// job line) and per lane print assist pulses (lane_cfg).
void espool_move_cfg(int lane, const char *line);
void espool_lane_cfg(int lane, const char *line);

int spool_any_active(void);
int spool_handle_cmd(const char *cmd, char *line);

#endif // spool.h
