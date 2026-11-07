// Events sent to the host, unasked, as {"ev":...} lines.
//
// Call these from task context only (never from a timer), since they
// format and queue a line.
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#ifndef __TURTLE_EVENTS_H
#define __TURTLE_EVENTS_H

#include <stdint.h>

// {"ev":"sw","sw":"prep1","v":1}
void ev_switch(int sw, int state);
// {"ev":"fault","code":"tmc_ot","lane":2,"msg":"..."}; lane < 0 = no lane.
// Codes are listed in PROTOCOL.md.
void ev_fault(const char *code, int lane, const char *msg);
// {"ev":"log","msg":"..."}
void ev_log(const char *msg);
// Keeps a lane's last move_done (EV_KEEP_MOVE) or job_done (EV_KEEP_JOB)
// line, so "replay" can send it again when the host missed it.
enum { EV_KEEP_MOVE, EV_KEEP_JOB, EV_KEEP_KINDS };
void ev_keep(int lane, int kind, const char *line, int len);
// Sends a lane's kept lines again.
void ev_replay(int lane);

#endif // events.h
