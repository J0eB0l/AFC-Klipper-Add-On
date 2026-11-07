// Boot guard (guard.c).
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#ifndef __TURTLE_GUARD_H
#define __TURTLE_GUARD_H

// Called first thing at boot with the reset cause.
void guard_boot(int power_on, int watchdog);
// The host sent a line with a known command: the build works.
void guard_host_ok(void);
// This boot started as a trial (just flashed, or after a watchdog reset).
int guard_boot_trial(void);
// Failed trials in a row before this boot.
int guard_boot_fails(void);
void guard_task(void);

#endif // guard.h
