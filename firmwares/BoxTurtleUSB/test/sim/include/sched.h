// Simulator stand-in for Klipper's sched.h
#ifndef SIM_SCHED_H
#define SIM_SCHED_H
#include <stdint.h>
#define DECL_INIT(f)
#define DECL_TASK(f)
#define DECL_SHUTDOWN(f)
struct timer { struct timer *next; uint_fast8_t (*func)(struct timer*); uint32_t waketime; };
enum { SF_DONE=0, SF_RESCHEDULE=1 };
struct task_wake { uint8_t wake; };
void sched_add_timer(struct timer*);
void sched_del_timer(struct timer*);
void sched_wake_task(struct task_wake *w);
uint8_t sched_check_wake(struct task_wake *w);
#endif
