#ifndef SIM_MISC_H
#define SIM_MISC_H
#include <stdint.h>
uint32_t timer_read_time(void);
static inline uint32_t timer_from_us(uint32_t us) { return us * 400; }
void bootloader_request(void);
#endif
