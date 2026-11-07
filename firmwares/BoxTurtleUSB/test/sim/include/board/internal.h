#include <stdint.h>
struct sim_rcc { volatile uint32_t RSR; };
extern struct sim_rcc sim_rcc;
#define RCC (&sim_rcc)
#define RCC_RSR_IWDG1RSTF (1u<<26)
#define RCC_RSR_WWDG1RSTF (1u<<28)
#define RCC_RSR_BORRSTF (1u<<21)
#define RCC_RSR_PORRSTF (1u<<23)
#define RCC_RSR_SFTRSTF (1u<<24)
#define RCC_RSR_LPWRRSTF (1u<<30)
#define RCC_RSR_PINRSTF (1u<<22)
#define RCC_RSR_RMVF (1u<<16)
void NVIC_SystemReset(void);
extern uint32_t sim_uid[3];
#define TURTLE_UID_ADDR (sim_uid)
struct sim_guard_mem { uint32_t w[8]; };
extern struct sim_guard_mem sim_guard;
#define TURTLE_GUARD_ADDR (&sim_guard)
#define SCB_CleanDCache_by_Addr(a, n) ((void)(a), (void)(n))
