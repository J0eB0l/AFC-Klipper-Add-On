static inline void irq_disable(void) {}
static inline void irq_enable(void) {}
static inline void irq_poll(void) {}
typedef unsigned long irqstatus_t;
static inline irqstatus_t irq_save(void) { return 0; }
static inline void irq_restore(irqstatus_t flag) {}
