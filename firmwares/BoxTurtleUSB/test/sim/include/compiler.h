#include <stddef.h>
#define container_of(ptr, type, member) ((type *)((char *)(ptr) - offsetof(type, member)))
#define DIV_ROUND_UP(n, d) (((n) + (d) - 1) / (d))
#define __always_inline inline
