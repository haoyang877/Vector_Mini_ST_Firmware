#ifndef TIME_HW_H
#define TIME_HW_H
#include <stdint.h>
/** Monotonic millisecond clock modulo 2^32, available from foreground/ISR. */
uint32_t time_hw_now_ms(void);
#endif
