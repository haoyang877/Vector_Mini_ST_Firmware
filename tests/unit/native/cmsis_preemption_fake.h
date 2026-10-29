/* 宿主CMSIS替身：仅模拟优先级阈值、内存顺序和排他访问，不操作设备。 */
#ifndef CMSIS_PREEMPTION_FAKE_H
#define CMSIS_PREEMPTION_FAKE_H
#include <stdint.h>
#define __NVIC_PRIO_BITS 4U
static uint32_t s_fake_basepri;
static inline uint32_t __get_BASEPRI(void)
{
    return s_fake_basepri;
}
static inline void __set_BASEPRI(uint32_t value)
{
    s_fake_basepri = value;
}
static inline void __set_BASEPRI_MAX(uint32_t value)
{
    if (value != 0U && (s_fake_basepri == 0U || value < s_fake_basepri))
    {
        s_fake_basepri = value;
    }
}
static inline void __DMB(void)
{
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
}
static inline void __DSB(void)
{
    __DMB();
}
static inline void __ISB(void)
{
    __DMB();
}
static inline uint32_t __LDREXW(volatile uint32_t *address)
{
    return *address;
}
static inline uint32_t __STREXW(uint32_t value, volatile uint32_t *address)
{
    *address = value;
    return 0U;
}
static inline void __CLREX(void)
{
}
#endif
