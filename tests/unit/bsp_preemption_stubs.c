/* 宿主原子访问替身；不模拟PRIMASK，控制锁可由行为夹具覆盖。 */
#include <stdbool.h>
#include <stdint.h>

__attribute__((weak)) uint32_t BspSystem_EnterControlCritical(void)
{
    return 0U;
}

__attribute__((weak)) void BspSystem_ExitControlCritical(uint32_t state)
{
    (void)state;
}

__attribute__((weak)) void BspSystem_MemoryBarrier(void)
{
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
}

__attribute__((weak)) bool
BspSystem_CompareExchange32(volatile uint32_t *address, uint32_t expected, uint32_t desired)
{
    return __atomic_compare_exchange_n(
        address, &expected, desired, false, __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST);
}
