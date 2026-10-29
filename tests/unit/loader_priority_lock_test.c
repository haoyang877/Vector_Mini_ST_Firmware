/* 运行实际Loader锁实现，验证优先级门槛及嵌套恢复。 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>

uint32_t LoaderPlatform_Lock(void);
void LoaderPlatform_Unlock(uint32_t key);

int main(void)
{
    const uint32_t ceiling = 1U << (8U - __NVIC_PRIO_BITS);
    const uint32_t initial[] = {0U, ceiling, 2U * ceiling};
    for (unsigned i = 0U; i < sizeof(initial) / sizeof(initial[0]); ++i)
    {
        __set_BASEPRI(initial[i]);
        uint32_t outer = LoaderPlatform_Lock();
        assert(outer == initial[i] && __get_BASEPRI() == ceiling);
        uint32_t inner = LoaderPlatform_Lock();
        assert(inner == ceiling && __get_BASEPRI() == ceiling);
        LoaderPlatform_Unlock(inner);
        assert(__get_BASEPRI() == ceiling);
        LoaderPlatform_Unlock(outer);
        assert(__get_BASEPRI() == initial[i]);
    }
    puts("PASS Loader priority lock: CAN masked, nested priority restored");
    return 0;
}
