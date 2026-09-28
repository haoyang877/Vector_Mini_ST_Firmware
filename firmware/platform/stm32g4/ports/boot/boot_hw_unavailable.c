/* 独立APP目标未重定位也未保留启动邮箱，平台明确不提供Loader能力。 */
#include "boot_hw.h"

bool BootHw_Init(void)
{
    return false;
}

bool BootHw_EnterLoader(uint8_t node)
{
    (void)node;
    return false;
}

void BootHw_Watchdog(bool healthy)
{
    (void)healthy;
}
