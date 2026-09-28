/* 独立APP目标未重定位也未保留启动邮箱，平台明确不提供Loader能力。 */
#include "boot_hw.h"

#include <stddef.h>

bool BootHw_Init(void)
{
    return false;
}

void BootHw_GetInfo(BootHwInfo *info)
{
    if (info != NULL)
    {
        *info = (BootHwInfo){0};
    }
}

bool BootHw_Confirm(uint64_t boot_epoch, uint32_t image_token)
{
    (void)boot_epoch;
    (void)image_token;
    return false;
}

bool BootHw_EnterLoader(uint64_t boot_epoch, uint8_t node)
{
    (void)boot_epoch;
    (void)node;
    return false;
}

void BootHw_Watchdog(bool healthy)
{
    (void)healthy;
}
