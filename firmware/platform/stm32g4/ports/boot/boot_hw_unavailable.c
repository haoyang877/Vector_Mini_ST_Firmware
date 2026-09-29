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
bool BootHw_Confirm(uint32_t version, uint32_t crc32)
{
    (void)version;
    (void)crc32;
    return false;
}
bool BootHw_IsConfirmed(void)
{
    return false;
}
uint32_t BootHw_ImageCrc32(void)
{
    return 0U;
}
