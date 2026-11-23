#include "boot_hw.h"
#include "../../bsp_internal.h"

/* 当前仅独立APP从Flash起点启动；没有AT32 Loader，明确报告能力不可用。 */
bool BootHw_Init(void)
{
    return false;
}
bool BootHw_EnterLoader(uint8_t node, uint32_t started_ms, uint32_t timeout_ms)
{
    (void)node;
    (void)started_ms;
    (void)timeout_ms;
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

bool BootHw_IsAvailable(void)
{
    return false;
}
