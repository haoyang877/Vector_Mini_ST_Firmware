/* 注入真实Flash等待和边界代码；W1C仅由模型模拟，不连接硬件。 */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "platform.h"

static struct
{
    uint32_t sts;
} s_flash;
#define FLASH (&s_flash)
static bool s_flash_failed;
static uint32_t s_now, s_release;
static bool flash_ready(bool completed);
static bool inside(uint32_t address, uint32_t bytes, const LoaderFlashRegion *region);

uint32_t LoaderPlatform_NowMs(void)
{
    if (s_now == s_release)
    {
        s_flash.sts = FLASH_ODF_FLAG;
    }
    return s_now++;
}
static void model_clear(uint32_t flags)
{
    s_flash.sts &= ~flags;
}
static void setup(uint32_t flags)
{
    s_flash.sts = flags;
    s_flash_failed = false;
    s_now = 0U;
    s_release = UINT32_MAX;
}
int main(void)
{
    const LoaderFlashRegion app = {0x08004000U, 98304U};
    assert(inside(0x08004000U, 2048U, &app));
    assert(inside(0x0801BFF8U, 8U, &app));
    assert(!inside(0x08003FF8U, 8U, &app));
    assert(!inside(0x0801C000U, 8U, &app));
    assert(!inside(UINT32_MAX - 3U, 8U, &app));
    assert(!inside(app.base, 0U, &app));
    setup(FLASH_PRGMERR_FLAG);
    assert(!flash_ready(false) && s_flash_failed);
    s_flash.sts = FLASH_ODF_FLAG;
    assert(!flash_ready(true));
    setup(FLASH_EPPERR_FLAG);
    assert(!flash_ready(true));
    setup(0U);
    assert(!flash_ready(true));
    setup(0U);
    assert(flash_ready(false));
    setup(FLASH_OBF_FLAG);
    assert(!flash_ready(true) && s_flash_failed && s_now <= 1002U);
    setup(FLASH_OBF_FLAG);
    s_release = 20U;
    assert(flash_ready(true));
    setup(FLASH_OBF_FLAG);
    s_now = UINT32_MAX - 10U;
    s_release = 10U;
    assert(flash_ready(true));
    puts("PASS real AT32 Flash bounds, error latch, timeout and timer wrap");
    return 0;
}
