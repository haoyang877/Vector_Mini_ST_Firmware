/* 检查真实平台只读入口的边界；物理 Flash 由内存记录替代。 */
#include "platform.h"
#include "loader_target.h"
#include <assert.h>
#include <stddef.h>
#include <stdio.h>

static unsigned calls;
static bool readable = true;
bool LoaderFlash_Read(uint32_t address, void *destination, uint32_t bytes)
{
    assert(destination != NULL && bytes != 0U);
    assert(address >= TARGET_PARAM_BASE && address + bytes <= TARGET_IDENTITY_BASE);
    ++calls;
    return readable;
}
int main(void)
{
    uint8_t data[8];
    const uint32_t capacity = TARGET_IDENTITY_BASE - TARGET_PARAM_BASE;
    assert(LoaderPlatform_ReadParameters(0, data, sizeof(data)));
    assert(LoaderPlatform_ReadParameters(capacity - 1U, data, 1U));
    assert(calls == 2U);
    assert(!LoaderPlatform_ReadParameters(capacity, data, 1U));
    assert(!LoaderPlatform_ReadParameters(capacity - 1U, data, 2U));
    assert(!LoaderPlatform_ReadParameters(UINT32_MAX, data, 1U));
    assert(!LoaderPlatform_ReadParameters(0, data, UINT32_MAX));
    assert(!LoaderPlatform_ReadParameters(0, data, 0));
    assert(!LoaderPlatform_ReadParameters(0, NULL, 1U));
    assert(calls == 2U);
    readable = false;
    assert(!LoaderPlatform_ReadParameters(0, data, sizeof(data)));
    puts("PASS parameter reader rejects overflow, identity page, zero and null ranges");
    return 0;
}
