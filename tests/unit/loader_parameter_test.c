/* 注入真实Loader参数扫描；SN页独立，全部Flash读取由内存替身完成。 */
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "platform.h"
#include "parameters.h"
#include "vector_mini_st_g431.h"

static uint8_t memory[TARGET_PARAM_BYTES];
static bool unreadable;
void LoaderPlatform_Watchdog(void)
{
}
bool LoaderFlash_Read(uint32_t address, void *out, uint32_t bytes);
bool LoaderPlatform_ReadParameters(uint32_t offset, void *out, uint32_t bytes)
{
    uint32_t address = TARGET_PARAM_BASE + offset;
    return LoaderFlash_Read(address, out, bytes);
}
bool LoaderFlash_Read(uint32_t address, void *out, uint32_t bytes)
{
    assert(address >= TARGET_PARAM_BASE && address + bytes <= TARGET_IDENTITY_BASE);
    if (unreadable)
    {
        return false;
    }
    memcpy(out, memory + address - TARGET_PARAM_BASE, bytes);
    return true;
}
int main(void)
{
    uint8_t node = 1U;
    memset(memory, 255, sizeof(memory));
    memset(memory + TARGET_IDENTITY_BASE - TARGET_PARAM_BASE, 0, TARGET_IDENTITY_BYTES);
    assert(LoaderParameters_ReadNode(&node) == YI_PARAMS_BLANK && node == 1U);
    uint32_t schema = TARGET_SCHEMA, magic = TARGET_RAW_MAGIC;
    float id = 3.0f;
    memcpy(memory, &id, 4U);
    memcpy(memory + TARGET_RAW_SCHEMA_OFFSET, &schema, 4U);
    memcpy(memory + TARGET_RAW_MAGIC_OFFSET, &magic, 4U);
    assert(LoaderParameters_ReadNode(&node) == YI_PARAMS_RAW11 && node == 3U);
    for (schema = 12U; schema <= 14U; ++schema)
    {
        memcpy(memory + TARGET_RAW_SCHEMA_OFFSET, &schema, 4U);
        assert(LoaderParameters_ReadNode(&node) == YI_PARAMS_RAW11 && node == 3U);
    }
    schema = 10U;
    memcpy(memory + TARGET_RAW_SCHEMA_OFFSET, &schema, 4U);
    assert(LoaderParameters_ReadNode(&node) == YI_PARAMS_UNSAFE);
    schema = TARGET_SCHEMA;
    memcpy(memory + TARGET_RAW_SCHEMA_OFFSET, &schema, 4U);
    unreadable = true;
    assert(LoaderParameters_ReadNode(&node) == YI_PARAMS_UNSAFE);
    unreadable = false;
    id = 3.5f;
    memcpy(memory, &id, 4U);
    assert(LoaderParameters_ReadNode(&node) == YI_PARAMS_UNSAFE);
    puts("PASS production SN page excluded from calibration blank/valid/ECC decisions");
    return 0;
}
