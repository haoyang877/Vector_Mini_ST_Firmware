#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "boot_epoch_flash.h"

#define PARAM_FLASH_ADDR TARGET_PARAM_BASE

static uint8_t memory[TARGET_PARAM_BYTES];
static bool output_off = true;
static unsigned writes, erases;
static bool epoch_range(uint32_t address, uint32_t size_bytes);

bool power_stage_hw_phases_disabled(void)
{
    return output_off;
}

static bool flash_write_data(uint32_t address, const void *record, uint32_t size_bytes)
{
    assert(address >= TARGET_PARAM_BASE &&
           address - TARGET_PARAM_BASE + size_bytes <= sizeof(memory));
    ++writes;
    memcpy(memory + address - TARGET_PARAM_BASE, record, size_bytes);
    return true;
}

static bool flash_erase_pages(uint32_t first, uint32_t last)
{
    assert(first >= TARGET_PARAM_BASE && last < TARGET_PARAM_BASE + sizeof(memory));
    ++erases;
    memset(memory + first - TARGET_PARAM_BASE, 0xFF, last - first + 1U);
    return true;
}

bool BootEpochFlash_Read(uint32_t address, void *record, uint32_t size_bytes)
{
    if (record == NULL || !epoch_range(address, size_bytes))
    {
        return false;
    }
    memcpy(record, memory + address - TARGET_PARAM_BASE, size_bytes);
    return true;
}

/* BOOT_EPOCH_FLASH_PRODUCTION：脚本插入真实参数和epoch写域生产体。 */

int main(void)
{
    static uint8_t record[TARGET_RAW_STORAGE_BYTES];
    memset(memory, 0xA5, sizeof(memory));
    assert(!flash_write_param(record, 0U));
    assert(!flash_write_param(NULL, 4304U));
    assert(!flash_write_param(record, TARGET_RAW_STORAGE_BYTES + 1U));
    assert(!flash_write_param(record, 0x3800U));
    assert(!flash_write_param(record, UINT32_MAX));
    assert(writes == 0U && erases == 0U);
    output_off = false;
    assert(!flash_write_param(record, 4304U));
    assert(writes == 0U && erases == 0U);
    output_off = true;
    assert(flash_write_param(record, 4304U));
    assert(flash_write_param(record, TARGET_RAW_STORAGE_BYTES));
    for (unsigned i = TARGET_RAW_STORAGE_BYTES; i < sizeof(memory); ++i)
    {
        assert(memory[i] == 0xA5U);
    }
    assert(!epoch_range(TARGET_EPOCH_BASE - 1U, 32U));
    assert(!epoch_range(TARGET_EPOCH_BASE, UINT32_MAX));
    assert(!epoch_range(TARGET_IDENTITY_BASE, 1U));
    assert(!epoch_range(TARGET_EPOCH_BASE, 0U));
    assert(epoch_range(TARGET_EPOCH_BASE, TARGET_EPOCH_BYTES));
    assert(epoch_range(TARGET_IDENTITY_BASE - 32U, 32U));
    unsigned before = writes + erases;
    assert(!BootEpochFlash_Erase(TARGET_PARAM_BASE));
    assert(!BootEpochFlash_Erase(TARGET_IDENTITY_BASE));
    assert(!BootEpochFlash_Erase(TARGET_EPOCH_BASE + 1U));
    assert(!BootEpochFlash_Program(TARGET_EPOCH_BASE - 8U, record));
    assert(!BootEpochFlash_Program(TARGET_IDENTITY_BASE - 8U, record));
    assert(!BootEpochFlash_Program(TARGET_EPOCH_BASE + 1U, record));
    assert(writes + erases == before);
    assert(BootEpochFlash_Erase(TARGET_EPOCH_BASE));
    output_off = false;
    before = writes + erases;
    assert(!BootEpochFlash_Program(TARGET_EPOCH_BASE, record));
    assert(!BootEpochFlash_Erase(TARGET_EPOCH_BASE));
    assert(writes + erases == before);
    output_off = true;
    assert(BootEpochFlash_Program(TARGET_EPOCH_BASE, record));
    assert(!BootEpochFlash_Program(TARGET_EPOCH_BASE, record));
    puts("PASS production flash guards: RAW 3-page boundary, reservations preserved, epoch-only "
         "erase/program, alignment, output-disable guard, no overwrite; no hardware");
    return 0;
}
