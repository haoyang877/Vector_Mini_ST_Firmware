#include "parameter_storage_hw.h"

#include <string.h>
#include "main.h"
#include "boot_contract.h"

#define PARAM_FLASH_ADDR BOOT_PARAM_BASE

typedef char ParameterRecordFits[(PARAMETER_STORAGE_RECORD_BYTES <= BOOT_PARAM_CAPACITY &&
    PARAMETER_STORAGE_RECORD_BYTES % 8U == 0U) ? 1 : -1];

uint32_t parameter_storage_hw_enter(void)
{
    uint32_t state = __get_PRIMASK();
    __disable_irq();
    return state;
}

void parameter_storage_hw_exit(uint32_t state)
{
    __set_PRIMASK(state);
}

void parameter_storage_hw_read(void *record)
{
    memcpy(record, (const void *)PARAM_FLASH_ADDR, PARAMETER_STORAGE_RECORD_BYTES);
}

ParameterStorageResult parameter_storage_hw_write(const void *record)
{
    ParameterStorageResult result = PARAMETER_STORAGE_UNLOCK_FAILED;
    FLASH_EraseInitTypeDef erase_init;
    uint32_t page_error = 0U;
    uint32_t first_page = (PARAM_FLASH_ADDR - FLASH_BASE) / FLASH_PAGE_SIZE;
    uint32_t last_page = (PARAM_FLASH_ADDR + PARAMETER_STORAGE_RECORD_BYTES - 1U - FLASH_BASE) / FLASH_PAGE_SIZE;

    if (memcmp((const void *)PARAM_FLASH_ADDR, record, PARAMETER_STORAGE_RECORD_BYTES) == 0)
    {
        return PARAMETER_STORAGE_UNCHANGED;
    }
    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        goto lock_flash;
    }
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
    erase_init.Banks = FLASH_BANK_1;
    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.NbPages = 1U;
    for (uint32_t page = first_page; page <= last_page; ++page)
    {
        erase_init.Page = page;
        if (HAL_FLASHEx_Erase(&erase_init, &page_error) != HAL_OK)
        {
            result = PARAMETER_STORAGE_ERASE_FAILED;
            goto lock_flash;
        }
    }
    for (uint32_t offset = 0U; offset < PARAMETER_STORAGE_RECORD_BYTES; offset += 8U)
    {
        uint64_t value;
        /* 当前记录固定为2384字节，恰好298个双字，不访问结构以外的字节。 */
        memcpy(&value, (const uint8_t *)record + offset, sizeof(value));
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, PARAM_FLASH_ADDR + offset, value) != HAL_OK)
        {
            result = PARAMETER_STORAGE_PROGRAM_FAILED;
            goto lock_flash;
        }
    }
    result = memcmp((const void *)PARAM_FLASH_ADDR, record, PARAMETER_STORAGE_RECORD_BYTES) == 0 ?
        PARAMETER_STORAGE_WRITTEN : PARAMETER_STORAGE_VERIFY_FAILED;

lock_flash:
    if (HAL_FLASH_Lock() != HAL_OK && result == PARAMETER_STORAGE_WRITTEN)
    {
        result = PARAMETER_STORAGE_LOCK_FAILED;
    }
    return result;
}
