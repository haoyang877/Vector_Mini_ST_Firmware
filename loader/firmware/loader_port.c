/* G431固定分区硬件边界；只允许记录页与APP区域写入。 */
#include "loader_port.h"
#include "stm32g4xx_hal.h"
#include <string.h>

static bool erase_pages(uint32_t page, uint32_t count)
{
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t error;
    bool ok;
    if (!((page == 6 && count == 1) || (page == 8 && count == 48)))
    {
        return false;
    }
    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return false;
    }
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks = FLASH_BANK_1;
    erase.Page = page;
    erase.NbPages = count;
    ok = HAL_FLASHEx_Erase(&erase, &error) == HAL_OK;
    if (HAL_FLASH_Lock() != HAL_OK)
    {
        ok = false;
    }
    return ok;
}

static bool blank(uint32_t address, uint32_t size)
{
    uint32_t i;
    const volatile uint8_t *p = (const volatile uint8_t *)address;
    for (i = 0; i < size; ++i)
    {
        if (p[i] != 0xFF)
        {
            return false;
        }
    }
    return true;
}

bool LoaderPort_EraseRecord(void)
{
    return erase_pages(6, 1) && blank(BOOT_RECORD_BASE, BOOT_PAGE_BYTES);
}

bool LoaderPort_EraseApp(void)
{
    return erase_pages(8, 48) && blank(BOOT_APP_BASE, BOOT_APP_CAPACITY);
}

static bool program(uint32_t address, const uint8_t *data, uint32_t size)
{
    uint32_t i;
    uint64_t value;
    bool ok = true;
    if (!blank(address, size) || HAL_FLASH_Unlock() != HAL_OK)
    {
        return false;
    }
    for (i = 0; i < size; i += 8)
    {
        memcpy(&value, data + i, 8);
        /* 全FF双字无需编程，避免消耗ECC写入机会。 */
        if (value != UINT64_MAX &&
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address + i, value) != HAL_OK)
        {
            ok = false;
            break;
        }
    }
    if (HAL_FLASH_Lock() != HAL_OK)
    {
        ok = false;
    }
    return ok && memcmp((const void *)address, data, size) == 0;
}

bool LoaderPort_Program(uint32_t offset, const uint8_t *data, uint32_t size)
{
    if (data == NULL || size < 8 || size > 40 || (size & 7U) != 0 || (offset & 7U) != 0 ||
        offset > BOOT_APP_CAPACITY - size || size > BOOT_PAGE_BYTES - offset % BOOT_PAGE_BYTES)
    {
        return false;
    }
    return program(BOOT_APP_BASE + offset, data, size);
}

const uint8_t *LoaderPort_App(void) { return (const uint8_t *)BOOT_APP_BASE; }

void LoaderPort_ReadRecord(uint8_t out[BOOT_RECORD_BYTES])
{
    memcpy(out, (const void *)BOOT_RECORD_BASE, BOOT_RECORD_BYTES);
}

bool LoaderPort_Commit(const uint8_t raw[BOOT_RECORD_BYTES])
{
    BootImage image;
    return Boot_RecordDecode(raw, &image) && blank(BOOT_RECORD_BASE, BOOT_RECORD_BYTES) &&
           program(BOOT_RECORD_BASE, raw, 40) && program(BOOT_RECORD_BASE + 40, raw + 40, 8);
}

void LoaderPort_Uid(uint8_t out[12]) { memcpy(out, (const void *)UID_BASE, 12); }

uint32_t LoaderPort_ParamCrc(void)
{
    return Boot_Crc32((const uint8_t *)BOOT_PARAM_BASE, BOOT_PARAM_CAPACITY);
}
