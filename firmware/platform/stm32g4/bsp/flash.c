#include "flash.h"

#include <string.h>
#include "power_stage_hw.h"

#define PARAM_FLASH_ADDR ADDR_FLASH_PAGE_56

/**
    * @brief  按地址计算 Flash 页号
    * @param  addr Flash 地址
    * @return 页号
 **/
static uint32_t get_page(uint32_t addr)
{
    return (addr - FLASH_BASE) / FLASH_PAGE_SIZE;
}

/**
    * @brief  擦除覆盖起止地址的 Flash 页
    * @param  start_addr 起始 Flash 地址
    * @param  end_addr 结束 Flash 地址
    * @return 擦除成功返回 true
 **/
static bool flash_erase_pages(uint32_t start_addr, uint32_t end_addr)
{
    uint32_t page_error = 0U;
    FLASH_EraseInitTypeDef erase_init;
    uint32_t first_page = get_page(start_addr);
    uint32_t page_count = get_page(end_addr) - first_page + 1U;

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
    erase_init.Banks = FLASH_BANK_1;
    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.Page = first_page;
    erase_init.NbPages = page_count;
    bool result = HAL_FLASHEx_Erase(&erase_init, &page_error) == HAL_OK;
    HAL_FLASH_Lock();
    return result;
}

/**
    * @brief  写入并回读校验 Flash 字节
    * @param  addr Flash 地址
    * @param  data 稳定输入缓冲
    * @param  size_bytes 写入字节数
 **/
static bool flash_write_data(uint32_t addr, const void *data, uint32_t size_bytes)
{
    const uint8_t *bytes = (const uint8_t *)data;
    uint32_t doubleword_count = (size_bytes + 7U) / 8U;

    HAL_FLASH_Unlock();
    for (uint32_t i = 0U; i < doubleword_count; ++i)
    {
        uint64_t value = UINT64_MAX;
        uint32_t offset = i * 8U;
        uint32_t copy_size = size_bytes - offset;
        if (copy_size > 8U)
        {
            copy_size = 8U;
        }
        memcpy(&value, &bytes[offset], copy_size);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr + offset, value) != HAL_OK)
        {
            HAL_FLASH_Lock();
            return false;
        }
    }
    HAL_FLASH_Lock();
    return memcmp((const void *)addr, data, size_bytes) == 0;
}

bool flash_read_param(void *record, uint32_t size_bytes)
{
    if (record == NULL || size_bytes == 0U || size_bytes > 0x4000U)
    {
        return false;
    }
    memcpy(record, (const void *)PARAM_FLASH_ADDR, size_bytes);
    return true;
}
bool flash_write_param(const void *record, uint32_t size_bytes)
{
    if (record == NULL || size_bytes == 0U || size_bytes > 0x4000U ||
        !power_stage_hw_phases_disabled())
    {
        return false;
    }
    return flash_erase_pages(PARAM_FLASH_ADDR, PARAM_FLASH_ADDR + size_bytes - 1U) &&
           flash_write_data(PARAM_FLASH_ADDR, record, size_bytes);
}
