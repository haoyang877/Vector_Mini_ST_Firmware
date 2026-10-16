#include "flash.h"
#include "vector_mini_st_g431.h"

#include <string.h>
#include "power_stage_hw.h"

#define PARAM_FLASH_ADDR ADDR_FLASH_PAGE_56

/* 寄存器级 Flash：G431 单 bank、2 KiB 页、双字写；不依赖 HAL Flash，省其链接开销。 */
#define FLASH_ERR_MASK                                                                             \
    (FLASH_SR_OPERR | FLASH_SR_PROGERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_PGSERR |     \
     FLASH_SR_SIZERR | FLASH_SR_MISERR | FLASH_SR_FASTERR | FLASH_SR_RDERR)
#define FLASH_KEY1_VALUE 0x45670123U
#define FLASH_KEY2_VALUE 0xCDEF89ABU

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
    * @brief  解锁 Flash 编程接口；已解锁返回 true
 **/
static bool flash_unlock(void)
{
    if ((FLASH->CR & FLASH_CR_LOCK) != 0U)
    {
        FLASH->KEYR = FLASH_KEY1_VALUE;
        FLASH->KEYR = FLASH_KEY2_VALUE;
    }
    return (FLASH->CR & FLASH_CR_LOCK) == 0U;
}

/**
    * @brief  重新锁住 Flash 编程接口
 **/
static void flash_lock(void)
{
    FLASH->CR |= FLASH_CR_LOCK;
}

/**
    * @brief  等待当前 Flash 操作完成并判错误位；完成后清 EOP 与错误标志
 **/
static bool flash_ready(void)
{
    while ((FLASH->SR & FLASH_SR_BSY) != 0U)
    {
    }
    bool ok = (FLASH->SR & FLASH_ERR_MASK) == 0U;
    FLASH->SR = FLASH_SR_EOP | FLASH_ERR_MASK | FLASH_SR_OPTVERR;
    return ok;
}

/**
    * @brief  擦除覆盖起止地址的 Flash 页
    * @param  start_addr 起始 Flash 地址
    * @param  end_addr 结束 Flash 地址
    * @return 擦除成功返回 true
 **/
static bool flash_erase_pages(uint32_t start_addr, uint32_t end_addr)
{
    uint32_t first_page = get_page(start_addr);
    uint32_t page_count = get_page(end_addr) - first_page + 1U;
    bool ok = flash_unlock();

    FLASH->SR = FLASH_SR_OPTVERR;
    (void)flash_ready();
    for (uint32_t i = 0U; ok && i < page_count; ++i)
    {
        FLASH->CR =
            (FLASH->CR & ~FLASH_CR_PNB_Msk) | FLASH_CR_PER | ((first_page + i) << FLASH_CR_PNB_Pos);
        FLASH->CR |= FLASH_CR_STRT;
        ok = flash_ready();
        FLASH->CR &= ~FLASH_CR_PER;
    }
    flash_lock();
    return ok;
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
    uint32_t acr = FLASH->ACR;
    bool ok = flash_unlock();

    /* 写前关数据缓存、写后复位再恢复，避免读回不一致。 */
    FLASH->ACR = acr & ~FLASH_ACR_DCEN;
    (void)flash_ready();
    for (uint32_t i = 0U; ok && i < doubleword_count; ++i)
    {
        uint64_t value = UINT64_MAX;
        uint32_t offset = i * 8U;
        uint32_t copy_size = size_bytes - offset;
        if (copy_size > 8U)
        {
            copy_size = 8U;
        }
        memcpy(&value, &bytes[offset], copy_size);
        FLASH->CR |= FLASH_CR_PG;
        *(volatile uint32_t *)(addr + offset) = (uint32_t)value;
        *(volatile uint32_t *)(addr + offset + 4U) = (uint32_t)(value >> 32U);
        ok = flash_ready();
        FLASH->CR &= ~FLASH_CR_PG;
    }
    FLASH->ACR = (acr & ~FLASH_ACR_DCEN) | FLASH_ACR_DCRST;
    FLASH->ACR = acr;
    flash_lock();
    return ok && memcmp((const void *)addr, data, size_bytes) == 0;
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
    if (record == NULL || size_bytes == 0U || size_bytes > TARGET_RAW_BYTES ||
        !power_stage_hw_phases_disabled())
    {
        return false;
    }
    if (memcmp((const void *)PARAM_FLASH_ADDR, record, size_bytes) == 0)
    {
        return true;
    }
    return flash_erase_pages(PARAM_FLASH_ADDR, PARAM_FLASH_ADDR + size_bytes - 1U) &&
           flash_write_data(PARAM_FLASH_ADDR, record, size_bytes);
}
