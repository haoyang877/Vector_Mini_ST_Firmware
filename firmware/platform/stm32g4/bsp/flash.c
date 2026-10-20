#include "flash.h"
#include "vector_mini_st_g431.h"

#include <string.h>
#include "power_stage_hw.h"

#define PARAM_FLASH_ADDR TARGET_PARAM_BASE

/* 寄存器级 Flash：G431 单 bank、2 KiB 页、双字写；不依赖 HAL Flash，省其链接开销。 */
#define FLASH_ERR_MASK                                                                             \
    (FLASH_SR_OPERR | FLASH_SR_PROGERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_PGSERR |     \
     FLASH_SR_SIZERR | FLASH_SR_MISERR | FLASH_SR_FASTERR | FLASH_SR_RDERR)
#define FLASH_KEY1_VALUE 0x45670123U
#define FLASH_KEY2_VALUE 0xCDEF89ABU
/* 与启动确认路径一样限制软件轮询次数；不是毫秒期限，不依赖IRQ时基。
 * 同bank取指停顿可能阻止执行本循环，不宣称硬件超时保证。 */
#define FLASH_WAIT_MAX_POLLS 1000000U
/* 忙超时可能遗留PG/PER或缓存状态；复位前禁止新写入，不能猜测已经恢复。 */
static bool s_flash_write_blocked;

/* 按已冻结目标布局计算页号。 */
static uint32_t get_page(uint32_t addr)
{
    return (addr - FLASH_BASE) / FLASH_PAGE_SIZE;
}

/* 解锁Flash编程接口；调用方已确认控制器空闲。 */
static bool flash_unlock(void)
{
    if ((FLASH->CR & FLASH_CR_LOCK) != 0U)
    {
        FLASH->KEYR = FLASH_KEY1_VALUE;
        FLASH->KEYR = FLASH_KEY2_VALUE;
    }
    return (FLASH->CR & FLASH_CR_LOCK) == 0U;
}

/* 忙超时后不再改控制寄存器，留待控制器恢复或硬件复位。 */
static void flash_lock(void)
{
    if ((FLASH->SR & FLASH_SR_BSY) == 0U)
    {
        FLASH->CR |= FLASH_CR_LOCK;
    }
}

/* 等待有界次数后判错误位；仅空闲时清标志，忙超时不再写寄存器。 */
static bool flash_ready(void)
{
    if (s_flash_write_blocked)
    {
        return false;
    }
    uint32_t remaining = FLASH_WAIT_MAX_POLLS;
    while ((FLASH->SR & FLASH_SR_BSY) != 0U)
    {
        if (remaining == 0U)
        {
            s_flash_write_blocked = true;
            return false;
        }
        --remaining;
    }
    bool ok = (FLASH->SR & FLASH_ERR_MASK) == 0U;
    FLASH->SR = FLASH_SR_EOP | FLASH_ERR_MASK | FLASH_SR_OPTVERR;
    return ok;
}

/* 擦除参数记录覆盖的页；首个失败立即结束，不继续擦后续页。 */
static bool flash_erase_pages(uint32_t start_addr, uint32_t end_addr)
{
    uint32_t first_page = get_page(start_addr);
    uint32_t page_count = get_page(end_addr) - first_page + 1U;
    bool ok = flash_ready() && flash_unlock();
    for (uint32_t i = 0U; ok && i < page_count; ++i)
    {
        FLASH->CR =
            (FLASH->CR & ~FLASH_CR_PNB_Msk) | FLASH_CR_PER | ((first_page + i) << FLASH_CR_PNB_Pos);
        FLASH->CR |= FLASH_CR_STRT;
        ok = flash_ready();
        if ((FLASH->SR & FLASH_SR_BSY) == 0U)
        {
            FLASH->CR &= ~FLASH_CR_PER;
        }
    }
    flash_lock();
    return ok;
}

/* 双字编程并回读；失败立即返回，不重试或补写。 */
static bool flash_write_data(uint32_t addr, const void *data, uint32_t size_bytes)
{
    const uint8_t *bytes = (const uint8_t *)data;
    uint32_t doubleword_count = (size_bytes + 7U) / 8U;
    uint32_t acr = FLASH->ACR;
    bool ok = flash_ready() && flash_unlock();

    if (!ok)
    {
        flash_lock();
        return false;
    }

    /* 写前关数据缓存、写后复位再恢复，避免读回不一致。 */
    FLASH->ACR = acr & ~FLASH_ACR_DCEN;
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
        if ((FLASH->SR & FLASH_SR_BSY) == 0U)
        {
            FLASH->CR &= ~FLASH_CR_PG;
        }
    }
    if ((FLASH->SR & FLASH_SR_BSY) == 0U)
    {
        FLASH->ACR = (acr & ~FLASH_ACR_DCEN) | FLASH_ACR_DCRST;
        FLASH->ACR = acr;
    }
    flash_lock();
    return ok && memcmp((const void *)addr, data, size_bytes) == 0;
}

bool flash_read_param(void *record, uint32_t size_bytes)
{
    if (record == NULL || size_bytes == 0U || size_bytes > TARGET_PARAM_BYTES)
    {
        return false;
    }
    memcpy(record, (const void *)PARAM_FLASH_ADDR, size_bytes);
    return true;
}
bool flash_write_param(const void *record, uint32_t size_bytes)
{
    if (s_flash_write_blocked || record == NULL || size_bytes == 0U ||
        size_bytes > TARGET_RAW_BYTES || !power_stage_hw_phases_disabled())
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
