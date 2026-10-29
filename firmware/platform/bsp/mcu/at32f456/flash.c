#include "bsp_flash.h"
#include "bsp_system.h"
#include "bsp_internal.h"

#include <stddef.h>
#include <string.h>
#include "at32f45x.h"
#include "bsp_pwm.h"
#include "bsp_config.h"

/* 仅停机前台，BSP内部拥有中断屏蔽和恢复；不访问业务对象、不改变参数schema。
 * 同bank取指可能被Flash忙状态阻塞，轮询上限不是硬件墙钟保证。
 * 超时后保留控制器现场并阻止后续读写，必须硬件复位后才能再次写。 */
#define R40_FLASH_WAIT_POLLS 1000000U
#define R40_FLASH_ERRORS (FLASH_PRGMERR_FLAG | FLASH_EPPERR_FLAG)
static bool s_flash_blocked;
static bool s_storage_initialized;

static const uint8_t *flash_memory(uint32_t address)
{
    return (const uint8_t *)(uintptr_t)address;
}

static void flash_store_word(uint32_t address, uint32_t value)
{
    *(volatile uint32_t *)(uintptr_t)address = value;
}

static bool flash_ready(bool require_completed)
{
    if (s_flash_blocked)
    {
        return false;
    }
    uint32_t polls = R40_FLASH_WAIT_POLLS;
    while (flash_flag_get(FLASH_OBF_FLAG) != RESET)
    {
        if (--polls == 0U)
        {
            s_flash_blocked = true;
            return false;
        }
    }
    bool ok = flash_flag_get(R40_FLASH_ERRORS) == RESET &&
              (!require_completed || flash_flag_get(FLASH_ODF_FLAG) != RESET);
    flash_flag_clear(FLASH_ODF_FLAG | R40_FLASH_ERRORS);
    return ok;
}

static void lock_if_idle(void)
{
    if (!s_flash_blocked && flash_flag_get(FLASH_OBF_FLAG) == RESET)
    {
        flash_lock();
    }
}

static bool write_record(const void *record, uint32_t size_bytes)
{
    if (record == NULL || size_bytes == 0U || size_bytes > R40_PARAM_RAW_BYTES || s_flash_blocked ||
        __get_IPSR() != 0U || __get_PRIMASK() == 0U || !BspPwm_ArePhasesDisabled())
    {
        return false;
    }
    uintptr_t source = (uintptr_t)record;
    if (source > UINTPTR_MAX - size_bytes ||
        (source < R40_PARAM_BASE + R40_PARAM_BYTES && source + size_bytes > R40_PARAM_BASE))
    {
        return false;
    }
    if (!flash_ready(false))
    {
        lock_if_idle();
        return false;
    }
    if (memcmp(flash_memory(R40_PARAM_BASE), record, size_bytes) == 0)
    {
        lock_if_idle();
        return true;
    }
    flash_unlock();
    if (FLASH->ctrl_bit.oplk)
    {
        return false;
    }
    if (!McuAdc_PauseForStorage())
    {
        lock_if_idle();
        return false;
    }
    bool ok = true;
    for (uint32_t offset = 0U; ok && offset < size_bytes; offset += R40_FLASH_SECTOR_BYTES)
    {
        FLASH->ctrl_bit.secers = TRUE;
        FLASH->addr = R40_PARAM_BASE + offset;
        FLASH->ctrl_bit.erstr = TRUE;
        ok = flash_ready(true);
        if (!s_flash_blocked)
        {
            FLASH->ctrl_bit.secers = FALSE;
        }
    }
    const uint8_t *bytes = record;
    for (uint32_t offset = 0U; ok && offset < size_bytes; offset += 4U)
    {
        uint32_t word = UINT32_MAX;
        uint32_t count = size_bytes - offset;
        if (count > 4U)
        {
            count = 4U;
        }
        memcpy(&word, bytes + offset, count);
        FLASH->ctrl_bit.fprgm = TRUE;
        flash_store_word(R40_PARAM_BASE + offset, word);
        ok = flash_ready(true);
        if (!s_flash_blocked)
        {
            FLASH->ctrl_bit.fprgm = FALSE;
        }
    }
    lock_if_idle();
    bool verified = ok && memcmp(flash_memory(R40_PARAM_BASE), record, size_bytes) == 0;
    /* 控制器仍忙时不恢复采样；由既有故障/看门狗路径保持关断。 */
    bool resumed = !s_flash_blocked && McuAdc_ResumeAfterStorage();
    return verified && resumed;
}

bool Flash_Init(void)
{
    if (s_storage_initialized || s_flash_blocked)
    {
        return false;
    }
    s_storage_initialized = true;
    return true;
}

BspResult BspFlash_Read(void *data, size_t length)
{
    if ((data == NULL && length != 0U) || length > R40_PARAM_BYTES)
    {
        return BSP_INVALID_ARGUMENT;
    }
    if (!s_storage_initialized)
    {
        return BSP_NOT_READY;
    }
    if (s_flash_blocked)
    {
        return BSP_IO_ERROR;
    }
    if (length == 0U)
    {
        return BSP_OK;
    }
    if (flash_flag_get(FLASH_OBF_FLAG) != RESET)
    {
        return BSP_IO_ERROR;
    }
    memcpy(data, flash_memory(R40_PARAM_BASE), length);
    return BSP_OK;
}

BspResult BspFlash_Write(const void *data, size_t length)
{
    uintptr_t source = (uintptr_t)data;
    if ((data == NULL && length != 0U) || length > R40_PARAM_RAW_BYTES ||
        source > UINTPTR_MAX - length ||
        (length != 0U && source < R40_PARAM_BASE + R40_PARAM_BYTES &&
         source + length > R40_PARAM_BASE))
    {
        return BSP_INVALID_ARGUMENT;
    }
    if (!s_storage_initialized)
    {
        return BSP_NOT_READY;
    }
    if (__get_IPSR() != 0U)
    {
        return BSP_REJECTED;
    }
    if (s_flash_blocked)
    {
        return BSP_IO_ERROR;
    }
    if (length == 0U)
    {
        return BSP_OK;
    }
    uint32_t key = BspSystem_EnterCritical();
    if (!BspPwm_ArePhasesDisabled())
    {
        BspSystem_ExitCritical(key);
        return BSP_REJECTED;
    }
    bool ok = write_record(data, (uint32_t)length);
    if (!ok)
    {
        s_flash_blocked = true;
        BspPwm_Disable();
    }
    BspSystem_ExitCritical(key);
    return ok ? BSP_OK : BSP_IO_ERROR;
}
