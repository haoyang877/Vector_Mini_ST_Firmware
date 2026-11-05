/* 仅Flash地址读写原语和硬件忙/完成副作用由模拟器提供；擦写算法完整执行。 */
#include "at32f45x.h"
#include "board_config.h"
#include "parameter_record.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const uint8_t *flash_memory(uint32_t address);
static void flash_store_word(uint32_t address, uint32_t value);
#include "at32f456_storage_production.inc"
static bool param_store_write(const void *data, uint32_t length)
{
    return BspFlash_Write(data, length) == BSP_OK;
}
static bool param_store_read(void *data, uint32_t length)
{
    return BspFlash_Read(data, length) == BSP_OK;
}

flash_type sim_flash;
static uint8_t storage[AT32_PARAM_BYTES + 2U * AT32_FLASH_SECTOR_BYTES];
static bool power_off = true, blocked_unlock, missing_completion;
static unsigned irq = 1U, ipsr, erases, programs, reads, polls, locks;
static unsigned error_after, busy_after, corrupt_after, operation, remaining;
static uint32_t pending_address, pending_word, error_flag;
static bool sampling_paused;
static bool flash_watchdog_active;
static bool fail_watchdog_prepare, fail_watchdog_restore;
bool McuSystem_SetFlashWatchdog(bool saving)
{
    assert(irq && !ipsr && power_off);
    if (saving)
    {
        assert(!flash_watchdog_active);
        if (fail_watchdog_prepare)
        {
            return false;
        }
        flash_watchdog_active = true;
        return true;
    }
    flash_watchdog_active = false;
    return !fail_watchdog_restore;
}
void Adc_Stop(void)
{
    assert(irq && !ipsr && power_off && !sampling_paused && flash_watchdog_active);
    sampling_paused = true;
}
void Adc_Start(void)
{
    assert(sampling_paused && !operation && irq && power_off);
    sampling_paused = false;
}

static const uint8_t *flash_memory(uint32_t address)
{
    assert(address >= AT32_PARAM_BASE && address < AT32_PARAM_BASE + AT32_PARAM_BYTES);
    ++reads;
    return storage + AT32_FLASH_SECTOR_BYTES + address - AT32_PARAM_BASE;
}

static void flash_store_word(uint32_t address, uint32_t value)
{
    assert(sampling_paused);
    assert(!operation && !sim_flash.ctrl_bit.oplk && sim_flash.ctrl_bit.fprgm);
    assert(address % 4U == 0U && address >= AT32_PARAM_BASE);
    assert(address + 4U <= AT32_PARAM_BASE + 2U * AT32_FLASH_SECTOR_BYTES);
    pending_address = address;
    pending_word = value;
    ++programs;
    operation = 2U;
    remaining = 2U;
}
bool BspPwm_ArePhasesDisabled(void)
{
    return power_off;
}
unsigned __get_PRIMASK(void)
{
    return irq;
}
uint32_t __get_IPSR(void)
{
    return ipsr;
}
uint32_t BspSystem_EnterCritical(void)
{
    uint32_t old = irq;
    irq = 1U;
    return old;
}
void BspSystem_ExitCritical(uint32_t key)
{
    irq = key;
}
void BspPwm_Disable(void)
{
    power_off = true;
}
void flash_unlock(void)
{
    assert(!operation && irq && !ipsr && power_off);
    if (!blocked_unlock)
    {
        sim_flash.ctrl_bit.oplk = 0U;
    }
}
void flash_lock(void)
{
    assert(!operation);
    sim_flash.ctrl_bit.oplk = 1U;
    ++locks;
}
void flash_flag_clear(uint32_t flags)
{
    assert(!operation);
    error_flag &= ~flags;
}
flag_status flash_flag_get(uint32_t flags)
{
    if (flags & FLASH_OBF_FLAG)
    {
        ++polls;
        if (sim_flash.ctrl_bit.erstr && sim_flash.ctrl_bit.secers)
        {
            assert(sampling_paused);
            assert(!operation && !sim_flash.ctrl_bit.oplk);
            sim_flash.ctrl_bit.erstr = 0U;
            pending_address = sim_flash.addr;
            assert(pending_address % AT32_FLASH_SECTOR_BYTES == 0U);
            assert(pending_address >= AT32_PARAM_BASE);
            assert(pending_address < AT32_PARAM_BASE + 2U * AT32_FLASH_SECTOR_BYTES);
            ++erases;
            operation = 1U;
            remaining = 2U;
        }
        if (operation)
        {
            if (busy_after && erases + programs == busy_after)
            {
                return SET;
            }
            if (remaining-- != 0U)
            {
                return SET;
            }
            if (error_after && erases + programs == error_after)
            {
                error_flag = FLASH_PRGMERR_FLAG;
            }
            else if (operation == 1U)
            {
                memset((void *)flash_memory(pending_address), 0xFF, AT32_FLASH_SECTOR_BYTES);
            }
            else
            {
                uint8_t *dest = (uint8_t *)flash_memory(pending_address);
                for (unsigned i = 0U; i < 4U; ++i)
                {
                    dest[i] &= (uint8_t)(pending_word >> (8U * i));
                }
                if (corrupt_after && programs == corrupt_after)
                {
                    dest[0] ^= 1U;
                }
            }
            if (error_flag == 0U && !missing_completion)
            {
                error_flag = FLASH_ODF_FLAG;
            }
            operation = 0U;
        }
    }
    return (flags & error_flag) != 0U ? SET : RESET;
}

static void reset(void)
{
    memset(storage, 0xA5, sizeof(storage));
    memset(&sim_flash, 0, sizeof(sim_flash));
    sim_flash.ctrl_bit.oplk = 1U;
    power_off = true;
    irq = 1U;
    ipsr = erases = programs = reads = polls = locks = 0U;
    error_after = busy_after = corrupt_after = operation = error_flag = 0U;
    blocked_unlock = s_flash_blocked = missing_completion = false;
    sampling_paused = false;
    flash_watchdog_active = fail_watchdog_prepare = fail_watchdog_restore = false;
}

static void guards_unchanged(void)
{
    for (unsigned i = 0; i < AT32_FLASH_SECTOR_BYTES; ++i)
    {
        assert(storage[i] == 0xA5U);
        assert(storage[AT32_FLASH_SECTOR_BYTES + AT32_PARAM_BYTES + i] == 0xA5U);
    }
    for (unsigned i = 2U * AT32_FLASH_SECTOR_BYTES; i < AT32_PARAM_BYTES; ++i)
    {
        assert(storage[AT32_FLASH_SECTOR_BYTES + i] == 0xA5U);
    }
}

int main(void)
{
    _Static_assert(sizeof(InterfaceParam) == AT32_PARAM_RAW_BYTES, "RAW capacity drift");
    uint8_t input[AT32_PARAM_RAW_BYTES + 1U], result[AT32_PARAM_BYTES + 1U];
    for (unsigned i = 0U; i < sizeof(input); ++i)
    {
        input[i] = (uint8_t)(i * 17U);
    }
    reset();
    s_storage_initialized = false;
    assert(Flash_Init());
    assert(!param_store_read(NULL, 1U) && BspFlash_Read(NULL, 0U) == BSP_OK);
    assert(!param_store_read(result, sizeof(result)));
    assert(!param_store_write(NULL, 1U) && BspFlash_Write(NULL, 0U) == BSP_OK);
    assert(!param_store_write(input, sizeof(input)));
    assert(!param_store_write((void *)(uintptr_t)AT32_PARAM_BASE, 1U));
    assert(!param_store_write((void *)(UINTPTR_MAX - 1U), 4U));
    assert(!erases && !programs && !reads);
    power_off = false;
    assert(!param_store_write(input, 12U));
    power_off = true;
    irq = 0U;
    assert(param_store_write(input, 12U) && irq == 0U);
    reset();
    irq = 1U;
    ipsr = 17U;
    assert(!param_store_write(input, 12U));
    ipsr = 0U;
    assert(!erases && !programs);

    const unsigned sizes[] = {1U, 2U, 3U, 4U, 2047U, 2048U, 2049U, AT32_PARAM_RAW_BYTES};
    for (unsigned i = 0U; i < sizeof(sizes) / sizeof(sizes[0]); ++i)
    {
        reset();
        unsigned size = sizes[i];
        assert(param_store_write(input, size));
        assert(!flash_watchdog_active);
        assert(erases == (size + 2047U) / 2048U && programs == (size + 3U) / 4U);
        assert(sim_flash.ctrl_bit.oplk && !sim_flash.ctrl_bit.fprgm && !sim_flash.ctrl_bit.secers);
        assert(param_store_read(result, size) && memcmp(input, result, size) == 0);
        unsigned padded = (size + 3U) & ~3U;
        for (unsigned j = size; j < padded; ++j)
        {
            assert(flash_memory(AT32_PARAM_BASE)[j] == 0xFFU);
        }
        unsigned before_erases = erases, before_programs = programs;
        assert(param_store_write(input, size) && erases == before_erases &&
               programs == before_programs);
        guards_unchanged();
    }
    reset();
    fail_watchdog_prepare = true;
    assert(!param_store_write(input, 12U) && !erases && !programs && s_flash_blocked);
    reset();
    fail_watchdog_restore = true;
    assert(!param_store_write(input, 12U) && !flash_watchdog_active && s_flash_blocked);
    reset();
    blocked_unlock = true;
    assert(!param_store_write(input, sizeof(input) - 1U) && !erases && !programs);
    reset();
    error_flag = FLASH_EPPERR_FLAG;
    assert(!param_store_write(input, 12U) && !erases && !programs && sim_flash.ctrl_bit.oplk);
    for (unsigned failure = 1U; failure <= 10U; ++failure)
    {
        reset();
        error_after = failure;
        assert(!param_store_write(input, sizeof(input) - 1U));
        assert(erases + programs == failure && sim_flash.ctrl_bit.oplk);
        guards_unchanged();
    }
    for (unsigned failure = 1U; failure <= 4U; ++failure)
    {
        reset();
        busy_after = failure;
        assert(!param_store_write(input, sizeof(input) - 1U) && s_flash_blocked);
        assert(sampling_paused);
        assert(erases + programs == failure && operation && !sim_flash.ctrl_bit.oplk);
        unsigned before = polls;
        assert(!param_store_write(input, 12U) && !param_store_read(result, 12U));
        assert(polls == before);
        assert((operation == 1U && sim_flash.ctrl_bit.secers) ||
               (operation == 2U && sim_flash.ctrl_bit.fprgm));
        guards_unchanged();
    }
    reset();
    missing_completion = true;
    assert(!param_store_write(input, 12U) && erases == 1U && programs == 0U);
    assert(sim_flash.ctrl_bit.oplk);
    reset();
    corrupt_after = 3U;
    assert(!param_store_write(input, 12U) && sim_flash.ctrl_bit.oplk);
    assert(BspFlash_Read(result, 12U) == BSP_IO_ERROR);
    guards_unchanged();
    puts("PASS AT32 store: RAW ABI, page boundaries, readback, write guards, padding, errors, busy "
         "lockout and isolation");
    return 0;
}
