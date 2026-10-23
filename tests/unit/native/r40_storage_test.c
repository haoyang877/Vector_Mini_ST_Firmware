/* 仅Flash地址读写原语和硬件忙/完成副作用由模拟器提供；擦写算法完整执行。 */
#include "at32f45x.h"
#include "r40_storage_layout.h"
#include "parameter_record.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const uint8_t *flash_memory(uint32_t address);
static void flash_store_word(uint32_t address, uint32_t value);
#include "r40_storage_production.inc"

flash_type sim_flash;
static uint8_t storage[R40_PARAM_BYTES + 2U * R40_FLASH_SECTOR_BYTES];
static bool power_off = true, blocked_unlock, missing_completion;
static unsigned irq = 1U, ipsr, erases, programs, reads, polls, locks;
static unsigned error_after, busy_after, corrupt_after, operation, remaining;
static uint32_t pending_address, pending_word, error_flag;

static const uint8_t *flash_memory(uint32_t address)
{
    assert(address >= R40_PARAM_BASE && address < R40_PARAM_BASE + R40_PARAM_BYTES);
    ++reads;
    return storage + R40_FLASH_SECTOR_BYTES + address - R40_PARAM_BASE;
}

static void flash_store_word(uint32_t address, uint32_t value)
{
    assert(!operation && !sim_flash.ctrl_bit.oplk && sim_flash.ctrl_bit.fprgm);
    assert(address % 4U == 0U && address >= R40_PARAM_BASE);
    assert(address + 4U <= R40_PARAM_BASE + 2U * R40_FLASH_SECTOR_BYTES);
    pending_address = address;
    pending_word = value;
    ++programs;
    operation = 2U;
    remaining = 2U;
}
bool power_stage_hw_phases_disabled(void)
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
            assert(!operation && !sim_flash.ctrl_bit.oplk);
            sim_flash.ctrl_bit.erstr = 0U;
            pending_address = sim_flash.addr;
            assert(pending_address % R40_FLASH_SECTOR_BYTES == 0U);
            assert(pending_address >= R40_PARAM_BASE);
            assert(pending_address < R40_PARAM_BASE + 2U * R40_FLASH_SECTOR_BYTES);
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
                memset((void *)flash_memory(pending_address), 0xFF, R40_FLASH_SECTOR_BYTES);
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
}

static void guards_unchanged(void)
{
    for (unsigned i = 0; i < R40_FLASH_SECTOR_BYTES; ++i)
    {
        assert(storage[i] == 0xA5U);
        assert(storage[R40_FLASH_SECTOR_BYTES + R40_PARAM_BYTES + i] == 0xA5U);
    }
    for (unsigned i = 2U * R40_FLASH_SECTOR_BYTES; i < R40_PARAM_BYTES; ++i)
    {
        assert(storage[R40_FLASH_SECTOR_BYTES + i] == 0xA5U);
    }
}

int main(void)
{
    _Static_assert(sizeof(InterfaceParam) == R40_PARAM_RAW_BYTES, "RAW capacity drift");
    uint8_t input[R40_PARAM_RAW_BYTES + 1U], result[R40_PARAM_BYTES + 1U];
    for (unsigned i = 0U; i < sizeof(input); ++i)
    {
        input[i] = (uint8_t)(i * 17U);
    }
    reset();
    assert(!param_store_read(NULL, 1U) && !param_store_read(result, 0U));
    assert(!param_store_read(result, sizeof(result)));
    assert(!param_store_write(NULL, 1U) && !param_store_write(input, 0U));
    assert(!param_store_write(input, sizeof(input)));
    assert(!param_store_write((void *)(uintptr_t)R40_PARAM_BASE, 1U));
    assert(!param_store_write((void *)(UINTPTR_MAX - 1U), 4U));
    assert(!erases && !programs && !reads);
    power_off = false;
    assert(!param_store_write(input, 12U));
    power_off = true;
    irq = 0U;
    assert(!param_store_write(input, 12U));
    irq = 1U;
    ipsr = 17U;
    assert(!param_store_write(input, 12U));
    ipsr = 0U;
    assert(!erases && !programs);

    const unsigned sizes[] = {1U, 2U, 3U, 4U, 2047U, 2048U, 2049U, R40_PARAM_RAW_BYTES};
    for (unsigned i = 0U; i < sizeof(sizes) / sizeof(sizes[0]); ++i)
    {
        reset();
        unsigned size = sizes[i];
        assert(param_store_write(input, size));
        assert(erases == (size + 2047U) / 2048U && programs == (size + 3U) / 4U);
        assert(sim_flash.ctrl_bit.oplk && !sim_flash.ctrl_bit.fprgm && !sim_flash.ctrl_bit.secers);
        assert(param_store_read(result, size) && memcmp(input, result, size) == 0);
        unsigned padded = (size + 3U) & ~3U;
        for (unsigned j = size; j < padded; ++j)
        {
            assert(flash_memory(R40_PARAM_BASE)[j] == 0xFFU);
        }
        unsigned before_erases = erases, before_programs = programs;
        assert(param_store_write(input, size) && erases == before_erases &&
               programs == before_programs);
        guards_unchanged();
    }
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
    assert(param_store_read(result, 12U) && memcmp(result, input, 12U) != 0);
    guards_unchanged();
    puts("PASS AT32 store: RAW ABI, page boundaries, readback, write guards, padding, errors, busy "
         "lockout and isolation");
    return 0;
}
