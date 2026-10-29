/* 真实APP Flash源码的寄存器模型；仅映射进程内存，不访问设备或探针。 */
#ifndef _WIN32
#define _GNU_SOURCE
#endif
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#ifdef _WIN32
#include <windows.h>
#else
#include <sys/mman.h>
#endif
#include "vector_mini_st_g431.h"
#include "flash_register_bits.h"

typedef struct
{
    volatile uint32_t ACR, KEYR, SR, CR;
} FlashRegisters;

static FlashRegisters registers, busy_snapshot;
static unsigned accesses, erase_calls, program_calls;
static unsigned erase_error_at, program_error_at, erase_busy_at, program_busy_at;
static unsigned release_at, busy_started_at;
static bool unlock_allowed, phases_disabled, program_active;

static FlashRegisters *model_access(void);
#define FLASH model_access()
#define FLASH_BASE TARGET_FLASH_BASE
#define FLASH_PAGE_SIZE TARGET_ERASE_BYTES
uint32_t __get_IPSR(void);
#include "flash_under_test.c"
static bool param_store_write(const void *data, uint32_t length)
{
    return BspFlash_Write(data, length) == BSP_OK;
}
static bool param_store_read(void *data, uint32_t length)
{
    return BspFlash_Read(data, length) == BSP_OK;
}

bool BspPwm_ArePhasesDisabled(void)
{
    return phases_disabled;
}

static uint32_t test_primask;
uint32_t __get_IPSR(void)
{
    return 0U;
}
uint32_t BspSystem_EnterCritical(void)
{
    uint32_t old = test_primask;
    test_primask = 1U;
    return old;
}
void BspSystem_ExitCritical(uint32_t key)
{
    test_primask = key;
}
void BspPwm_Disable(void)
{
    phases_disabled = true;
}
bool McuAdc_PauseForStorage(void)
{
    assert(test_primask && phases_disabled);
    return true;
}
bool McuAdc_ResumeAfterStorage(void)
{
    assert(test_primask && phases_disabled);
    return true;
}

static void start_result(bool error, bool busy)
{
    if (error)
    {
        registers.SR |= FLASH_SR_PROGERR;
    }
    if (busy)
    {
        registers.SR |= FLASH_SR_BSY;
        busy_snapshot = registers;
        busy_started_at = accesses;
    }
}

/* 在下一次寄存器访问兑现W1C/解锁/操作启动，使被测体仍是完整生产函数。 */
static FlashRegisters *model_access(void)
{
    ++accesses;
    if (registers.KEYR == FLASH_KEY2_VALUE)
    {
        if (unlock_allowed)
        {
            registers.CR &= ~FLASH_CR_LOCK;
        }
        registers.KEYR = 0U;
    }
    if (registers.SR == (FLASH_SR_EOP | FLASH_ERR_MASK | FLASH_SR_OPTVERR))
    {
        registers.SR = 0U;
    }
    if ((registers.CR & FLASH_CR_STRT) != 0U)
    {
        uint32_t page = (registers.CR & FLASH_CR_PNB_Msk) >> FLASH_CR_PNB_Pos;
        uint32_t address = TARGET_FLASH_BASE + page * TARGET_ERASE_BYTES;
        assert(address >= TARGET_PARAM_BASE && address < TARGET_PARAM_BASE + TARGET_RAW_BYTES);
        ++erase_calls;
        registers.CR &= ~FLASH_CR_STRT;
        memset((void *)(uintptr_t)address, 0xFF, TARGET_ERASE_BYTES);
        start_result(erase_calls == erase_error_at, erase_calls == erase_busy_at);
    }
    if ((registers.CR & FLASH_CR_PG) != 0U && !program_active)
    {
        program_active = true;
        ++program_calls;
        start_result(program_calls == program_error_at, program_calls == program_busy_at);
    }
    else if ((registers.CR & FLASH_CR_PG) == 0U)
    {
        program_active = false;
    }
    if (release_at != 0U && accesses >= release_at)
    {
        registers.SR &= ~FLASH_SR_BSY;
    }
    return &registers;
}

static void setup(void)
{
    memset((void *)(uintptr_t)TARGET_FLASH_BASE, 0xFF, TARGET_FLASH_BYTES);
    memset((void *)(uintptr_t)TARGET_IDENTITY_BASE, 0xA5, TARGET_IDENTITY_BYTES);
    registers = (FlashRegisters){.ACR = FLASH_ACR_DCEN | 4U, .CR = FLASH_CR_LOCK};
    busy_snapshot = registers;
    accesses = erase_calls = program_calls = 0U;
    erase_error_at = program_error_at = erase_busy_at = program_busy_at = release_at = 0U;
    busy_started_at = 0U;
    unlock_allowed = phases_disabled = true;
    program_active = false;
    s_storage_initialized = false;
    s_flash_write_blocked = false;
    assert(Flash_Init());
#ifdef FLASH_WAIT_MAX_POLLS
    s_flash_write_blocked = false;
#endif
}

static void check_identity(void)
{
    const uint8_t *identity = (const uint8_t *)(uintptr_t)TARGET_IDENTITY_BASE;
    for (unsigned i = 0U; i < TARGET_IDENTITY_BYTES; ++i)
    {
        assert(identity[i] == 0xA5U);
    }
}

static void check_idle_cleanup(uint32_t original_acr)
{
    assert((registers.CR & (FLASH_CR_PG | FLASH_CR_PER)) == 0U);
    assert((registers.CR & FLASH_CR_LOCK) != 0U);
    assert(registers.ACR == original_acr);
    check_identity();
}

static void check_busy_preserved(void)
{
    assert((registers.SR & FLASH_SR_BSY) != 0U);
    assert(registers.CR == busy_snapshot.CR);
    assert(registers.ACR == busy_snapshot.ACR);
    assert(registers.KEYR == busy_snapshot.KEYR);
    assert(accesses - busy_started_at <= 1000010U);
    check_identity();
}

static void check_latched_write_rejection(const uint8_t *data, uint32_t bytes)
{
    /* 即使控制器稍后清BSY，遗留模式/缓存不能被下一事务继续使用。 */
    registers.SR &= ~FLASH_SR_BSY;
    FlashRegisters before = registers;
    unsigned before_accesses = accesses;
    assert(!param_store_write(data, bytes));
    assert(accesses == before_accesses);
    assert(memcmp(&before, &registers, sizeof(before)) == 0);
    check_identity();
}

static void check_failures(const uint8_t *data, uint32_t bytes)
{
    /* 首次错误不得被清除后忽略；旧实现会继续擦写并错误返回成功。 */
    setup();
    uint32_t original_acr = registers.ACR;
    registers.SR = FLASH_SR_PROGERR;
    assert(!param_store_write(data, bytes));
    assert(erase_calls == 0U && program_calls == 0U);
    check_idle_cleanup(original_acr);

    setup();
    unlock_allowed = false;
    assert(!param_store_write(data, bytes));
    assert(erase_calls == 0U && program_calls == 0U);
    check_idle_cleanup(original_acr);

    setup();
    registers.SR = FLASH_SR_PROGERR;
    assert(!flash_write_data(TARGET_PARAM_BASE, data, bytes));
    assert(program_calls == 0U);
    check_idle_cleanup(original_acr);

    setup();
    erase_error_at = 1U;
    assert(!param_store_write(data, bytes));
    assert(erase_calls == 1U && program_calls == 0U);
    check_idle_cleanup(original_acr);

    setup();
    program_error_at = 2U;
    assert(!param_store_write(data, bytes));
    assert(erase_calls == 1U && program_calls == 2U);
    assert(memcmp((const void *)(uintptr_t)TARGET_PARAM_BASE, data, 16U) == 0);
    assert(*(const uint32_t *)(uintptr_t)(TARGET_PARAM_BASE + 16U) == UINT32_MAX);
    check_idle_cleanup(original_acr);

    setup();
    registers.SR = FLASH_SR_BSY;
    busy_snapshot = registers;
    assert(!param_store_write(data, bytes));
    assert(erase_calls == 0U && program_calls == 0U);
    check_busy_preserved();
    check_latched_write_rejection(data, bytes);

    setup();
    registers.SR = FLASH_SR_BSY;
    busy_snapshot = registers;
    assert(!flash_write_data(TARGET_PARAM_BASE, data, bytes));
    assert(program_calls == 0U);
    check_busy_preserved();
    check_latched_write_rejection(data, bytes);

    setup();
    erase_busy_at = 1U;
    assert(!param_store_write(data, bytes));
    assert(erase_calls == 1U && program_calls == 0U);
    check_busy_preserved();
    check_latched_write_rejection(data, bytes);

    setup();
    program_busy_at = 1U;
    assert(!param_store_write(data, bytes));
    assert(erase_calls == 1U && program_calls == 1U);
    check_busy_preserved();
    check_latched_write_rejection(data, bytes);
}

static void check_ready_and_success(const uint8_t *data, uint32_t bytes)
{
    const uint32_t errors[] = {FLASH_SR_OPERR,
                               FLASH_SR_PROGERR,
                               FLASH_SR_WRPERR,
                               FLASH_SR_PGAERR,
                               FLASH_SR_PGSERR,
                               FLASH_SR_SIZERR,
                               FLASH_SR_MISERR,
                               FLASH_SR_FASTERR,
                               FLASH_SR_RDERR};
    for (unsigned i = 0U; i < sizeof(errors) / sizeof(errors[0]); ++i)
    {
        setup();
        registers.SR = errors[i];
        assert(!flash_ready());
    }
    setup();
    registers.SR = FLASH_SR_EOP | FLASH_SR_OPTVERR;
    assert(flash_ready()); /* 沿用OPTVERR清理，不改变原错误掩码。 */
    setup();
    registers.SR = FLASH_SR_BSY;
    release_at = 5U;
    assert(flash_ready() && accesses < 10U);

    setup();
    uint32_t original_acr = registers.ACR;
    assert(param_store_write(data, bytes));
    assert(erase_calls == 1U && program_calls == 3U);
    assert(memcmp((const void *)(uintptr_t)TARGET_PARAM_BASE, data, bytes) == 0);
    assert(param_store_write(data, bytes));
    assert(erase_calls == 1U && program_calls == 3U); /* 相同记录仍免擦写。 */
    check_idle_cleanup(original_acr);

    setup();
    phases_disabled = false;
    assert(!param_store_write(data, bytes));
    phases_disabled = true;
    assert(!param_store_write(NULL, bytes));
    assert(BspFlash_Write(NULL, 0U) == BSP_OK);
    assert(!param_store_write(data, TARGET_RAW_BYTES + 1U));
    assert(erase_calls == 0U && program_calls == 0U);
    assert(!param_store_read(NULL, 1U));
    assert(!param_store_read((void *)data, TARGET_PARAM_BYTES + 1U));
    check_idle_cleanup(original_acr);
}

int main(void)
{
    /* 固定地址仅为进程内测试内存；不以MAP_FIXED覆盖任何已有映射。 */
    void *base = (void *)(uintptr_t)TARGET_FLASH_BASE;
#ifdef _WIN32
    void *memory = VirtualAlloc(base, TARGET_FLASH_BYTES, MEM_RESERVE | MEM_COMMIT, PAGE_READWRITE);
#else
    void *memory =
        mmap(base, TARGET_FLASH_BYTES, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
#endif
    if (memory != base)
    {
        fputs("Cannot reserve isolated Flash model address\n", stderr);
        return 2;
    }
    const uint8_t data[24] = {1U,  2U,  3U,  4U,  5U,  6U,  7U,  8U,  9U,  10U, 11U, 12U,
                              13U, 14U, 15U, 16U, 17U, 18U, 19U, 20U, 21U, 22U, 23U, 24U};
    check_failures(data, sizeof(data));
    check_ready_and_success(data, sizeof(data));
#ifdef _WIN32
    VirtualFree(memory, 0U, MEM_RELEASE);
#else
    munmap(memory, TARGET_FLASH_BYTES);
#endif
    puts("PASS APP Flash error propagation, bounded busy polling, cleanup, write domain and no "
         "rewrite");
    return 0;
}
