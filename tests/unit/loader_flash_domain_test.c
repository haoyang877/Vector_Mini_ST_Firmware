/* 真实 G431 平台 Flash 入口的保护域测试；寄存器操作替换为内存，不连接设备。 */
#include "platform.h"
#include "loader_target.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static const LoaderFlashRegion image_domain = {TARGET_APP_BASE, TARGET_APP_BYTES};
static uint8_t flash_bytes[TARGET_FLASH_BYTES];
static bool outputs_off = true;
static bool io_ok = true;
static unsigned erase_calls, program_calls;

bool LoaderPlatform_OutputsOff(void)
{
    return outputs_off;
}
static bool flash_erase_page(uint32_t page)
{
    ++erase_calls;
    memset(flash_bytes + page * TARGET_ERASE_BYTES, 255, TARGET_ERASE_BYTES);
    return io_ok;
}
bool LoaderFlash_EraseCompleted(uint32_t address)
{
    (void)address;
    return io_ok;
}
static bool flash_program_doubleword(uint32_t address, uint64_t value)
{
    ++program_calls;
    memcpy(flash_bytes + address - TARGET_FLASH_BASE, &value, sizeof(value));
    return io_ok;
}
bool LoaderFlash_Read(uint32_t address, void *destination, uint32_t bytes)
{
    assert(address >= TARGET_FLASH_BASE && bytes <= TARGET_FLASH_BYTES &&
           address - TARGET_FLASH_BASE <= TARGET_FLASH_BYTES - bytes);
    memcpy(destination, flash_bytes + address - TARGET_FLASH_BASE, bytes);
    return io_ok;
}
static void check_protected_regions(void)
{
    for (uint32_t i = 0; i < TARGET_FLASH_BYTES; ++i)
    {
        if (i < TARGET_APP_BASE - TARGET_FLASH_BASE ||
            i >= TARGET_APP_BASE - TARGET_FLASH_BASE + TARGET_APP_BYTES)
        {
            assert(flash_bytes[i] == 0x5AU);
        }
    }
}
int main(void)
{
    LoaderFlashIo io;
    uint64_t value = 0x123456789ABCDEF0ULL;
    uint8_t readback[8];
    memset(flash_bytes, 0x5A, sizeof(flash_bytes));
    LoaderPlatform_ImageIo(NULL);
    LoaderPlatform_ImageIo(&io);
    assert(io.base == TARGET_APP_BASE && io.bytes == TARGET_APP_BYTES);
    const uint32_t forbidden[] = {TARGET_FLASH_BASE,
                                  TARGET_APP_BASE - 8U,
                                  TARGET_APP_BASE + TARGET_APP_BYTES,
                                  TARGET_PARAM_BASE,
                                  TARGET_IDENTITY_BASE,
                                  UINT32_MAX - 7U};
    for (unsigned i = 0; i < sizeof(forbidden) / sizeof(forbidden[0]); ++i)
    {
        assert(!io.erase(io.context, forbidden[i], TARGET_ERASE_BYTES));
        assert(!io.program(io.context, forbidden[i], &value, sizeof(value)));
        assert(!io.read(io.context, forbidden[i], readback, sizeof(readback)));
    }
    assert(erase_calls == 0U && program_calls == 0U);
    assert(!io.program(io.context, TARGET_APP_BASE + 1U, &value, sizeof(value)));
    assert(!io.program(io.context, TARGET_APP_BASE, NULL, sizeof(value)));
    assert(!io.program(io.context, TARGET_APP_BASE, &value, 4U));
    assert(!io.erase(io.context, TARGET_APP_BASE + 8U, TARGET_ERASE_BYTES));
    assert(!io.erase(io.context, TARGET_APP_BASE, 0U));
    outputs_off = false;
    assert(!io.erase(io.context, TARGET_APP_BASE, TARGET_ERASE_BYTES));
    assert(!io.program(io.context, TARGET_APP_BASE, &value, sizeof(value)));
    assert(erase_calls == 0U && program_calls == 0U);
    outputs_off = true;
    /* 擦遍 APP 区后写首尾，保护区必须逐字节不变。 */
    for (uint32_t offset = 0; offset < TARGET_APP_BYTES; offset += TARGET_ERASE_BYTES)
    {
        assert(io.erase(io.context, TARGET_APP_BASE + offset, TARGET_ERASE_BYTES));
    }
    assert(io.program(io.context, TARGET_APP_BASE, &value, sizeof(value)));
    assert(io.program(io.context, TARGET_APP_BASE + TARGET_APP_BYTES - 8U, &value, sizeof(value)));
    assert(io.read(io.context, TARGET_APP_BASE, readback, sizeof(readback)));
    assert(memcmp(readback, &value, sizeof(value)) == 0);
    io_ok = false;
    assert(!io.erase(io.context, TARGET_APP_BASE, TARGET_ERASE_BYTES));
    assert(!io.program(io.context, TARGET_APP_BASE, &value, sizeof(value)));
    assert(!io.read(io.context, TARGET_APP_BASE, readback, sizeof(readback)));
    check_protected_regions();
    puts("PASS G431 APP-only erase/program, Loader/parameters/SN guards and I/O failure returns");
    return 0;
}
