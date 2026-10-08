#include <assert.h>
#include <stdio.h>
#include <string.h>

/* 编译真实provider于同一翻译单元，仅重置其RAM模拟程序重启；Flash为有界内存模型。 */
#include "boot_epoch_stm32g4.c"

static uint8_t memory[TARGET_EPOCH_BYTES];
static bool output_disabled = true, read_failed;
static int program_cut = -1, erase_cut = -1;
static unsigned programs, erases, reads, read_fail_at, read_corrupt_at;

bool power_stage_hw_phases_disabled(void)
{
    return output_disabled;
}

static bool inside(uint32_t address, uint32_t bytes)
{
    return address >= TARGET_EPOCH_BASE && bytes <= sizeof(memory) &&
           address - TARGET_EPOCH_BASE <= sizeof(memory) - bytes;
}

bool BootEpochFlash_Read(uint32_t address, void *record, uint32_t bytes)
{
    ++reads;
    assert(inside(address, bytes));
    if (read_failed || (read_fail_at != 0U && reads == read_fail_at))
    {
        return false;
    }
    memcpy(record, memory + address - TARGET_EPOCH_BASE, bytes);
    if (read_corrupt_at != 0U && reads == read_corrupt_at)
    {
        ((uint8_t *)record)[8] ^= 1U;
    }
    return true;
}

bool BootEpochFlash_Program(uint32_t address, const void *record)
{
    assert(output_disabled && inside(address, BOOT_EPOCH_RECORD_BYTES) && address % 8U == 0U);
    ++programs;
    uint8_t *out = memory + address - TARGET_EPOCH_BASE;
    const uint8_t *in = record;
    unsigned bytes = program_cut < 0 ? BOOT_EPOCH_RECORD_BYTES : (unsigned)program_cut;
    for (unsigned i = 0U; i < bytes; ++i)
    {
        assert(out[i] == 0xFFU);
        out[i] &= in[i];
    }
    return program_cut < 0;
}

bool BootEpochFlash_Erase(uint32_t address)
{
    assert(output_disabled && inside(address, TARGET_ERASE_BYTES) &&
           address % TARGET_ERASE_BYTES == 0U);
    ++erases;
    unsigned bytes = erase_cut < 0 ? TARGET_ERASE_BYTES : (unsigned)erase_cut;
    memset(memory + address - TARGET_EPOCH_BASE, 0xFF, bytes);
    return erase_cut < 0;
}

static void restart(void)
{
    s_status = BOOT_EPOCH_UNINITIALIZED;
    s_value = 0U;
    reads = read_fail_at = read_corrupt_at = 0U;
    read_failed = false;
    program_cut = erase_cut = -1;
    output_disabled = true;
}

static void blank(void)
{
    memset(memory, 0xFF, sizeof(memory));
    programs = erases = 0U;
    restart();
}

static void seed(uint32_t slot, uint64_t value)
{
    uint32_t record[8] = {BOOT_EPOCH_RECORD_MAGIC,
                          BOOT_EPOCH_RECORD_VERSION,
                          (uint32_t)value,
                          (uint32_t)(value >> 32U),
                          ~(uint32_t)value,
                          ~(uint32_t)(value >> 32U),
                          0U,
                          BOOT_EPOCH_RECORD_COMMIT};
    record[6] = Crc32_Compute(record, 24U, 0U);
    memcpy(memory + slot * sizeof(record), record, sizeof(record));
}

static void expect_unavailable(BootEpochStatus expected)
{
    unsigned before = programs + erases;
    assert(BootEpoch_Init() == expected);
    assert(BootEpoch_Value() == 0U && BootEpoch_Status() == expected);
    assert(BootEpoch_Init() == expected && programs + erases == before);
}

static void test_monotonic_rotation(void)
{
    blank();
    for (uint64_t boot = 1U; boot <= 1024U; ++boot)
    {
        restart();
        assert(BootEpoch_Value() == 0U);
        assert(BootEpoch_Init() == BOOT_EPOCH_READY && BootEpoch_Value() == boot);
        unsigned before = programs;
        assert(BootEpoch_Init() == BOOT_EPOCH_READY && BootEpoch_Value() == boot);
        assert(programs == before);
    }
    assert(erases == 14U && programs == 1024U);
}

static void test_program_interruptions(void)
{
    for (int cut = 0; cut <= 32; ++cut)
    {
        blank();
        assert(BootEpoch_Init() == BOOT_EPOCH_READY && BootEpoch_Value() == 1U);
        restart();
        program_cut = cut;
        assert(BootEpoch_Init() == BOOT_EPOCH_WRITE_FAILED && BootEpoch_Value() == 0U);
        unsigned before = programs;
        assert(BootEpoch_Init() == BOOT_EPOCH_WRITE_FAILED && programs == before);
        restart();
        if (cut == 0 || cut == 32)
        {
            assert(BootEpoch_Init() == BOOT_EPOCH_READY);
            assert(BootEpoch_Value() == (cut == 0 ? 2U : 3U));
        }
        else
        {
            expect_unavailable(BOOT_EPOCH_UNKNOWN_RECORD);
        }
    }
}

static void fill_both_pages(void)
{
    for (unsigned i = 0U; i < 128U; ++i)
    {
        seed(i, i + 1U);
    }
}

static void test_erase_interruptions(void)
{
    const int cuts[] = {0, 1, 8, 31, 32, 2047, 2048};
    for (unsigned i = 0U; i < sizeof(cuts) / sizeof(cuts[0]); ++i)
    {
        blank();
        fill_both_pages();
        erase_cut = cuts[i];
        assert(BootEpoch_Init() == BOOT_EPOCH_WRITE_FAILED && BootEpoch_Value() == 0U);
        restart();
        if (cuts[i] == 0 || cuts[i] == 2048)
        {
            assert(BootEpoch_Init() == BOOT_EPOCH_READY && BootEpoch_Value() == 129U);
        }
        else
        {
            expect_unavailable(BOOT_EPOCH_UNKNOWN_RECORD);
        }
    }
}

static void test_invalid_records(void)
{
    blank();
    memory[sizeof(memory) - 1U] = 0xFEU;
    expect_unavailable(BOOT_EPOCH_UNKNOWN_RECORD);
    blank();
    seed(0U, 7U);
    memory[4] = 2U;
    expect_unavailable(BOOT_EPOCH_UNKNOWN_RECORD);
    blank();
    seed(0U, 7U);
    memory[24] ^= 1U;
    expect_unavailable(BOOT_EPOCH_UNKNOWN_RECORD);
    blank();
    seed(1U, 1U);
    expect_unavailable(BOOT_EPOCH_UNKNOWN_RECORD);
    blank();
    seed(0U, 1U);
    seed(1U, 3U);
    expect_unavailable(BOOT_EPOCH_UNKNOWN_RECORD);
    blank();
    seed(0U, 1U);
    seed(64U, 2U);
    expect_unavailable(BOOT_EPOCH_UNKNOWN_RECORD);
    blank();
    fill_both_pages();
    seed(64U, 64U);
    expect_unavailable(BOOT_EPOCH_UNKNOWN_RECORD);
    blank();
    seed(0U, 0U);
    expect_unavailable(BOOT_EPOCH_UNKNOWN_RECORD);
    blank();
    seed(0U, 1U);
    seed(1U, 1U);
    expect_unavailable(BOOT_EPOCH_UNKNOWN_RECORD);
}

static void test_failure_and_exhaustion(void)
{
    blank();
    output_disabled = false;
    expect_unavailable(BOOT_EPOCH_OUTPUT_ENABLED);
    blank();
    read_failed = true;
    expect_unavailable(BOOT_EPOCH_READ_FAILED);
    blank();
    read_fail_at = 129U; /* 两页128次扫描完成，持久写入后读回失败。 */
    assert(BootEpoch_Init() == BOOT_EPOCH_READ_FAILED && BootEpoch_Value() == 0U);
    restart();
    assert(BootEpoch_Init() == BOOT_EPOCH_READY && BootEpoch_Value() == 2U);
    blank();
    read_corrupt_at = 129U;
    assert(BootEpoch_Init() == BOOT_EPOCH_WRITE_FAILED && BootEpoch_Value() == 0U);
    restart();
    assert(BootEpoch_Init() == BOOT_EPOCH_READY && BootEpoch_Value() == 2U);
    blank();
    seed(0U, UINT64_MAX - 1U);
    assert(BootEpoch_Init() == BOOT_EPOCH_READY && BootEpoch_Value() == UINT64_MAX);
    restart();
    expect_unavailable(BOOT_EPOCH_EXHAUSTED);
    blank();
    seed(0U, UINT64_C(0xFFFFFFFF));
    assert(BootEpoch_Init() == BOOT_EPOCH_READY && BootEpoch_Value() == UINT64_C(0x100000000));
}

static void test_golden_crc(void)
{
    const uint64_t values[] = {1U, 2U, UINT64_C(0x100000001), UINT64_MAX};
    const uint32_t crc[] = {0xD96CA977U, 0x7E2B1A66U, 0xFA75827DU, 0x68A6B537U};
    for (unsigned i = 0U; i < 4U; ++i)
    {
        blank();
        seed(0U, values[i]);
        uint32_t record[8];
        memcpy(record, memory, sizeof(record));
        assert(record[6] == crc[i] && record_valid(record));
    }
}

int main(void)
{
    test_monotonic_rotation();
    test_program_interruptions();
    test_erase_interruptions();
    test_invalid_records();
    test_failure_and_exhaustion();
    test_golden_crc();
    puts("PASS boot epoch: 1024 resets, rotation, 33 program cuts, erase cuts, unknown/CRC/gap, "
         "readback failure, u64 boundary/exhaustion, golden records; no hardware");
    return 0;
}
