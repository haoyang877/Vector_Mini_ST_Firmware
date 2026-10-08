#include "boot_epoch.h"
#include "boot_epoch_flash.h"
#include "crc32.h"
#include "power_stage_hw.h"

/* 目标改动必须显式重审页布局；不能静默扩大扫描/擦除域。 */
typedef char
    BootEpochLayoutChecked[(TARGET_EPOCH_BYTES == 2U * TARGET_ERASE_BYTES &&
                            TARGET_ERASE_BYTES % BOOT_EPOCH_RECORD_BYTES == 0U &&
                            TARGET_EPOCH_BASE >= TARGET_PARAM_BASE + TARGET_RAW_STORAGE_BYTES &&
                            TARGET_EPOCH_BASE + TARGET_EPOCH_BYTES == TARGET_IDENTITY_BASE)
                               ? 1
                               : -1];

/* 一条32B小端记录：magic/version、u64计数、计数补码、CRC/提交字；最后双字提交。 */
typedef struct
{
    uint64_t first, last;
    uint32_t count;
} EpochPage;

static BootEpochStatus s_status;
static uint64_t s_value;

static bool record_blank(const uint32_t record[8])
{
    for (unsigned i = 0U; i < 8U; ++i)
    {
        if (record[i] != UINT32_MAX)
        {
            return false;
        }
    }
    return true;
}

static bool record_valid(const uint32_t record[8])
{
    return record[0] == BOOT_EPOCH_RECORD_MAGIC && record[1] == BOOT_EPOCH_RECORD_VERSION &&
           (record[2] != 0U || record[3] != 0U) && record[4] == ~record[2] &&
           record[5] == ~record[3] && record[6] == Crc32_Compute(record, 24U, 0U) &&
           record[7] == BOOT_EPOCH_RECORD_COMMIT;
}

/* 扫描整页而非遇空即停：不把未知尾部或撕裂记录当自由空间。 */
static BootEpochStatus scan_page(uint32_t address, EpochPage *page)
{
    uint32_t record[8];
    bool blank_seen = false;
    *page = (EpochPage){0};
    for (uint32_t offset = 0U; offset < TARGET_ERASE_BYTES; offset += sizeof(record))
    {
        if (!BootEpochFlash_Read(address + offset, record, sizeof(record)))
        {
            return BOOT_EPOCH_READ_FAILED;
        }
        if (record_blank(record))
        {
            blank_seen = true;
            continue;
        }
        if (blank_seen || !record_valid(record))
        {
            return BOOT_EPOCH_UNKNOWN_RECORD;
        }
        uint64_t value = ((uint64_t)record[3] << 32U) | record[2];
        if (page->count != 0U && (page->last == UINT64_MAX || value != page->last + 1U))
        {
            return BOOT_EPOCH_UNKNOWN_RECORD;
        }
        if (page->count == 0U)
        {
            page->first = value;
        }
        page->last = value;
        ++page->count;
    }
    return BOOT_EPOCH_READY;
}

BootEpochStatus BootEpoch_Init(void)
{
    if (s_status != BOOT_EPOCH_UNINITIALIZED)
    {
        return s_status;
    }
    s_status = BOOT_EPOCH_OUTPUT_ENABLED;
    if (!power_stage_hw_phases_disabled())
    {
        return s_status;
    }
    EpochPage pages[2];
    for (unsigned i = 0U; i < 2U; ++i)
    {
        s_status = scan_page(TARGET_EPOCH_BASE + i * TARGET_ERASE_BYTES, &pages[i]);
        if (s_status != BOOT_EPOCH_READY)
        {
            return s_status;
        }
    }
    unsigned active = pages[1].last > pages[0].last ? 1U : 0U;
    unsigned other = active ^ 1U;
    if (pages[other].count != 0U &&
        (pages[other].count != TARGET_ERASE_BYTES / BOOT_EPOCH_RECORD_BYTES ||
         pages[other].last == UINT64_MAX || pages[active].first != pages[other].last + 1U))
    {
        s_status = BOOT_EPOCH_UNKNOWN_RECORD;
        return s_status;
    }
    if (pages[active].last == UINT64_MAX)
    {
        s_status = BOOT_EPOCH_EXHAUSTED;
        return s_status;
    }
    uint64_t next = pages[active].last + 1U;
    uint32_t address = TARGET_EPOCH_BASE + active * TARGET_ERASE_BYTES;
    if (pages[active].count == TARGET_ERASE_BYTES / BOOT_EPOCH_RECORD_BYTES)
    {
        /* 最新页保留到新记录完整提交；只擦已验证的旧页，空白页无需擦除。 */
        address = TARGET_EPOCH_BASE + other * TARGET_ERASE_BYTES;
        if (pages[other].count != 0U && !BootEpochFlash_Erase(address))
        {
            s_status = BOOT_EPOCH_WRITE_FAILED;
            return s_status;
        }
    }
    else
    {
        address += pages[active].count * BOOT_EPOCH_RECORD_BYTES;
    }
    uint32_t record[8] = {BOOT_EPOCH_RECORD_MAGIC,
                          BOOT_EPOCH_RECORD_VERSION,
                          (uint32_t)next,
                          (uint32_t)(next >> 32U),
                          ~(uint32_t)next,
                          ~(uint32_t)(next >> 32U),
                          0U,
                          BOOT_EPOCH_RECORD_COMMIT};
    record[6] = Crc32_Compute(record, 24U, 0U);
    s_status = BOOT_EPOCH_WRITE_FAILED;
    if (!BootEpochFlash_Program(address, record))
    {
        return s_status;
    }
    uint32_t readback[8];
    if (!BootEpochFlash_Read(address, readback, sizeof(readback)))
    {
        s_status = BOOT_EPOCH_READ_FAILED;
        return s_status;
    }
    if (!record_valid(readback) || readback[2] != record[2] || readback[3] != record[3])
    {
        return s_status;
    }
    /* 只有完整持久提交和读回后发布；本次启动不再重分配，失败不允许重试。 */
    s_value = next;
    s_status = BOOT_EPOCH_READY;
    return s_status;
}

uint64_t BootEpoch_Value(void)
{
    return s_value;
}

BootEpochStatus BootEpoch_Status(void)
{
    return s_status;
}
