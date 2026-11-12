/* 产品参数只读解析：保留原有存储格式和节点策略，不访问芯片寄存器。 */
#include "parameters.h"
#include <stddef.h>
#include "platform.h"
#include "loader_target.h"
#include "can_bitrate.h"
#include "crc32.h"

uint8_t LoaderParameters_ReadBitrate(LoaderParameterState state)
{
#if defined(TARGET_CONFIGURABLE_CAN) && TARGET_CONFIGURABLE_CAN
#if CAN_BITRATE_RECORD_OFFSET != TARGET_RAW_SCHEMA_OFFSET + 8U
#error Incompatible schema, magic and bitrate offsets
#endif
    uint32_t header[3];
    if (state != YI_PARAMS_RAW11 ||
        !LoaderPlatform_ReadParameters(TARGET_RAW_SCHEMA_OFFSET, header, sizeof(header)) ||
        header[0] != TARGET_SCHEMA || header[1] != TARGET_RAW_MAGIC)
    {
        return CAN_BITRATE_DEFAULT_PROFILE;
    }
    uint8_t profile = CanBitrate_FromRecord(header[2]);
    uint32_t expected_crc;
    if (profile == CAN_BITRATE_DEFAULT_PROFILE ||
        !LoaderPlatform_ReadParameters(
            CAN_BITRATE_RECORD_CRC_OFFSET, &expected_crc, sizeof(expected_crc)))
    {
        return CAN_BITRATE_DEFAULT_PROFILE;
    }

    /* 与 schema14 所有者相同，跳过 CRC 槽本身；分块读取限制启动栈占用。 */
    uint32_t crc = 0U;
    uint8_t block[64];
    for (uint32_t offset = 0U; offset < CAN_BITRATE_BASE_RECORD_BYTES;)
    {
        if (offset == CAN_BITRATE_RECORD_CRC_OFFSET)
        {
            offset += sizeof(expected_crc);
        }
        uint32_t end = offset < CAN_BITRATE_RECORD_CRC_OFFSET ? CAN_BITRATE_RECORD_CRC_OFFSET
                                                              : CAN_BITRATE_BASE_RECORD_BYTES;
        uint32_t count = end - offset;
        if (count > sizeof(block))
        {
            count = sizeof(block);
        }
        LoaderPlatform_Watchdog();
        if (!LoaderPlatform_ReadParameters(offset, block, count))
        {
            return CAN_BITRATE_DEFAULT_PROFILE;
        }
        crc = Crc32_Compute(block, count, crc);
        offset += count;
    }
    return crc == expected_crc ? profile : CAN_BITRATE_DEFAULT_PROFILE;
#else
    (void)state;
    return 0U;
#endif
}

LoaderParameterState LoaderParameters_ReadNode(uint8_t *node)
{
    uint8_t block[64];
    uint32_t offset, i, words[2], node_bits, candidate, shift;
    bool blank = true;
    /* 完整扫描只确认可读/空白；CRC及配置合法性由APP所有者校验。 */
    /* 生产身份页独立；不能把已烧SN当作存在校准，也不能因该页ECC错误拒绝参数域。 */
    for (offset = 0; offset < TARGET_IDENTITY_BASE - TARGET_PARAM_BASE; offset += sizeof(block))
    {
        LoaderPlatform_Watchdog();
        if (!LoaderPlatform_ReadParameters(offset, block, sizeof(block)))
        {
            return YI_PARAMS_UNSAFE;
        }
        for (i = 0; i < sizeof(block); ++i)
        {
            if (block[i] != 255)
            {
                blank = false;
            }
        }
    }
    if (blank)
    {
        return YI_PARAMS_BLANK;
    }
    if (node == NULL || !LoaderPlatform_ReadParameters(TARGET_RAW_SCHEMA_OFFSET, words, 8) ||
        (words[0] != TARGET_SCHEMA && words[0] != 12U && words[0] != 13U) ||
        words[1] != TARGET_RAW_MAGIC || !LoaderPlatform_ReadParameters(0, &node_bits, 4))
    {
        return YI_PARAMS_UNSAFE;
    }
    /* node_id 的现有ABI是 IEEE754 float，逐位接纳精确整数，拒绝NaN/负值/小数。 */
    if (node_bits == 0)
    {
        *node = 0;
        return YI_PARAMS_RAW11;
    }
    if ((node_bits >> 23) < 127U || (node_bits >> 23) > 134U)
    {
        return YI_PARAMS_UNSAFE;
    }
    shift = 150U - (node_bits >> 23);
    candidate = (node_bits & 0x7FFFFFU) | 0x800000U;
    if ((candidate & ((1U << shift) - 1U)) != 0 || (candidate >> shift) >= 255U)
    {
        return YI_PARAMS_UNSAFE;
    }
    *node = (uint8_t)(candidate >> shift);
    return YI_PARAMS_RAW11;
}
