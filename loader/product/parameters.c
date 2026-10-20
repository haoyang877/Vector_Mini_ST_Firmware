/* 产品参数只读解析：保留原有存储格式和节点策略，不访问芯片寄存器。 */
#include "parameters.h"
#include <stddef.h>
#include "platform.h"
#include "loader_target.h"

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
