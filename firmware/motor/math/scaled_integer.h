#ifndef SCALED_INTEGER_H
#define SCALED_INTEGER_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <float.h>

typedef char ScaledIntegerRequiresBinary32[(sizeof(float) == sizeof(uint32_t) && FLT_RADIX == 2 &&
                                            FLT_MANT_DIG == 24 && FLT_MAX_EXP == 128)
                                               ? 1
                                               : -1];

typedef struct
{
    uint32_t magnitude;
    bool negative, exact_half;
} ScaledInteger;

/**
 * @brief 对 binary32 输入与整数比例的精确乘积取最近整数，半值远离零。
 * @param value 有符号输入值。
 * @param scale 正整数单位比例。
 * @param rounded 成功时写入符号、舍入后的幅值及精确半值标记。
 * @return 输入有限且结果可用 uint32_t 幅值表示时为 true；失败保持输出。
 * @note 只做数值转换。线路哨兵、有效位和有符号端点由各自编码器决定。
 */
static inline bool ScaledInteger_Round(float value, uint32_t scale, ScaledInteger *rounded)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    unsigned exponent = (bits >> 23U) & 255U;
    if (exponent == 255U)
    {
        return false;
    }
    /* 24 位尾数与 32 位比例的乘积可精确装入 uint64_t。 */
    uint64_t product = (uint64_t)((bits & 0x7fffffU) | (exponent ? 0x800000U : 0U)) * scale;
    int shift = exponent ? 150 - (int)exponent : 149;
    uint64_t magnitude;
    bool half = false;
    if (shift > 63)
    {
        magnitude = 0U;
    }
    else if (shift > 0)
    {
        uint64_t midpoint = (uint64_t)1U << (shift - 1);
        magnitude = (product >> shift) + ((product & midpoint) != 0U);
        half = (product & ((midpoint << 1) - 1U)) == midpoint;
    }
    else
    {
        if (shift < -31 || product > (UINT32_MAX >> -shift))
        {
            return false;
        }
        magnitude = product << -shift;
    }
    if (magnitude > UINT32_MAX)
    {
        return false;
    }
    *rounded = (ScaledInteger){(uint32_t)magnitude, (bits >> 31U) != 0U, half};
    return true;
}

#endif
