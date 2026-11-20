#include "scaled_integer.h"

#include <string.h>
#include <float.h>

typedef char ScaledIntegerRequiresBinary32[(sizeof(float) == sizeof(uint32_t) && FLT_RADIX == 2 &&
                                            FLT_MANT_DIG == 24 && FLT_MAX_EXP == 128)
                                               ? 1
                                               : -1];

bool ScaledInteger_Round(float value, uint32_t scale, ScaledInteger *rounded)
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
