#include "crc32.h"

uint32_t Crc32_Compute(const void *data, uint32_t bytes, uint32_t seed)
{
    /* 反射（LSB-first）实现：初值与结果各取反一次，等价 zlib.crc32 的约定。 */
    const uint8_t *p = (const uint8_t *)data;
    uint32_t crc = ~seed;
    while (bytes--)
    {
        crc ^= *p++;
        for (unsigned bit = 0; bit < 8; ++bit)
        {
            crc = (crc >> 1) ^ (0xedb88320U & (0U - (crc & 1U)));
        }
    }
    return ~crc;
}
