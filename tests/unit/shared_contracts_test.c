/* 离线契约测试：CRC 与独立 zlib 向量对照，字节序及请求条件覆盖边界。 */
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "crc32.h"
#include "yg_protocol_byte_order.h"
#include "yg_protocol_request.h"
#include "crc_vectors.h"

int main(void)
{
    assert(Crc32_Compute("123456789", 9U, 0U) == 0xCBF43926U);
    assert(Crc32_Compute(NULL, 0U, 0x12345678U) == 0x12345678U);
    for (unsigned i = 0; i < sizeof(vectors) / sizeof(vectors[0]); ++i)
    {
        CrcVector v = vectors[i];
        assert(Crc32_Compute(payload, v.bytes, v.seed) == v.expected);
        const unsigned chunks[] = {1, 2, 7, 16, 63, 128};
        for (unsigned c = 0; c < sizeof(chunks) / sizeof(chunks[0]); ++c)
        {
            uint32_t crc = v.seed;
            for (unsigned offset = 0; offset < v.bytes; offset += chunks[c])
            {
                unsigned n = v.bytes - offset < chunks[c] ? v.bytes - offset : chunks[c];
                crc = Crc32_Compute(payload + offset, n, crc);
                assert(Crc32_Compute(NULL, 0, crc) == crc);
            }
            assert(crc == v.expected);
        }
    }
    const uint8_t golden[] = {0xEF, 0xCD, 0xAB, 0x89, 0x67, 0x45, 0x23, 0x01};
    for (unsigned offset = 0; offset < 8; ++offset)
    {
        uint8_t buffer[24];
        memset(buffer, 0xA5, sizeof(buffer));
        YgProtocol_WriteU64Le(buffer + offset, UINT64_C(0x0123456789ABCDEF));
        assert(memcmp(buffer + offset, golden, 8) == 0);
        assert(YgProtocol_ReadU64Le(buffer + offset) == UINT64_C(0x0123456789ABCDEF));
        assert(YgProtocol_ReadU32Le(buffer + offset) == 0x89ABCDEFU);
        assert(YgProtocol_ReadU16Le(buffer + offset) == 0xCDEFU);
        for (unsigned j = 0; j < offset; ++j)
        {
            assert(buffer[j] == 0xA5);
        }
        for (unsigned j = offset + 8; j < sizeof(buffer); ++j)
        {
            assert(buffer[j] == 0xA5);
        }
        YgProtocol_WriteU32Le(buffer + offset, UINT32_MAX);
        assert(YgProtocol_ReadU32Le(buffer + offset) == UINT32_MAX);
        YgProtocol_WriteU16Le(buffer + offset, 0);
        assert(YgProtocol_ReadU16Le(buffer + offset) == 0);
    }
    assert(!YgProtocol_IsAckUnicastRequest(NULL));
    const uint8_t nodes[] = {0, 1, 2, 254, 255};
    for (unsigned f = 0; f < 256; ++f)
    {
        for (unsigned v = 0; v < 2; ++v)
        {
            for (unsigned r = 0; r < 2; ++r)
            {
                for (unsigned src = 0; src < 5; ++src)
                {
                    for (unsigned dst = 0; dst < 5; ++dst)
                    {
                        yg_protocol_message_t m = {0};
                        m.version = (uint8_t)v;
                        m.reserved = (uint8_t)r;
                        m.flags = (uint8_t)f;
                        m.source_node = nodes[src];
                        m.destination_node = nodes[dst];
                        bool expected =
                            v == 1 && r == 0 && src != 4 && dst != 4 && (f == 0x20 || f == 0x24);
                        assert(YgProtocol_IsAckUnicastRequest(&m) == expected);
                    }
                }
            }
        }
    }
    puts("PASS CRC zlib vectors/streaming, unaligned byte order, 25600 request combinations");
    return 0;
}
