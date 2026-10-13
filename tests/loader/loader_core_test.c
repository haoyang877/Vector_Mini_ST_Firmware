/* 假Flash故障注入；仅主机运行，不接触硬件。 */
#include "boot_handoff.h"
#include "golden_vectors.h"
#include "loader_core.h"
#include "loader_port.h"
#include "loader_proto.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint8_t flash[0x20000], image[BOOT_APP_CAPACITY];
static unsigned erase_count, program_count;
static int cut_bytes = -1;
static bool fail_program;

bool LoaderPort_EraseRecord(void)
{
    memset(flash + 0x3000, 0xFF, 2048);
    ++erase_count;
    return true;
}
bool LoaderPort_EraseApp(void)
{
    memset(flash + 0x4000, 0xFF, BOOT_APP_CAPACITY);
    ++erase_count;
    return true;
}
const uint8_t *LoaderPort_App(void) { return flash + 0x4000; }
void LoaderPort_ReadRecord(uint8_t out[BOOT_RECORD_BYTES]) { memcpy(out, flash + 0x3000, 48); }
void LoaderPort_Uid(uint8_t out[12])
{
    unsigned i;
    for (i = 0; i < 12; ++i)
    {
        out[i] = (uint8_t)i;
    }
}
uint32_t LoaderPort_ParamCrc(void) { return Boot_Crc32(flash + 0x1C000, 0x4000); }
bool LoaderPort_Program(uint32_t offset, const uint8_t *data, uint32_t size)
{
    ++program_count;
    if (fail_program)
    {
        return false;
    }
    assert(size >= 8 && size <= 40 && (offset & 7) == 0 && (size & 7) == 0);
    assert(offset <= BOOT_APP_CAPACITY - size && size <= 2048 - offset % 2048);
    memcpy(flash + 0x4000 + offset, data, size);
    return true;
}
bool LoaderPort_Commit(const uint8_t raw[BOOT_RECORD_BYTES])
{
    unsigned n = cut_bytes < 0 ? 48 : (unsigned)cut_bytes;
    memcpy(flash + 0x3000, raw, n);
    return n == 48;
}

static unsigned request(LoaderCore *c, unsigned op, unsigned seq, uint32_t session,
                        const uint8_t *p, unsigned n, uint8_t reply[64])
{
    uint8_t raw[64];
    unsigned len = ldr_build_reply((uint16_t)seq, session, (uint8_t)op, p, (uint16_t)n, raw);
    len = LoaderCore_Request(c, raw, (uint8_t)len, seq * 10, reply);
    assert(len >= 16);
    return ldr_get_u16(reply + 12);
}

static void begin(LoaderCore *c, uint32_t size)
{
    uint8_t p[40] = {0}, reply[64];
    ldr_put_u32(p, size);
    ldr_put_u32(p + 4, Boot_Crc32(image, size));
    ldr_put_u32(p + 8, 0x00040000);
    ldr_put_u16(p + 12, 1);
    ldr_put_u32(p + 16, BOOT_LAYOUT_ID);
    ldr_put_u32(p + 20, BOOT_HARDWARE_ID);
    ldr_put_u32(p + 24, BOOT_PARAM_ABI);
    LoaderPort_Uid(p + 28);
    assert(request(c, 0x20, 1, 123, p, 40, reply) == 0);
    assert(request(c, 0x20, 2, 456, p, 40, reply) == 10);
    assert(request(c, 0x21, 2, 456, NULL, 0, reply) == 9);
    assert(request(c, 0x21, 2, 123, NULL, 0, reply) == 0);
    /* 查询不能覆盖ERASE重试缓存。 */
    assert(request(c, 1, 99, 0, p, 4, reply) == 0);
    assert(request(c, 0x21, 2, 123, NULL, 0, reply) == 0);
}

static unsigned transfer(LoaderCore *c, uint32_t size)
{
    uint8_t p[44], reply[64];
    uint32_t offset = 0;
    unsigned seq = 3, n, before;
    while (offset < size)
    {
        n = size - offset;
        if (n > 40)
        {
            n = 40;
        }
        if (n > 2048 - offset % 2048)
        {
            n = 2048 - offset % 2048;
        }
        ldr_put_u32(p, offset);
        memcpy(p + 4, image + offset, n);
        assert(request(c, 0x22, seq, 123, p, n + 4, reply) == 0);
        before = program_count;
        assert(request(c, 0x22, seq, 123, p, n + 4, reply) == 0);
        assert(program_count == before && ldr_get_u32(reply + 14) == offset + n);
        offset += n;
        ++seq;
        assert(!LoaderCore_BootValid());
    }
    return seq;
}

static void core_tests(void)
{
    LoaderCore c;
    BootImage b;
    uint8_t raw[48], reply[64], p[44], nonce[4] = {1, 2, 3, 4};
    unsigned i, seq, before;
    ldr_request_t decoded;
    for (i = 0; i < golden_count; ++i)
    {
        assert(ldr_parse_request(golden[i].bytes, golden[i].length, &decoded));
        assert(decoded.plen == golden[i].payload_length);
    }
    assert(Boot_Crc32((const uint8_t *)"123456789", 9) == 0xCBF43926);
    memset(flash, 0x55, sizeof(flash));
    memset(flash + 0x3000, 0xFF, 2048);
    memset(image, 0xA5, sizeof(image));
    image[0] = 0;
    image[1] = 0x70;
    image[2] = 0;
    image[3] = 0x20;
    image[4] = 0x11;
    image[5] = 0x40;
    image[6] = 0;
    image[7] = 8;
    LoaderCore_Init(&c, 4);
    assert(request(&c, 1, 1, 0, nonce, 4, reply) == 0 && memcmp(reply + 14, nonce, 4) == 0);
    memcpy(p, golden[2].bytes + 12, 40);
    p[28] ^= 1;
    assert(request(&c, 0x20, 1, 123, p, 40, reply) == 12 && c.session == 0);
    p[28] ^= 1;
    ldr_put_u32(p + 16, 0);
    assert(request(&c, 0x20, 1, 123, p, 40, reply) == 13 && c.session == 0);
    for (i = 0; i < 48; ++i)
    {
        memset(flash + 0x3000, 0xFF, 2048);
        LoaderCore_Init(&c, 4);
        begin(&c, 64);
        before = erase_count;
        assert(request(&c, 0x21, 2, 123, NULL, 0, reply) == 0 && erase_count == before);
        seq = transfer(&c, 64);
        assert(request(&c, 0x23, seq++, 123, NULL, 0, reply) == 0);
        cut_bytes = (int)i;
        assert(request(&c, 0x24, seq, 123, NULL, 0, reply) == 6);
        assert(!LoaderCore_BootValid());
    }
    cut_bytes = -1;
    LoaderCore_Init(&c, 4);
    begin(&c, BOOT_APP_CAPACITY);
    seq = transfer(&c, BOOT_APP_CAPACITY);
    assert(request(&c, 0x23, seq++, 123, NULL, 0, reply) == 0);
    assert(request(&c, 0x24, seq, 123, NULL, 0, reply) == 0 && LoaderCore_BootValid());
    LoaderPort_ReadRecord(raw);
    assert(Boot_RecordDecode(raw, &b));
    flash[0x4008] ^= 1;
    assert(!LoaderCore_BootValid());
    flash[0x4008] ^= 1;
    for (i = 0; i < 0x3000; ++i)
    {
        assert(flash[i] == 0x55);
    }
    for (i = 0x3800; i < 0x4000; ++i)
    {
        assert(flash[i] == 0x55);
    }
    for (i = 0x1C000; i < 0x20000; ++i)
    {
        assert(flash[i] == 0x55);
    }
    LoaderCore_Init(&c, 4);
    begin(&c, 64);
    ldr_put_u32(p, UINT32_MAX);
    memcpy(p + 4, image, 8);
    assert(request(&c, 0x22, 3, 123, p, 12, reply) != 0 && !LoaderCore_BootValid());
    fail_program = true;
    ldr_put_u32(p, 0);
    assert(request(&c, 0x22, 3, 123, p, 12, reply) == 6 && c.offset == 0);
    LoaderCore_Tick(&c, 31000);
    assert(c.session == 0 && !LoaderCore_BootValid());
    puts("PASS real C: golden parsing, UID/layout/session, retry after query, max image, 48 commit "
         "power cuts, bounds, flash failure and protected regions");
}

int main(int argc, char **argv)
{
    if (argc > 1 && strcmp(argv[1], "server") == 0)
    {
        LoaderCore c;
        char line[160];
        uint8_t raw[64], reply[64];
        unsigned n, i, x, k = 0;
        memset(flash, 0x55, sizeof(flash));
        memset(flash + 0x3000, 0xFF, 2048);
        LoaderCore_Init(&c, 4);
        while (fgets(line, sizeof(line), stdin))
        {
            if (line[0] == 'V')
            {
                printf("%d\n", LoaderCore_BootValid());
                fflush(stdout);
                continue;
            }
            n = (unsigned)strcspn(line, "\r\n") / 2;
            if (n > 64)
            {
                return 2;
            }
            for (i = 0; i < n; ++i)
            {
                if (sscanf(line + 2 * i, "%2x", &x) != 1)
                {
                    return 2;
                }
                raw[i] = (uint8_t)x;
            }
            n = LoaderCore_Request(&c, raw, (uint8_t)n, ++k, reply);
            for (i = 0; i < n; ++i)
            {
                printf("%02x", reply[i]);
            }
            printf("\n");
            fflush(stdout);
        }
        return 0;
    }
    if (argc == 1)
    {
        core_tests();
        return 0;
    }
    assert(BootHandoff_Request(4, 0));
    assert(!BootHandoff_Request(5, 50) && BootHandoff_Node() == 4);
    if (strcmp(argv[1], "stop-failure") == 0)
    {
        assert(BootHandoff_Poll(101, false, false, false) == BOOT_REPLY_FAILED);
        BootHandoff_ReplyQueued(102);
        assert(BootHandoff_Poll(105, true, true, true) != BOOT_RESET);
    }
    else if (strcmp(argv[1], "tx-failure") == 0)
    {
        assert(BootHandoff_Poll(1, true, true, false) == BOOT_REPLY_OK);
        BootHandoff_ReplyQueued(2);
        assert(BootHandoff_Poll(103, true, true, false) != BOOT_RESET);
        assert(BootHandoff_Poll(104, true, true, true) != BOOT_RESET);
    }
    else
    {
        assert(BootHandoff_Poll(1, false, false, false) == BOOT_STOP);
        assert(BootHandoff_Poll(2, true, true, false) == BOOT_REPLY_OK);
        BootHandoff_ReplyQueued(3);
        assert(BootHandoff_Poll(4, true, false, true) != BOOT_RESET);
        assert(BootHandoff_Poll(5, true, true, true) == BOOT_RESET);
    }
    assert(BootHandoff_Pending());
    puts("PASS real C APP handoff: exclusive latch, stop/power/TX qualification, timeout and no "
         "automatic unlock");
    return 0;
}
