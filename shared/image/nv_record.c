#include "nv_record.h"
#include <string.h>

static uint32_t get32(const uint8_t *p)
{
    return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3];
}
static void put32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)v;
}
static uint32_t crc_update(uint32_t c, const uint8_t *p, uint32_t n)
{
    unsigned i;
    while (n--)
    {
        c ^= *p++;
        for (i = 0; i < 8; i++)
        {
            c = (c >> 1) ^ ((0U - (c & 1U)) & 0xedb88320U);
        }
    }
    return c;
}
static bool all_ff(const uint8_t *p, uint32_t n)
{
    while (n--)
    {
        if (*p++ != 255)
        {
            return false;
        }
    }
    return true;
}
static bool contained(uint32_t base, uint32_t bytes, uint32_t address, uint32_t n)
{
    return address >= base && n <= bytes && address - base <= bytes - n;
}
static LoaderRecordResult
region_geometry(const LoaderFlashIo *io, const LoaderFlashRegion *s, const LoaderRecordFormat *f)
{
    uint32_t u;
    if (!io || !s || !f || !io->read || !f->payload_bytes)
    {
        return NVR_ARGUMENT;
    }
    u = io->program_bytes;
    if (!u || u > 32 || (u & (u - 1)) || !io->erase_bytes || io->erase_bytes % u || s->bytes < 96 ||
        s->bytes % u || s->base % u || f->payload_bytes > s->bytes - 64)
    {
        return NVR_GEOMETRY;
    }
    if (io->bytes > UINT32_MAX - io->base || s->bytes > UINT32_MAX - s->base ||
        !contained(io->base, io->bytes, s->base, s->bytes))
    {
        return NVR_RANGE;
    }
    return NVR_OK;
}
static LoaderRecordResult
geometry(const LoaderFlashIo *io, const LoaderFlashRegion *s, const LoaderRecordFormat *f)
{
    LoaderRecordResult result = region_geometry(io, s, f);
    if (result != NVR_OK)
    {
        return result;
    }
    return s->bytes % io->erase_bytes || s->base % io->erase_bytes ? NVR_GEOMETRY : NVR_OK;
}
static LoaderRecordResult read_at(const LoaderFlashIo *io, uint32_t a, void *p, uint32_t n)
{
    if (!contained(io->base, io->bytes, a, n))
    {
        return NVR_RANGE;
    }
    return io->read(io->context, a, p, n) ? NVR_OK : NVR_IO;
}
static LoaderRecordResult erased_range(const LoaderFlashIo *io, uint32_t a, uint32_t n)
{
    uint8_t b[32];
    uint32_t take;
    LoaderRecordResult r;
    while (n)
    {
        take = n > 32 ? 32 : n;
        r = read_at(io, a, b, take);
        if (r != NVR_OK)
        {
            return r;
        }
        if (!all_ff(b, take))
        {
            return NVR_INVALID;
        }
        a += take;
        n -= take;
    }
    return NVR_OK;
}
static LoaderRecordResult inspect_record(const LoaderFlashIo *io,
                                         const LoaderFlashRegion *s,
                                         const LoaderRecordFormat *f,
                                         LoaderRecordInfo *out)
{
    uint8_t h[32], b[32];
    uint32_t crc, k, t;
    LoaderRecordInfo info;
    LoaderRecordResult r;
    if (!out)
    {
        return NVR_ARGUMENT;
    }
    r = read_at(io, s->base, h, 32);
    if (r != NVR_OK)
    {
        return r;
    }
    if (all_ff(h, 32))
    {
        r = erased_range(io, s->base, s->bytes);
        return r == NVR_OK ? NVR_ERASED : r;
    }
    if (get32(h) != f->magic || get32(h + 4) != f->kind || get32(h + 8) != f->payload_bytes ||
        get32(h + 20) != f->layout || get32(h + 28) != f->version)
    {
        return NVR_INVALID;
    }
    info.sequence = get32(h + 12);
    info.context = get32(h + 16);
    info.crc = get32(h + 24);
    r = read_at(io, s->base + s->bytes - 32, b, 32);
    if (r != NVR_OK)
    {
        return r;
    }
    if (get32(b) != f->commit_magic || get32(b + 4) != ~f->commit_magic ||
        get32(b + 8) != info.sequence || get32(b + 12) != info.crc || !all_ff(b + 16, 16))
    {
        return NVR_INVALID;
    }
    crc = crc_update(UINT32_MAX, h, 24);
    crc = crc_update(crc, h + 28, 4);
    for (k = 0; k < f->payload_bytes; k += t)
    {
        t = f->payload_bytes - k;
        if (t > 32)
        {
            t = 32;
        }
        r = read_at(io, s->base + 32 + k, b, t);
        if (r != NVR_OK)
        {
            return r;
        }
        crc = crc_update(crc, b, t);
    }
    if (~crc != info.crc)
    {
        return NVR_INVALID;
    }
    *out = info;
    return NVR_OK;
}
LoaderRecordResult LoaderRecord_Inspect(const LoaderFlashIo *io,
                                        const LoaderFlashRegion *s,
                                        const LoaderRecordFormat *f,
                                        LoaderRecordInfo *out)
{
    LoaderRecordResult result = geometry(io, s, f);
    return result == NVR_OK ? inspect_record(io, s, f, out) : result;
}

static LoaderRecordResult read_payload(const LoaderFlashIo *io,
                                       const LoaderFlashRegion *s,
                                       const LoaderRecordFormat *f,
                                       void *out,
                                       uint32_t capacity,
                                       LoaderRecordInfo *info)
{
    LoaderRecordInfo local;
    LoaderRecordResult r;
    if (!out || !info || !f || capacity < f->payload_bytes)
    {
        return NVR_ARGUMENT;
    }
    r = inspect_record(io, s, f, &local);
    if (r != NVR_OK)
    {
        return r;
    }
    r = read_at(io, s->base + 32, out, f->payload_bytes);
    if (r != NVR_OK)
    {
        return r;
    }
    /* 调用方必须独占。对复制后的实际字节再次校验，避免读取变化被误报为成功。 */
    {
        uint8_t h[32];
        uint32_t crc;
        r = read_at(io, s->base, h, 32);
        if (r != NVR_OK)
        {
            return r;
        }
        crc = crc_update(UINT32_MAX, h, 24);
        crc = crc_update(crc, h + 28, 4);
        if (~crc_update(crc, (const uint8_t *)out, f->payload_bytes) != local.crc)
        {
            return NVR_INVALID;
        }
    }
    *info = local;
    return NVR_OK;
}
LoaderRecordResult LoaderRecord_Read(const LoaderFlashIo *io,
                                     const LoaderFlashRegion *s,
                                     const LoaderRecordFormat *f,
                                     void *out,
                                     uint32_t capacity,
                                     LoaderRecordInfo *info)
{
    LoaderRecordResult result = geometry(io, s, f);
    return result == NVR_OK ? read_payload(io, s, f, out, capacity, info) : result;
}

static LoaderRecordResult program_one(const LoaderFlashIo *io, uint32_t a, const uint8_t *p)
{
    uint8_t v[32];
    LoaderRecordResult r = erased_range(io, a, io->program_bytes);
    if (r != NVR_OK)
    {
        return r;
    }
    if (!io->program(io->context, a, p, io->program_bytes))
    {
        return NVR_IO;
    }
    r = read_at(io, a, v, io->program_bytes);
    if (r != NVR_OK)
    {
        return r;
    }
    return memcmp(v, p, io->program_bytes) ? NVR_INVALID : NVR_OK;
}
static LoaderRecordResult program_record(const LoaderFlashIo *io,
                                         const LoaderFlashRegion *s,
                                         const LoaderRecordFormat *f,
                                         uint32_t seq,
                                         uint32_t context,
                                         const void *payload)
{
    const uint8_t *p = (const uint8_t *)payload;
    uint8_t h[32], b[32], footer[32];
    uint32_t k, t, crc, u;
    LoaderRecordInfo check;
    LoaderRecordResult r;
    u = io->program_bytes;
    put32(h, f->magic);
    put32(h + 4, f->kind);
    put32(h + 8, f->payload_bytes);
    put32(h + 12, seq);
    put32(h + 16, context);
    put32(h + 20, f->layout);
    put32(h + 28, f->version);
    crc = crc_update(UINT32_MAX, h, 24);
    crc = crc_update(crc, h + 28, 4);
    crc = ~crc_update(crc, p, f->payload_bytes);
    put32(h + 24, crc);
    for (k = 0; k < 32; k += u)
    {
        r = program_one(io, s->base + k, h + k);
        if (r != NVR_OK)
        {
            return r;
        }
    }
    for (k = 0; k < f->payload_bytes; k += u)
    {
        t = f->payload_bytes - k;
        if (t > u)
        {
            t = u;
        }
        memset(b, 255, u);
        memcpy(b, p + k, t);
        r = program_one(io, s->base + 32 + k, b);
        if (r != NVR_OK)
        {
            return r;
        }
    }
    memset(footer, 255, 32);
    put32(footer, f->commit_magic);
    put32(footer + 4, ~f->commit_magic);
    put32(footer + 8, seq);
    put32(footer + 12, crc);
    /* 包含commit_magic的单元必须最后写；发布后不能再修改头或载荷。 */
    for (k = u; k < 32; k += u)
    {
        r = program_one(io, s->base + s->bytes - 32 + k, footer + k);
        if (r != NVR_OK)
        {
            return r;
        }
    }
    r = program_one(io, s->base + s->bytes - 32, footer);
    if (r != NVR_OK)
    {
        return r;
    }
    r = inspect_record(io, s, f, &check);
    if (r != NVR_OK)
    {
        return r;
    }
    return check.sequence == seq && check.context == context && check.crc == crc ? NVR_OK
                                                                                 : NVR_INVALID;
}
LoaderRecordResult LoaderRecord_Replace(const LoaderFlashIo *io,
                                        const LoaderFlashRegion *s,
                                        const LoaderRecordFormat *f,
                                        uint32_t seq,
                                        uint32_t context,
                                        const void *payload)
{
    uint32_t k;
    LoaderRecordResult result = geometry(io, s, f);
    if (result != NVR_OK)
    {
        return result;
    }
    if (!payload || !io->erase || !io->program)
    {
        return NVR_ARGUMENT;
    }
    for (k = 0; k < s->bytes; k += io->erase_bytes)
    {
        if (!io->erase(io->context, s->base + k, io->erase_bytes))
        {
            return NVR_IO;
        }
        result = erased_range(io, s->base + k, io->erase_bytes);
        if (result != NVR_OK)
        {
            return result;
        }
    }
    return program_record(io, s, f, seq, context, payload);
}
