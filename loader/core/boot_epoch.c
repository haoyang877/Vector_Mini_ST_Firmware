#include "boot_epoch.h"
#include <string.h>
typedef struct
{
    bool valid, dirty;
    uint64_t generation, high;
    uint32_t next;
} page_state;
static uint16_t r16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] << 8 | p[1]);
}
static uint32_t r32(const uint8_t *p)
{
    return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3];
}
static uint64_t r64(const uint8_t *p)
{
    return (uint64_t)r32(p) << 32 | r32(p + 4);
}
static void w16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)v;
}
static void w32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)v;
}
static void w64(uint8_t *p, uint64_t v)
{
    w32(p, (uint32_t)(v >> 32));
    w32(p + 4, (uint32_t)v);
}
static bool filled(const uint8_t *p, unsigned n, uint8_t value)
{
    while (n--)
    {
        if (*p++ != value)
        {
            return false;
        }
    }
    return true;
}
static uint32_t crc_more(uint32_t crc, const uint8_t *p, unsigned n)
{
    unsigned j;
    while (n--)
    {
        crc ^= *p++;
        for (j = 0; j < 8; j++)
        {
            crc = (crc >> 1) ^ (0xedb88320U & (0U - (crc & 1U)));
        }
    }
    return crc;
}
static uint32_t header_crc(const uint8_t *p)
{
    return ~crc_more(crc_more(~0U, p, 40), p + 44, 12);
}
static uint32_t entry_crc(const uint8_t header[64], uint32_t index, const uint8_t *p)
{
    uint8_t pos[4];
    uint32_t crc = crc_more(~0U, header, 40);
    w32(pos, index);
    crc = crc_more(crc, pos, 4);
    crc = crc_more(crc, p, 16);
    return ~crc_more(crc, p + 20, 4);
}
static LoaderBootResult geometry(const LoaderBootStore *s)
{
    unsigned i;
    uint64_t end, other;
    if (!s || !s->io.read || !s->io.erase || !s->io.program || !s->magic || !s->version ||
        !s->header_commit || s->header_commit == UINT64_MAX || !s->entry_commit ||
        s->entry_commit == UINT64_MAX)
    {
        return YGB_ARGUMENT;
    }
    if (s->io.program_bytes != 8 || s->io.erase_bytes < 96 || s->io.erase_bytes % 32 ||
        (uint64_t)s->io.base + s->io.bytes > UINT64_C(0x100000000))
    {
        return YGB_GEOMETRY;
    }
    for (i = 0; i < 2; i++)
    {
        end = (uint64_t)s->page[i].base + s->page[i].bytes;
        if (s->page[i].bytes != s->io.erase_bytes || s->page[i].base % s->io.erase_bytes ||
            s->page[i].base < s->io.base || end > (uint64_t)s->io.base + s->io.bytes)
        {
            return YGB_GEOMETRY;
        }
    }
    end = (uint64_t)s->page[0].base + s->page[0].bytes;
    other = (uint64_t)s->page[1].base + s->page[1].bytes;
    if (s->page[0].base < other && s->page[1].base < end)
    {
        return YGB_GEOMETRY;
    }
    return YGB_OK;
}
static bool read_at(const LoaderBootStore *s, unsigned page, uint32_t off, void *out, uint32_t n)
{
    return s->io.read(s->io.context, s->page[page].base + off, out, n);
}
static LoaderBootResult scan(const LoaderBootStore *s, unsigned page, page_state *state)
{
    uint8_t header[64], entry[32];
    uint32_t i, slots = (s->page[page].bytes - 64) / 32;
    uint64_t epoch;
    memset(state, 0, sizeof(*state));
    if (!read_at(s, page, 0, header, 64))
    {
        return YGB_IO;
    }
    if (r64(header + 56) != s->header_commit)
    {
        /* 未提交或空白页不是可信来源，不能猜floor=0。 */
        return YGB_OK;
    }
    if (r32(header) != s->magic || r16(header + 4) != s->version || r16(header + 6) != 64 ||
        memcmp(header + 8, s->uid, 12) || !filled(header + 20, 4, 0) ||
        !filled(header + 44, 12, 0) || r32(header + 40) != header_crc(header) || !r64(header + 24))
    {
        return YGB_MAINTENANCE;
    }
    state->valid = true;
    state->generation = r64(header + 24);
    state->high = r64(header + 32);
    for (i = 0; i < slots; i++)
    {
        if (!read_at(s, page, 64 + i * 32, entry, 32))
        {
            return YGB_IO;
        }
        if (filled(entry, 32, 255))
        {
            continue;
        }
        state->next = i + 1;
        if (r64(entry + 24) != s->entry_commit)
        {
            state->dirty = true;
            continue;
        }
        epoch = r64(entry);
        if (r64(entry + 8) != ~epoch || !filled(entry + 20, 4, 0) ||
            r32(entry + 16) != entry_crc(header, i, entry) || epoch <= state->high)
        {
            return YGB_MAINTENANCE;
        }
        state->high = epoch;
    }
    return YGB_OK;
}
static LoaderBootResult
write_units(const LoaderBootStore *s, unsigned page, uint32_t off, const uint8_t *p, uint32_t n)
{
    uint8_t check[8];
    uint32_t i;
    for (i = 0; i < n; i += 8)
    {
        if (!s->io.program(s->io.context, s->page[page].base + off + i, p + i, 8))
        {
            return YGB_IO;
        }
        if (!read_at(s, page, off + i, check, 8))
        {
            return YGB_IO;
        }
        if (memcmp(check, p + i, 8))
        {
            return YGB_MAINTENANCE;
        }
    }
    return YGB_OK;
}
static LoaderBootResult
rewrite(const LoaderBootStore *s, unsigned page, uint64_t generation, uint64_t floor)
{
    uint8_t header[64], check[32];
    uint32_t off;
    LoaderBootResult result;
    page_state state;
    if (!s->io.erase(s->io.context, s->page[page].base, s->page[page].bytes))
    {
        return YGB_IO;
    }
    for (off = 0; off < s->page[page].bytes; off += 32)
    {
        if (!read_at(s, page, off, check, 32))
        {
            return YGB_IO;
        }
        if (!filled(check, 32, 255))
        {
            return YGB_MAINTENANCE;
        }
    }
    memset(header, 0, sizeof(header));
    w32(header, s->magic);
    w16(header + 4, s->version);
    w16(header + 6, 64);
    memcpy(header + 8, s->uid, 12);
    w64(header + 24, generation);
    w64(header + 32, floor);
    w32(header + 40, header_crc(header));
    w64(header + 56, s->header_commit);
    result = write_units(s, page, 0, header, 64);
    if (result != YGB_OK)
    {
        return result;
    }
    result = scan(s, page, &state);
    if (result != YGB_OK)
    {
        return result;
    }
    return state.valid && state.high == floor && state.generation == generation && !state.dirty
               ? YGB_OK
               : YGB_MAINTENANCE;
}
static LoaderBootResult
append(const LoaderBootStore *s, unsigned page, page_state *state, uint64_t epoch)
{
    uint8_t header[64], entry[32], check[32];
    uint32_t off = 64 + state->next * 32;
    LoaderBootResult result;
    page_state after;
    if (off > s->page[page].bytes - 32 || epoch <= state->high)
    {
        return YGB_MAINTENANCE;
    }
    if (!read_at(s, page, 0, header, 64) || !read_at(s, page, off, check, 32))
    {
        return YGB_IO;
    }
    if (!filled(check, 32, 255))
    {
        return YGB_MAINTENANCE;
    }
    memset(entry, 0, sizeof(entry));
    w64(entry, epoch);
    w64(entry + 8, ~epoch);
    w32(entry + 16, entry_crc(header, state->next, entry));
    w64(entry + 24, s->entry_commit);
    result = write_units(s, page, off, entry, 32);
    if (result != YGB_OK)
    {
        return result;
    }
    result = scan(s, page, &after);
    if (result != YGB_OK)
    {
        return result;
    }
    if (!after.valid || after.high != epoch)
    {
        return YGB_MAINTENANCE;
    }
    *state = after;
    return YGB_OK;
}
LoaderBootResult LoaderBoot_Provision(const LoaderBootStore *s, uint64_t floor)
{
    uint8_t check[32];
    unsigned i;
    uint32_t off;
    LoaderBootResult result = geometry(s);
    if (result != YGB_OK)
    {
        return result;
    }
    for (i = 0; i < 2; i++)
    {
        for (off = 0; off < s->page[i].bytes; off += 32)
        {
            if (!read_at(s, i, off, check, 32))
            {
                return YGB_IO;
            }
            if (!filled(check, 32, 255))
            {
                return YGB_MAINTENANCE;
            }
        }
    }
    result = rewrite(s, 0, 1, floor);
    if (result != YGB_OK)
    {
        return result;
    }
    return rewrite(s, 1, 1, floor);
}
LoaderBootResult LoaderBoot_Allocate(const LoaderBootStore *s, uint64_t *out)
{
    page_state state[2];
    uint64_t high = 0, generation = 0, next;
    unsigned i, peer;
    uint32_t slots;
    LoaderBootResult result;
    if (!out)
    {
        return YGB_ARGUMENT;
    }
    result = geometry(s);
    if (result != YGB_OK)
    {
        return result;
    }
    for (i = 0; i < 2; i++)
    {
        result = scan(s, i, &state[i]);
        if (result != YGB_OK)
        {
            return result;
        }
        if (state[i].valid)
        {
            if (state[i].high > high)
            {
                high = state[i].high;
            }
            if (state[i].generation > generation)
            {
                generation = state[i].generation;
            }
        }
    }
    if (!state[0].valid && !state[1].valid)
    {
        return YGB_MAINTENANCE;
    }
    /* 两份见证损坏或唯一来源不完整时必须外部诊断，不能自动猜测启动历史。 */
    if ((!state[0].valid || state[0].dirty) && (!state[1].valid || state[1].dirty))
    {
        return YGB_MAINTENANCE;
    }
    if (high == UINT64_MAX)
    {
        return YGB_EXHAUSTED;
    }
    if (generation == UINT64_MAX)
    {
        return YGB_EXHAUSTED;
    }
    slots = (s->io.erase_bytes - 64) / 32;
    /* 擦除任何来源前先使两份追平已消费高水位；擦页期间始终保留high==H的同伴。 */
    for (i = 0; i < 2; i++)
    {
        peer = 1 - i;
        if (!state[i].valid || state[i].high < high)
        {
            if (!state[peer].valid || state[peer].high != high)
            {
                return YGB_MAINTENANCE;
            }
            if (state[i].valid && !state[i].dirty && state[i].next < slots)
            {
                result = append(s, i, &state[i], high);
            }
            else
            {
                if (generation == UINT64_MAX)
                {
                    return YGB_EXHAUSTED;
                }
                result = rewrite(s, i, ++generation, high);
                if (result == YGB_OK)
                {
                    result = scan(s, i, &state[i]);
                }
            }
            if (result != YGB_OK)
            {
                return result;
            }
        }
    }
    for (i = 0; i < 2; i++)
    {
        if (state[i].dirty || state[i].next >= slots)
        {
            if (generation == UINT64_MAX)
            {
                return YGB_EXHAUSTED;
            }
            result = rewrite(s, i, ++generation, high);
            if (result != YGB_OK)
            {
                return result;
            }
            result = scan(s, i, &state[i]);
            if (result != YGB_OK)
            {
                return result;
            }
        }
    }
    next = high + 1;
    for (i = 0; i < 2; i++)
    {
        result = append(s, i, &state[i], next);
        if (result != YGB_OK)
        {
            return result;
        }
    }
    /* 完成所有写入后重读两份；成功返回必须有双持久见证。 */
    for (i = 0; i < 2; i++)
    {
        result = scan(s, i, &state[i]);
        if (result != YGB_OK)
        {
            return result;
        }
        if (!state[i].valid || state[i].dirty || state[i].high != next)
        {
            return YGB_MAINTENANCE;
        }
    }
    *out = next;
    return YGB_OK;
}
