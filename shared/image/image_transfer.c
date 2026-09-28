#include "image_transfer.h"
#include <string.h>
static uint32_t be32(const uint8_t *p)
{
    return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3];
}
static uint32_t le32(const uint8_t *p)
{
    return (uint32_t)p[3] << 24 | (uint32_t)p[2] << 16 | (uint32_t)p[1] << 8 | p[0];
}
static uint32_t crc32(const uint8_t *p, uint32_t n)
{
    uint32_t c = ~0U;
    unsigned i;
    while (n--)
    {
        c ^= *p++;
        for (i = 0; i < 8; i++)
        {
            c = (c >> 1) ^ ((0U - (c & 1U)) & 0xEDB88320U);
        }
    }
    return ~c;
}
static bool inside(uint32_t a, uint32_t n, LoaderFlashRegion s)
{
    return n && a >= s.base && n <= s.bytes && a - s.base <= s.bytes - n;
}
static bool overlap(LoaderFlashRegion a, LoaderFlashRegion b)
{
    return (uint64_t)a.base + a.bytes > (uint64_t)b.base &&
           (uint64_t)b.base + b.bytes > (uint64_t)a.base;
}
static bool geometry(const LoaderImageConfig *c)
{
    LoaderFlashRegion all;
    if (!c || !c->io.read || !c->io.erase || !c->io.program || !c->allowed || !c->watchdog ||
        c->io.program_bytes != 8 || !c->io.erase_bytes || c->io.erase_bytes % 8 ||
        c->record.bytes != c->io.erase_bytes || c->record.bytes < 160 || c->app.bytes > 131072 ||
        c->app.bytes < 8 || c->app.bytes % c->io.erase_bytes || c->app.base % c->io.erase_bytes ||
        c->record.base % c->io.erase_bytes || c->schema != 11 || c->ram_start >= c->ram_end ||
        c->descriptor_offset < 8 || (uint64_t)c->descriptor_offset + 64U > c->app.bytes ||
        c->descriptor_offset % 8 || !c->layout || !c->mcu || !c->product || !c->hardware ||
        !c->param_abi)
    {
        return false;
    }
    all.base = c->io.base;
    all.bytes = c->io.bytes;
    return (uint64_t)all.base + all.bytes <= 0x100000000ULL &&
           inside(c->app.base, c->app.bytes, all) && inside(c->record.base, c->record.bytes, all) &&
           !overlap(c->app, c->record);
}
bool LoaderImage_DecodeManifest(const LoaderImageConfig *c,
                                const uint8_t p[96],
                                LoaderImageIdentity *out)
{
    LoaderImageIdentity m;
    unsigned i;
    uint32_t flags;
    uint16_t low, high;
    if (!geometry(c) || !p || !out || be32(p) != YGI_MANIFEST_MAGIC || be32(p + 4) != YGI_PROFILE ||
        be32(p + 8) != c->layout || be32(p + 12) != c->mcu || be32(p + 16) != c->product ||
        be32(p + 20) != c->hardware || be32(p + 40) != c->param_abi)
    {
        return false;
    }
    m.version = be32(p + 24);
    m.token = be32(p + 28);
    m.bytes = be32(p + 32);
    m.role = be32(p + 36);
    flags = be32(p + 80);
    low = (uint16_t)((uint16_t)p[44] << 8 | p[45]);
    high = (uint16_t)((uint16_t)p[46] << 8 | p[47]);
    if (!m.token || m.bytes < c->descriptor_offset + 64 || m.bytes > c->app.bytes || m.bytes % 8 ||
        low > c->schema || high < c->schema || low > high ||
        (m.role != YGI_ROLE_NORMAL && m.role != YGI_ROLE_MAINTENANCE) ||
        flags != (m.role == YGI_ROLE_MAINTENANCE ? 1U : 0U))
    {
        return false;
    }
    for (i = 84; i < 96; i++)
    {
        if (p[i])
        {
            return false;
        }
    }
    memcpy(m.sha, p + 48, 32);
    *out = m;
    return true;
}
static bool vectors(const LoaderImageConfig *c, const LoaderImageIdentity *m)
{
    uint8_t p[8];
    uint32_t sp, pc;
    if (!c->io.read(c->io.context, c->app.base, p, 8))
    {
        return false;
    }
    sp = le32(p);
    pc = le32(p + 4);
    return sp > c->ram_start && sp <= c->ram_end && !(sp & 7U) && (pc & 1U) &&
           (uint64_t)(pc & ~1U) >= (uint64_t)c->app.base + c->descriptor_offset + 64U &&
           (uint64_t)(pc & ~1U) < (uint64_t)c->app.base + m->bytes;
}
static bool
descriptor(const LoaderImageConfig *c, const LoaderImageIdentity *m, const uint8_t *manifest)
{
    uint8_t p[64];
    unsigned i;
    uint16_t low, high;
    if (!c->io.read(c->io.context, c->app.base + c->descriptor_offset, p, 64) ||
        be32(p) != YGI_DESCRIPTOR_MAGIC || be32(p + 4) != YGI_DESCRIPTOR_VERSION ||
        be32(p + 8) != YGI_PROFILE || be32(p + 12) != c->layout || be32(p + 16) != c->mcu ||
        be32(p + 20) != c->product || be32(p + 24) != c->hardware || be32(p + 28) != m->version ||
        be32(p + 32) != m->role || be32(p + 36) != c->param_abi ||
        be32(p + 44) != (m->role == YGI_ROLE_MAINTENANCE ? YGI_MAINTENANCE_WORKFLOW_ONLY : 0U))
    {
        return false;
    }
    low = (uint16_t)((uint16_t)p[40] << 8 | p[41]);
    high = (uint16_t)((uint16_t)p[42] << 8 | p[43]);
    if (low > c->schema || high < c->schema || low > high ||
        (manifest && memcmp(p + 40, manifest + 44, 4)))
    {
        return false;
    }
    for (i = 48; i < 64; i++)
    {
        if (p[i])
        {
            return false;
        }
    }
    return true;
}
static LoaderImageResult fail(LoaderImageTransfer *t, LoaderImageResult r)
{
    t->phase = YI_ERROR;
    t->reason = r;
    return r;
}
bool LoaderImage_Init(LoaderImageTransfer *t, const LoaderImageConfig *c)
{
    if (!t)
    {
        return false;
    }
    memset(t, 0, sizeof(*t));
    if (!geometry(c))
    {
        return false;
    }
    t->config = c;
    return true;
}
void LoaderImage_Abort(LoaderImageTransfer *t)
{
    if (t && !t->in_call)
    {
        const LoaderImageConfig *c = t->config;
        memset(t, 0, sizeof(*t));
        t->config = c;
    }
}
LoaderImageResult LoaderImage_Begin(LoaderImageTransfer *t, const uint8_t manifest[96])
{
    LoaderImageIdentity parsed;
    if (!t || !t->config || !manifest || t->in_call)
    {
        return YI_ARGUMENT;
    }
    if (t->phase == YI_ERASING || t->phase == YI_VERIFYING || t->phase == YI_COMMITTED)
    {
        return YI_BUSY;
    }
    if (!LoaderImage_DecodeManifest(t->config, manifest, &parsed))
    {
        return YI_IDENTITY;
    }
    t->image = parsed;
    memcpy(t->manifest, manifest, 96);
    t->manifest_crc = crc32(manifest, 96);
    t->next = 0;
    t->erased = 0;
    t->reason = YI_OK;
    t->phase = YI_READY;
    return YI_OK;
}
LoaderImageResult LoaderImage_Erase(LoaderImageTransfer *t)
{
    if (!t || !t->config || t->in_call)
    {
        return YI_ARGUMENT;
    }
    if (t->phase == YI_ERASING || t->phase == YI_RECEIVING)
    {
        return YI_OK;
    }
    if (t->phase != YI_READY)
    {
        return YI_STATE;
    }
    t->phase = YI_ERASING;
    t->erased = 0;
    return YI_OK;
}
LoaderImageResult
LoaderImage_Write(LoaderImageTransfer *t, uint32_t offset, const uint8_t *data, uint32_t bytes)
{
    uint8_t old[8];
    uint32_t i;
    LoaderImageResult result = YI_OK;
    const LoaderImageConfig *c;
    if (!t || !(c = t->config) || !data || t->in_call || !bytes || bytes > 128 || bytes % 8 ||
        offset % 8)
    {
        return YI_ARGUMENT;
    }
    if (t->phase != YI_RECEIVING)
    {
        return YI_STATE;
    }
    if (bytes > t->image.bytes || offset > t->image.bytes - bytes || offset > t->next)
    {
        return YI_RANGE;
    }
    t->in_call = true;
    for (i = 0; i < bytes; i += 8)
    {
        uint32_t position = offset + i;
        if (!c->allowed(c->owner))
        {
            result = fail(t, YI_DENIED);
            break;
        }
        if (position < t->next)
        {
            if (!c->io.read(c->io.context, c->app.base + position, old, 8))
            {
                result = fail(t, YI_FLASH);
                break;
            }
            if (memcmp(old, data + i, 8))
            {
                result = YI_CONFLICT;
                break;
            }
            continue;
        }
        /* 每个 doubleword 成功读回后才推进；失败后只能重新整页擦除，不能原位补写 ECC。 */
        if (!c->io.program(c->io.context, c->app.base + position, data + i, 8) ||
            !c->io.read(c->io.context, c->app.base + position, old, 8) || memcmp(old, data + i, 8))
        {
            result = fail(t, YI_FLASH);
            break;
        }
        t->next += 8;
        c->watchdog(c->owner);
    }
    t->in_call = false;
    return result;
}
LoaderImageResult LoaderImage_Verify(LoaderImageTransfer *t)
{
    if (!t || !t->config || t->in_call)
    {
        return YI_ARGUMENT;
    }
    if (t->phase == YI_VERIFYING || t->phase == YI_VERIFIED)
    {
        return YI_OK;
    }
    if (t->phase != YI_RECEIVING || t->next != t->image.bytes)
    {
        return YI_STATE;
    }
    t->next = 0;
    t->phase = YI_VERIFYING;
    LoaderSha256_Init(&t->hash);
    return YI_OK;
}
LoaderImageResult LoaderImage_Poll(LoaderImageTransfer *t)
{
    const LoaderImageConfig *c;
    uint32_t a, n, i;
    uint8_t p[64], digest[32];
    LoaderImageResult r = YI_OK;
    if (!t || !(c = t->config) || t->in_call)
    {
        return YI_ARGUMENT;
    }
    if (t->phase != YI_ERASING && t->phase != YI_VERIFYING)
    {
        return YI_OK;
    }
    t->in_call = true;
    if (!c->allowed(c->owner))
    {
        r = fail(t, YI_DENIED);
        goto done;
    }
    c->watchdog(c->owner);
    if (t->phase == YI_ERASING)
    {
        a = t->erased ? c->app.base + t->erased - c->io.erase_bytes : c->record.base;
        if (!c->io.erase(c->io.context, a, c->io.erase_bytes))
        {
            r = fail(t, YI_FLASH);
            goto done;
        }
        for (n = 0; n < c->io.erase_bytes; n += sizeof(p))
        {
            uint32_t count = c->io.erase_bytes - n;
            if (count > sizeof(p))
            {
                count = sizeof(p);
            }
            if (!c->io.read(c->io.context, a + n, p, count))
            {
                r = fail(t, YI_FLASH);
                goto done;
            }
            for (i = 0; i < count; i++)
            {
                if (p[i] != 255)
                {
                    r = fail(t, YI_FLASH);
                    goto done;
                }
            }
        }
        t->erased += c->io.erase_bytes;
        if (t->erased - c->io.erase_bytes == c->app.bytes)
        {
            t->phase = YI_RECEIVING;
            t->next = 0;
        }
    }
    else
    {
        n = t->image.bytes - t->next;
        if (n > sizeof(p))
        {
            n = sizeof(p);
        }
        if (!c->io.read(c->io.context, c->app.base + t->next, p, n) ||
            !LoaderSha256_Update(&t->hash, p, n))
        {
            r = fail(t, YI_FLASH);
            goto done;
        }
        t->next += n;
        if (t->next == t->image.bytes)
        {
            if (!LoaderSha256_Finish(&t->hash, digest) || memcmp(digest, t->image.sha, 32) ||
                !vectors(c, &t->image) || !descriptor(c, &t->image, t->manifest))
            {
                r = fail(t, YI_DIGEST);
            }
            else
            {
                t->phase = YI_VERIFIED;
            }
        }
    }
done:
    t->in_call = false;
    return r;
}
static LoaderRecordFormat format(const LoaderImageConfig *c)
{
    LoaderRecordFormat f = {0x59495231U, 0x59494331U, YGI_PROFILE, 0, 1, 96};
    f.layout = c->layout;
    return f;
}
static bool gate_read(void *v, uint32_t a, void *p, uint32_t n)
{
    const LoaderImageConfig *c = v;
    return inside(a, n, c->record) && c->io.read(c->io.context, a, p, n);
}
static bool gate_erase(void *v, uint32_t a, uint32_t n)
{
    const LoaderImageConfig *c = v;
    return a == c->record.base && n == c->record.bytes && c->allowed(c->owner) &&
           c->io.erase(c->io.context, a, n);
}
static bool gate_program(void *v, uint32_t a, const void *p, uint32_t n)
{
    const LoaderImageConfig *c = v;
    return inside(a, n, c->record) && c->allowed(c->owner) && c->io.program(c->io.context, a, p, n);
}
static bool record_write(const LoaderImageConfig *c, const uint8_t p[96], uint32_t context)
{
    LoaderFlashIo io = c->io;
    LoaderRecordFormat f = format(c);
    io.context = (void *)c;
    io.read = gate_read;
    io.erase = gate_erase;
    io.program = gate_program;
    return LoaderRecord_Replace(&io, &c->record, &f, be32(p + 28), context, p) == NVR_OK;
}
bool LoaderImage_ReadRecord(const LoaderImageConfig *c, uint8_t p[96], uint32_t *context)
{
    uint8_t raw[96];
    LoaderRecordInfo info;
    LoaderImageIdentity image;
    LoaderRecordFormat f;
    if (!geometry(c) || !p || !context)
    {
        return false;
    }
    f = format(c);
    if (LoaderRecord_Read(&c->io, &c->record, &f, raw, 96, &info) != NVR_OK || info.context < 1 ||
        info.context > 3 || !LoaderImage_DecodeManifest(c, raw, &image) ||
        info.sequence != image.token)
    {
        return false;
    }
    memcpy(p, raw, 96);
    *context = info.context;
    return true;
}
bool LoaderImage_Check(const LoaderImageConfig *c, const LoaderImageIdentity *m)
{
    LoaderSha256 h;
    uint8_t p[64], digest[32];
    uint32_t n, offset = 0;
    if (!geometry(c) || !m || m->bytes < 8 || m->bytes > c->app.bytes || m->bytes % 8)
    {
        return false;
    }
    LoaderSha256_Init(&h);
    while (offset < m->bytes)
    {
        n = m->bytes - offset;
        if (n > 64)
        {
            n = 64;
        }
        c->watchdog(c->owner);
        if (!c->io.read(c->io.context, c->app.base + offset, p, n) ||
            !LoaderSha256_Update(&h, p, n))
        {
            return false;
        }
        offset += n;
    }
    return LoaderSha256_Finish(&h, digest) && !memcmp(digest, m->sha, 32) && vectors(c, m) &&
           descriptor(c, m, 0);
}
LoaderImageResult LoaderImage_Commit(LoaderImageTransfer *t)
{
    LoaderImageResult r;
    LoaderImageIdentity checked;
    if (!t || !t->config || t->in_call || t->phase != YI_VERIFIED)
    {
        return YI_STATE;
    }
    t->in_call = true;
    if (!t->config->allowed(t->config->owner))
    {
        r = fail(t, YI_DENIED);
    }
    else if (crc32(t->manifest, 96) != t->manifest_crc ||
             !LoaderImage_DecodeManifest(t->config, t->manifest, &checked) ||
             checked.version != t->image.version || checked.token != t->image.token ||
             checked.bytes != t->image.bytes || checked.role != t->image.role ||
             memcmp(checked.sha, t->image.sha, 32) || !LoaderImage_Check(t->config, &t->image) ||
             !descriptor(t->config, &t->image, t->manifest))
    {
        r = fail(t, YI_DIGEST);
    }
    else if (!record_write(t->config, t->manifest, 1))
    {
        r = fail(t, YI_FLASH);
    }
    else
    {
        t->phase = YI_COMMITTED;
        r = YI_OK;
    }
    t->in_call = false;
    return r;
}
static LoaderBootDecision admission(const LoaderImageIdentity *m, LoaderParameterState p)
{
    if (m->role == YGI_ROLE_NORMAL)
    {
        return p == YI_PARAMS_RAW11 ? YI_BOOT_NORMAL : YI_RESCUE;
    }
    if (m->role == YGI_ROLE_MAINTENANCE && (p == YI_PARAMS_RAW11 || p == YI_PARAMS_BLANK))
    {
        return YI_BOOT_MAINTENANCE_INHIBITED;
    }
    return YI_RESCUE;
}
LoaderBootDecision LoaderImage_PrepareBoot(const LoaderImageConfig *c,
                                           LoaderParameterState p,
                                           bool forced,
                                           LoaderImageIdentity *out)
{
    uint8_t raw[96];
    uint32_t state;
    LoaderImageIdentity m;
    LoaderBootDecision result;
    if (forced || !out || !LoaderImage_ReadRecord(c, raw, &state) || state == 2 ||
        !LoaderImage_DecodeManifest(c, raw, &m) || !LoaderImage_Check(c, &m) ||
        !descriptor(c, &m, raw))
    {
        return YI_RESCUE;
    }
    result = admission(&m, p);
    if (result == YI_RESCUE)
    {
        return result;
    }
    if (state == 1 && !record_write(c, raw, 2))
    {
        return YI_RESCUE;
    }
    *out = m;
    return result;
}
LoaderImageResult LoaderImage_Confirm(const LoaderImageConfig *c,
                                      LoaderParameterState p,
                                      const LoaderImageIdentity *expected)
{
    uint8_t raw[96];
    uint32_t state;
    LoaderImageIdentity m;
    if (!expected || !LoaderImage_ReadRecord(c, raw, &state) ||
        !LoaderImage_DecodeManifest(c, raw, &m) || state == 1 || m.token != expected->token ||
        m.version != expected->version || m.bytes != expected->bytes || m.role != expected->role ||
        memcmp(m.sha, expected->sha, 32) || admission(&m, p) == YI_RESCUE ||
        !LoaderImage_Check(c, &m) || !descriptor(c, &m, raw))
    {
        return YI_DENIED;
    }
    if (state == 3)
    {
        return YI_OK;
    }
    return record_write(c, raw, 3) ? YI_OK : YI_FLASH;
}
