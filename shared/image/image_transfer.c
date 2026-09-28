/* 镜像事务与跳前校验：只用描述符声明的长度与整包 CRC32，不保留持久提交记录。 */
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
static uint32_t crc_more(uint32_t crc, const uint8_t *p, uint32_t n)
{
    unsigned i;
    while (n--)
    {
        crc ^= *p++;
        for (i = 0; i < 8; i++)
        {
            crc = (crc >> 1) ^ ((0U - (crc & 1U)) & 0xEDB88320U);
        }
    }
    return crc;
}
static uint32_t crc32(const uint8_t *p, uint32_t n)
{
    return ~crc_more(~0U, p, n);
}
static bool inside(uint32_t a, uint32_t n, LoaderFlashRegion s)
{
    return n && a >= s.base && n <= s.bytes && a - s.base <= s.bytes - n;
}
static bool geometry(const LoaderImageConfig *c)
{
    LoaderFlashRegion all;
    if (!c || !c->io.read || !c->io.erase || !c->io.program || !c->allowed || !c->watchdog ||
        c->io.program_bytes != 8 || !c->io.erase_bytes || c->io.erase_bytes % 8 ||
        c->app.bytes > 131072 || c->app.bytes < 8 || c->app.bytes % c->io.erase_bytes ||
        c->app.base % c->io.erase_bytes || c->schema != 11 || c->ram_start >= c->ram_end ||
        c->descriptor_offset < 8 || (uint64_t)c->descriptor_offset + 64U > c->app.bytes ||
        c->descriptor_offset % 8 || !c->layout || !c->mcu || !c->product || !c->hardware ||
        !c->param_abi)
    {
        return false;
    }
    all.base = c->io.base;
    all.bytes = c->io.bytes;
    return (uint64_t)all.base + all.bytes <= 0x100000000ULL &&
           inside(c->app.base, c->app.bytes, all);
}
static bool descriptor_decode(const LoaderImageConfig *c,
                              const uint8_t *p,
                              LoaderImageIdentity *out,
                              const LoaderImageIdentity *manifest)
{
    uint32_t version, role, bytes, crc;
    uint16_t low, high;
    unsigned i;
    if (be32(p) != YGI_DESCRIPTOR_MAGIC || be32(p + 4) != YGI_DESCRIPTOR_VERSION ||
        be32(p + 8) != YGI_PROFILE || be32(p + 12) != c->layout || be32(p + 16) != c->mcu ||
        be32(p + 20) != c->product || be32(p + 24) != c->hardware || be32(p + 36) != c->param_abi)
    {
        return false;
    }
    version = be32(p + 28);
    role = be32(p + 32);
    bytes = be32(p + YGI_DESCRIPTOR_BYTES_OFFSET);
    crc = be32(p + YGI_DESCRIPTOR_CRC_OFFSET);
    low = (uint16_t)((uint16_t)p[40] << 8 | p[41]);
    high = (uint16_t)((uint16_t)p[42] << 8 | p[43]);
    if (role != YGI_ROLE_NORMAL || low > c->schema || high < c->schema || low > high ||
        bytes < c->descriptor_offset + 64U || bytes > c->app.bytes || bytes % 8)
    {
        return false;
    }
    for (i = YGI_DESCRIPTOR_CRC_OFFSET + 4U; i < YGI_DESCRIPTOR_BYTES; ++i)
    {
        if (p[i])
        {
            return false;
        }
    }
    if (manifest != NULL && (manifest->version != version || manifest->role != role ||
                             manifest->bytes != bytes || manifest->crc32 != crc))
    {
        return false;
    }
    if (out != NULL)
    {
        out->version = version;
        out->token = manifest != NULL ? manifest->token : 0U;
        out->bytes = bytes;
        out->role = role;
        out->crc32 = crc;
    }
    return true;
}
static bool read_descriptor(const LoaderImageConfig *c,
                            LoaderImageIdentity *out,
                            const LoaderImageIdentity *manifest)
{
    uint8_t p[YGI_DESCRIPTOR_BYTES];
    return c->io.read(c->io.context, c->app.base + c->descriptor_offset, p, sizeof(p)) &&
           descriptor_decode(c, p, out, manifest);
}
static bool vectors(const LoaderImageConfig *c, uint32_t bytes)
{
    uint8_t p[8];
    uint32_t sp, pc;
    if (!c->io.read(c->io.context, c->app.base, p, sizeof(p)))
    {
        return false;
    }
    sp = le32(p);
    pc = le32(p + 4);
    return sp > c->ram_start && sp <= c->ram_end && !(sp & 7U) && (pc & 1U) &&
           (uint64_t)(pc & ~1U) >= (uint64_t)c->app.base + c->descriptor_offset + 64U &&
           (uint64_t)(pc & ~1U) < (uint64_t)c->app.base + bytes;
}
/* 整包 CRC32；描述符自身的4B CRC字段按0参与，以便打包时回填后仍可复算。 */
static bool compute_crc(const LoaderImageConfig *c, uint32_t bytes, uint32_t *out)
{
    uint8_t p[64];
    uint32_t off = 0, crc = ~0U, crc_at = c->descriptor_offset + YGI_DESCRIPTOR_CRC_OFFSET;
    while (off < bytes)
    {
        uint32_t n = bytes - off, k;
        if (n > sizeof(p))
        {
            n = sizeof(p);
        }
        if (!c->io.read(c->io.context, c->app.base + off, p, n))
        {
            return false;
        }
        for (k = 0; k < n; ++k)
        {
            uint32_t a = off + k;
            if (a >= crc_at && a < crc_at + 4U)
            {
                p[k] = 0U;
            }
        }
        crc = crc_more(crc, p, n);
        c->watchdog(c->owner);
        off += n;
    }
    *out = ~crc;
    return true;
}
static LoaderImageResult fail(LoaderImageTransfer *t, LoaderImageResult r)
{
    t->phase = YI_ERROR;
    t->reason = r;
    return r;
}
bool LoaderImage_DecodeManifest(const LoaderImageConfig *c,
                                const uint8_t p[96],
                                LoaderImageIdentity *out)
{
    LoaderImageIdentity m;
    uint16_t low, high;
    unsigned i;
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
    m.crc32 = be32(p + YGI_MANIFEST_CRC_OFFSET);
    low = (uint16_t)((uint16_t)p[44] << 8 | p[45]);
    high = (uint16_t)((uint16_t)p[46] << 8 | p[47]);
    if (!m.token || m.bytes < c->descriptor_offset + 64U || m.bytes > c->app.bytes || m.bytes % 8 ||
        low > c->schema || high < c->schema || low > high || m.role != YGI_ROLE_NORMAL)
    {
        return false;
    }
    for (i = YGI_MANIFEST_CRC_OFFSET + 4U; i < YGI_MANIFEST_BYTES; ++i)
    {
        if (p[i])
        {
            return false;
        }
    }
    *out = m;
    return true;
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
    if (t->phase == YI_ERASING || t->phase == YI_VERIFYING)
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
    t->hash = ~0U;
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
    const LoaderImageConfig *c = t ? t->config : NULL;
    if (!t || !c || !data || t->in_call || !bytes || bytes > 128 || bytes % 8 || offset % 8)
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
    t->hash = ~0U;
    t->phase = YI_VERIFYING;
    return YI_OK;
}
LoaderImageResult LoaderImage_Poll(LoaderImageTransfer *t)
{
    const LoaderImageConfig *c = t ? t->config : NULL;
    uint32_t a, n, i, crc_at;
    uint8_t p[64];
    LoaderImageResult r = YI_OK;
    if (!t || !c || t->in_call)
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
        a = c->app.base + t->erased;
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
        if (t->erased >= c->app.bytes)
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
        if (!c->io.read(c->io.context, c->app.base + t->next, p, n))
        {
            r = fail(t, YI_FLASH);
            goto done;
        }
        crc_at = c->descriptor_offset + YGI_DESCRIPTOR_CRC_OFFSET;
        for (i = 0; i < n; ++i)
        {
            uint32_t position = t->next + i;
            if (position >= crc_at && position < crc_at + 4U)
            {
                p[i] = 0U;
            }
        }
        t->hash = crc_more(t->hash, p, n);
        t->next += n;
        if (t->next == t->image.bytes)
        {
            if (~t->hash != t->image.crc32 || !vectors(c, t->image.bytes) ||
                !read_descriptor(c, NULL, &t->image))
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
bool LoaderImage_Check(const LoaderImageConfig *c, const LoaderImageIdentity *m)
{
    uint32_t crc;
    if (!geometry(c) || !m || m->bytes < 8 || m->bytes > c->app.bytes || m->bytes % 8)
    {
        return false;
    }
    return compute_crc(c, m->bytes, &crc) && crc == m->crc32 && vectors(c, m->bytes) &&
           read_descriptor(c, NULL, m);
}
bool LoaderImage_CheckImage(const LoaderImageConfig *c)
{
    LoaderImageIdentity id;
    uint32_t crc;
    if (!geometry(c) || !read_descriptor(c, &id, NULL))
    {
        return false;
    }
    return compute_crc(c, id.bytes, &crc) && crc == id.crc32 && vectors(c, id.bytes);
}
