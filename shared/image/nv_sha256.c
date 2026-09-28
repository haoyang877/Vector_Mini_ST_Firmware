/* SHA256按NIST FIPS180-4第4/5/6节实现；无堆/HAL依赖，不等于签名认证。 */
#include "nv_sha256.h"
#include <string.h>
static uint32_t rotate(uint32_t x, unsigned n)
{
    return x >> n | x << (32 - n);
}
static uint32_t r32(const uint8_t *p)
{
    return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3];
}
static void w32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)v;
}
static const uint32_t round_k[64] = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};
static void transform(LoaderSha256 *ctx, const uint8_t *p)
{
    uint32_t w[16], a = ctx->h[0], b = ctx->h[1], c = ctx->h[2], d = ctx->h[3], e = ctx->h[4],
                    f = ctx->h[5], g = ctx->h[6], h = ctx->h[7];
    uint32_t x, y, t1, t2;
    unsigned i;
    for (i = 0; i < 16; i++)
    {
        w[i] = r32(p + i * 4);
    }
    for (i = 0; i < 64; i++)
    {
        if (i >= 16)
        {
            x = w[(i - 15) & 15];
            y = w[(i - 2) & 15];
            w[i & 15] += (rotate(x, 7) ^ rotate(x, 18) ^ (x >> 3)) + w[(i - 7) & 15] +
                         (rotate(y, 17) ^ rotate(y, 19) ^ (y >> 10));
        }
        t1 = h + (rotate(e, 6) ^ rotate(e, 11) ^ rotate(e, 25)) + ((e & f) ^ (~e & g)) +
             round_k[i] + w[i & 15];
        t2 = (rotate(a, 2) ^ rotate(a, 13) ^ rotate(a, 22)) + ((a & b) ^ (a & c) ^ (b & c));
        h = g;
        g = f;
        f = e;
        e = d + t1;
        d = c;
        c = b;
        b = a;
        a = t1 + t2;
    }
    ctx->h[0] += a;
    ctx->h[1] += b;
    ctx->h[2] += c;
    ctx->h[3] += d;
    ctx->h[4] += e;
    ctx->h[5] += f;
    ctx->h[6] += g;
    ctx->h[7] += h;
}
void LoaderSha256_Init(LoaderSha256 *ctx)
{
    static const uint32_t initial[8] = {0x6a09e667,
                                        0xbb67ae85,
                                        0x3c6ef372,
                                        0xa54ff53a,
                                        0x510e527f,
                                        0x9b05688c,
                                        0x1f83d9ab,
                                        0x5be0cd19};
    if (!ctx)
    {
        return;
    }
    memset(ctx, 0, sizeof(*ctx));
    memcpy(ctx->h, initial, sizeof(initial));
}
bool LoaderSha256_Update(LoaderSha256 *ctx, const void *data, uint32_t bytes)
{
    const uint8_t *p = data;
    uint32_t count;
    if (!ctx || (!data && bytes) || ctx->finished || ctx->used >= 64 ||
        ctx->bytes > UINT64_MAX / 8 || bytes > UINT64_MAX / 8 - ctx->bytes)
    {
        return false;
    }
    ctx->bytes += bytes;
    while (bytes)
    {
        count = 64 - ctx->used;
        if (count > bytes)
        {
            count = bytes;
        }
        memcpy(ctx->block + ctx->used, p, count);
        ctx->used += count;
        p += count;
        bytes -= count;
        if (ctx->used == 64)
        {
            transform(ctx, ctx->block);
            ctx->used = 0;
        }
    }
    return true;
}
bool LoaderSha256_Finish(LoaderSha256 *ctx, uint8_t out[32])
{
    uint64_t bits;
    unsigned i;
    if (!ctx || !out || ctx->finished || ctx->used >= 64 || ctx->bytes > UINT64_MAX / 8)
    {
        return false;
    }
    bits = ctx->bytes * 8;
    ctx->block[ctx->used++] = 0x80;
    if (ctx->used > 56)
    {
        memset(ctx->block + ctx->used, 0, 64 - ctx->used);
        transform(ctx, ctx->block);
        ctx->used = 0;
    }
    memset(ctx->block + ctx->used, 0, 56 - ctx->used);
    w32(ctx->block + 56, (uint32_t)(bits >> 32));
    w32(ctx->block + 60, (uint32_t)bits);
    transform(ctx, ctx->block);
    for (i = 0; i < 8; i++)
    {
        w32(out + 4 * i, ctx->h[i]);
    }
    ctx->finished = true;
    return true;
}
bool LoaderSha256_Digest(const void *p, uint32_t n, uint8_t out[32])
{
    LoaderSha256 ctx;
    LoaderSha256_Init(&ctx);
    return LoaderSha256_Update(&ctx, p, n) && LoaderSha256_Finish(&ctx, out);
}
