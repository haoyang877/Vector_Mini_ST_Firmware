#include "image_mailbox.h"
#include "image_format.h"
static uint32_t get(const uint8_t *p)
{
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}
static void put(uint8_t *p, uint32_t n)
{
    p[0] = (uint8_t)n;
    p[1] = (uint8_t)(n >> 8);
    p[2] = (uint8_t)(n >> 16);
    p[3] = (uint8_t)(n >> 24);
}
static uint32_t crc(const uint8_t *p)
{
    uint32_t c = ~0U, i, j;
    for (i = 0; i < 28; i++)
    {
        c ^= p[i];
        for (j = 0; j < 8; j++)
        {
            c = (c >> 1) ^ ((0U - (c & 1U)) & 0xEDB88320U);
        }
    }
    return ~c;
}
static bool valid(const BootMailboxMessage *m)
{
    if (!m || m->node >= 255)
    {
        return false;
    }
    if (m->command == YG_BOOT_ENTER)
    {
        return true;
    }
    return (m->command == YG_BOOT_TRIAL || m->command == YG_BOOT_NORMAL ||
            m->command == YGI_BOOT_START) &&
           m->token && m->epoch;
}
bool BootMailbox_Encode(const BootMailboxMessage *m, uint8_t *p)
{
    if (!p || !valid(m))
    {
        return false;
    }
    put(p, YGIM_MAGIC);
    put(p + 4, YGIM_VERSION);
    put(p + 8, m->command);
    put(p + 12, m->node);
    put(p + 16, m->token);
    put(p + 20, (uint32_t)m->epoch);
    put(p + 24, (uint32_t)(m->epoch >> 32));
    put(p + 28, crc(p));
    return true;
}
bool BootMailbox_Decode(const uint8_t *p, BootMailboxMessage *out)
{
    BootMailboxMessage m;
    if (!p || !out || get(p) != YGIM_MAGIC || get(p + 4) != YGIM_VERSION || get(p + 28) != crc(p))
    {
        return false;
    }
    m.command = get(p + 8);
    m.node = get(p + 12);
    m.token = get(p + 16);
    m.epoch = get(p + 20) | (uint64_t)get(p + 24) << 32;
    if (!valid(&m))
    {
        return false;
    }
    *out = m;
    return true;
}
