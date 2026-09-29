/* APP→Loader 一次性进入邮箱：固定 32B 布局，magic/version 与前 28B CRC32。 */
#include "image_mailbox.h"
#include "../checksum/crc32.h"
#include "../protocol/yg_protocol_byte_order.h"

static bool valid(const BootMailboxMessage *m)
{
    return m && m->command == YG_BOOT_ENTER && m->node < 255U;
}
bool BootMailbox_Encode(const BootMailboxMessage *m, uint8_t *p)
{
    if (!p || !valid(m))
    {
        return false;
    }
    YgProtocol_WriteU32Le(p, YGIM_MAGIC);
    YgProtocol_WriteU32Le(p + 4, YGIM_VERSION);
    YgProtocol_WriteU32Le(p + 8, m->command);
    YgProtocol_WriteU32Le(p + 12, m->node);
    YgProtocol_WriteU32Le(p + 16, 0U);
    YgProtocol_WriteU32Le(p + 20, 0U);
    YgProtocol_WriteU32Le(p + 24, 0U);
    YgProtocol_WriteU32Le(p + 28, Crc32_Compute(p, 28U, 0U));
    return true;
}
bool BootMailbox_Decode(const uint8_t *p, BootMailboxMessage *out)
{
    BootMailboxMessage m;
    if (!p || !out || YgProtocol_ReadU32Le(p) != YGIM_MAGIC ||
        YgProtocol_ReadU32Le(p + 4) != YGIM_VERSION ||
        YgProtocol_ReadU32Le(p + 28) != Crc32_Compute(p, 28U, 0U) ||
        YgProtocol_ReadU32Le(p + 16) != 0U || YgProtocol_ReadU32Le(p + 20) != 0U ||
        YgProtocol_ReadU32Le(p + 24) != 0U)
    {
        return false;
    }
    m.command = YgProtocol_ReadU32Le(p + 8);
    m.node = YgProtocol_ReadU32Le(p + 12);
    if (!valid(&m))
    {
        return false;
    }
    *out = m;
    return true;
}
