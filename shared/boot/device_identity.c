/* 生产身份记录与只读页的纯计算契约，APP和Loader共享，无平台依赖。 */
#include "device_identity.h"
#include "../checksum/crc32.h"
#include "../image/image_format.h"
#include "../protocol/yg_protocol_common.h"
#include <string.h>
bool DeviceIdentity_Decode(const uint8_t record[64], DeviceIdentity *identity)
{
    bool ended = false, nonempty = false;
    if (identity == NULL)
    {
        return false;
    }
    memset(identity, 0, sizeof(*identity));
    if (record == NULL || YgProtocol_ReadU32Le(record) != DEVICE_IDENTITY_MAGIC ||
        YgProtocol_ReadU32Le(record + 4) != 1U || YgProtocol_ReadU32Le(record + 24) == 0U ||
        YgProtocol_ReadU32Le(record + 60) != ~DEVICE_IDENTITY_MAGIC ||
        Crc32_Compute(record, 56U, 0U) != YgProtocol_ReadU32Le(record + 56))
    {
        return false;
    }
    for (uint32_t i = 0; i < 16U; ++i)
    {
        uint8_t ch = record[8U + i];
        if (ch == 0U)
        {
            ended = true;
        }
        else if (ended || !((ch >= 'A' && ch <= 'Z') || (ch >= 'a' && ch <= 'z') ||
                            (ch >= '0' && ch <= '9') || ch == '-'))
        {
            return false;
        }
        else
        {
            nonempty = true;
        }
    }
    for (uint32_t i = 28U; i < 56U; ++i)
    {
        if (record[i] != 0U)
        {
            return false;
        }
    }
    if (!nonempty)
    {
        return false;
    }
    memcpy(identity->sn, record + 8U, 16U);
    identity->motor_type = YgProtocol_ReadU32Le(record + 24U);
    identity->valid = true;
    return true;
}
void DeviceIdentity_EncodeInfo(const DeviceIdentity *identity,
                               uint8_t node,
                               uint8_t role,
                               uint8_t boot_state,
                               uint32_t image_crc32,
                               uint8_t payload[32])
{
    memset(payload, 0, 32U);
    if (identity != NULL && identity->valid)
    {
        memcpy(payload, identity->sn, 16U);
        YgProtocol_WriteU32Le(payload + 16U, identity->motor_type);
        payload[20] = 1U;
    }
    payload[21] = node;
    payload[22] = role;
    payload[23] = boot_state;
    YgProtocol_WriteU32Le(payload + 24U, image_crc32);
    YgProtocol_WriteU32Le(payload + 28U, YGI_PROFILE);
}
