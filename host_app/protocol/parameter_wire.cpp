/** 参数线路只做纯值转换；高位 u32 不经过有符号裁剪。 */
#include "parameter_wire.h"
#include <stdexcept>
extern "C"
{
#include "shared/protocol/yg_protocol_byte_order.h"
}

namespace yg::host
{
QByteArray Parameter_ReadPayload(quint8 id)
{
    QByteArray bytes(4, 0);
    bytes[0] = char(id);
    return bytes;
}

QByteArray Parameter_WritePayload(const ParameterValue &value)
{
    if (value.value_type != 1 && value.value_type != 2)
    {
        throw std::invalid_argument("参数类型必须为 i32 或 u32");
    }
    QByteArray bytes(12, 0);
    bytes[0] = char(value.id);
    bytes[1] = char(value.value_type);
    bytes[2] = 4;
    auto *data = reinterpret_cast<uint8_t *>(bytes.data());
    YgProtocol_WriteU32Le(data + 4, value.revision);
    YgProtocol_WriteU32Le(data + 8, value.raw);
    return bytes;
}

QByteArray Parameter_SavePayload(quint32 revision)
{
    QByteArray bytes(4, 0);
    YgProtocol_WriteU32Le(reinterpret_cast<uint8_t *>(bytes.data()), revision);
    return bytes;
}

ParameterReply Parameter_DecodeReply(quint16 type,
                                     const QByteArray &payload,
                                     quint8 expected_id,
                                     quint8 expected_value_type)
{
    if (type < 120 || type > 122 || payload.size() < 4)
    {
        throw std::invalid_argument("参数回复类型或长度错误");
    }
    const auto *data = reinterpret_cast<const uint8_t *>(payload.constData());
    ParameterReply reply;
    reply.result = YgProtocol_ReadU16Le(data);
    reply.detail = YgProtocol_ReadU16Le(data + 2);
    if (reply.result > 8 && reply.result != 11 && reply.result != 17)
    {
        throw std::invalid_argument("未知参数结果");
    }
    if (reply.result > 1)
    {
        if (payload.size() != 4)
        {
            throw std::invalid_argument("失败回复不得附带参数值");
        }
        return reply;
    }
    if (type == 122)
    {
        if (payload.size() != 8)
        {
            throw std::invalid_argument("SAVE 回复必须包含已确认持久版本");
        }
        reply.persisted_revision = YgProtocol_ReadU32Le(data + 4);
        return reply;
    }
    if (reply.result == 1 && type == 121 && payload.size() == 4)
    {
        return reply;
    }
    if (reply.result != 0 || payload.size() != 16 || data[4] != expected_id ||
        (type == 121 && data[5] != expected_value_type) || (data[5] != 1 && data[5] != 2) ||
        data[6] != 4 || (type == 120 ? (data[7] & ~7U) != 0 : data[7] != 0))
    {
        throw std::invalid_argument("活动值回复字段错误或受理冒充完成");
    }
    reply.value = ParameterValue{
        data[4], data[5], data[7], YgProtocol_ReadU32Le(data + 8), YgProtocol_ReadU32Le(data + 12)};
    return reply;
}
} // namespace yg::host
