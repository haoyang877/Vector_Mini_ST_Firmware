/** 示例参数及保存作业只有此模型能改变；拒绝和完成均消费当前线路载荷解释器。 */
#include "parameter_sandbox.h"
#include <limits>
extern "C"
{
#include "shared/protocol/yg_protocol_byte_order.h"
}

namespace yg::host
{
namespace
{
ParameterReply scalar_reply(quint16 type, ParameterValue value)
{
    QByteArray bytes(16, 0);
    auto *data = reinterpret_cast<uint8_t *>(bytes.data());
    data[4] = value.id;
    data[5] = value.value_type;
    data[6] = 4;
    data[7] = type == 120 ? value.access : 0;
    YgProtocol_WriteU32Le(data + 8, value.revision);
    YgProtocol_WriteU32Le(data + 12, value.raw);
    return Parameter_DecodeReply(type, bytes, value.id, value.value_type);
}

ParameterReply save_reply(quint16 result, quint32 revision)
{
    QByteArray bytes(8, 0);
    auto *data = reinterpret_cast<uint8_t *>(bytes.data());
    YgProtocol_WriteU16Le(data, result);
    YgProtocol_WriteU32Le(data + 4, revision);
    return Parameter_DecodeReply(122, bytes);
}
} // namespace

ParameterSandbox::ParameterSandbox()
{
    // 这些值只用于页面交互，不宣称是产品默认值或设备读取结果。
    values_.insert(0, {0, 2, 1, revision_, revision_});
    values_.insert(1, {1, 1, 3, revision_, 10000});
    values_.insert(2, {2, 1, 3, revision_, 3000});
    values_.insert(3, {3, 2, 1, revision_, 3});
    values_.insert(5, {5, 2, 1, revision_, 7});
    values_.insert(21, {21, 1, 3, revision_, 1000000});
    values_.insert(22, {22, 1, 3, revision_, 100000});
    values_.insert(23, {23, 1, 3, revision_, 5000});
    values_.insert(0x4d, {0x4d, 2, 3, revision_, 0});
    values_.insert(0x4f, {0x4f, 2, 1, revision_, 3});
    values_.insert(0x51, {0x51, 2, 1, revision_, 0});
}

QVector<ParameterDefinition> ParameterSandbox::Sandbox_Definitions() const
{
    return {{0, "活动配置修订号", "revision", 1},
            {1, "最大速度", "rad/s", 1000},
            {2, "最大扭矩电流 Iq", "A", 1000},
            {3, "通信节点", "node", 1},
            {5, "电机极对数", "对", 1},
            {21, "位置环 Kp", "1/s", 1000000},
            {22, "位置环 Kd", "系数", 1000000},
            {23, "位置环最大速度", "rad/s", 1000},
            {0x4d, "停机配置确认", "0 / 1", 1},
            {0x4f, "启动受限位", "bitmask", 1},
            {0x51, "可启动模式位", "bitmask", 1}};
}

ParameterReply ParameterSandbox::Sandbox_Read(quint8 id) const
{
    if (!values_.contains(id))
    {
        return {2, 1, {}, {}};
    }
    auto value = values_.value(id);
    value.revision = revision_;
    if (id == 0)
    {
        value.raw = revision_;
    }
    return scalar_reply(120, value);
}

ParameterReply ParameterSandbox::Sandbox_Write(const ParameterValue &value)
{
    const auto snapshot = Sandbox_Read(value.id);
    if (!snapshot.value)
    {
        return snapshot;
    }
    if (save_pending_)
    {
        return {6, 0, {}, {}};
    }
    if (!(snapshot.value->access & 2))
    {
        return {5, 2, {}, {}};
    }
    if (value.revision != revision_)
    {
        return {4, 3, {}, {}};
    }
    if (value.value_type != snapshot.value->value_type)
    {
        return {4, 6, {}, {}};
    }
    // 仅示例模型约束，实际产品范围由设备所有者检查。
    if ((value.id == 0x4d && value.raw > 1) ||
        (value.id != 0x4d && (value.raw == 0 || value.raw > 10000000)) ||
        (value.id == 23 && value.raw > values_.value(1).raw) ||
        (value.id == 1 && value.raw < values_.value(23).raw) ||
        revision_ == std::numeric_limits<quint32>::max())
    {
        return {11, 0, {}, {}};
    }
    auto next = value;
    next.revision = ++revision_;
    next.access = snapshot.value->access;
    values_[value.id] = next;
    return scalar_reply(121, next);
}

ParameterReply ParameterSandbox::Sandbox_Save(quint32 expected_revision)
{
    if (save_pending_)
    {
        return {6, 0, {}, {}};
    }
    if (expected_revision != revision_)
    {
        return {4, 3, {}, {}};
    }
    save_pending_ = true;
    return save_reply(1, persisted_revision_);
}

ParameterReply ParameterSandbox::Sandbox_CompleteSave(bool successful)
{
    if (!save_pending_)
    {
        return {5, 5, {}, {}};
    }
    save_pending_ = false;
    if (!successful)
    {
        return {8, 4, {}, {}};
    }
    persisted_revision_ = revision_;
    return save_reply(0, persisted_revision_);
}

quint32 ParameterSandbox::Sandbox_PersistedRevision() const
{
    return persisted_revision_;
}

bool ParameterSandbox::Sandbox_SavePending() const
{
    return save_pending_;
}
} // namespace yg::host
