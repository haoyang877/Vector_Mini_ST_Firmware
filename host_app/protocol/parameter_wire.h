#pragma once
#include <QByteArray>
#include <optional>

namespace yg::host
{
/** 参数标量保持原始 u32 位模式，value_type 决定 i32/u32 解释。 */
struct ParameterValue
{
    quint8 id = 0, value_type = 1, access = 0;
    quint32 revision = 0, raw = 0;
};
/** 回复类型明确区分拒绝、受理、活动值及真实持久版本。 */
struct ParameterReply
{
    quint16 result = 0, detail = 0;
    std::optional<ParameterValue> value;
    std::optional<quint32> persisted_revision;
};

/**
 * @brief 构造当前参数 READ 的四字节载荷。
 * @param id 单字节参数编号。
 * @return 显式小端载荷，不含公司帧头。
 */
QByteArray Parameter_ReadPayload(quint8 id);
/**
 * @brief 按活动修订号构造 WRITE，不隐含保存。
 * @param value 已读取的类型、参数编号与 expected_revision，raw 为待写位模式。
 * @return 十二字节载荷；非法类型抛出 invalid_argument。
 */
QByteArray Parameter_WritePayload(const ParameterValue &value);
/**
 * @brief 构造单独的 SAVE 请求。
 * @param revision 已确认的活动修订号。
 * @return 四字节小端载荷。
 */
QByteArray Parameter_SavePayload(quint32 revision);
/**
 * @brief 校验参数回复，禁止受理结果携带伪造活动值。
 * @param type 120、121 或 122。
 * @param payload 完整业务载荷。
 * @param expected_id READ/WRITE 期望的参数编号。
 * @param expected_value_type WRITE 请求类型，必须为 1 或 2；其他操作可传 0。
 * @return 显式结果；非法长度、字段、保留位或未知结果抛出 invalid_argument。
 */
ParameterReply Parameter_DecodeReply(quint16 type,
                                     const QByteArray &payload,
                                     quint8 expected_id = 0,
                                     quint8 expected_value_type = 0);
} // namespace yg::host
