#pragma once

#include "protocol/parameter_wire.h"
#include <QMap>
#include <QString>
#include <QVector>

namespace yg::host
{
/** 页面静态显示项；可写性仍以读取所得 access 为准。 */
struct ParameterDefinition
{
    quint8 id;
    QString name, unit;
    double scale;
};
/** 离线活动参数唯一所有者；不从参数模型获得硬件操作权。 */
class ParameterSandbox
{
  public:
    /** @brief 初始化明确标记的示例参数；不访问设备或修改文件。 */
    ParameterSandbox();
    /**
     * @brief 返回本模型有真实演示来源的页面字段。
     * @return 描述副本，数值必须通过 Read 取得。
     */
    QVector<ParameterDefinition> Sandbox_Definitions() const;
    /**
     * @brief 读取演示活动值及全局 revision。
     * @param id 单字节参数编号。
     * @return 当前标量快照；未登记项返回 UNSUPPORTED。
     */
    ParameterReply Sandbox_Read(quint8 id) const;
    /**
     * @brief 校验类型、读回权限、revision 和示例约束后修改活动值。
     * @param value 编号、类型、expected_revision 与待写位模式。
     * @return 已应用演示值或明确拒绝；不修改 persisted_revision。
     */
    ParameterReply Sandbox_Write(const ParameterValue &value);
    /**
     * @brief 受理独立的演示保存作业，不提前确认持久化。
     * @param expected_revision 当前活动修订号。
     * @return ACCEPTED 与先前 persisted_revision，失败保持原作业。
     */
    ParameterReply Sandbox_Save(quint32 expected_revision);
    /**
     * @brief 推进当前演示保存到成功或失败终态。
     * @param successful 是否模拟存储提交成功。
     * @return 成功后更新的 persisted_revision；失败不改变旧版本。
     */
    ParameterReply Sandbox_CompleteSave(bool successful);
    /**
     * @brief 查询已完成保存的演示修订号。
     * @return 已确认的持久修订号，不是参数 schema。
     */
    quint32 Sandbox_PersistedRevision() const;
    /**
     * @brief 查询演示保存作业是否仍在等待完成。
     * @return 待完成时返回 true，页面不得恢复写入入口。
     */
    bool Sandbox_SavePending() const;

  private:
    QMap<quint8, ParameterValue> values_;
    quint32 revision_ = 5, persisted_revision_ = 5;
    bool save_pending_ = false;
};
} // namespace yg::host
