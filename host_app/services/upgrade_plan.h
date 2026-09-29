#pragma once

#include <QByteArray>
#include <QJsonObject>
#include <QStringList>
#include <QVector>

namespace yg::host
{
/** 升级模式只选择业务写域，不授予硬件操作权限。 */
enum class UpgradeMode
{
    Loader,
    App,
    Combined
};
/** 不可变文件快照的安装区域；参数区从不成为区域成员。 */
struct ProgramRegion
{
    QString component, source;
    quint32 address = 0;
    QByteArray bytes;
};
/** 本地审核任务，不代表设备兼容、烧录、激活或发布完成。 */
struct UpgradePlan
{
    UpgradeMode mode = UpgradeMode::App;
    bool valid = false, synthetic = false;
    QString error, transport;
    QStringList checks, steps;
    QVector<ProgramRegion> regions;
    QByteArray manifest;
    QJsonObject target;
};

/**
 * @brief 校验当前目标及镜像，生成独立写域和可审核的离线任务。
 * @param mode Loader、APP 或联合安装。
 * @param loader_path Loader BIN 文件，APP 模式忽略。
 * @param app_path 打包后的 image.bin，清单取同目录 manifest.bin。
 * @param synthetic 是否使用内置合成样本；此模式永远不能用作部署证据。
 * @return 本地校验结果；失败仅含原因，不返回可执行任务。
 * @note 只读取文件和共享纯校验核心，不连接设备或加载驱动。
 */
UpgradePlan Upgrade_Inspect(UpgradeMode mode,
                            const QString &loader_path,
                            const QString &app_path,
                            bool synthetic = false);

/**
 * @brief 导出任务和冻结镜像副本；不覆盖既有目录。
 * @param plan 已通过本地校验的任务。
 * @param directory 新的绝对输出目录，推荐 outputs 下。
 * @return 新 job.json 的绝对路径；无效任务或写入失败抛出 runtime_error。
 * @note 不产生烧录脚本、不触发探针；任务保留参数备份和设备兼容性待核对项。
 */
QString Upgrade_Export(const UpgradePlan &plan, const QString &directory);

/**
 * @brief 从单份共享目标资源返回页面可呈现的配置，非实测设备身份。
 * @return 当前目标 JSON；资源损坏抛出 runtime_error。
 */
QJsonObject Upgrade_Target();
} // namespace yg::host
