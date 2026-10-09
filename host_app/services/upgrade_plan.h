#pragma once

#include <QString>
#include <QStringList>

namespace yg::host
{
/** APP 镜像的本地校验结果；不代表设备已安装或发布通过。 */
struct UpgradePlan
{
    bool valid = false;
    QString error;
    QStringList checks;
};

/**
 * @brief 校验同目录 image.bin 与 manifest.bin 的目标、布局、向量、长度和 CRC。
 * @param app_path 实际 image.bin 路径。
 * @return 本地校验结果；失败含原因，不触碰设备。
 */
UpgradePlan Upgrade_Inspect(const QString &app_path);
} // namespace yg::host
