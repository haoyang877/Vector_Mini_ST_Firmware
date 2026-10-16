#pragma once

#include <QJsonObject>
#include <QString>
#include <QStringList>
#include <cstdint>

namespace yg::host
{
/** APP 镜像的本地校验结果；不代表设备已安装或发布通过。 */
struct UpgradePlan
{
    bool valid = false;
    uint32_t version = 0, crc32 = 0;
    QString error;
    QStringList checks;
};

/**
 * @brief 校验同目录 image.bin 与 manifest.bin 的目标、布局、向量、长度和 CRC。
 * @param app_path 实际 image.bin 路径。
 * @return 本地校验结果；失败含原因，不触碰设备。
 */
UpgradePlan Upgrade_Inspect(const QString &app_path);
/**
 * @brief 将客户端结果与本次已校验的包及固定 UID 对照。
 * @param plan 本次输入包的实际版本和 CRC。
 * @param expected_uid 本会话固定的真实设备 UID。
 * @param report 客户端 summary.json 内容。
 * @return UID、版本、CRC 和成功标志全部一致时返回 true。
 */
bool Upgrade_ResultMatches(const UpgradePlan &plan,
                           const QString &expected_uid,
                           const QJsonObject &report);
} // namespace yg::host
