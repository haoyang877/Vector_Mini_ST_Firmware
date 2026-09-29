#pragma once
#include "workbench_session.h"

namespace yg::host
{
/**
 * @brief 导出有界完整演示记录，保留 SI 数值和数据间隙。
 * @param path 新 CSV 文件路径，不覆盖现有文件。
 * @param samples 完整记录，1..30000 点。
 * @note 只写本地文件，不是设备采集证明；失败抛出 runtime_error。
 */
void Capture_Write(const QString &path, const std::vector<ScopeSample> &samples);
/**
 * @brief 读取本程序导出的演示 CSV，不接受未知设备来源或畸形数据。
 * @param path 最大 16 MiB 的 CSV 文件。
 * @return 原始顺序的 SI 演示样本；格式错误抛出 runtime_error。
 */
std::vector<ScopeSample> Capture_Read(const QString &path);
} // namespace yg::host
