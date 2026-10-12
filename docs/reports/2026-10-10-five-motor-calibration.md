# 五台电机标定参数与编码器曲线

版本：v1.0，2026-10-10。

本报告从已核验 Flash 快照离线提取，不连接 J-Link、不重新标定。左右轮、Roll、Pitch 的来源为 2026-10-10 回读；Yaw 为 2026-09-30 回读。UID 用于区分板卡，不能把旧角色名称替代该板最新配置。

## 编码器个体标定

| 电机 / ID | UID 前缀 | 回读日期 | 电零位 counts / 编码器参考° | 机械零位 counts / ° | 方向 | 标志 |
| --- | --- | --- | --- | --- | --- | --- |
| 左轮 / 1 | `18004A` | 2026-10-10 | 15655 / 85.995° | 未标定（存储 0） | 0 | 3 |
| 右轮 / 2 | `500051` | 2026-10-10 | 12605 / 69.241° | 未标定（存储 0） | 0 | 3 |
| Roll / 3 | `3E001F` | 2026-10-10 | 3834 / 21.061° | 56100 / 308.167° | 0 | 7 |
| Pitch / 4 | `680048` | 2026-10-10 | 3000 / 16.479° | 12747 / 70.021° | 0 | 7 |
| Yaw / 5 | `460060` | 2026-09-30 | 3707 / 20.363° | 43503 / 238.969° | 0 | 7 |

标志 bit0=角度 LUT、bit1=电零位、bit2=机械零位；3 表示前两项有效，7 表示三项有效。方向 0 表示不反转编码器。标志是固件保存的状态，本报告没有重新验证标定物理有效性。

电零位字段保存的是线性化编码器参考位置，不是直接的电角度；固件用 `电角 = (线性化编码器角 − 电零位参考角) × 极对数`，按一电周期取模。机械零位的度数也是一圈内编码器参考角，不是当前轴位置。

## 电流采样与电机模型

| 电机 | A / B / C 采样零偏 counts | Rs / Ω | Ld / Lq / mH | 磁链 / Wb | 运行 / 标定电流 / A | 摩擦模型正 / 反向幅值 / A |
| --- | --- | --- | --- | --- | --- | --- |
| 左轮 | 2012 / 2046 / 2036 | 2.144 | 0.900 / 0.900 | 0.0175025 | 6 / 3 | 0.000 / 0.000 |
| 右轮 | 2041 / 2034 / 2054 | 2.144 | 0.900 / 0.900 | 0.0175025 | 6 / 3 | 0.000 / 0.000 |
| Roll | 2064 / 2063 / 2051 | 2.144 | 0.900 / 0.900 | 0.0175025 | 8 / 5 | 2.946 / 3.259 |
| Pitch | 2032 / 2047 / 2041 | 2.144 | 0.900 / 0.900 | 0.0175025 | 8 / 5 | 3.455 / 3.101 |
| Yaw | 2051 / 2044 / 2035 | 2.144 | 0.900 / 0.900 | 0.0175025 | 8 / 5 | 3.103 / 2.653 |

五台均为 21 极对、6 mΩ。采样零偏为 Flash 保存值；复位启动时可能重新采样 RAM 零偏。Rs/Ld/Lq/磁链相同，来源为共用模型配置，不能据此声称五台各自完成了 R/L/磁链辨识。运行/标定电流是限制或指令配置，不是实际辨识结果。

Roll/Pitch 摩擦系数在角色切换时按总体参数表配置，未在当前两台重新辨识；Yaw 系数为历史保存的模型，本次没有复测。它们与编码器 LUT/零位属于不同数据。

## LUT 曲线与统计

![编码器补偿纹波](../../outputs/five_motor_calibration_20261010/encoder_lut_shape.png)

| 电机 | 原始补偿均值 / ° | 补偿纹波峰峰值 / ° | 去均值 RMS / ° | 最大相邻点差 / counts |
| --- | --- | --- | --- | --- |
| 左轮 | -51.544 | 0.747 | 0.209 | 9 |
| 右轮 | +146.599 | 0.577 | 0.143 | 9 |
| Roll | -29.050 | 0.588 | 0.150 | 7 |
| Pitch | +146.997 | 1.154 | 0.261 | 10 |
| Yaw | -14.777 | 0.621 | 0.162 | 8 |

每台 LUT 为 1024 个 int16 点，横轴步长 360/1024=0.3515625°，纵轴 counts×360/65536。固件在相邻点间插值，并减去补偿值。上图只为比较形状减去每台均值，实际保存值没有修改。

![原始 LUT 补偿](../../outputs/five_motor_calibration_20261010/encoder_lut_absolute.png)

LUT 常量偏置包含标定对齐参考差异，不能按均值绝对大小判断电机好坏。峰峰值/RMS描述补偿曲线本身，不能解释为补偿后真实角度误差或定位噪声。缺少五台一致条件下的独立角度参考与标定后残余误差采样，因此不绘制虚构的精度验收曲线或电流环响应曲线。

## 数据来源与复现

离线脚本、完整数值及导出图位于 `outputs/five_motor_calibration_20261010/`：`summarize.py`、`summary.json`、`encoder_lut_points.csv`、PNG/SVG 曲线与 `manifest.json`。

- 左轮，UID `18004a0010504e4831303320`：`outputs/software_version_update_20261010_02/flash_after_reboot.bin`；核验 `outputs/software_version_update_20261010_02/download_result.json`。Flash SHA-256：`e386af3e12a43979dc846420cc2780946bc47b775e49d0670f3f088f622db34d`；LUT SHA-256：`881b61ce8891ef7baffb4b3cfa491050bd20a584757dd43a11e12de59a4e3457`。
- 右轮，UID `500051001750503759353020`：`outputs/software_version_update_20261010/flash_after_reboot.bin`；核验 `outputs/software_version_update_20261010/download_result.json`。Flash SHA-256：`706d1b8af97df50ac006da109a8bcfc98297f536f966505282aa9e3070f7637b`；LUT SHA-256：`71befef5ce0dc72f7051c9446563709aaac036f704feaa3cb89347ee89aa9e89`。
- Roll，UID `3e001f000e50325534303120`：`outputs/software_version_update_20261010_03/flash_after_reboot.bin`；核验 `outputs/software_version_update_20261010_03/download_result.json`。Flash SHA-256：`b5bec3bd3983900f309cf24a0bf575e887a1a613ba0a62e37786255354d525ca`；LUT SHA-256：`0894725968f471fc7f90d20b95ef0b422fe9688a92ac146384392ffbacd775c6`。
- Pitch，UID `6800480010504e4831303320`：`outputs/software_version_update_20261010_04/flash_after_reboot.bin`；核验 `outputs/software_version_update_20261010_04/download_result.json`。Flash SHA-256：`66c90fb8e0426c30c2cf759e9505985e0a2705d289c6ad0a7c0b9b429042331a`；LUT SHA-256：`2daac47045e1740250273100d9b7dd1c95a3025ba90274da8661a7e6fd749309`。
- Yaw，UID `4600600012504e4831303320`：`outputs/yaw_filter5_limit90_20260930/flash_after_write.bin`；核验 `outputs/yaw_filter5_limit90_20260930/verify_result.json`。Flash SHA-256：`047eacc0ed6e8de4a4cc02211f144de9a8ade8c4827f8641d1cbb61e60ce2902`；LUT SHA-256：`46b5562dce067723d1aff42d676eafbd9fbc84a9c24250cddde9c2d380f7adfa`。
