# 编码器标定与反馈说明

本文档对应当前 TLE5012B 板载编码器重构后的实现。系统仅保留 SPI2 板载 TLE5012B；旧的外置传感器、动态传感器类型切换和方向判别均已删除。

## 1. 角度与速度数据流

1. `Encoder_Update()` 每个电流控制周期（20 kHz）读取一帧 TLE5012B SSC 数据。
2. 帧必须通过 SPI、CRC6、磁场状态和超速状态检查；连续 `100` 帧失败（5 ms）时编码器离线。
3. TLE5012B 的 15 位角度左移一位，得到无符号 Q15 环形角度 `raw_q15`：`0..65535`。
4. 方向参数 `encoder_reverse`（USB 参数 `erv`）决定 LUT 输入坐标：`0` 保持原始方向，`1` 将角度反向；当前 Product Profile 默认值为 `0`，最终方向必须通过实机确认。
5. 线性化表有 `1024` 个 `int16_t` Q15 修正值。对方向修正后的 `directed_q15` 查表并线性插值，得到 `linearized_q15`。
6. 电角度只使用线性化角度和电零位：

```c
electrical_q15 = (linearized_q15 - electrical_zero_q15) * pole_pairs;
```

7. 机械位置使用多圈 `shadow_q15`；机械零位只改变机械位置原点，不影响电角度。
8. 机械速度每 `10` 个电流控制周期更新一次，即 `2 kHz`。最近 `16` 个速度增量做滑动平均，等效窗口 `8 ms`；若累积增量绝对值不超过 `8 Q15`，输出速度为 `0`。没有 PLL 或角度插值。

内部环形表示为 `uint16_t 0..65535`；对外诊断可按 `int16_t -32768..32767` 解释。位置控制使用连续多圈机械弧度值。

## 2. 持久化标定数据

Flash 保存的数据严格分为三类：

| 数据 | 用途 | 是否影响编码器闭环 |
| --- | --- | --- |
| `linearization_lut_q15[1024]` | 补偿单圈角度非线性 | 是 |
| `electrical_zero_q15` | 使 d 轴对齐到电机电零位 | 是 |
| `mechanical_zero_q15` | 当前机械位置原点 | 否 |

闭环的前置条件是 `ENC_CALIB_LINEARIZED` 和 `ENC_CALIB_ELECTRICAL_ZERO` 同时有效。机械零位仅用于位置坐标定义。

Flash 格式已升级并更换魔术字；旧格式的编码器标定数据会被拒绝，因此升级该固件后必须重新完成线性化和电零位标定。

## 3. 标定模式

| 模式 | 名称 | 当前行为 |
| --- | --- | --- |
| 5 | `Calib_EncoderOffset` | 电流闭环旋转矢量单向线性化标定 |
| 7 | `Set_ZeroPosition` | 将当前线性化机械角设为机械零位；不设置电零位 |
| 13 | `Calib_EncoderObserver` | 无感启动后使用观测器闭环位置的 1024 点线性化标定 |
| 15 | `Calib_EleAngelOffset` | 固定 d 轴电流对齐并标定电零位 |

模式 5 同时作为新电机首次调试的基础标定入口；它不依赖尚未验证的磁链观察器，
也不再走旧的电压开环/反向平均算法。

模式 13 的磁链观测器把真实相电阻与观测器等效电阻分开：

```text
observer_Rs = phase_resistance_ohm × flux_observer_resistance_scale
```

真实 `phase_resistance_ohm` 继续用于电流环 PI 和电机参数模型；等效系数来自
`ControlTuningProfile`。当前 HT8115-4 台架值为 `1.905 Ω × 0.4199475 ≈ 0.800 Ω`。
这可避免为了让观察器锁定而把持久化相电阻改成非物理值。

## 4. 电流闭环旋转矢量线性化标定（模式 5）

任务函数：`Task_Calib_EncoderOffset()`。

1. 检查 TLE5012B 在线、极对数、电机参数，并确认标定电流不超过当前电流上限。
2. 在 `1.5 s` 内将受 PI 限制的 d 轴电流从 0 平滑升至 `calib_current`，使转子对齐。
3. 将开环电角速度在 `3 s` 内升至 `2π × 20 rad/s`。
4. 使用已知的连续驱动电角度作为参考，单方向完成一圈机械角度扫描；编码器电角速度
   低于目标的 50% 持续 `1 s` 时中止，避免堵转时生成无效 LUT。
5. 按 TLE5012B `raw_q15` 所在的 `1024` 个扇区累积误差，构造总线性化修正表。
6. 采样完成后：置位线性化标志、清除电零位和机械零位标志、保存 Flash。

阻尼负载会在驱动电流矢量与转子之间形成近似固定的相位差；模式 15 会随后独立标定
电角零位并消除该常量偏移。模式 5 不做编码器方向判别或正反转平均，安装方向必须与
当前电机正方向约定一致。

## 5. 电零位标定（模式 15）

任务函数：`Task_Calib_EleAngelOffset()`。

1. 先完成线性化标定。
2. 在 `0.5 s` 内将 `Id` 从零平滑上升至 `calib_current`，保持 `Iq = 0`。
3. 维持 `Id = calib_current`、`Iq = 0` 继续 `1 s`。
4. 仅在满预定位电流的这 `1 s` 内平均 `linearized_q15`，跨零点时按环形角度解卷绕。
5. 平均值写入 `electrical_zero_q15`，置位电零位标志并保存 Flash。

采样对象是线性化后的 Q15 角度，不是原始角度；因此不会覆盖线性化基准。

模式 5 或模式 13 生成新 LUT 后都会使旧电角零位失效，因此每次重新线性化后必须
重新执行模式 15。只看到 Mode 13 自动回到模式 0，并不代表编码器速度/位置闭环的
全部标定前置条件已经恢复。

## 6. 机械零位（模式 7）

模式 7 将当前 `linearized_q15` 及当前多圈影子计数保存为机械零位。之后机械位置立即为零，并触发参数保存。

它不会写 `electrical_zero_q15`，也不会置位电零位标定标志；执行模式 7 不能替代模式 15。

## 7. 推荐重新标定顺序

1. 上电等待自动电流 ADC 零偏标定完成（模式 11 回到模式 0）。
2. 在 Disable 状态设置 `erv`：`1` 表示反向、`0` 表示正向；当前代码默认值为 `0`，实际安装方向必须在限能量条件下确认。切换该参数会清空 LUT、电零位和机械零位标定，随后执行模式 9 保存。
3. 设置并保存准确的 `pol`、`mrs`、`mld`、`mlq`、`mfx`，以及合适的 `ica`。
4. 读取 `e_s`，确认响应为 `encoder=Online`。
5. 新电机或观察器尚未验证时优先写模式 5；只有磁链观察器已通过锁定验证时才使用
   模式 13。等待线性化标定完成并自动保存。
6. 清除可能的错误后，写入模式 15，完成电零位标定并自动保存。
7. 需要位置坐标原点时，再写入模式 7。
8. 最后进入电流、转速或位置闭环验证。

连续错误帧达到 100 帧时，当前需要编码器反馈的模式会报 `Encoder_Error`；SPI 传输诊断状态保留在 `Encoder_TypeDef` 中供调试读取。

## 8. LUT 读取与误差波形操作表

必须在执行下一种线性化标定前导出当前 LUT。模式 5 和模式 13 共用同一个持久化
`linearization_lut_q15[1024]`，后执行的标定会覆盖前一次结果；`-Prefix` 只决定导出
文件名，不能从设备中读取已经被覆盖的历史 Mode 5 或 Mode 13 LUT。

| 步骤 | 目的 | 命令 | 主要输出/通过条件 |
| --- | --- | --- | --- |
| 1 | 确认允许导出 | `\r_mod`、`\r_err`、`\r_e_s` | 必须为 `mode=0`、`error=0`、编码器 Online |
| 2 | 手工读取当前 LUT | 串口发送 `\r_lut\r\n` | 首行 `lut_begin,count=1024,reverse=...`，随后 1024 行数据，末行 `lut_end` |
| 3 | 导出 Mode 5 LUT | `pwsh -NoProfile -File tools/capture_encoder_lut.ps1 -Port COM4 -OutputDirectory validation/lut/mode5 -Prefix mode5` | 生成 metadata、CSV 和原始串口文本；必须收到 1024 点 |
| 4 | 导出 Mode 13 LUT | `pwsh -NoProfile -File tools/capture_encoder_lut.ps1 -Port COM4 -OutputDirectory validation/lut/mode13 -Prefix mode13` | 必须在 Mode 13 完成后、执行其他线性化标定前导出 |
| 5 | 绘制单次 LUT | `<python> tools/plot_encoder_lut.py <csv> <png> --summary <summary.json> --title "Mode 5 encoder LUT"` | 上图为含常量偏置的修正量，下图为去均值后的编码器非线性 |
| 6 | 比较 Mode 5/13 | `<python> tools/compare_encoder_luts.py <mode5.csv> <mode13.csv> <output-dir>` | 生成逐点对比 CSV、JSON 摘要和三联波形图 |
| 7 | 标定电角零位 | `\w_mod=15` | 新 LUT 会清除旧电角零位；完成后必须回到 `mode=0/error=0` |
| 8 | 复位回读 | J-Link/硬件复位后再次执行步骤 1，并读取 `\r_mrs` | LUT、方向、电机参数及电角零位应从 Flash 正常恢复 |

Python 3 环境需要提供 Pillow。示例：

```powershell
python tools\plot_encoder_lut.py `
  validation\lut\mode5\mode5_encoder_lut.csv `
  validation\lut\mode5\mode5_encoder_lut.png `
  --summary validation\lut\mode5\mode5_lut_summary.json `
  --title 'Mode 5 encoder LUT'
```

其他环境使用 Python 3，并确保已安装 Pillow。`capture_encoder_lut.ps1` 输出文件含义：

| 文件 | 内容 | 用途 |
| --- | --- | --- |
| `<prefix>_metadata.json` | 模式、错误、编码器方向、极对数、母线、电流限制及 R/L/磁链 | 证明 LUT 的标定环境，比较时必须一起保存 |
| `<prefix>_encoder_lut_raw.txt` | 设备原始响应，包括 begin/end 标记 | 协议审计和解析问题追溯 |
| `<prefix>_encoder_lut.csv` | `index/raw_deg/error_deg` 三列 | 绘图和逐点数值分析的标准输入 |
| `<prefix>_encoder_lut.png` | 原始修正和去均值非线性波形 | 人工检查周期形状、毛刺和局部异常 |
| `<prefix>_lut_summary.json` | 极值、峰峰值、均值、去均值 RMS 和峰值 | 自动化比较和历史趋势记录 |

单行 LUT 协议格式：

```text
lut=<index>,raw_deg=<原始机械角度>,err_deg=<LUT修正角度>
```

误差统计判读表：

| 指标 | 含义 | 判读方式 |
| --- | --- | --- |
| `mean_error_deg` | 标定参考带来的整体常量相位 | 不用于比较编码器非线性；由 Mode 15 重新建立电角零位 |
| `peak_to_peak_error_deg` | LUT 最大值与最小值之差 | 反映单圈总修正范围，但同时受局部异常影响 |
| `demeaned_rms_error_deg` | 去掉常量后的非线性 RMS | 适合比较同一编码器不同标定方法/不同批次的总体形状幅度 |
| `demeaned_peak_abs_error_deg` | 去掉常量后的最大绝对修正 | 用于定位最差机械角区域 |
| `demeaned_waveform_correlation` | 两套去均值 LUT 的形状相关性 | 越接近 1，说明识别到的周期非线性形状越一致 |
| `demeaned_difference_rms_deg` | 两套去均值 LUT 的逐点差异 RMS | 应结合相同方法的重复性基线判断，不能孤立设定统一阈值 |
| `demeaned_difference_peak_abs_deg` | 两种方法局部最大差异 | 回到对比 CSV 和波形中定位对应机械角 |

当前实机 Mode 5/Mode 13 的对比结果见
`docs/mode5_vs_mode13_calibration_comparison_2026-09-05.md`。两者去均值波形相关系数
为 `0.94470`，差异 RMS 为 `0.07212°`；同一 Mode 13 重复性差异 RMS 为
`0.03324°`。
