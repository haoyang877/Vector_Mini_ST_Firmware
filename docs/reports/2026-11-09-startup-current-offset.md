# 上电自动电流零偏校准：V0.7.1 build 1

日期：2026-11-09。分支：`feature/roll-motion-telemetry`。
需求：上电保持三相关断，自动测量电流偏置，完成后停机，不自动保存 Flash，并下载验证。

## 实现

- 移除 Loader APP 启动时强制跳过 mode 11 的分支；保留原有电机配置有效性检查。
- 启动不使能三相 PWM，只使用 TIM1 CH4 触发 ADC。mode 11 也不走通用 PWM 启用路径。
- 电流偏置采样前强制确认关相；累积 `FOC_FREQ` 次原始 ADC 数据，即 20 kHz 下 1 秒。
- 三相均值全部在 1948～2148 counts 内才更新 RAM 偏置，否则保留原值、置电流偏置故障并停机。
- 停机或中止清除未完成的采样；结束直接进入 mode 0，不进入 mode 9，不写 Flash。
- 采样期间不使用旧偏置换算相电流，避免旧偏置造成误报；校准任务仍读取真实 ADC 原码。

构建为 V0.7.1 build 1，版本编码 `0x00070101`。点到点位置参考，经典 CAN。
Keil 普通工程及隔离候选构建均为 0 错误、0 警告；离线测试全套通过，覆盖正常采样、中止重启、
无效均值回退、边界值、关相约束以及现有控制、Loader 和通信回归。

## 下载与实机结果

探针 J-Link `601012403`；STM32G431CB UID `33004b0010504e4831303320`；当前左轮，节点 1。
仅写 APP 与有效记录；原 Loader、参数区、保留页逐字节不变。
原 LUT/电零位标志 `calib_flag=3` 保留。本轮未执行电机转动测试或角度重标定。

| 检查 | 复位 1 | 复位 2 |
| --- | --- | --- |
| 自复位到校准结束 | 1.176 s | 1.186 s |
| 最后观测到的累积计数 | 19,857 | 19,952 |
| 初始 Flash 偏置 A/B/C | 2037 / 1986 / 2022 | 2037 / 1986 / 2022 |
| 完成后 RAM 偏置 A/B/C | 2037 / 1985 / 2023 | 2036 / 1984 / 2022 |
| 最终模式 / 故障 | 0 / 0 | 0 / 0 |
| 观测期间三相 CCER 使能位 | 全部为 0 | 全部为 0 |
| 校准及结束后相引脚 | 高阻 | 高阻 |
| 复位前后整片 Flash | 完全一致 | 完全一致 |
| `last_save_result` | 0，未保存 | 0，未保存 |

J-Link 是离散采样，表中计数为最后读到的值，并非任务实际少采了样本；任务达到 20,000 次后立即清零并停机。
两轮均完成 RAM 更新，证明启动确实重新测量，未直接沿用 Flash 数值。

APP CRC32：`D0B65355`；APP BIN SHA256：
`d88c57cb5be74f9b08852f0cc1cc1aa57e2c71fc00d4a9c79e1047466412f4f0`。
整片烧录后及两次复位后的 Flash SHA256：
`0a4f55d7323d1c59d67138aa2f9f727454d199a1f4e668f0627d0a482b0b80ab`。

## 附加通信复验

USB-CAN `USBCANFD202609107432`，物理通道 2（SDK 通道 1），1 Mbit/s、标准经典帧。
节点查询 `0x109` 及随后 5 次版本查询 `0x16F` 均未收到回复，不能记为通信通过。
适配器错误计数为 0；J-Link 同时确认 CPU 运行、时基递增、版本 `0x00070101`、mode 0、故障 0、三相关闭。
此结果只说明本次外部通信复验未通过；尚未定位其原因，也不以此前通过记录替代。

## 证据文件

- [构建产物与版本清单](../../outputs/current_offset_boot_20261109/build/Vector_Mini_ST.boot.json)
- [下载回读验证](../../outputs/current_offset_boot_20261109/device/download.json)
- [两次启动测试结果](../../outputs/current_offset_boot_20261109/device/startup_validation.json)
- [启动采样 CSV](../../outputs/current_offset_boot_20261109/device/startup_samples.csv)
- [离线回归目录](../../outputs/current_offset_boot_20261109/regression_final)
- [CAN 首次复验](../../outputs/current_offset_boot_20261109/can/result.json)
- [CAN 重试与错误计数](../../outputs/current_offset_boot_20261109/can/retry.json)
- [J-Link 最终运行状态](../../outputs/current_offset_boot_20261109/can/jlink_state.json)

`outputs/` 为本机产物，不随源码自动推送。原始 Flash 和参数备份保存在同一 `device/` 目录。
