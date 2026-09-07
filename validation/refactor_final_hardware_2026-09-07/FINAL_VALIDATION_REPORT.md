# 重构固件最终实机验证报告

日期：2026-09-07

分支：`codex/production-firmware-refactor`

硬件：Vector Mini ST、28 V 母线、实验电源限流 5 A、约 1.5 Nm 阻尼器、TLE5012B、J-Link、USB CDC、USB-CAN

## 最终候选固件

- ARMCC/Keil 链接结果：Code 107580 B，RO-data 5180 B，RW-data 460 B，ZI-data 30844 B
- HEX SHA-256：`E157230A2A5347860D0AFE766A7DF0EC039060A561EDC5DDF620DA16BC0DEDDE`
- J-Link 下载：114688 B，Program & Verify 成功

## 验证结果

| 项目 | 结果 | 关键数据 |
| --- | --- | --- |
| CAN Classic 全协议 | PASS | 106/106；节点号/波特率切换、全部读写、非法帧、心跳停机与恢复、200 次压力读取、适配器错误计数均通过 |
| CAN 心跳安全 | PASS | 超时进入停机；恢复首帧清除 `CAN_DISCONNECTED`；未再误报 `POWER_STAGE` |
| USB CDC 只读 | PASS | `mode=0`、`error=0`、编码器在线、遥测可读 |
| USB CDC 写入与位置闭环 | PASS | `mode=18`，负向移动 0.01 圈；目标 1.0312 rad，最终 1.0340 rad，误差 -0.0028 rad；停机和参数恢复读回通过 |
| CAN 转速闭环 | PASS | 目标 ±0.10 r/s（±0.6283 rad/s）；稳态均值 +0.6665/-0.5854 rad/s |
| 带载电气边界 | PASS | 峰值相电流 1.7326 A；最大母线电流 0.3619 A；最低母线 27.7683 V；最高 MCU 内温度 38 °C |
| 20 kHz 快环 | PASS | 最大 7166 cycles；截止 8500 cycles；超时计数 0；最终滤波值 6286 cycles |
| 停机输出状态 | PASS | TIM1 CH1～CH3 主/互补输出关闭，仅 CH4 ADC 同步触发保留；活动/锁存故障集合均为 0 |
| Flash 持久化 | PASS | 重启后编码器标定标志 `0x0F`、摩擦模型有效、默认电流限值恢复为 6.00 A |

## 构建与测试说明

- 最终链接前重新编译了全部 17 个已修改的目标源文件，避免使用时间戳过期的增量对象。
- 主机测试脚本未运行，原因是当前 Windows 环境没有 clang/GCC/MSVC/Zig；对应通信恢复行为已由实机 CAN 全协议测试覆盖，目标源文件均通过 ARMCC 编译和链接。
- 温度使用 MCU 内部温度观测；按当前产品配置仅监视，不启用温度跳闸保护。

## 原始结果

- `final_candidate_can_full/can_protocol_results.csv`
- `final_candidate_can_full/can_protocol_results.json`
- `final_candidate_speed_0p10/speed_closed_loop_samples.csv`
- `final_candidate_speed_0p10/speed_closed_loop_summary.json`
