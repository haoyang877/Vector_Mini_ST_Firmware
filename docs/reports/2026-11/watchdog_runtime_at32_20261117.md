# AT32F456 看门狗修复设备验证

日期：2026-11-17。适用当前节点 7、APP v1.8.26、Loader v2.3.4。
基线为 `2a7cbf52` 加本轮工作树，源码补丁及哈希归档于
`outputs/watchdog-device-20261117/manifest.json`，标记 `dirty=true`。

**结论：看门狗故障子场景通过；本轮整体验收为 PARTIAL，完整安全验收未通过。**
未发送电机运动命令。测试结束已通过 CAN 56 转入 Loader，确认六路 GPIO 低电平；
设备保留本轮 APP，未回滚。修复行为见[状态机契约](../../architecture/motor_state_machine_v1.md#运行期健康与看门狗)，
计划及范围见[设备验证记录](../../plans/completed/2026-11-17-watchdog-device-validation.md)。

## 下载与保护区

写入前核对设备并双读备份完整 256 KiB Flash。仅更新 APP 槽
`0x08004000..0x0801BFFF`；96,272 B 镜像逐字节回读一致，
APP 槽外所有字节与备份一致，包括 Loader、参数、身份及剩余物理 Flash。
本轮使用已激活、已尝试、已确认的 debug 镜像，不把此结果等同于 CAN 升级确认验证。

APP BIN SHA-256：
`cc2c8b4508bb7e4509ee6e1ab20510ab48acb98a6110b539d3cdcda21b2dac7f`。
下载证据为 `program-result.json`，结束后的保护区与 APP 复核见 `handoff-result.json`；
以上文件均位于 `outputs/watchdog-device-20261117/`。

## 看门狗与通信

设备正常经过 Loader 进入 APP。保留自然出现的过压故障，未清错或重新标定。
最终 APP 状态来自 `runtime-final.json`：

| 观察量 | 最终值 |
| --- | --- |
| 向量表 | `0x08004000`，仍在 APP |
| APP 运行时间 | 135,494 ms |
| 主状态 / 故障码 | FAULT / 8（过压） |
| 电流零偏 | 5 个采样，未完成 |
| 快环 / 监督 / 外环完成数 | 2,710,039 / 271,007 / 542,018 |
| 看门狗调试冻结 | 0 |
| CFSR / HFSR | 0 / 0 |
| 通信就绪 | 1 |

这些计数为顺序读取的运行态字段，不承诺同一采样瞬间。
14 次 CAN 查询成功，身份回复核对 APP v1.8.26 / Loader v2.3.4，
108 状态回复仍保留故障。已确认 APP 在零偏未完成的该故障下持续运行，
超过此前约 28.9 s 后进入 Loader 的现场；没有用调试冻结看门狗维持运行。

## 未通过的输出检查

计划的 120 s 定时观测因六路输出检查失败中断。后续只读诊断确认 APP
运行时间已达 135.5 s，但不能将它表述为完整 120 s 安全观测通过。

TMR1 `CCTRL=0`、`BRK=0x0C24`（主输出关闭），GPIOA 的上桥掩码为 0；
GPIOB 输入数据在 `0x7304/0x6304` 与 `0x7307/0x6307` 之间出现变化，
即 PB0/PB1 偶尔读为高电平。1 MHz、100 kHz SWD 及 CPU 暂停、定时器仍运行时均复现。
原始证据为 `observation-result.json`、`output-inspection.json`、
`gpio-live-comparison.json` 和 `gpio-halted-comparison.json`。

现有 BSP 关闭状态查询只核对相通道使能位，不能据此证明引脚持续为低。
本轮未确定引脚读回异常的来源，不把它解释为正常，不放宽六路输出验收条件。
结束时 CAN 56 交接成功，Loader 的输出检查确认六路 GPIO 低；
这仅证明结束状态，不消除 APP 运行期间的异常。

## 验证边界

本轮目标独立重建零错误、零警告。`release2` 检查证据为
`outputs/runs/watchdog-device-20261117-release2/summary.json`：
doctor、工程布局、架构、接口和卫生检查通过；非干净源码、已有 `main_ticks`
格式及入口抽取问题、120 条历史文档证据链接缺失使完整 release 未通过。
不宣称正式发布或整仓通过。

GPIO 异常与上电过压触发源仍待定位。未验证电机转动、带载波形、故障恢复后重新启相、
未确认镜像的实机退回、真实任务停滞或关相失败注入；后两类危险场景保持离线回归范围。
