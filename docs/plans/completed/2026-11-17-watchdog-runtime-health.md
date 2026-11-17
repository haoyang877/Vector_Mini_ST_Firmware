# 运行期看门狗与电机故障解耦

日期：2026-11-17。状态：源码修复和离线验证范围完成；已知整仓检查失败及未测范围见下文。

## 目标与基线

基线为 `the-system-refactor` 分支 `2a7cbf52` 及本次开始时的工作树。
已有 Keil 工作区、AT32 工程和 main、构建工具、相关测试与文档改动予以保留，
不覆盖或回退用户改动。适用 STM32G4 / AT32F456 配套 LoaderApp。

AT32F456 APP v1.8.25 的 J-Link 现场显示：上电过压使状态进入 FAULT，
电流零偏停在 5 个样本；快环与监督仍推进，但原基础采样健康判定持续失败，
最终看门狗复位后留在 Loader。现场证据位于 `outputs/hang-debug-20261117/`。
过压触发源尚未确定，不将本次修复解释为过压原因已解决。

本次将电机运行许可、运行期看门狗健康与新镜像确认资格分开。
已确认 APP 在普通故障下可靠关相且关键任务继续执行时维持通信和诊断；
任务停滞、非法状态或关相不可信仍停止喂狗。

## 行为与范围

- 状态机提供快环、外环与监督完成计数及运行期健康只读快照；
  AppLoader 每 100 ms 比较三项进展并检查通信栈就绪。
- 已确认 APP 的 IDLE、SELF_TEST、FAULT 及已锁存故障必须实际关相，
  不以零偏未完成或测量 NaN 单独否决运行期喂狗。
- 无故障的 RUNNING / CALIBRATION 仍要求基础采样有效；
  `fault_locked` 和非法状态不得通过运行期健康检查。
- 新镜像确认仍要求有效采样、无故障、持续健康窗口与实际关相；
  未确认镜像喂狗仍使用严格健康条件，保留 5 s 确认期限。

不改保护阈值、启相许可、CLEAR 恢复条件、零偏校准调度、协议或存储 ABI。
不增加硬件写入和电机运动命令，不自动下载新固件。

## 验收与证据

1. 原生回归覆盖早期过压且零偏无效、可靠关相时已确认 APP 可喂狗，
   但不能确认新镜像；保留故障锁存和运动禁止。
2. 快环、外环、监督任一停止推进，通信未就绪、非法状态或关相失败均不喂狗；
   比较计数保留无符号回绕行为。
3. 未确认镜像不因放宽运行期条件而通过严格健康窗口。
4. 执行 PR 档验证及两板配套 LoaderApp 构建，记录当前失败、资源变化与未测范围。

## 实施结果

状态机新增 `MotorRuntimeProgress` 与 `MotorStateMachine_ReadRuntimeHealth`，
由 AppLoader 比较三个任务的完成数；`ReadBootHealth` 保留严格采样要求，
并明确拒绝故障状态及已锁存电机故障。已确认镜像采用运行期健康喂狗，
未确认镜像仍采用严格健康，11 次确认窗口和 5 s 期限保留。
现行契约见[状态机设计](../../architecture/motor_state_machine_v1.md#运行期健康与看门狗)。

本次针对性原生回归全部通过：`outputs/tests/watchdog-loader/` 的 Loader 运行时子项、
`outputs/tests/watchdog-state/` 的双板状态机测试，以及
`outputs/tests/watchdog-orchestration/` 的双板、有无 Loader 四组合编排。
覆盖早期故障、零偏无效、可靠关相、三项任务停滞、无符号回绕及严格试运行门禁。

完整 PR 验证证据为 `outputs/runs/watchdog-runtime-20261117/summary.json`，总结果为 FAIL：

| 检查 | 当前结果与范围 |
| --- | --- |
| doctor、project-layout、architecture、interfaces、hygiene | PASS |
| format、lint | FAIL；用户既有 `projects/at32f456/application/main.c` 中 `main_ticks` 缩进不符合规则，未改动该处 |
| tests | 两项失败：Python harness 检查出 120 条历史文档指向缺失 outputs 证据；`run_position_servo_tests` 抽取 main 函数时未包含用户既有 `main_ticks` 声明，编译报未声明。其余原生套件通过 |
| docs | FAIL；同上述 120 条历史证据链接问题，本次新增文档无坏链接 |

两板配套 LoaderApp 均完成当前源码的 Keil 构建，均为 0 errors、0 warnings：

| 目标 | APP 镜像字节数 | 构建日志 |
| --- | --- | --- |
| AT32F456 | 96,272 B | `outputs/build/logs/ESC-APP-AT32F456.log` |
| STM32G431 | 82,888 B | `outputs/build/logs/ESC-APP-STM32G431-LoaderApp.log` |

AppLoader 因增加两个观测计数而多占 8 B RAM。AT32 静态栈预算仍为 3,496 / 4,096 B，
余量 600 B，未变化；该预算不是实机栈水位测量。

本次没有下载新固件、没有实机复测，也没有正式 release 验收。
过压触发源仍待定位；CLEAR 和零偏采样的原条件保持不变，
因此保留故障诊断不代表本次同时解决故障恢复或重新启动问题。
归档表示本计划的源码与离线修复范围结束，不将局部通过表述为整仓或硬件通过。

上述未下载、未实测描述对应源码修复阶段。后续同日已在节点 7 下载 v1.8.26，
看门狗子场景通过，完整安全验收因 GPIO 读回异常未通过；
详见[设备实测报告](../../reports/2026-11/watchdog_runtime_at32_20261117.md)。

## 回退

仅回退本计划涉及的状态机健康接口、AppLoader 判定、对应测试与契约文档，
恢复原 `ReadBootHealth` 喂狗路径；不得回退上述用户既有改动。
新固件尚未下载时，设备仍保留原镜像。
