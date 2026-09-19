# CAN 接收中断瘦身（S5）设计与验收计划 v1.0

日期：2026-09-19。状态：**立项，待授权实施**（用户已裁决 S5 执行，但按约定必须先完成本
最坏时延分析与台架授权，本次不动代码）。
依据：[通信分层优化设计](2026-09-19-communication-layering-optimization.md) §6 S5 与 §8 D3、
[通信分层说明](../../architecture/communication_layering.md) §3 已接受的偏差。

## 1. 意图

把 `CANRxIRQHandler` 的"中断内完整命令派发"改为**有界采集 + 有界队列 + 前台派发**，
使中断只做有界采集（满足 `ARCHITECTURE.md` 的 ISR 规则），同时保持：

- 线协议、编号、编码、48 字节状态流逐位不变；
- 全部公共签名与调用方不变（`CANRxIRQHandler`、`CAN_ReceiveMessage_Update` 等）；
- 心跳存活语义不变（`can_rx_en`/`can_hb_count` 仍在中断内刷新）。

## 2. 现状事实（本次实测证据）

| 事实 | 证据 |
| --- | --- |
| 中断优先级（数值越小越高）：`ADC1_2`(20 kHz FOC)=0、`FDCAN1_IT0`(CAN RX)=1、`TIM7`(1 kHz)=3、`DMA1_Channel1`=5 | `firmware/platform/stm32g4/cubemx/Core/Src/fdcan.c:104`、`adc.c:271,317`、`tim.c:263`、`dma.c:48` |
| CAN 中断入口：`HAL_FDCAN_RxFifo0Callback` → `CANRxIRQHandler()`，即派发全程在 `FDCAN1_IT0_IRQHandler` 内 | `stm32g4xx_it.c:342-350`、`firmware/communication/can/can_command_binding.c` |
| 快环 20 kHz（50 µs 周期）在注入转换回调内 | `stm32g4xx_it.c:333-342`、`firmware/app/foc_task.c:22` |
| ISR 内工作量：取帧 + 帧级过滤 + ≤4 字节解码 + 两级 `switch`（写 28 case / 读 60 case）+ 命中 case 体，其中 `CAN_SET_POS`/`CAN_SET_CURRENT`/`CAN_SET_SPEED` 会调用 `ModeSwitch_Handle` | `can_command_binding.c:37-95`、`can_binding_commands.c`、`can_binding_queries.c` |
| 主循环最坏迭代：`CAN_SendMessage()` → `FocCogging_Service()` → 若 `Save_Param` 则 `__disable_irq(); param_store_save(); __enable_irq();` | `firmware/platform/stm32g4/cubemx/Core/Src/main.c:122-147` |
| **Flash 保存期间全局中断关闭**，同时阻塞 FOC 快环与 CAN 中断 | 同上（`main.c:135-139`） |
| 原子性事实：`MotorControl.iqRef` 由快环与命令路径双写；`current_limit` 由命令写、快环读并用于限幅 | 快环 `firmware/app/foc_run.c:341,415,568,642`；命令 `can_binding_commands.c:45,107,108` |

**由优先级得出的关键结论**：`CAN_SET_CURRENT_LIMIT` 现在的"写 `current_limit` + 钳位 `iqRef`"
之所以安全，是因为命令运行在 prio 1、快环在 prio 0——**快环无法抢占命令**，成对更新对快环原子。
这条性质一旦把派发搬到前台就会消失。

## 3. 不能"简单外移"的设计约束

1. **原子性**：前台派发可被 20 kHz 快环抢占 ⇒ 成对/多字段更新会撕裂（例如先写
   `current_limit`、未及钳位 `iqRef` 就被快环读取），属控制安全风险。因此每个命令体的应用
   必须包在一段**短临界区**内，恢复对快环的原子性。
2. **控制相位**：命令生效时刻从"中断内即时"变为"前台某一拍"，对控制环路相位对齐有实际影响，
   必须实机回归（不可用离线夹具替代）。
3. **新的丢帧点**：队列满成为新的丢弃位置，必须**有界、可观测（计数）、不静默**。
4. **心跳必须在中断内**：`can_rx_en = true; can_hb_count = 0;` 是存活信号，留在中断。

## 4. 设计（候选方案）

```text
ISR (FDCAN1_IT0, prio 1)                      前台 (main loop / CAN_ServiceCommands)
  取帧 → 帧级过滤 → 解码(≤4B)                   出队 → 短临界区 { CAN_ReceiveMessage_Update }
  刷新心跳(can_rx_en=1, can_hb_count=0)               ↑
  入队 {param_id, data}  ──── SPSC 定长环(8) ─────────┘
  队满：丢弃最新 + drop_count++（只读计数）
```

- **队列**：单生产者（ISR）/单消费者（前台）定长环，容量 8；元素 `{CAN_PARAM_ID, float}`；
  `volatile` 头尾索引，无锁、不等待、不分配。
- **中断侧**：保持现有取帧/过滤/解码/心跳逻辑，仅把"派发"替换为"入队"。
- **前台侧**：新增 `CAN_ServiceCommands()`，在 `main.c` 循环入口（`CAN_SendMessage()` 之前）调用；
  出队后仍走既有 `CAN_ReceiveMessage_Update`（握手与路由不变），**每个元素的应用包在短临界区内**。
- **索引类查询**（`CAN_GET_COGGING_POINT` 的非法索引拒绝）保持在入队前判定，避免把拒绝路径
  也带进队列。
- **可观测**：新增只读 `CAN_GetQueueDropCount()`（契约 + 中文 Doxygen），供台架与诊断读取。

## 5. 会变化的可观测面（必须回归，不得声称"零行为变化"）

1. 命令生效时刻（控制相位）由"中断即时"变为"前台某拍"；
2. 新增队列满丢帧路径与计数；
3. ISR 时长下降、主循环单次迭代时长上升。

## 6. 验收

**离线（可在本机完成）**
- 新增原生夹具：FIFO 顺序、队满丢帧与计数、心跳仍在 ISR 刷新、非法索引不入队；
- 差分等价：同一命令序列在"旧直派"与"新队列派发"两条路径下，`MotorControl`/`FOC`/参数状态
  的**最终值逐位一致**（复用 run-state 差分的做法）；
- `check_architecture` / `check_interfaces` / `check_project_layout` 0 新增；PR 档全绿；
- Keil 双目标 0 Error / 0 Warning，Code/ZI 变化逐项记录。

**实机（需用户授权 + 显式台架上下文，当前不可执行）**
- CAN 命令表回归：模式切换、使能、禁用、设定值、限幅、状态流 0x64/0x65、协议版本 0x67、
  心跳断连与恢复矩阵；
- 时延测量：`CANRxIRQHandler` 入口 → 命令生效（GPIO 翻转 / RTT 时间戳），给出 p50/p99/max；
  以及主循环最坏迭代时长；
- 故障注入：队列满、Flash 保存期间并发命令、断连后恢复；
- 高速率压力：总线满负载下的丢帧计数与恢复行为。

## 7. 回滚

单提交 revert；队列、前台服务与计数接口均为新增符号，回滚无残留数据或协议痕迹。

## 8. 待裁决（实施前必须确认）

- **D5-a**：是否为"模式/禁用类命令"保留中断内快速路径？若保留，须先证明 `ModeSwitch_Handle`
  工作量有界（`foc_errhandle.c:100` 起）及其副作用集合。
- **D5-b**：前台消费点放主循环入口，还是并入 1 kHz 任务（`BSP1kHzIRQHandler`）？后者时延更确定，
  但仍在中断上下文，需权衡与本计划目标的一致性。
- **D5-c**：实机 CAN 回归窗口、台架配置与场景（可复用 `tools/bench/canfd_diagnostics.py`、
  `tools/bench/dual_axis_can_test.py`）。
- **D5-d**：队列容量与队满策略（丢弃最新 vs 丢弃最旧）——当前设计取"丢弃最新 + 计数"。

## 9. 证据与状态

- 未实施，未改动任何代码；本文档只记录分析结论与设计约束。
- 前置条件：D5-a..d 裁决 + 用户提供实机台架与 CAN 回归授权。
