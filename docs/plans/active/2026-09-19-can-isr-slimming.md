# CAN 接收中断瘦身（S5）设计与验收计划 v1.0

日期：2026-09-19。状态：**立项，待授权实施**（用户已裁决 S5 执行，但按约定必须先完成本
最坏时延分析与台架授权，本次不动代码）。
依据：[通信分层优化设计](2026-09-19-communication-layering-optimization.md) §6 S5 与 §8 D3、
[通信分层说明](../../architecture/communication_layering.md) §3 已接受的偏差。

## 1. 意图

按用户给出的通信时序（2026-09-19 澄清）重构收发路径：**中断只做有界采集 → 1 kHz 时基
派发与组包 → 1 kHz 时基发送**。

```text
FDCAN1_IT0 (prio 1)  取帧 → 帧级过滤 → 解码(≤4B) → 刷新心跳 → 入 RX ring
TIM7 1 kHz (prio 3)  ① 排空 RX ring：逐条派发（写路径/读路径，结果入 TX ring）
                     ② 排空 TX ring：发应答；空闲时按周期发 48 B 状态流
主循环               不再承担 CAN 收发（保留 FocCogging_Service 与保存会话）
```

本阶段（S5）的**核心**是把 RX 派发移出 CAN 中断；**TX ring 与 1 kHz 发送节拍**属同一次
重构的配套改动（用户设计的一部分），因此一并纳入本计划范围。

同时保持：

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
| 1 kHz tick 现有内容：`FOC1kHzSupervisor` + LED(每 200 拍) + RGB(每 50 拍) + `CAN_BaudRateSwitching`(每 100 拍) + `CAN_DisConnect_Handle`(每拍)，由 TIM7 中断（prio 3）调用 | `firmware/app/bsp_task.c:46-69`、`stm32g4xx_it.c:360-365` |
| TX 目前是**单槽**：`CAN_SendMessage_Update` 覆盖 `CANMsg.tx_param_id/tx_data_u8/tx_data_len` 并置 `can_tx_en`，由主循环 `CAN_SendMessage()` 发出后清零 | `interface_can.c`（`CAN_SendMessage_Update`/`CAN_SendMessage`） |
| ⇒ 由此得出**现状缺陷**：主循环未及发送时又到达一条查询，上一条应答被覆盖丢弃（突发应答丢失） | 同上（单槽无队列） |
| 状态流发送：主循环每次迭代调用 `CanMotorStatus_Prepare(time_hw_now_ms(), …)`，用 1 ms 相位累积限速，且只在无待发应答时提交 | `interface_can.c: CAN_SendMessage`、`protocol/can_motor_status.c` |

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

## 4. 设计（用户规格，2026-09-19）

```text
FDCAN1_IT0 (prio 1)                            TIM7 1 kHz (prio 3)
  取帧 → 帧级过滤 → 解码(≤4B)                     ① 排空 RX ring（有界）
  刷新心跳(can_rx_en=1, can_hb_count=0)             → 短临界区 { CAN_ReceiveMessage_Update }
  入 RX ring {param_id, data} ──────────────┐       → 组包入 TX ring {param_id, data}
  队满：丢弃最新 + rx_drop++                 │     ② 排空 TX ring（有界）
                                            └──►      → 发应答（FIFO）
                                                        → 无应答待发时按周期提交 48 B 状态流
主循环：移除 CAN_SendMessage()，只保留 FocCogging_Service 与保存会话
```

- **RX ring**：单生产者（CAN ISR）/单消费者（1 kHz tick）定长环，元素 `{CAN_PARAM_ID, float}`；
  `volatile` 头尾索引；无锁、不等待、不分配；满则丢弃最新并计数。
- **CAN 中断侧**：保持取帧/帧级过滤/解码/心跳刷新，仅把"派发"替换为"入队"；工作量有界。
- **TX ring**：生产者是 1 kHz 里的派发结果（`CAN_SendMessage_Update`），消费者是同一 tick 的
  发送段；FIFO 顺序；**应答优先于状态流**（保持现状"reply first"语义）；满则丢弃最新并计数。
- **发送节拍**：由主循环改为 1 kHz 固定节拍；状态流仍走 `CanMotorStatus_Prepare` 的 1 ms
  相位累积与"队列空闲才发"约束（`comm_hw_can_try_send_status`），不再依赖主循环迭代频率。
- **临界区**：1 kHz tick 在 prio 3，20 kHz 快环（prio 0）可抢占它，因此派发仍需**每命令短临界区**
  以恢复"命令对快环原子"（现状靠 prio 0/1 关系天然成立）。临界区预算必须远小于 50 µs 快环周期。
- **索引类查询**（`CAN_GET_COGGING_POINT` 非法索引拒绝）在派发前判定，不入队拒绝路径。
- **可观测**：新增只读 `CAN_GetRxDropCount()` / `CAN_GetTxDropCount()`（中文契约），供台架与诊断。
- **顺序约束**：`CAN_BaudRateSwitching`（每 100 拍）与 `CAN_DisConnect_Handle`（每拍）在 tick 内的
  调用次序需与现有语义核对（`SET_CAN_BR` 生效后最多 100 ms 切换，保持不变）。

## 5. 会变化的可观测面（必须回归，不得声称"零行为变化"）

1. 命令生效时刻（控制相位）由"CAN 中断即时"变为"1 kHz 拍沿"，最坏增加一个 1 kHz 周期；
2. **应答发送时机**由主循环改为 1 kHz（上位机看到的应答时延变为 ≤1 ms + 临界区/总线时间）；
3. **突发应答丢失被修复**：现状单槽覆盖会丢掉前一条应答，TX ring 后按 FIFO 全部发出
   （这是**行为变化**，需在回归中明确验证为改善且不破坏顺序/编号）；
4. 新增 RX/TX 队列满的丢帧路径与计数（有界、可观测、不静默）；
5. CAN 中断时长下降；1 kHz tick 与主循环单次迭代时长变化；
6. 主循环不再调用 `CAN_SendMessage()`（`main.c:128` 移除），状态流不再受主循环迭代频率影响。

## 6. 验收

**离线（可在本机完成）**
- 新增原生夹具：RX FIFO 顺序、RX 队满丢帧与计数、心跳仍在 CAN ISR 刷新、非法索引不入队；
- 新增原生夹具（TX）：应答 FIFO 顺序、**突发多条查询不再丢应答**（现状单槽会丢，作为回归基线）、
  应答优先于状态流、TX 队满计数；
- 1 kHz 节拍：在夹具中以显式 tick 驱动，验证状态流按 1 ms 相位累积与 `CanMotorStatus_Prepare`
  的既有速率语义不变；
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

## 8. 裁决记录与待裁决项（2026-09-19 用户答复）

| 编号 | 议题 | 结论 |
| --- | --- | --- |
| S5 实施方式 | 无台架时如何推进 | **等台架：先不写代码**。本阶段不产生固件变更，避免提交无法验证的控制时序改动。 |
| D5-b | 派发消费点 | **已按用户规格更正**：1 kHz 时基（TIM7 tick，prio 3），不再放主循环入口。 |
| D5-d | 队列容量与队满策略 | **容量 8 + 丢弃最新 + `drop_count` 计数**（设计默认）。 |
| D4（通信层收尾） | 是否为 `communication/transport` 单列 harness 允许面 | **暂不收紧**，保持现有 `communication/can` 允许面；后续如需再立项。 |
| D5-a | 是否保留中断内快速路径 | **未决**：用户同时选择了"不保留 ISR 快速路径"与"保留模式/禁用类快速路径"，两者对同一命令类互斥。 |

**D5-a 综合提案（实施前需确认其一）**：默认**全部命令统一入队**，仅为模式/禁用类
（`CAN_SET_MODE` 及依赖最短时延的禁用路径）保留中断内快速路径。该快速路径的**前提**是
先在实施分析中给出 `ModeSwitch_Handle`（`foc_errhandle.c:100` 起）及其副作用的**有界性证明**；
若证明不成立，则回到"全部入队"。

**D5-c（仍未决，属外部条件）**：实机 CAN 回归窗口、台架配置与场景
（可复用 `tools/bench/canfd_diagnostics.py`、`tools/bench/dual_axis_can_test.py`）。

**D5-e**：TX ring 与 1 kHz 发送节拍是否与 RX 一起纳入本阶段？（用户 2026-09-19 规格已包含，
默认**纳入**；若拆分实施，则先做 RX、TX 保持主循环，但需接受"应答时延仍受主循环最坏迭代影响"。）

**D5-f**：RX/TX ring 容量与溢出策略（用户规格未指定；设计默认 RX=8/TX=8、满则丢弃最新并计数）。

**D5-g**：命令临界区形式——短 `__disable_irq()`（等效现状"对快环原子"，但会连 20 kHz 快环一起
屏蔽，需严格预算）vs 以 BASEPRI 屏蔽到快环之上（不屏蔽快环则无法保证原子）。默认取前者并给预算。

**D5-h**：状态流是否也经 TX ring（与应答同队、按"应答优先"插入）vs 保持"无应答时直接提交"。
默认**同队但优先发应答**，以保持现状的优先序并让队满计数覆盖两种帧。

**D5-i**：`ARCHITECTURE.md`"ISR 只做有界采集、编码/调度放前台"的规则张力——本设计把派发与
组包放在 1 kHz 中断内（有界、定节拍）。用户规格明确接受该形态；需在实施时同步更新
`communication_layering.md` 的偏差记录。

## 9. 证据与状态

- **未实施，未改动任何固件代码**；本文档只记录分析结论、设计与裁决。
- 已就绪：D5-b、D5-d 裁决；D4 结论（不收紧）；分析证据（§2）。
- 阻塞项：D5-a 二选一确认 + D5-c 实机台架窗口。
- 实施入口（按用户规格更正）：
  `can_transport.{c,h}`（RX ring 入队 + TX ring 出队与发送节拍）、
  `can_command_binding.c`（改为从 RX ring 派发）、`interface_can.{c,h}`（TX ring 入队接口
  `CAN_SendMessage_Update` 语义调整 + `CAN_GetRxDropCount/CAN_GetTxDropCount` 契约 + 1 kHz 服务入口）、
  `bsp_task.c`（1 kHz tick 内追加 RX/TX 排空）、`main.c`（移除 `CAN_SendMessage()` 调用）、
  夹具 `run_can_status_tests.py`（RX/TX 队列与突发应答用例）与差分夹具。
- 用户规格澄清（2026-09-19）：RX 中断只入队；1 kHz tick 排空 RX、派发、组包入 TX ring 并发应答；
  发送同样按 1 kHz 节拍。已据此把 D5-b 由"主循环入口"更正为"1 kHz 时基"，并把 TX ring 纳入范围。
