# CAN接收积压与回复FIFO修复 v0.7.1

后续实机结果：节点2已由用户下载v0.7.1，60秒静止通信测试达到49.986Hz，详见[实测报告](2026-10-27-can-v071-live-communication.md)。下文保留修复阶段的验证范围。

日期：2026-10-27。分支：`feature/roll-motion-telemetry`，基于 `c7e22ac`，本次修改未提交。
本轮只修改源码、运行离线测试与构建；没有连接板卡、烧录或启动运动。保留本轮开始时已有工程配置与未提交修改，初始差异另存 `outputs/can_queue_fix_20261027/before.diff`。

## 问题与证据边界

1126B主控侧的先前只读实测：节点1/2/3均为v0.7.0。主控v0.57.0将1/2的核心查询目标设为50Hz，3/4/5设为20Hz后，节点2有效位置反馈仍为5.29Hz，原始回复13.81Hz；4760/4760个可比较回复固定匹配倒数第三次查询。控制器错误及内核丢弃为0，但未读取电机驱动内部FIFO溢出计数。
原报告为 `C:/Users/haoya/.codex/worktrees/monitor-jog-release/motor_ctrl_api/outputs/feedback-rates-v0570/RESULT.md`。这些是修复前的测量，不能当作本版实机结果，也不足以证明全部根因。

源码确认：原HAL回调每次仅调用一次单帧分派。三槽FIFO在同次中断前积累三帧时，处理一帧后仍留两帧，而HAL已清新消息标志。回复端原来只有一个槽，直接增加RX循环会导致后续查询无法保存回复。
本轮将原生产回调放入新的三帧测试，断言如预期失败；替换成修复代码后通过。记录在 `outputs/can_queue_fix_20261027/rx_before_fix/result.log`。

## 修改

- `stm32g4xx_it.c`：每次IRQ最多取3帧，拒绝帧也计入预算；仍有积压则挂起FDCAN IRQ，不依赖下一条报文。HAL已清标志、不再回调时，由IRQ尾部续处理。同一IRQ中的HAL回调与尾部入口共享预算。代码位于CubeMX用户区域，ADC优先级未改。
- `interface_can.c`：8项单生产者/单消费者FIFO。每项独立保存回复ID、长度、载荷、经典/FD/BRS格式及UID/CRC状态。RX写完后用内存屏障发布写索引；前台在HAL接受后才释放读索引。队满拒绝新回复，不覆盖旧项。
- UID/CRC仍只在前台计算一次，TX忙保留原队头。STOP执行不依赖回复槽；BOOT仍按原维护状态机抢占普通回复。状态流仅在普通回复队列空闲时发送。
- 开启FIFO0消息丢失通知；增加峰值与预算耗尽诊断。原协议编码、过滤器、心跳准入、参数区、控制增益、模式与Loader契约保持不变。
- APP版本改为v0.7.1/build0，`0x6F`返回`00 07 01 00`，用于区分板上v0.7.0。版本向量随之更新。

## 诊断

通过Keil/J-Link符号读取 `CAN_Diagnostics`，不要在运行中手动清零。

| 字段 | 含义 |
| --- | --- |
| `rx_fifo_lost` | 硬件丢帧标志的回调事件数；多个丢帧可能合并，不能当精确丢帧总数 |
| `rx_peak_depth` | 观察到的硬件FIFO深度峰值，0～3 |
| `rx_budget_exhausted` | 处理3帧后仍有积压并安排续调度的次数 |
| `reply_busy` | 8项回复FIFO已满的拒绝次数 |
| `reply_peak_depth` | 回复FIFO深度峰值，0～8 |
| `tx_busy` | 前台发送遇到硬件忙/失败的重试次数，不等于丢帧次数 |

事件计数达到UINT32_MAX后饱和。计数上电清零，保留原有字段顺序，新项附加到末尾。队列实时深度可由`can_reply_write - can_reply_read`的无符号差观察；运行中分开读取可能跨越更新，精确快照需在停机调试环境读取。

## 验证

- Python：56项unit、36项integration全部通过。
- 原生CAN回归：三种不同GET在不再注入请求后全部正确回复；消费期间新帧到达时单IRQ仍最多3帧；无RF0N的续调度、丢帧计数饱和通过。
- 回复FIFO：8项满队列、STOP、BOOT、忙重试、ID/数据配对、UID/CRC nonce及逐项格式冻结、发送和CRC计算期间RX抢占、32位索引回绕与重复复用通过。经典/FD及默认/外部参考四个组合通过。
- 既有协议、温度保护、热参数保存、Loader经典静默与维护部署回归通过。
- 既有120秒离散时间轮询模拟，在0～3ms发送忙条件下每种条件30000个查询均有回复；这是模拟时间结果，不能替代总线实时性或20kHz FOC时序实测。
- 四个Keil构建全部0错误、0警告，最终AXF的加载段只覆盖记录与APP，记录CRC和版本核对通过。

| 构建 | Code | RO | RW | ZI | APP CRC |
| --- | ---: | ---: | ---: | ---: | --- |
| 默认点到点/经典CAN | 90192 | 3116 | 444 | 24132 | 7A59F6D7 |
| HIL/经典CAN | 91952 | 3124 | 652 | 24172 | A639BC0E |
| 外部参考/经典CAN | 91740 | 3116 | 448 | 24208 | 37E187A7 |
| 外部参考/FD兼容 | 93552 | 3116 | 464 | 24208 | 0F6A5ED2 |

原生日志：`outputs/can_queue_fix_20261027/native/tests.log`；其他测试分别位于`python/`、`deployment/`、`thermal/`、`temperature/`。完整工件SHA256及源文件哈希见`outputs/can_queue_fix_20261027/summary.json`。

## 可烧录产物与后续实机检查

默认普通Keil工程仍为`firmware/platform/stm32g4/cubemx/MDK-ARM/Vector_Mini_ST.uvprojx`，可直接Rebuild后F8更新APP。已冻结的普通版文件：

- `outputs/can_queue_fix_20261027/default-classic/Vector_Mini_ST.axf`
- `outputs/can_queue_fix_20261027/default-classic/Vector_Mini_ST.hex`

HEX含0x08003000有效记录与0x08004000起的APP；不含Loader或0x0801C000起的参数/标定区。必须保留已有匹配Loader，使用Erase Sectors、Program、Verify，不使用全片擦除。纯APP BIN用于原Loader升级流程，不作为空板完整镜像。
其他变体分别归档到`hil-classic/`、`external-classic/`、`external-fd/`。同一v0.7.1可有不同编译变体，下载前须选择与当前控制方式匹配的构建；共线部署使用经典CAN版本。

下一步由用户决定是否烧录。先保存目标板Flash/参数并核验板卡身份；下载后确认版本和Loader/标定区保留，再按相同50Hz查询条件复测节点2，采集有效反馈率、回复对应关系和新增诊断计数。同时测量FOC时序，最后再做运动验证。本轮没有这些实机结果，不宣称节点2已恢复50Hz或已排除全部根因。
