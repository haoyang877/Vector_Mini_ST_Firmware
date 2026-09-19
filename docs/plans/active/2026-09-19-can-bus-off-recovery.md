# CAN bus-off 自恢复 v1.0

日期：2026-09-19。状态：**已实施，离线验证通过；台架回归待补**。

## 1. 背景与实测证据（本次台架）

- 现象：适配器可打开、可发送，但驱动器始终不应答；两个独立客户端（本仓库验证过的
  `yg_host_static_cli` 与自建探针）、ch0/ch1、node 0–7、波特率 125k–5M、BRS 开关、
  适配器内部 120Ω 开关，全部 0 帧。
- J-Link 只读寄存器（STM32G431CB，只读、未复位、读完 `go`）：

| 寄存器 | 值 | 含义 |
| --- | --- | --- |
| CCCR | `0x00001301` | **INIT=1：控制器停在初始化态，未参与总线** |
| NBTP | `0x00090B03` | NBRP=10 / NTSEG1=12 / NTSEG2=4 → 仲裁段 1 Mbps（配置正确） |
| DBTP | `0x00090B30` | DBRP=10 / DTSEG1=12 / DTSEG2=4 → 数据段 1 Mbps（配置正确） |
| ECR | `0x001F00F8` | **TEC=248**（发送错误计数逼近 255）、REC=31 |
| PSR | `0x00002CE0` | **EW=1、BO=1 → bus-off**，ACT=0 |
| IE / ISER0 | `0x1` / bit21 | RX FIFO0 中断与 NVIC 均已使能（正确） |
| RXF0S | `0x0` | RX FIFO0 空 → 从未收到过帧 |

- 结论：**CAN 驱动配置无误，但缺少 bus-off 恢复**。触发链：CANH 脱落 → 无人 ACK →
  `AutoRetransmission=ENABLE` 下每帧失败 TEC+8 → 达 255 进 bus-off → M_CAN 置位
  `CCCR.INIT` → 控制器永久退出总线（既不收也不发），直到复位。
- 复位验证：J-Link `r` 复位后，`yg_host_static_cli` 立即恢复遥测
  （**节点 ID = 1**，`position_rad = 3.786`），确认根因与修复方向。

## 2. 变更（行为新增）

- `platform/api/comm_hw.h` 新增能力 `bool comm_hw_can_service_bus_off(void)`：
  检查 `PSR.BO` 或 `CCCR.INIT`，必要时重新上线；健康时返回 false 且不动硬件。
- `ports/comm/comm_control_stm32g4.c` 实现：置位时调用 `HAL_FDCAN_Start`（清 INIT 重回
  总线），失败按致命处理；不改变位时序、滤波器与中断配置。
- `communication/can/can_transport.c`：在既有 1 kHz 监督入口 `CAN_BaudRateSwitching`
  （每 100 拍调用一次）内按 10 拍限频（**约 1 Hz**）执行恢复，避免总线真断时反复抖动。

## 3. 不变行为

- 线协议、编号、编码、48 字节状态流、心跳语义、初始化与波特率切换序列均不变；
- 健康总线上的行为与耗时不变（仅多两次寄存器读，约 1 Hz 一次）；
- 恢复动作不重配滤波/时序/中断，因此不引入丢帧或重同步。

## 4. 验收

**离线（已完成）**
- `run_can_status_tests` 端口夹具新增断言：健康时 `comm_hw_can_service_bus_off()` 返回 false
  且不增加 `HAL_FDCAN_Start` 调用；`PSR.BO` 置位或 `CCCR.INIT` 置位时返回 true 并各调用一次
  `HAL_FDCAN_Start`。9 组 CAN 夹具全 PASS。
- PR 档全绿：`outputs/runs/20260919T151739722017Z-9cc22d36/summary.json`。
- Keil 双目标 **0 Error / 0 Warning**：Code=82148 / RO=4888 / RW=256 / ZI=31344。

**台架（待补）**
- 断开 CANL（或断对端供电）→ 观察驱动器进入 bus-off 后**自动恢复**（≤1 s 重新上线，
  只读 CLI 继续收到遥测）；插回后持续正常；
- 稳定性：连续插拔 5 次不应出现永久静默（复位前行为）。

## 5. 回滚

单提交 revert；新增能力为独立符号，回滚无残留。
