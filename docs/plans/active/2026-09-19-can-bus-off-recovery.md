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

**台架（2026-09-19 完成，含烧录验证）**
- 烧录：J-Link 下载 `outputs/build/keil/Vector_Mini_ST/Vector_Mini_ST.hex`（245,580 B），
  `Program & Verify → O.K.`（Erase 0.959 s / Program 0.981 s / Verify 0.051 s），复位后运行。
- 复位后连通性：只读 GET（node 1）→ `adapter_online=true`、`telemetry_observed=true`、
  位置 3.786 rad、母线 27.86 V、温度 26.61 °C、mode/fault = 0/0、4 s 内 76 帧 RX。
- **bus-off 自恢复回归（软件注入"无人 ACK"）**：
  | 步骤 | 条件 | 结果 |
  | --- | --- | --- |
  | A | 正常模式，发 `0x64=20` 启流，听 2 s | **40 帧**（20 Hz × 2 s，正在推 48 B 状态流） |
  | B | **listen-only（不 ACK）10 s** → 制造 bus-off | **18,625 帧**：10 s 内**持续**有帧，未出现永久静默 |
  | C | 切回正常模式 3 s | **62 帧**（≈20 Hz × 3 s），**无需复位即恢复** |
  - 判据说明：修复前同一条件（无 ACK）会让控制器锁在 bus-off/INIT 直到复位——已由本次
    J-Link 寄存器实测（CCCR.INIT=1、PSR.BO=1、TEC=248、RXF0S=0）证实；本次 B 步在 10 s 内
    持续出帧，即为**自恢复生效**的直接证据（D 步 CLI 因 revision/profile 不匹配拒绝发送，
    因此未做 STOP）。
- 遗留：本次为制造无 ACK，探针以 listen-only 观察，驱动器状态流仍保持 20 Hz 开启
  （未下发 `0x64=0`）；如需关闭请复位或由上位机停流。

## 5. 回滚

单提交 revert；新增能力为独立符号，回滚无残留。
