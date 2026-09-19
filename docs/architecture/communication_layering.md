# 通信分层说明（CAN 接入）

2026-09-19。本文将 `firmware/communication/` 的现有内容按三个层次归纳：**协议约定**、
**协议实现**、**与电机/服务/应用的耦合**。本文件只描述现状与边界，不改变任何实现；
落地分层需要另立行为保持重构计划（见文末）。

## 0. 结论速览

- 固件当前只实现**一套**线路协议：11 位标准帧参数协议 + 48 字节大端状态流。
- `docs/protocols/` 中的公司 CAN FD 1.2.3（29 位扩展帧、CRC8/CRC16、分片、固件升级）
  是**设计稿，固件未实现**，不要与已实现协议混淆。
- `firmware/communication/` 只有 5 个文件；协议"约定"与"实现"目前混在
  `interface_can.c` 内，电机耦合也集中在该文件的命令派发与状态快照里。

```text
① 协议约定层（"字节长什么样"，与 MCU、电机无关）
   interface_can.h:7-117        CAN_PARAM_ID 命令/参数注册表 0x00..0x6F
   can_parameter_format.h:9     CAN_PARAMETER_FORMAT_REVISION = 2
   can_motor_status.h:8-14      状态流常量 0x64/0x65、0x7F0、48B、扩展版本 1
   motor_status.h:10-19         MotorStatus 载荷结构（SI 单位）
   motor/data_type.h            线上枚举 ModeNow_TypeDef / ErrorNow_TypeDef
   interface_can.c:31-105       线路编码等级 + 读/写编码映射（事实约定，住在实现文件里）
   docs/protocols/*、tools/bench/*.py、tests 黄金向量
        │  引用
        ▼
② 协议实现层（编解码、调度、传输机制）
   interface_can.c              FDCAN 过滤/中断/发送 + 大端编解码 + 命令派发
   can_motor_status.c           纯编码 + 周期调度（无 HAL、无电机）
   platform/api/comm_hw.h       传输能力契约
   ports/comm/comm_status_stm32g4.c   STM32G4 HAL 移植
        │  直接读写全局量 / 调用
        ▼
③ 耦合层（与电机服务、电机控制、应用调度的胶水）
   interface_can.c:293-713      CAN_ReceiveMessage_Update 直接写 MotorControl/FOC/…
   interface_can.c:828-860      CAN_BuildMotorStatusSnapshot 直接读电机状态
   interface_can.c:207-228      CAN_DisConnect_Handle 心跳看门狗 → Set_ErrorNow
   telemetry/motor_status.c     状态邮箱（当前唯一生产者是通信自身）
   app: board_config.c / bsp_task.c / main.c / stm32g4xx_it.c   集成与调度
```

---

## 1. 协议约定层

这一层描述线路格式本身，与硬件、电机实现无关。**它是跨版本兼容的判定依据**。

| 契约项 | 位置 | 内容 |
| --- | --- | --- |
| 命令/参数注册表 | `firmware/communication/can/interface_can.h:7-117` | `CAN_PARAM_ID` 0x00–0x6F：模式/电流/速度/位置、节点与极对数、编码器、限幅与增益、母线/相电流、dq、温度、摩擦结果 0x58–0x62、状态流 0x64/0x65、协议版本 0x67、齿槽 0x68–0x6D、温度源/有效位 0x6E/0x6F。0x6B/0x6C 的 Q15 与安培满量程在注释里约定。 |
| 参数格式版本 | `firmware/communication/protocol/can_parameter_format.h:9` | 只读 0x67 返回 `CAN_PARAMETER_FORMAT_REVISION = 2`：位置 int32 毫弧度、速度/加速度 int32 百分一、电流 int16 毫安、状态 48 字节；未知版本不得自动回退遗留格式。 |
| 状态流常量 | `firmware/communication/protocol/can_motor_status.h:8-14` | 命令 0x64、回复 0x65、ID 基址 0x7F0、负载 48 字节、offset 38 扩展版本 1（offset 36 有符号毫安母线电流）。`CAN_MOTOR_STATUS_COMMAND/REPLY` 与 `interface_can.h` 的 `CAN_SET/GET_STATUS_STREAM` 是重复定义，实际生效的是后者。 |
| 状态载荷模式 | `firmware/services/telemetry/motor_status.h:10-19` | fault、mode + 位置/速度/电流的 target–feedback–planned + 温度/母线电压/母线电流，全部 SI 单位；`current_*` 指 q 轴电流而非母线电流。 |
| 线上枚举 | `firmware/motor/data_type.h` | `ModeNow_TypeDef`（0..19，18 = `Position_Impedance_Mode`）与 `ErrorNow_TypeDef`（0..17，4 = `CAN_DisConnect`）直接作为线路数值。 |
| 线路编码等级 | `firmware/communication/can/interface_can.c:31-37` | `CanValueEncoding`：`float32` / `milli-i32` / `centi-i32` / `milli-i16`。 |
| 写命令编码映射 | `firmware/communication/can/interface_can.c:44-65` | 电流族（0x02/0x0E/0x10）→ int16 毫安；0x06 位置 → int32 毫弧度；速度族（0x04/0x12/0x14/0x16/0x1C/0x1E/0x20）→ int32 百分一；其余遗留 float32。 |
| 回复编码映射 | `firmware/communication/can/interface_can.c:72-105` | 电流族与摩擦结果 → int16 毫安；0x07/0x41 → int32 毫弧度；速度族 → int32 百分一；其余 float32。 |
| 比例、哨兵、字节序 | `interface_can.c:112-159`、`can_motor_status.c:39-80` | ×1000 / ×100，截断并饱和；非有限值 → `INT_MIN`/`INT16_MIN` 哨兵（接收端拒收）；状态帧与回复一律**大端**。 |
| CAN ID 编址 | `interface_can.c:775-777, 887`、`can_motor_status.c:128` | 命令 `node_id << 8 \| param_id`；状态 `0x7F0 + node`；node 0..7，过滤器按 `[node<<8, node<<8+0xFF]` 范围接收。 |
| 传输能力契约 | `firmware/platform/api/comm_hw.h:7-30` | `CommHwCanFrame` + 非阻塞收帧 + "仅发送队列空闲时才发状态帧"。这是通信实现与平台之间的能力边界。 |
| 设计稿（未实现） | `docs/protocols/motor_protocol_v1.md` 等 | 公司 CAN FD 1.2.3：magic 0xA55A、CRC8 多项式 0x9B / CRC16 0xBAAD、小端、类型 0–186、Q/R 会话、升级清单。固件无任何代码引用。 |
| 主机镜像 | `tools/bench/can_parameter_protocol.py`、`tools/bench/can_motor_status.py` | PC 侧同构编解码/发送模型，与固件共享同一约定（改协议必须同步）。 |
| 黄金向量 | `tests/unit/can_motor_status_test.c`、`tests/unit/test_can_motor_status.py`、`tests/unit/test_can_parameter_protocol.py`、`tests/unit/native/run_can_status_tests.py` | 48 字节状态帧、参数 SET/GET 编码、状态流命令的期望字节。 |

**边界观察**：其中"线路编码等级 + 读/写编码映射"物理上位于 `interface_can.c`（实现文件）。
按分层原则它们属于①，因此 `interface_can.c` 内部同时存在①与②的内容，是当前最明显的混层点。

---

## 2. 协议实现层

### 2a. 传输 / 硬件机制

| 机制 | 位置 | 说明 |
| --- | --- | --- |
| 过滤器、启动、中断使能 | `interface_can.c:165-202` | 直接使用 `HAL_FDCAN_ConfigFilter / ConfigGlobalFilter / Start / ActivateNotification` 与 `hfdcan1`。 |
| 波特率切换 | `interface_can.c:242-276` | `HAL_FDCAN_Stop / Init / Start`，由 1 kHz 任务驱动。 |
| 应答发送 | `interface_can.c:866-907` | `HAL_FDCAN_AddMessageToTxFifoQ`，标准帧 FD + BRS，最多重试 5 次。 |
| 接收取帧 | `interface_can.c:756-772` | 经 `comm_hw_can_receive` 取帧，校验非扩展/非远程、长度 2/4、ID ≤ 0x7FF 且不落在状态 ID 区间。 |
| 传输能力与移植 | `platform/api/comm_hw.h`、`ports/comm/comm_status_stm32g4.c:4-36` | 平台能力 + STM32G4 端口（`HAL_FDCAN_GetRxMessage`、DLC→长度映射、48 字节状态 DLC、发送前检查 TX 队列为空）。这是**正确的边界形态**。 |

### 2b. 编解码与调度（纯逻辑）

| 机制 | 位置 | 说明 |
| --- | --- | --- |
| 状态流编码 | `can_motor_status.c:81-99` | 48 字节大端定长布局，纯函数：不读时钟、不碰硬件、不建调度状态。 |
| 状态流周期调度 | `can_motor_status.c:100-130` | 32 位时间回绕安全；错过多个周期合并为最新一帧；构建期间收到命令则放弃本帧；命令由 `Configure`（19-33）原子更新。 |
| 回复编码 | `interface_can.c:720-750` | 按编码等级写 2/4 字节大端。 |
| 心跳判据 | `interface_can.c:207-228` | 状态机本身属②，但读取 `MotorControl` 并触发故障，归入③。 |

`can_motor_status.c` 是全模块**唯一完全干净**的协议实现：不含 HAL、不含电机头文件，只依赖
`services/telemetry/motor_status.h` 的载荷结构。

---

## 3. 耦合层

这一层是"通信 ↔ 电机/服务/应用"的胶水，也是分层的重点隔离对象。

| 耦合点 | 位置 | 方向 | 内容 |
| --- | --- | --- | --- |
| 命令派发 switch | `interface_can.c:293-713` | 通信 → 电机 | 直接写 `MotorControl.*`、`FOC.*`；调用 `ModeSwitch_Handle`、`Set_ErrorNow`、`Param_SetSpeedLimit`、`Encoder_*`、`FocFrictionIdentification_*`、`FocCogging_*`、`McuTemperature`。所依赖的 `MotorControl / ModeLast / FOC / OnBoard_Encoder` 以 `extern` 全局量导入（25-28）。 |
| 状态快照 | `interface_can.c:828-860` | 通信 → 电机 | 直接读 `MotorControl`、`FOC`、`Encoder_GetMecPos/Vel`，并按当前模式挑选 planned 字段。 |
| 心跳断连保护 | `interface_can.c:207-228` | 通信 → 电机/保护 | 读 `MotorControl.ModeNow / ErrorNow`；超时后 `Set_ErrorNow(CAN_DisConnect)`；使能条件限定在电流/速度/位置/阻抗模式。 |
| 状态邮箱 | `services/telemetry/motor_status.c:11-52` | 服务 ↔ 通信 | 无锁单生产者/单消费者邮箱（EMPTY/REQUESTED/READY）。但**唯一生产者**是 `interface_can.c:872-876`（前台 `CAN_SendMessage`），电机侧代码从不调用 `MotorStatus_Request/Publish`，因此它目前实际上是通信模块的私有缓冲，而不是电机主动上报的缝。 |
| 共享全局状态 | `interface_can.c:23` | 跨上下文 | `CANMsg`（node_id、baudrate、心跳设置/计数、收发暂存）被 RX 中断、1 kHz 任务、主循环共同读写。 |
| 应用集成 | `board_config.c:52`、`bsp_task.c:73-79`、`main.c:121`、`stm32g4xx_it.c:339-345` | 应用 → 通信 | 初始化 `FDCAN1_Param_Init`；1 kHz 做波特率切换与心跳；主循环调 `CAN_SendMessage`；CubeMX 回调 `HAL_FDCAN_RxFifo0Callback` 进 `CANRxIRQHandler`。 |

**两个必须记录的现状事实**：

- `CAN_SetEncoderState`（`interface_can.c:234-237`）是空实现，`CAN_SET_ENCODER_STATE`(0x0C) 当前不改变任何状态。
- `CANRxIRQHandler`（756-822）在 RX 中断里完成了取帧、校验、解码和完整命令派发，超出
  `ARCHITECTURE.md:60-61`"ISR 只做有界采集"的要求。

---

## 4. 依赖方向与规则对照

目标方向（`ARCHITECTURE.md:15-26`）：

```text
common/types  →  platform API + motor algorithms  →  services + protocol + communication  →  app
```

相关规则（`ARCHITECTURE.md:42-63`）：

- `motor/`、`services/`、`communication/` 不得暴露 STM32 HAL 类型、外设句柄、寄存器或厂商头文件；
  硬件细节必须经 `platform/api/`。
- 通信在边界校验线路数据后**调用显式服务**，而不是直接改写别的模块的全局量。
- 可变状态单一归属；ISR 只做有界采集，编码/调度放前台。

现状差距已被架构棘轮登记（`tools/harness/architecture_debt.json:5-8`）：

| 已登记越界 include | 违反的规则 |
| --- | --- |
| `interface_can.c → platform/stm32g4/bsp/delay.h` | 未经 `platform/api` |
| `interface_can.c → platform/stm32g4/bsp/hw_conf.h` | 未经 `platform/api` |
| `interface_can.c → platform/stm32g4/cubemx/Core/Inc/fdcan.h` | 暴露 HAL 句柄 |
| `interface_can.h → platform/stm32g4/cubemx/Core/Inc/main.h` | 头文件暴露 CubeMX 类型 |

注意：架构检查器只按 include 判定违规；"直接写别模块全局量"和"ISR 内完整派发"不在
自动判定范围，但仍属 `ARCHITECTURE.md:33-38、54-63` 的运行边界要求，本文件按现状如实标注。

`can_motor_status.h → services/telemetry/motor_status.h` 属同层（services + protocol）引用，
未计入债务，是当前唯一一处"协议 → 服务类型"的显式依赖。

---

## 5. 文件归属总表

| 文件 | 归属层 | 说明 |
| --- | --- | --- |
| `communication/can/interface_can.h` | ① + ② | 枚举 = 约定；结构体与函数声明 = 实现。 |
| `communication/can/interface_can.c` | ② + ③ | 传输/编解码 + 大量电机耦合，混层最重。 |
| `communication/protocol/can_parameter_format.h` | ① | 版本常量。 |
| `communication/protocol/can_motor_status.h` | ① | 线路常量 + 实现 API 声明。 |
| `communication/protocol/can_motor_status.c` | ② | 纯编码 + 周期调度。 |
| `services/telemetry/motor_status.h` | ① | 载荷结构（约定）。 |
| `services/telemetry/motor_status.c` | ③ | 状态邮箱（耦合缝）。 |
| `platform/api/comm_hw.h` | ①（能力契约）+ ② | 平台能力接口。 |
| `platform/stm32g4/ports/comm/comm_status_stm32g4.c` | ② | HAL 移植。 |
| `docs/protocols/*`、`tools/bench/can_*.py`、`tests` 向量 | ① | 设计稿、主机镜像、黄金向量。 |

---

## 6. 使用方式与后续

- 本文件是**边界参考**，不是重构计划，未改动任何实现。
- 若要按此分层落地代码（拆分 `interface_can.c`、把电机访问改为显式服务接口、把派发移出 ISR），
  属于行为保持重构，必须按 `docs/plans/README.md` 另立计划，记录：结构问题与改进目标、
  不变的外部行为（协议/参数 ABI、默认值、状态迁移、故障行为、控制输出、时序、栈、ROM/RAM）、
  重构前基线与黄金向量、分步迁移与回滚点、以及最终证据。
- 文档类改动只需 `verify --profile quick`；触及固件逻辑时按 `AGENTS.md` 升到 `pr` 或 `release`。
