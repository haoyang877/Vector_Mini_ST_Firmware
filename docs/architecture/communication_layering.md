# 通信分层说明（CAN 接入）

2026-09-19（v2，落地后）。本文描述 `firmware/communication/` 的三层划分，以及**已实施**的
文件边界与机械门禁。落地过程与验收见
[通信分层优化设计](../plans/active/2026-09-19-communication-layering-optimization.md)；
硬件域定则见 [硬件边界收口](../plans/active/2026-09-19-hardware-boundary-closure.md)。

## 0. 结论速览

- 固件当前只实现**一套**线路协议：11 位标准帧参数协议 + 48 字节大端状态流。
- `docs/protocols/` 中的公司 CAN FD 1.2.3（29 位扩展帧、CRC8/CRC16、分片、固件升级）
  是**设计稿，固件未实现**，不要与已实现协议混淆。
- 三层已落到独立文件：**协议约定**（`protocol/can_parameter_wire.*`）、
  **协议实现**（`can/can_transport`、`can/can_command_binding`、`protocol/can_motor_status`）、
  **耦合**（`can/can_binding_commands`、`can/can_binding_queries`、`can/can_status_source`、
  门面 `can/interface_can`）。
- 通信层零硬件依赖：`HAL_FDCAN_*` 与 `hfdcan1` 全部在
  `platform/stm32g4/ports/comm/`，通信只调用 `platform/api/comm_hw.h` 契约。
- 唯一的跨层取值缝是 `services/parameters/param_comm_bridge.h`（节点身份与心跳超时的
  窄接口），它替换了原先两个隐藏 `extern CANMsg`。

```text
① 协议约定层（"字节长什么样"，与 MCU、电机无关）
   protocol/can_parameter_wire.h      CAN_PARAM_ID 注册表 0x00..0x6F
   protocol/can_parameter_wire.{c}    编码等级 + 读/写映射 + 定点转换/哨兵 + ID/长度规则
   protocol/can_parameter_format.h    CAN_PARAMETER_FORMAT_REVISION = 2
   protocol/can_motor_status.h        状态流常量 0x7F0、48B、扩展版本 1
   services/telemetry/motor_status.h  MotorStatus 载荷结构（SI 单位）
   motor/data_type.h                  线上枚举 ModeNow_TypeDef / ErrorNow_TypeDef
   docs/protocols/*、tools/bench/*.py、tests 黄金向量
        │  引用
        ▼
② 协议实现层（编解码、调度、传输机制、派发）
   can/can_transport.{c,h}            波特率运行态 + 启停切换 + 帧级收帧过滤 + 发送重试
   can/can_command_binding.{c,h}      接收中断入口：取帧/校验/解码 + 前导握手 + 路由
   protocol/can_motor_status.c        纯编码 + 周期调度
   platform/api/comm_hw.h             传输能力契约
   ports/comm/comm_status_stm32g4.c   状态帧收发移植
   ports/comm/comm_control_stm32g4.c  滤波/启停/波特率/应答发送移植
        │  经显式接口调用
        ▼
③ 耦合层（与电机服务、电机控制、应用调度的胶水）
   can/can_binding_commands.{c,h}     写路径：模式/设定值/限幅/参数/会话命令
   can/can_binding_queries.{c,h}      读路径：状态/遥测/会话查询
   can/can_status_source.{c,h}        状态快照组装 + 心跳可见性判定
   can/interface_can.{c,h}            门面：CANMsg 运行态、应答暂存、心跳状态机、发送调度
   services/parameters/param_comm_bridge.h  节点身份/心跳超时的窄桥
   app: board_config.c / bsp_task.c / main.c / stm32g4xx_it.c   集成与调度
```

---

## 1. 协议约定层

这一层描述线路格式本身，与硬件、电机实现无关。**它是跨版本兼容的判定依据**。

| 契约项 | 位置 | 内容 |
| --- | --- | --- |
| 命令/参数注册表 | `communication/protocol/can_parameter_wire.h` | `CAN_PARAM_ID` 0x00–0x6F：模式/电流/速度/位置、节点与极对数、编码器、限幅与增益、母线/相电流、dq、温度、摩擦结果 0x58–0x62、状态流 0x64/0x65、协议版本 0x67、齿槽 0x68–0x6D、温度源/有效位 0x6E/0x6F。0x6B/0x6C 的 Q15 与安培满量程在注释里约定。 |
| 参数格式版本 | `communication/protocol/can_parameter_format.h` | 只读 0x67 返回 `CAN_PARAMETER_FORMAT_REVISION = 2`：位置 int32 毫弧度、速度/加速度 int32 百分一、电流 int16 毫安、状态 48 字节；未知版本不得自动回退遗留格式。 |
| 状态流常量 | `communication/protocol/can_motor_status.h` | ID 基址 0x7F0、负载 48 字节、offset 38 扩展版本 1（offset 36 有符号毫安母线电流）。命令 0x64/0x65 的线槽位由 `CAN_PARAM_ID` 单点定义，不再重复。 |
| 状态载荷模式 | `services/telemetry/motor_status.h` | fault、mode + 位置/速度/电流的 target–feedback–planned + 温度/母线电压/母线电流，全部 SI 单位；`current_*` 指 q 轴电流而非母线电流。 |
| 线上枚举 | `motor/data_type.h` | `ModeNow_TypeDef`（0..19，18 = `Position_Impedance_Mode`）与 `ErrorNow_TypeDef`（0..17，4 = `CAN_DisConnect`）直接作为线路数值。 |
| 线路编码等级与映射 | `communication/protocol/can_parameter_wire.{h,c}` | `CanValueEncoding`（float32 / milli-i32 / centi-i32 / milli-i16）与 `CanParamWire_CommandEncoding`/`CanParamWire_ReplyEncoding`；电流族 int16 毫安、位置 int32 毫弧度、速度族 int32 百分一，其余遗留 float32。 |
| 比例、哨兵、字节序 | `can_parameter_wire.c` | 单一实现 `CanParamWire_Milli32/Centi32/Milli16/Centi16`：×1000/×100，截断并饱和；非有限值 → `INT_MIN`/`INT16_MIN` 哨兵（接收端拒收）；状态帧与回复一律**大端**。 |
| CAN ID 编址与长度 | `can_parameter_wire.c` | `CanParamWire_Identifier(node, param)` = `node << 8 \| param`；`CanParamWire_Length`：int16 毫安 = 2，其余 = 4。状态帧 `0x7F0 + node`，node 0..7，滤波器按 `[node<<8, node<<8+0xFF]` 范围接收。 |
| 传输能力契约 | `platform/api/comm_hw.h` | 收帧、状态帧发送、启动/波特率切换/应答发送；全部"非阻塞、有界、无等待"。 |
| 设计稿（未实现） | `docs/protocols/motor_protocol_v1.md` 等 | 公司 CAN FD 1.2.3：magic 0xA55A、CRC8 多项式 0x9B / CRC16 0xBAAD、小端、类型 0–186、Q/R 会话、升级清单。固件无任何代码引用。 |
| 主机镜像 | `tools/bench/can_parameter_protocol.py`、`tools/bench/can_motor_status.py` | PC 侧同构编解码/发送模型，与固件共享同一约定（改协议必须同步）。 |
| 黄金向量 | `tests/unit/can_motor_status_test.c`、`tests/unit/test_can_motor_status.py`、`tests/unit/test_can_parameter_protocol.py`、`tests/unit/native/run_can_status_tests.py` | 48 字节状态帧、参数 SET/GET 编码、状态流命令的期望字节。 |

---

## 2. 协议实现层

### 2a. 传输机制与硬件移植

| 机制 | 位置 | 说明 |
| --- | --- | --- |
| 波特率运行态与切换 | `can/can_transport.c` | 模块持有当前/上次波特率；设置变化时才调用 `comm_hw_can_set_baudrate`，由 1 kHz 任务驱动。 |
| 帧级收帧与过滤 | `can/can_transport.c` | 经 `comm_hw_can_receive` 取帧并拒收扩展帧/远程帧/长度非 2,4/ID 越界/状态流 ID 区间。 |
| 应答发送与重试 | `can/can_transport.c` | 最多 5 次 `comm_hw_can_try_send_reply` 尝试，不等待；状态帧走队列空闲检查。 |
| FDCAN 滤波/启停/中断使能 | `ports/comm/comm_control_stm32g4.c` | `HAL_FDCAN_ConfigFilter/ConfigGlobalFilter/Start/ActivateNotification/Stop/Init` 与 `hfdcan1` 全在此文件。 |
| 应答与状态帧 HAL 提交 | `ports/comm/comm_control_stm32g4.c`、`comm_status_stm32g4.c` | TX Header 组装、DLC 映射、队列空闲检查。 |
| 传输能力契约 | `platform/api/comm_hw.h` | 能力 + 单位 + 失败语义，`firmware/platform/stm32g4/ports` 是实现域。 |

### 2b. 编解码与调度（纯逻辑）

| 机制 | 位置 | 说明 |
| --- | --- | --- |
| 线路编码/解码 | `can_parameter_wire.c`（约定层） + `can_command_binding.c`（解码入口） | 大端定长；写入侧经 `INBitToFloat`/定点还原，哨兵值直接拒收。 |
| 回复编码 | `interface_can.c: CAN_SendMessage_Update` | 按 `CanParamWire_ReplyEncoding` 写 2/4 字节大端。 |
| 状态流编码与调度 | `protocol/can_motor_status.c` | 48 字节大端纯函数 + 32 位时间回绕安全的周期调度；错过多个周期合并为最新一帧。 |

`can_motor_status.c` 不含 HAL、不含电机头文件，只依赖载荷结构与 `can_parameter_wire`。

---

## 3. 耦合层

这一层是"通信 ↔ 电机/服务/应用"的胶水。**写路径与读路径已物理分离**。

| 耦合点 | 位置 | 方向 | 内容 |
| --- | --- | --- | --- |
| 写路径命令 | `can/can_binding_commands.c` | 通信 → 电机 | `MotorControl.*`、`ModeSwitch_Handle`、`Param_SetSpeedLimit`、`Encoder_*`、`FocFrictionIdentification_ApplyCandidate`、`CoggingCompensation.request`、`CanTransport_SetBaudrate`。 |
| 读路径查询 | `can/can_binding_queries.c` | 电机 → 通信 | 只读 `MotorControl`/`FOC`/`Encoder_*`/`McuTemperature`/FocCogging/FocFriction 并暂存应答。 |
| 状态快照 | `can/can_status_source.c` | 通信 → 电机 | 前台按需组装 `MotorStatus`；按当前模式挑选 planned 字段；另提供只读的心跳可见性判定。 |
| 心跳断连保护 | `can/interface_can.c: CAN_DisConnect_Handle` | 通信 → 电机/保护 | 读模式可见性与心跳设置；超时后 `Set_ErrorNow(CAN_DisConnect)`；判据唯一实现见 `CAN_IsHeartbeatAlive`。 |
| 参数/心跳取值缝 | `services/parameters/param_comm_bridge.h` | 服务 ↔ 通信 | 只读/读写节点身份与心跳超时；替换原先 `foc_param.c`、`foc_run_state.c` 的隐藏 `extern CANMsg`。 |
| 状态邮箱 | `services/telemetry/motor_status.c` | 服务 ↔ 通信 | 无锁单生产者/单消费者邮箱；生产者仍是通信前台 `CAN_SendMessage`。 |
| 共享全局状态 | `can/interface_can.c: CANMsg` | 通信内部 | 节点身份、收发暂存、心跳状态；声明集中在 `interface_can.h`，**通信层之外不再有隐藏 `extern`**。 |
| 应用集成 | `board_config.c`、`bsp_task.c`、`main.c`、`stm32g4xx_it.c` | 应用 → 通信 | 初始化 `FDCAN1_Param_Init`；1 kHz 做波特率切换与心跳；主循环调 `CAN_SendMessage`；CubeMX 回调进 `CANRxIRQHandler`。 |

**两个必须记录的现状事实**：

- `CAN_SetEncoderState`（`can/can_binding_commands.c`）是空实现，`CAN_SET_ENCODER_STATE`(0x0C)
  当前不改变任何状态；槽位保留按设计决策 D1。
- `CANRxIRQHandler`（`can/can_command_binding.c`）在 RX 中断里完成取帧、校验、解码与完整
  命令派发，超出 `ARCHITECTURE.md`"ISR 只做有界采集"的要求。这是**已接受的偏差**
  （设计决策 D3/S5 默认不执行）：中断内工作量有界（单帧、≤4 字节解码、无等待/无分配），
  且模式/禁用类命令依赖最短时延；如 E 统一准入落地后再评估则另行立项。

---

## 4. 依赖方向与规则对照

目标方向（`ARCHITECTURE.md`）：

```text
common/types  →  platform API + motor algorithms  →  services + protocol + communication  →  app
```

允许的依赖边（`harness.toml` 的 `architecture.layers`，机械门禁）：

| 模块 | 允许依赖 |
| --- | --- |
| `communication/protocol` | common、motor、services、protocol |
| `communication/can` | common、platform_api、motor、services、protocol、communication |
| `platform/stm32g4/ports` | common、platform_api、motor、services、platform_stm32、cubemx |

相关规则：

- `motor/`、`services/`、`communication/` 不得暴露 STM32 HAL 类型、外设句柄、寄存器或厂商头文件；
  硬件细节必须经 `platform/api/`。**通信层已满足**（棘轮 `architecture_debt.json` 中
  communication 条目为 0）。
- 通信在边界校验线路数据后**调用显式服务**，而不是直接改写别的模块的全局量。
- 可变状态单一归属；ISR 只做有界采集（当前偏差见 §3）。

`can_motor_status.h → services/telemetry/motor_status.h` 属同层（services + protocol）引用，
未计入债务。

---

## 5. 文件归属总表

| 文件 | 归属层 | 说明 |
| --- | --- | --- |
| `communication/protocol/can_parameter_wire.{c,h}` | ①（+②的纯逻辑） | 注册表、编码映射、定点转换、ID/长度。 |
| `communication/protocol/can_parameter_format.h` | ① | 版本常量。 |
| `communication/protocol/can_motor_status.{c,h}` | ① + ② | 线路常量 + 纯编码/调度。 |
| `communication/can/can_transport.{c,h}` | ② | 波特率、帧级过滤、发送重试（仅 comm_hw）。 |
| `communication/can/can_command_binding.{c,h}` | ② + ③ | 接收中断入口与路由。 |
| `communication/can/can_binding_commands.{c,h}` | ③ | 写路径。 |
| `communication/can/can_binding_queries.{c,h}` | ③ | 读路径。 |
| `communication/can/can_status_source.{c,h}` | ③ | 状态快照与心跳可见性。 |
| `communication/can/interface_can.{c,h}` | ② + ③ | 门面：运行态、应答暂存、心跳状态机、发送调度。 |
| `services/parameters/param_comm_bridge.h` | ③ | 参数服务的窄取值桥。 |
| `services/telemetry/motor_status.h` / `.c` | ① / ③ | 载荷结构 / 状态邮箱。 |
| `platform/api/comm_hw.h` | ①（能力契约） | 传输能力接口。 |
| `platform/stm32g4/ports/comm/*.c` | ② | HAL 移植。 |
| `docs/protocols/*`、`tools/bench/can_*.py`、`tests` 向量 | ① | 设计稿、主机镜像、黄金向量。 |

---

## 6. 使用方式与后续

- 本文件是**当前边界参考**；重构过程与证据见
  [通信分层优化设计](../plans/active/2026-09-19-communication-layering-optimization.md)。
- 若继续推进：S5（RX 中断瘦身）与 D4（是否为 transport 单列 harness 允许面）尚未裁决，
  需用户决定后另行立项；任何进一步改动仍按行为保持重构记录基线与证据。
