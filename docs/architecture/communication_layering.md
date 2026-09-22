# CAN FD 通信分层说明

日期：2026-09-22。状态：CAN FD-only 运行入口已完成，旧标准 CAN 实现已从固件工程删除；参数、标定、完整运动控制命令仍按新协议目录逐项补齐。

## 1. 运行结论

固件只使用 `yg_protocol` CAN FD。线路固定为 29 位扩展 ID、FD+BRS、仲裁段 1 Mbit/s、数据段 5 Mbit/s。旧的 11 位标准 CAN 参数帧、48 字节状态流、旧心跳调度和旧波特率切换不再有源码或工程入口。

```text
FDCAN HAL
   │  CommHwCanFrame（ID、FD、BRS、DLC、数据）
   ▼
platform/api/comm_hw.h
   │  非阻塞收发能力
   ▼
communication/can/yg_protocol_canfd_irq.c
   │  RX 中断只复制 FD+BRS 扩展帧
   ▼
communication/can/yg_protocol_link.c
   │  固定队列、CRC、分片、序列号和响应调度
   ▼
communication/protocol/yg_protocol_*
   │  公司帧头、CAN ID、路由、只读服务、业务值对象
   ▼
电机/参数/标定服务
```

## 2. 启动和信息流

```text
Board_Init
  ├─ param_store_load
  ├─ MotorControl_Init
  ├─ board_hw_start
  ├─ YgProtocolRuntime_Init
  │    ├─ YgProtocolLink_Init
  │    └─ comm_hw_can_start_fd
  └─ YgProtocolStatusAdapter_Init（应用层绑定快照源）

FDCAN RX FIFO0 中断
  └─ YgProtocolCanfd_RxIrqHandler
       └─ YgProtocolLink_OnRxFrame（只入 RX 队列）

while(1)
  ├─ YgProtocolStatusAdapter_Refresh
  └─ YgProtocolRuntime_Service(now_ms)
       └─ YgProtocolLink_Service(now_ms)
       ├─ 取队列帧
       ├─ CRC/地址/长度/分片校验
       ├─ registry + router
       ├─ 调用业务服务
       └─ 发送 TX 队列中的 CAN FD 帧
```

ISR 不执行 CRC、分片、Flash、日志或电机业务。后台服务每次调用限制输入和输出处理数量，队列满时丢弃最新帧并保留统计。

## 3. 文件职责

| 文件 | 职责 |
| --- | --- |
| `firmware/platform/api/comm_hw.h` | CAN FD 收发能力契约，不含公司协议语义 |
| `firmware/platform/stm32g4/ports/comm/comm_control_stm32g4.c` | FDCAN 启动、滤波、通知和硬件恢复 |
| `firmware/platform/stm32g4/ports/comm/comm_status_stm32g4.c` | HAL 收帧、DLC 转换和扩展 FD 帧发送 |
| `communication/can/yg_protocol_runtime.*` | 节点运行态、协议初始化和参数桥 |
| `communication/can/yg_protocol_canfd_irq.*` | 唯一 RX 中断入口 |
| `communication/can/yg_protocol_link.*` | RX/TX 队列、端点服务、响应发送 |
| `app/yg_protocol_status_adapter.*` | 在短临界区采集电机与 FOC 状态，映射线上 state/mode/fault 位 |
| `communication/can/yg_protocol_canfd.*` | CAN FD 承载帧打包和解包 |
| `communication/protocol/yg_protocol_frame_codec.*` | 公司应用头、CRC 和 payload 编解码 |
| `communication/protocol/yg_protocol_router.*` | 类型注册、方向检查和业务路由 |
| `communication/protocol/yg_protocol_motor.*` | 电机请求和业务结果值对象 |
| `communication/protocol/yg_protocol_motor_status.*` | 位置、速度、电流、状态、故障、母线和温度编码 |
| `services/parameters/param_comm_bridge.h` | 参数服务访问节点身份和历史超时字段的窄桥 |

## 4. 业务边界

电机控制模块是模式和功率输出的唯一所有者。协议层只产生经过校验的请求值对象，不直接写 `MotorControl`，也不直接操作 HAL。状态反馈必须来自同一次采样快照，位置使用 i32 mrad，速度和电流使用协议定义的 i16/i32 单位与小端编码。

STOP/DISABLE 的协议适配已经预留 `yg_protocol_motor_service_t`，当前尚未绑定电机操作服务，因此返回明确的 UNSUPPORTED；业务命令、参数、标定、升级和 1 kHz 多轴同步按新协议类型继续接入。未绑定的服务不能隐式回落到旧协议。

## 5. 删除范围

以下旧协议文件和工程条目已删除：旧接口门面、标准 CAN transport、旧命令/查询绑定、旧状态源、旧参数线编码、旧 48 字节状态流及其专属测试和主机镜像。历史设计文档保留为迁移记录，不能作为当前线路实现依据。

## 6. 验证

- `tests/unit` 和 `tests/integration` 验证通过；普通验证不接触硬件。
- 新协议的核心、分片、路由、参数适配和链路测试位于 `tests/unit/` 与 `tests/unit/native/`。
- release 构建需使用当前提交重新生成 `summary.json`、AXF/HEX/MAP 哈希；旧输出不能作为新协议完成证明。
- 硬件回归顺序为只读查询 → STOP/DISABLE → 状态反馈 → 参数/标定 → 运动控制，整机连接前确认 PWM 已关闭。

对应的删除计划见 [2026-09-22 删除旧标准 CAN 协议计划](../plans/active/2026-09-22-remove-legacy-can-protocol.md)。
