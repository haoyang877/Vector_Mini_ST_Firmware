# CAN FD 通信分层说明

日期：2026-09-29。当前 APP 只使用公司 `yg_protocol` CAN FD。共享编解码由 APP 与
Loader 共用，产品运行态和业务所有者分别组合；旧标准 CAN 与 USB 服务已经删除。

## 线路与处理路径

线路为 29 位扩展 ID、FD+BRS、仲裁段 1 Mbit/s、数据段 5 Mbit/s。
旧 11 位参数帧、48 B 状态流、心跳调度及波特率切换不再提供兼容入口。

```text
FDCAN HAL → platform/api/comm_hw.h
          → communication/can/yg_protocol_canfd_irq（只复制入队）
          → communication/can/yg_protocol_link（有界队列与端点服务）
          → shared/protocol（CAN ID、公司帧、CRC、分片）
          → communication/protocol（注册、路由与业务值对象）
          → APP 所有者适配器 → 状态机 / 参数服务
```

Board_Init 先初始化电机对象及 schema 11 参数，再启动平台和协议运行态，并绑定状态源。
旧 schema 或无效参数回退默认值，启动不迁移或回写参数区。
FDCAN RX ISR 不执行 CRC、分片、Flash、日志或电机业务。
前台循环服务维护认领与保存、刷新状态快照、处理协议队列；每次收发数量有界，
队列满时丢弃最新帧并保留统计。

## 文件归属与所有权

| 模块 | 职责 |
| --- | --- |
| `firmware/platform/api/comm_hw.h` | CAN FD 能力、单位与失败契约，不含业务语义 |
| `firmware/platform/stm32g4/ports/comm/` | HAL 收发、DLC、滤波、通知及硬件恢复 |
| `shared/protocol/` | APP/Loader 共用 CAN ID、应用头、CRC、分片及升级线路契约 |
| `firmware/communication/can/` | APP 节点运行态、中断入队、链路与参数桥 |
| `firmware/communication/protocol/` | APP 类型注册、路由、请求/反馈编码，不持有可写电机对象 |
| `firmware/app/yg_protocol_owner_adapter.*` | 参数、清错及标定请求到领域值的显式映射 |
| `firmware/app/yg_protocol_status_adapter.*` | 同一时刻状态快照到线路 state/mode/fault 的投影 |
| `firmware/app/motor_state_machine.*` | 生命周期、功率输出许可、维护身份及故障恢复的唯一所有者 |
| `firmware/app/board_config.c` | 组合 APP 进 Loader 服务与实际停机、TX 完成、ENTER 交接 |

CAN 116 与前台目标使用 `MotorStateMachine_SubmitControl()`，只提交值命令；
通信层不暴露电机对象可写 extern，不直接操作 HAL 或更改模式。
状态通过 `MotorControl_ReadStatus()` 获取一致快照，反馈单位及布局以线路契约为准。

## 当前已接入范围

- STOP 与 DISABLE 已绑定真实停机所有者；受理与确认关相不同，不自动重启。
- CLEAR 经主状态机清锁存，随后仍需自检；不隐含使能。
- Type 116 只更新已经 RUNNING 的相同模式目标，不隐式启动或切换模式。
- 参数读写与保存已接入停机资格、revision 和保存 ticket，写 RAM 不等于已持久化。
- Type 160～162 显式映射 LUT、电角零位及齿槽任务，并支持查询、取消及原始结果身份。
- LoaderApp 的 Type 56 交接先禁止输出，确认停止及精确 TX 完成后写 ENTER 邮箱并复位；
  普通独立 APP 使用不可用端口，不能宣称两种链接目标具备同样升级能力。

远程普通使能尚未开放；Type 101 的 enable=1 是未开放的低侧制动候选，不是普通使能。
GET_CAPS 的 supported_modes 当前为 0。摩擦任务及原始采样导出没有新的 CAN 入口。
未支持的服务返回明确结果，不回落到旧协议。

当前契约见 [电机线路](../protocols/yg_protocol_motor_wire_v1.md)、
[标定线路](../protocols/yg_protocol_calibration_wire_v1.md)、
[升级 v2](../protocols/yg_protocol_upgrade_wire_v2.md) 和
[内部所有权](code_structure.md)。

## 验证与限制

单元/集成/native 检查与 release 编译均不接触硬件。当前证据必须绑定提交、工具和产物哈希，
旧输出不能证明新版本可运行。离线 Loader 模型不替代实机升级、输出关断、执行时序与栈峰值验收。
历史协议删除过程见 [删除计划](../plans/active/2026-09-22-remove-legacy-can-protocol.md)。
