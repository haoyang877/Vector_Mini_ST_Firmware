# 电机驱动 APP 源码导航

本目录是 STM32G4 / AT32F456 共用的单轴裸机应用。工程按 BSP_BOARD 选择硬件实现，
控制时基为 20 kHz 快环和 2 kHz 监督拍；AT32F456/YQ-5708 每五个有效快拍派发一次 4 kHz 位置或速度控制，前台主循环处理通信与标定候选发布。

| 目录 | 职责 |
| --- | --- |
| [config](config/README.md) | 板卡、电机、应用参数及派生值 |
| [app](app/README.md) | 应用组装、单轴对象、生命周期和五态迁移 |
| [motor](motor/README.md) | 控制、反馈、测量、标定和故障处理 |
| [parameters](parameters/README.md) | 默认值、配置校验、标量服务与 schema14 记录 |
| communication | CAN FD 承载、命令边界及保留的 RTT 模块 |
| [platform](platform/README.md) | 公共硬件接口及两种 MCU 实现 |
| third_party | 第三方源码，保持上游格式 |

两板入口分别为 `projects/stm32g4/application/main.c` 和
`projects/at32f456/application/main.c`。启动顺序为
`Bsp_Init → App_Init → Bsp_Start → App_Start`，主循环调用 `App_Process`。
AT32 ADC 入口确认完成后直接执行 `MotorAxis_SampleFast → MotorAxis_ControlFast`，生命周期登记本拍；
STM32 保留 HAL 完成回调，经 `BspApp_OnFastSample → MotorAxis_SampleFast → MotorAxis_ControlFast`；
监督事件经 `BspApp_OnControlTick → MotorAxis_SupervisorTick`。

当前 APP 开放零目标启动、STOP、清错和三类编码器标定。
同模式目标通过 116 提交；三类角度标定完成后在关相前台自动保存到 Flash。在线参数读写/保存与标定进度查询返回不支持。
启动零偏在关相的 IDLE 快拍累计 2048 次，20 kHz 下约 102.4 ms。
完整链路见[软件架构](../docs/architecture/software_architecture.md)。

## Keil Watch 对象

板级实例的这些成员供只读观测，运行写入仍归所属模块。

| 对象 | 所属源码 | 内容 |
| --- | --- | --- |
| `g_app.axis.configuration` | app/motor_axis.c | model/application/device 配置与来源、确认位 |
| `g_app.axis.motor` | app/motor_axis.c | 模式、故障、目标和动态控制状态 |
| `g_app.axis.foc` | app/motor_axis.c | 测量值、PI、坐标变换、调制和估算量 |
| `g_app.axis.rotor_encoder` | app/motor_axis.c | 角度、位置和速度估计；不含独立硬件错误诊断 |
| `g_app.axis.lifecycle` | app/motor_lifecycle.c | 生命周期、启停意图、标定类型和会话代际 |

见[Watch 指南](../docs/guides/watch_motor_commands.md)。调试前确认板上镜像与 AXF 匹配；
源码检查不代表实机验收，电机出力时不通过暂停 CPU 停车。
