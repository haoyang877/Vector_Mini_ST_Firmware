# APP 文件职责与阅读顺序

两块板各自在 main.c 持有一个应用实例。先读板级入口和组合根，再沿相关事件进入业务模块。

| 入口 | 状态写入者 | 跟踪路径 |
| --- | --- | --- |
| 上电与前台 | AppContext | 板级 main → App_Init / App_Start / App_Process |
| 快采样、控制、外环 | MotorAxis | ADC → SampleFast → ControlFast；监督或 PendSV → OuterTick → BeginOuter / CommitOuter |
| 启停与故障 | MotorLifecycle | 协议请求 → 生命周期准入 → 快拍复核 → 实际 PWM 输出；五态迁移在同一编译单元内按值执行 |
| 标定与保存 | MotorCalibration | StepFast → PostOutcome → 关相 → ProcessPublication → 原会话复核 → 参数槽写入 |
| CAN 命令 | ProtocolEndpoint、AppProtocol | 中断入队 → 前台解码 → 绑定的处理器 → 状态快照或 STOP 回复 |
| 镜像确认 | AppLoader | 停机请求 → 监督健康窗口 → 票据发送完成、复核、交接 |

公共接口、并发约定和代码链接见 [APP 目录](../../firmware/app/README.md)。控制律及编码器算法归 motor，线上格式归 communication，记录格式归 parameters。Keil 可展开板级符号 `g_app.axis` 和 `g_app.endpoint`。
