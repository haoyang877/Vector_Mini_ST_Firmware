# APP 文件职责与阅读顺序

两块板各自在 main.c 持有一个应用实例。先读板级入口和组合根，再沿相关事件进入业务模块。

| 入口 | 状态写入者 | 跟踪路径 |
| --- | --- | --- |
| 上电与前台 | AppContext | 板级 main → App_Init / App_Start / App_Process |
| 快采样、控制、外环 | MotorAxis | ADC → SampleFast → ControlFast；监督或 PendSV → OuterTick → BeginOuter / CommitOuter |
| 启停与故障 | MotorLifecycle | 协议请求 → 生命周期准入 → 快拍复核 → 实际 PWM 输出；五态迁移在同一编译单元内按值执行 |
| 标定与保存 | MotorCalibration | 关相后由前台登记 LUT 会话，ADC 在本拍读角前复核并安装，前台复核同一会话后保存参数 |
| CAN 反馈 | ProtocolEndpoint、AppProtocol | 108 查询或 116 目标处理 → 读取所属轴一次 → 直接填写 108 或 124 回复 |
| 请求停机 | MotorLifecycle | 110 或失能 → 撤销启相许可 → 按实际关相状态回报 |
| 升级与试运行 | AppLoader | 56 请求锁定停机，票据实际发出后复核并交接。57 在连续健康且关相后确认镜像 |

公共接口、并发约定和代码链接见 [APP 目录](../../firmware/app/README.md)。控制律及编码器算法归 motor，线上格式归 communication，记录格式归 parameters。Keil 可展开板级符号 `g_app.axis` 和 `g_app.endpoint`。
