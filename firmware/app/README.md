# APP 层

两块板各自在 `main.c` 静态分配一个 `AppContext g_app`。ADC 和监督中断以及前台主循环都从这里取得所属实例。每块板的生产 APP 使用同一份参数、PWM 和 CAN 硬件。软件实例可以独立分配，初始化后地址必须稳定。

| 文件 | 公共入口与职责 | 执行上下文 |
| --- | --- | --- |
| [app.h](app.h)、[app.c](app.c) | `App_Init` 装配轴、协议和 Loader，`App_Start` 启动通信，`App_Process` 推进前台；`app.h` 同时定义板级 `AppContext` | 启动、前台、监督 |
| [motor_axis.h](motor_axis.h)、[motor_axis.c](motor_axis.c) | `SampleFast` 采样，`ControlFast` 执行控制或标定，`SupervisorTick` 监督，`OuterTick` 计算外环，`ProcessPositionPlan` 在前台建表 | ADC、监督、外环、前台 |
| [motor_state_machine.c](motor_state_machine.c)、[motor_state_machine.h](motor_state_machine.h) | 在本轴处理状态事件、目标写入、外环输入与提交、反馈快照。公开入口统一使用 `MotorStateMachine_*` | 前台、ADC、监督与外环 |
| [motor_calibration.h](motor_calibration.h)、[motor_calibration.c](motor_calibration.c) | `StepFast` 推进 LUT、电气零位或机械零位；`ProcessPublication` 处理停稳、发布、保存和失败 | ADC、停机前台 |
| [app_protocol.h](app_protocol.h)、[app_protocol.c](app_protocol.c) | `Init` 绑定本端点的请求和状态读取回调；108 查询或 124 回包各读一次所属轴的快照；目标重发记录属于本端点，STOP 按实际关相状态回复 | 前台 |
| [app_loader.h](app_loader.h)、[app_loader.c](app_loader.c) | `Init/Process` 管理两板 LoaderApp 的停机票据、发送完成、超时复核和镜像确认 | 前台 |

## 镜像边界

普通 G431 和 AT32F456 共享 app、motor_axis、motor_state_machine、motor_calibration、app_protocol 五个生产模块。每轴的状态、功率许可和目标读写都通过 `axis.state_machine`。各事件的提交时机见[五态主线](../../docs/architecture/motor_state_machine_v1.md)。普通 APP 没有应用看门狗。采样新鲜度、功率保护、STOP 和 Flash 事务仍由轴与 BSP 执行。

独立 Loader 负责接收并写入升级镜像；配套 LoaderApp 通过 `YG_APP_LOADER_IMAGE` 编入 app_loader，保留停机交接、健康窗口、镜像确认和继承看门狗。STM32G431 和 AT32F456 均由目标清单生成配套 LoaderApp 工程。[Loader 说明](../../loader/README.md)

## 调度和状态所有权

- 上电顺序为 `Bsp_Init → App_Init → Bsp_Start → App_Start`。一个已初始化实例再次进入 `App_Init` 会立即返回。
- 每个有效 ADC 事件推进一次 20 kHz 快拍，监督保持 2 kHz。AT32 的 4 kHz 配置每五个有效快拍派发一次外环，其他配置从监督拍推进 2 kHz 外环。
- 状态和会话号可从 `axis.state_machine.state` 与 `axis.state_machine.epoch` 读取。状态机独占写入。空闲且无启动请求时，外环和监督拍跳过整套参数校验。
- 前台在目标校验成功后直接写本轴。外环在栈上计算，提交时核对会话、控制模式、STOP 和故障，然后直接更新电流参考及速度 PI。ADC 可在多字段写入期间抢占，并在启相前复核许可。Hall 由外环选定读槽，ADC 只写另一槽。
- 标定工作区、候选和停稳计时器属于轴。关相后前台登记待安装 LUT 的会话，ADC 在读取本拍编码器前复核并安装，随后前台保存，监督拍收尾。完成后由前台复核原会话、故障及实际关相状态，将结果写入该轴绑定的参数槽。写失败锁存存储故障。
- CAN 前台核对 ADC、外环和监督拍号，持续争用时按[反馈元数据 v1](../../docs/protocols/README.md#反馈元数据-v1caps-bit28)返回无效测量；124 保留本次目标的结果码。
- 通信队列、重发序号与 Loader 票据分别随应用或端点存储。生产 ISR 按物理外设固定路由。同一轴的快拍、同一端点的前台消费、同一物理 Flash 的提交均按原硬件顺序串行。不同轴可在调用中嵌套，绑定独立 PWM 和参数槽时可独立提交输出。
- `firmware/motor` 负责控制律、位置级联与编码器标定算法，`firmware/communication` 负责帧与队列，`firmware/parameters` 负责 schema14 记录编解码。新增业务时在持有相应状态的模块实现，板级只负责绑定和派发。

文件名使用小写下划线，公共函数使用模块前缀和动词，私有函数使用小写下划线。每个公共头描述输入、使用时机及副作用。离线嵌套实例用例和两块板的入口均通过正常生产接口调用。[工程规范](../../STYLE.md) · [双实例回归](../../tests/unit/native/test_app_instances.py)

## 事件阅读路径

两块板各自在 main.c 持有一个应用实例。先读板级入口和组合根，再沿相关事件进入业务模块。

| 入口 | 状态写入者 | 跟踪路径 |
| --- | --- | --- |
| 上电与前台 | AppContext | 板级 main → App_Init / App_Start / App_Process |
| 快采样、控制、外环 | MotorAxis | ADC → SampleFast → ControlFast；监督或 PendSV → OuterTick → ReadOuterInputs / CommitOuterResult |
| 启停与故障 | MotorStateMachine | [电机状态机](motor_state_machine.c) 在本轴上复核事件条件、操作功率并提交主状态 |
| 标定与保存 | MotorCalibration | 关相后由前台登记 LUT 会话，ADC 在本拍读角前复核并安装，前台复核同一会话后保存参数 |
| CAN 反馈 | ProtocolEndpoint、AppProtocol | 108 查询或 116 目标处理 → 读取所属轴一次 → 直接填写 108 或 124 回复 |
| 请求停机 | MotorStateMachine | 110 或失能 → 撤销启相许可 → 按实际关相状态回报 |
| 升级与试运行 | AppLoader | 56 请求锁定停机，票据实际发出后复核并交接。57 在连续健康且关相后确认镜像 |

公共接口与并发约定见本页前文。控制律及编码器算法归 motor，线上格式归 communication，记录格式归 parameters。Keil 可展开板级符号 `g_app.axis` 和 `g_app.endpoint`。
