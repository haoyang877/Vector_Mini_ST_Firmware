# APP 层

两块板各自在 `main.c` 静态分配一个 `AppContext g_app`。ADC 和监督中断以及前台主循环都从这里取得所属实例。每块板的生产 APP 使用同一份参数、PWM 和 CAN 硬件。软件实例可以独立分配，初始化后地址必须稳定。

| 文件 | 公共入口与职责 | 执行上下文 |
| --- | --- | --- |
| [app.h](app.h)、[app.c](app.c) | `App_Init` 装配轴、协议和 Loader，`App_Start` 启动通信，`App_Process` 推进前台；`app.h` 同时定义板级 `AppContext` | 启动、前台、监督 |
| [motor_axis.h](motor_axis.h)、[motor_axis.c](motor_axis.c) | `SampleFast` 采样，`ControlFast` 执行控制或标定，`SupervisorTick` 监督，`OuterTick` 计算外环，`ProcessPositionPlan` 在前台建表 | ADC、监督、外环、前台 |
| [motor_lifecycle.h](motor_lifecycle.h)、[motor_lifecycle.c](motor_lifecycle.c) | `RequestEnable/RequestStop/RequestClear/SetTarget` 处理请求，`BeginFastTick/PostOutcome` 交接快拍，`BeginOuter/CommitOuter` 复核并提交外环。五态迁移只读取状态和事件 | 前台请求、快拍、慢拍 |
| [motor_calibration.h](motor_calibration.h)、[motor_calibration.c](motor_calibration.c) | `StepFast` 推进 LUT、电气零位或机械零位；`ProcessPublication` 处理停稳、发布、保存和失败 | ADC、停机前台 |
| [app_protocol.h](app_protocol.h)、[app_protocol.c](app_protocol.c) | `Init` 绑定本端点的请求和状态读取回调；108 查询或 124 回包各读一次所属轴的快照；目标重发记录属于本端点，STOP 按实际关相状态回复 | 前台 |
| [app_loader.h](app_loader.h)、[app_loader.c](app_loader.c) | `Init/Process` 管理 G431 LoaderApp 的停机票据、发送完成、超时复核和镜像确认 | 前台 |

## 镜像边界

普通 G431 和 AT32F456 工程只编入五个生产模块：app、motor_axis、motor_lifecycle、motor_calibration、app_protocol。普通 APP 没有应用看门狗，前台或双环进展停滞不再触发由它发起的自动复位。采样新鲜度、功率保护、STOP 和 Flash 事务仍由轴与 BSP 执行。

独立 Loader 负责接收并写入升级镜像；配套 LoaderApp 通过 `YG_APP_LOADER_IMAGE` 编入 app_loader，保留停机交接、健康窗口、镜像确认和继承看门狗。AT32 以相同协议为后续移植目标，当前分支提供普通 AT32 APP 和调试口下载。[Loader 说明](../../loader/README.md)

## 调度和状态所有权

- 上电顺序为 `Bsp_Init → App_Init → Bsp_Start → App_Start`。一个已初始化实例再次进入 `App_Init` 会立即返回。
- 每个有效 ADC 事件推进一次 20 kHz 快拍，监督保持 2 kHz。AT32 的 4 kHz 配置每五个有效快拍派发一次外环，其他配置从监督拍推进 2 kHz 外环。
- 外环在自己的栈上计算。提交前在同一短临界区核对所属实例、epoch、控制模式、STOP 与故障，再发布电流参考、速度 PI 和启相许可。迟到的结果按会话丢弃。
- 标定工作区、候选和停稳计时器属于轴。关相后由一个交接阶段记录等待前台发布或等待监督拍收尾。完成后由前台复核原会话、故障及实际关相状态，将结果写入该轴绑定的参数槽。写失败锁存存储故障。
- 通信队列、重发序号与 Loader 票据分别随应用或端点存储。生产 ISR 按物理外设固定路由。同一轴的快拍、同一端点的前台消费、同一物理 Flash 的提交均按原硬件顺序串行。不同轴可在调用中嵌套，绑定独立 PWM 和参数槽时可独立提交输出。
- `firmware/motor` 负责控制律、位置级联与编码器标定算法，`firmware/communication` 负责帧与队列，`firmware/parameters` 负责 schema14 记录编解码。新增业务时在持有相应状态的模块实现，板级只负责绑定和派发。

文件名使用小写下划线，公共函数使用模块前缀和动词，私有函数使用小写下划线。每个公共头描述输入、使用时机及副作用。离线嵌套实例用例和两块板的入口均通过正常生产接口调用。[工程规范](../../STYLE.md) · [双实例回归](../../tests/unit/native/test_app_instances.py) · [板级入口与文件职责](../../docs/architecture/app_file_responsibilities_v1.md)
