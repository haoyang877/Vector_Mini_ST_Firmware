# APP文件职责与分层对照

更新：2026-10-22。适用当前`ESC-APP-STM32G431`主工程及`ESC-APP-STM32G431-LoaderApp`配套工程。
下面按Keil现有七组及工程排列说明主要源文件；文件链接指向实际物理位置。
源文件有对应头文件时一并链接，头文件负责类型、函数契约和单位，实现仍属于同一模块。
另列没有同名`.c`的自有头文件，区分配置、值类型、平台能力和私有规则。

目录、Keil树和模块名已经同步。下面直接使用当前文件名，旧名到新名的对应见末尾迁移表。

## 七个工程分组分别负责什么

Keil分组是源码导航，数字顺序不是调用顺序、线程优先级或必须逐层经过的路径。
VS Code显示物理目录，Keil显示源文件清单；二者通过下表和逐文件路径对应。

| Keil分组 | 实际物理目录 | 核心职责 |
| --- | --- | --- |
| 00_App | `firmware/app/{composition,runtime,motor_state_machine,interfaces}` | 启动组装、运行所有权、调度和主状态机。 |
| 10_Motor | `firmware/motor/{foc,feedback,position,calibration,protection,math}` | 控制、反馈、单项标定与保护计算。 |
| 20_Parameters | `firmware/parameters` | 配置默认值、标量访问和参数记录转换；真实Flash归平台。 |
| 30_Communication | `firmware/communication/{can,protocol,telemetry}` | CAN承载、消息分派、业务字段和单位转换。 |
| 50_Shared | `shared/{protocol,checksum,boot}` | 真正跨APP/Loader共用的格式和纯计算。 |
| 90_Vendor | `platform/sdk`及CMSIS系统文件 | 厂商实现。 |

```text
请求：CAN中断 → 接收队列 → 前台消息解析/路由 → APP所有者接口 → 主状态机
控制：ADC完整采样中断 → APP快拍调度 → 反馈/保护/控制计算 → 平台PWM端口
监督：TIM7中断 → APP慢拍调度 → 输入与状态前处理 → 获准外环副本 → 复核提交
保存：APP后台事务 → 参数记录转换 → 平台Flash字节存储
```

功率启停许可归主状态机；算法给计算结果，通信给请求，平台兑现硬件动作。
`app/composition`是组装边界，可以绑定这些模块；控制算法不反向调用APP。

## 从启动与执行入口开始读

先读[app.h](../../firmware/app/composition/app.h)和[bsp.h](../../firmware/platform/bsp/include/bsp.h)，
再按[BSP架构总图](bsp_architecture_v1.md)区分硬件初始化、应用装配和中断事件。

```text
各板application/main.c
  Bsp_Init      MCU system.c：时钟、安全GPIO、外设准备
  App_Init      app_startup.c：参数、电机对象、状态机、协议绑定
  Bsp_Start     启动采样及监督时钟
  App_Start     启动绑定后的通信和看门狗
  while(1) App_Process：通信、存储与健康后台任务
ADC完整采样IRQ → BspApp_OnFastSample → App_FastTick → MotorAxis_FastTick
监督IRQ → BspApp_OnControlTick → App_SupervisorTick → MotorAxis_SupervisorTick
```

这些函数不是多个线程。20kHz快环、2kHz监督和前台循环各自有唯一状态所有者。
零偏满窗口后的计算与发布在监督中完成；采样累计仍在快环。

## 与Keil树逐文件对应

### 00_App 应用组织

决定何时初始化、受理请求、启停、维护和保存；拥有单轴运行对象。

| 文件与物理目录 | 核心功能 | 归属和边界 |
| --- | --- | --- |
| [app_startup.c](../../firmware/app/composition/app_startup.c) / [app.h](../../firmware/app/composition/app.h)<br>`firmware/app/composition` | 以App_Init装配电机、采样和通信，以App_Process有序推进后台；Loader确认/交接为私有实现。 | 仅组合所有者与平台能力；不实现FOC、协议codec或直接HAL操作。 |
| [motor_axis.c](../../firmware/app/runtime/motor_axis.c) / [.h](../../firmware/app/runtime/motor_axis.h)<br>`firmware/app/runtime` | 持有单份配置及Watch运行对象，编排20kHz快环、2kHz监督和后台保存。 | 按顺序借用显式对象并转递结果；控制计算、采样换算和热算法归motor，启停许可归主状态机。 |
| [motor_parameter_transaction.c](../../firmware/app/runtime/motor_parameter_transaction.c) / [.h](../../firmware/app/runtime/motor_parameter_transaction.h)<br>`firmware/app/runtime` | 拥有参数读写去重、活动revision及异步保存票据，并装载/保存配置记录。 | 只在既有准入下提交配置；不拥有主状态、功率使能或运行对象副本。 |
| [motor_state_machine.c](../../firmware/app/motor_state_machine/motor_state_machine.c) / [.h](../../firmware/app/motor_state_machine/motor_state_machine.h)<br>`firmware/app/motor_state_machine` | 独占INIT至FATAL生命周期，处理启停、维护、故障、请求身份和后台完成。 | 消费真实采样/资源事实并执行转换动作；不计算控制律，CLEAR成功仅回待机。 |
| [motor_state_rules.c](../../firmware/app/motor_state_machine/motor_state_rules.c)<br>`firmware/app/motor_state_machine` | 依据状态、事件和条件计算主状态转换及所需动作。 | 纯规则核心；不访问硬件、不执行控制或存储，实际动作由主状态机兑现。 |
| [motor_command.c](../../firmware/app/runtime/motor_command.c) / [.h](../../firmware/app/runtime/motor_command.h)<br>`firmware/app/runtime` | 校验控制命令未使用字段、有限性及当前电流/速度/位置/电压目标范围。 | 只校验目标，不授予启动许可、不修改运行状态、不执行控制算法。 |
| [motor_maintenance.c](../../firmware/app/runtime/motor_maintenance.c) / [.h](../../firmware/app/runtime/motor_maintenance.h)<br>`firmware/app/runtime` | 对单项LUT、零位、默认值和保存进行准入、任务分发、取消及资源归还。 | 不拥有主生命周期或标定内部阶段；保存由后台真实存储提交，任务不能自行启相。 |
| [app_tasks.c](../../firmware/app/composition/app_tasks.c) / [app.h](../../firmware/app/composition/app.h)<br>`firmware/app/composition` | 提供App_FastTick与App_SupervisorTick，执行20kHz快环、2kHz监督及原指示器分频。 | 合并原两个任务源；不持有控制对象，调用MotorAxis与指示器能力。 |
| [yg_protocol_owner_adapter.c](../../firmware/app/composition/yg_protocol_owner_adapter.c) / [.h](../../firmware/app/composition/yg_protocol_owner_adapter.h)<br>`firmware/app/composition` | 把协议生命周期、普通启动、参数与维护请求映射到对应所有者并映射结果。 | 组合业务接口及身份/进展；不直接写Watch对象、不替代codec或控制准入。 |
| [yg_protocol_status_adapter.c](../../firmware/app/composition/yg_protocol_status_adapter.c) / [.h](../../firmware/app/composition/yg_protocol_status_adapter.h)<br>`firmware/app/composition` | 将电机、主状态和启动健康的只读快照映射为协议遥测值。 | 只发布快照与状态映射；不反向修改控制状态或触发功率。 |
| [maintenance_workspace.c](../../firmware/app/runtime/maintenance_workspace.c) / [.h](../../firmware/app/runtime/maintenance_workspace.h)<br>`firmware/app/runtime` | 互斥借出参数记录与编码器采样共用的固定联合工作区，并核对归还。 | 只管理6KiB静态资源占用；无heap、算法或隐式第二份大数组。 |
| [motor_stop_request.c](../../firmware/app/interfaces/motor_stop_request.c) / [.h](../../firmware/app/interfaces/motor_stop_request.h)<br>`firmware/app/interfaces` | 将带来源/序号的STOP请求交给功率所有者，并根据真实关闭确认维护停机票据。 | 服务层只关联请求与确认，不直接关相、不拥有主生命周期。 |
| [motor_status_mailbox.c](../../firmware/app/interfaces/motor_status_mailbox.c) / [.h](../../firmware/app/interfaces/motor_status_mailbox.h)<br>`firmware/app/interfaces` | 通过请求、发布和取走三个阶段传递单槽只读电机状态快照。 | 不读取Axis全局，不执行控制；生产者和消费者遵守单核ISR/前台邮箱所有权。 |

### 10_Motor 电机计算与任务

完成控制、反馈、标定和保护；使用显式对象和平台能力，不反查APP主状态机。

| 文件与物理目录 | 核心功能 | 归属和边界 |
| --- | --- | --- |
| [motor_control.c](../../firmware/motor/foc/motor_control.c) / [.h](../../firmware/motor/foc/motor_control.h)<br>`firmware/motor/foc` | 执行电流/速度/位置/Vq模式分发及2kHz外环，独占外环复位和就绪状态。 | 显式接收配置与运行对象；不依赖服务命令、不查询APP生命周期、不使能功率。 |
| [motor_thermal_control.c](../../firmware/motor/protection/motor_thermal_control.c) / [.h](../../firmware/motor/protection/motor_thermal_control.h)<br>`firmware/motor/protection` | 持有单份热历史，编排采样窗/监督看门狗、热限流、故障准入和调试快照。 | 热保护应用策略层；不访问硬件或主状态机，调用者提供真实时间和关相事实。 |
| [motor_thermal.c](../../firmware/motor/protection/motor_thermal.c) / [.h](../../firmware/motor/protection/motor_thermal.h)<br>`firmware/motor/protection` | 计算三相热累积、加热/冷却、降额迟滞、故障锁存及向量限流。 | 热模型纯计算层；状态显式传入，不拥有系统时钟、采样外设或生命周期。 |
| [motor_thermal_config.c](../../firmware/motor/protection/motor_thermal_config.c) / [.h](../../firmware/motor/protection/motor_thermal_config.h)<br>`firmware/motor/protection` | 提供绕组热默认输入，校验并派生模型峰值、连续值和1ms采样更新配置。 | 热配置转换层；不保存热历史、不调度采样、不改变应用电流上限。 |
| [speed_feedforward.c](../../firmware/motor/foc/speed_feedforward.c) / [.h](../../firmware/motor/foc/speed_feedforward.h)<br>`firmware/motor/foc` | 按显式状态执行可选速度目标整形及加速度电流前馈。 | 纯计算且默认关闭；只收窄到传入电流额度，不访问硬件、启停或PI全局对象。 |
| [foc_algorithm.c](../../firmware/motor/foc/foc_algorithm.c) / [.h](../../firmware/motor/foc/foc_algorithm.h)<br>`firmware/motor/foc` | 完成坐标变换、d/q电流闭环、Vq控制、调制及PI配置和电压额度更新。 | 控制数学及PWM能力调用；不持有生命周期、Flash或HAL句柄，不自行启动功率级。 |
| [foc_pid.c](../../firmware/motor/foc/foc_pid.c) / [.h](../../firmware/motor/foc/foc_pid.h)<br>`firmware/motor/foc` | 执行PI配置、积分/饱和、输出跟踪和显式复位。 | 通用控制器纯计算；只更新传入PiController，不决定电机模式或保护阈值。 |
| [foc_sensing.c](../../firmware/motor/foc/foc_sensing.c) / [.h](../../firmware/motor/foc/foc_sensing.h)<br>`firmware/motor/foc` | 完成上电零偏采集、母线/相电流/温度换算、采样资格及对应故障发布。 | 经平台接口取样及查询关相；不碰寄存器、不启相、不写Flash或参数revision。 |
| [flux_observer.c](../../firmware/motor/foc/flux_observer.c) / [.h](../../firmware/motor/foc/flux_observer.h)<br>`firmware/motor/foc` | 根据电压电流计算磁链观测器角度、速度、多圈位置及有效性。 | 只更新传入观测器；不负责启动序列、编码器采样或主状态转换。 |
| [sensorless_startup.c](../../firmware/motor/foc/sensorless_startup.c) / [.h](../../firmware/motor/foc/sensorless_startup.h)<br>`firmware/motor/foc` | 为编码器标定提供对齐、开环、锁速、角度交接和无感闭环支持序列。 | 仅推进传入SensorlessStartupState和电流控制；不代表APP普通启动入口、不保存标定或拥有主状态。 |
| [speed_control.c](../../firmware/motor/foc/speed_control.c) / [.h](../../firmware/motor/foc/speed_control.h)<br>`firmware/motor/foc` | 推进速度斜坡并执行归一化速度PI、动态限流及可选前馈组合。 | 更新显式控制参考；普通2kHz外环与标定分频入口各自调用，不自行选择生命周期。 |
| [angle_feedback.c](../../firmware/motor/feedback/angle_feedback.c) / [.h](../../firmware/motor/feedback/angle_feedback.h)<br>`firmware/motor/feedback` | 经传感器接口采样并应用方向/LUT/零位，产生电角、多圈位置及速度估计。 | 常规采样只读配置、更新显式反馈状态；标定setter显式修改校准对象，不拥有标定流程或自动保存。 |
| [position_cascade.c](../../firmware/motor/position/position_cascade.c) / [.h](../../firmware/motor/position/position_cascade.h)<br>`firmware/motor/position` | 执行直接位置到速度再到电流的串级、反馈滤波、限幅和到位判断。 | 位置控制纯计算；不包含轨迹规划、硬件访问或主状态机，电流额度由调用方传入。 |
| [encoder_calibration_task.c](../../firmware/motor/calibration/encoder_calibration_task.c) / [.h](../../firmware/motor/calibration/encoder_calibration_task.h)<br>`firmware/motor/calibration` | 逐拍采集LUT并执行构表、验证、停转和结果提交，以及单项电零位任务。 | 只拥有标定内部步骤与显式借用采样空间；报告完成/取消，不拥有主启停许可或Flash保存。 |
| [encoder_calibration_core.c](../../firmware/motor/calibration/encoder_calibration_core.c) / [.h](../../firmware/motor/calibration/encoder_calibration_core.h)<br>`firmware/motor/calibration` | 执行Q15角差/均值、角度采样分桶、分步LUT构建及质量校验。 | 确定性标定计算核心；不访问传感器、出力、生命周期或持久存储。 |
| [motor_fault.c](../../firmware/motor/protection/motor_fault.c) / [.h](../../firmware/motor/protection/motor_fault.h)<br>`firmware/motor/protection` | 锁存首个故障，累计过流恢复资格并判断各故障源是否恢复。 | 只报告/核对故障事实；不自动CLEAR、重新启动或直接切换功率输出。 |
| [control_math.c](../../firmware/motor/math/control_math.c) / [.h](../../firmware/motor/math/control_math.h)<br>`firmware/motor/math` | 提供角度归一化、三角/开方近似、滤波、限幅和位转换等基础数学。 | 无I/O与APP状态依赖；由调用方提供输入及状态，不改变控制策略。 |

### 20_Parameters 配置与参数记录

负责配置默认值、参数标量访问及RAW记录转换；停机与快照已归APP公共接口。

| 文件与物理目录 | 核心功能 | 归属和边界 |
| --- | --- | --- |
| [parameter_record.c](../../firmware/parameters/parameter_record.c) / [.h](../../firmware/parameters/parameter_record.h)<br>`firmware/parameters` | 装配默认配置，并在MotorConfiguration与schema14及兼容旧RAW记录之间转换和校验CRC。 | 参数服务只转换配置，不执行Flash I/O，不持久化目标、积分或测量历史。 |
| [motor_axis_profile.c](../../firmware/parameters/motor_axis_profile.c) / [.h](../../firmware/parameters/motor_axis_profile.h)<br>`firmware/parameters` | 读取、校验或创建轴配置记录，解析关节配置，并判断位置和速度包络。 | 只处理显式轴记录及数值，不选择通信节点、不执行控制或硬件动作。 |
| [parameter_values.c](../../firmware/parameters/parameter_values.c) / [.h](../../firmware/parameters/parameter_values.h)<br>`firmware/parameters` | 按冻结参数ID目录处理类型、权限、单位缩放、数值校验和显式配置字段读写。 | 只操作调用者传入的配置，生命周期准入、去重、revision和保存由APP所有者管理。 |

### 30_Communication 通信接入与业务转换

把CAN帧转换为明确业务请求，把实际结果转换为回复；不直接改电机对象或写Flash。

| 文件与物理目录 | 核心功能 | 归属和边界 |
| --- | --- | --- |
| [yg_protocol_canfd.c](../../firmware/communication/can/yg_protocol_canfd.c) / [.h](../../firmware/communication/can/yg_protocol_canfd.h)<br>`firmware/communication/can` | CAN FD帧承载与固定队列：维护push/pop/peek及丢帧计数；应用帧编码/解码、DLC补零及校验，将BspCanFrame与队列帧互转，发送成功后才出队。 | communication/can；使用shared纯codec和BspCan能力，不配置位速率/过滤器，不执行电机命令，不与IRQ竞争读取FIFO。 |
| [yg_protocol_endpoint.c](../../firmware/communication/can/yg_protocol_endpoint.c) / [.h](../../firmware/communication/can/yg_protocol_endpoint.h)<br>`firmware/communication/can` | 命令注册与串行处理：校验重复CMD ID，登记函数与上下文；帧解码、分片收拢、查命令表并执行和应答入队；TX满时保留待发应答。 | communication/can；拥有通用端点/分片进度，借用队列与命令表，不拥有外设和电机业务规则。 |
| [yg_protocol_link.c](../../firmware/communication/can/yg_protocol_link.c) / [.h](../../firmware/communication/can/yg_protocol_link.h)<br>`firmware/communication/can` | APP通信入口：一次初始化和启动、每次中断最多读取3帧、前台按秒维护bus-off；组装8帧RX/TX队列、命令表、端点和业务服务绑定；驱动电机反馈发送接口，跟踪0x0038回复的队列位置与硬件发送完成；前台每次最多处理/发送各8帧。 | communication/can；产品级接线与链路进度，不是纯编解码器；通过注入服务和快照访问APP，不直接读写电机全局、启动功率或执行Flash。业务命令处理已移到对应协议模块。 |
| [yg_protocol_boot.c](../../firmware/communication/commands/yg_protocol_boot.c) / [.h](../../firmware/communication/commands/yg_protocol_boot.h)<br>`firmware/communication/commands` | 处理0x0038进入Loader和0x0039确认：解析8B nonce或版本/CRC，调用升级服务，保存待确认票据并编码4B结果。 | communication/commands；不写boot邮箱、不复位CPU、不传输镜像；真实静默证明、邮箱/复位由APP/boot端口负责。 |
| [yg_protocol_job.c](../../firmware/communication/commands/yg_protocol_job.c) / [.h](../../firmware/communication/commands/yg_protocol_job.h)<br>`firmware/communication/commands` | Type160/161/162维护任务服务边界：解码任务种类或job_id，调用注入后端，编码12B票据/阶段/提交效果。 | communication/commands；只表示任务请求和真实进展，不执行LUT/零位、不分配工作区、不将ACCEPTED/COMPLETED冒充Flash COMMITTED；退役3~5编号不代表实现。 |
| [yg_protocol_motor.c](../../firmware/communication/commands/yg_protocol_motor.c) / [.h](../../firmware/communication/commands/yg_protocol_motor.h)<br>`firmware/communication/commands` | 电机线路值契约：解码101使能、116目标、117启动请求，调用注入motor服务并编码4B结果；保存32条控制重试记录，关联控制完成状态，把快照编码为45B反馈；发送忙时保留反馈，成功入队后清除。 | communication/commands；检查长度/保留位/模式字段，不判真实状态/限位/功率许可，不读取运行对象。 |
| [yg_protocol_motor_adapter.c](../../firmware/communication/commands/yg_protocol_motor_adapter.c) / [.h](../../firmware/communication/commands/yg_protocol_motor_adapter.h)<br>`firmware/communication/commands` | 线路motor请求到业务接口的适配：STOP/禁用交停机服务，启动/清错交生命周期回调；启动与目标控制共用模式映射；mrad/mA/mV到SI转换后提交命令。 | communication/commands；不同于motor.c的字节解码，也不同于APP owner_adapter的实际绑定；只调用注入端口，拒绝未实现MIT/制动模式，不直接关相、清故障或写控制变量。 |
| [yg_protocol_parameter.c](../../firmware/communication/commands/yg_protocol_parameter.c) / [.h](../../firmware/communication/commands/yg_protocol_parameter.h)<br>`firmware/communication/commands` | Type120读、121写、122保存的线路解析/结果编码，保留参数类型、权限、expected_revision及异步保存结果，调用显式参数后端。 | communication/commands；不持有活动配置、不实现写入去重/修订或Flash；这些分别由Axis参数事务和存储层负责。 |
| [yg_protocol_readonly.c](../../firmware/communication/commands/yg_protocol_readonly.c) / [.h](../../firmware/communication/commands/yg_protocol_readonly.h)<br>`firmware/communication/commands` | 只读查询共用处理：检查请求，调用直接绑定的数据提供函数，限制回复长度；拥有身份、能力及维护页面，引用设备状态快照，完成查询命令处理和编码。 | 不再按 CMD ID 查第二张表，不采集硬件，不访问收发队列。 |

### 40_Platform 硬件功能

逐文件职责以[BSP架构总图](bsp_architecture_v1.md)为准，不在此复制第二套平台目录。

| 位置 | 职责 |
| --- | --- |
| bsp/include | 对上统一功能契约 |
| bsp根目录 | 真正共用的ADC、PWM、编码器、CAN、LED流程及未绑定UART |
| bsp/bsp_internal.h、devices | 私有跨文件契约和真实器件协议 |
| bsp/mcu | MCU的系统、外设、IRQ与Flash操作；STM32保留生成目录 |
| platform/config | 板选择、资源、硬件比例及能力校验 |
| app/config | 控制时基、应用限流、功能要求与测试许可 |
| projects | 相同main模板和各目标编译/链接输入 |

### 50_Shared 跨产品契约

APP与Loader共用的字节协议、校验及身份格式；不拥有硬件或APP运行态。

| 文件与物理目录 | 核心功能 | 归属和边界 |
| --- | --- | --- |
| [yg_protocol_can_id.c](../../shared/protocol/yg_protocol_can_id.c) / [.h](../../shared/protocol/yg_protocol_can_id.h)<br>`shared/protocol` | 纯编码和解码29位CAN ID中的优先级、PF与源/目标节点并校验保留位。 | 跨产品共享字节契约，不访问CAN硬件或决定电机业务。 |
| [yg_protocol_crc.c](../../shared/protocol/yg_protocol_crc.c) / [.h](../../shared/protocol/yg_protocol_crc.h)<br>`shared/protocol` | 按协议约定计算CRC8头校验和CRC16帧校验。 | 纯函数，无硬件或可变全局状态，不承担业务鉴权。 |
| [yg_protocol_fragment.c](../../shared/protocol/yg_protocol_fragment.c) / [.h](../../shared/protocol/yg_protocol_fragment.h)<br>`shared/protocol` | 使用调用者固定缓冲进行单会话分片重组、重复片识别和片间/绝对超时处理。 | 只管理传入重组状态，不分配内存、不执行消息重传或实时电机命令。 |
| [yg_protocol_frame_codec.c](../../shared/protocol/yg_protocol_frame_codec.c) / [.h](../../shared/protocol/yg_protocol_frame_codec.h)<br>`shared/protocol` | 编码和校验统一帧头、显式小端字段、载荷长度及头/帧双层CRC。 | 只处理协议字节和值，不访问外设、不路由业务或持有运行对象。 |
| [crc32.c](../../shared/checksum/crc32.c) / [.h](../../shared/checksum/crc32.h)<br>`shared/checksum` | 提供与zlib约定一致、支持seed续算的反射CRC32纯计算。 | 供参数、身份和镜像等真实调用者复用，不读取全局状态或执行I/O。 |
| [device_identity.c](../../shared/boot/device_identity.c) / [.h](../../shared/boot/device_identity.h)<br>`shared/boot` | 校验生产身份记录的magic、版本、SN和CRC，并编码设备信息回复载荷。 | 只解析调用者提供的身份字节，不直接读写生产页、不管理CAN链路或电机节点存储。 |

### 90_Vendor 厂商基础库

ST HAL、LL和CMSIS实现，由平台调用；不承载项目业务。

| 文件与物理目录 | 核心功能 | 归属和边界 |
| --- | --- | --- |

## 没有同名实现文件的头文件

下表覆盖APP自有目录中没有同名`.c`的39个头文件。平台能力的实现在本板`bsp`或共用驱动，
值契约和默认宏通常无需独立实现文件。生成的CubeMX/HAL头文件仍由对应工具和厂商维护。

| 实际头文件 | 类型 | 功能和归属 |
| --- | --- | --- |
| [app/composition/app.h](../../firmware/app/composition/app.h) | 四个应用入口契约 | App_Init、App_Process、App_FastTick、App_SupervisorTick；分别限定启动、前台、20kHz与2kHz上下文。 |
| [app/interfaces/motor_control_command.h](../../firmware/app/interfaces/motor_control_command.h) | 请求与只读进展契约 | SI目标、START身份、受理/应用进度及submit端口。 |
| [app/interfaces/motor_lifecycle_request.h](../../firmware/app/interfaces/motor_lifecycle_request.h) | 请求与只读进展契约 | 维护任务类别、请求身份、票据与阶段/提交效果。 |
| [app/interfaces/motor_readiness.h](../../firmware/app/interfaces/motor_readiness.h) | 请求与只读进展契约 | 启动阻断原因、自检事实及可启动模式快照。 |
| [app/motor_state_machine/motor_state_internal.h](../../firmware/app/motor_state_machine/motor_state_internal.h) | 模块私有规则契约 | 主状态机事件/动作/guards及规则状态，不是公共访问入口。 |
| [communication/commands/yg_protocol_service.h](../../firmware/communication/commands/yg_protocol_service.h) | 通信内部结果值契约 | 内部服务状态/reply及显式线路结果码映射。 |
| [motor/axis_config.h](../../firmware/motor/axis_config.h) | 配置值契约 | 轴类型、AXS记录及关节控制基线值；内联位置范围校验。 |
| [motor/encoder_config.h](../../firmware/motor/encoder_config.h) | 配置值契约 | 设备角度LUT、方向、零位和标定标志结构；不包含反馈运行态。 |
| [motor/motor_config.h](../../firmware/motor/motor_config.h) | 配置值契约 | model/application/device三类活动配置及配置来源；Axis单份持有。 |
| [motor/motor_control_mode.h](../../firmware/motor/motor_control_mode.h) | 运行值契约 | 电流/速度/位置/VQ内部模式枚举。 |
| [motor/motor_hardware_profile.h](../../firmware/motor/motor_hardware_profile.h) | 默认输入与profile选择 | MOTOR_HAS_DAMPING_RING装配选择；不是MCU引脚或ADC能力。 |
| [motor/motor_profile_defaults.h](../../firmware/motor/motor_profile_defaults.h) | 默认输入与profile选择 | 已实现电机profile及物理模型/整定默认输入。 |
| [motor/motor_startup_profile.h](../../firmware/motor/motor_startup_profile.h) | 默认输入与profile选择 | 无感启动及LUT/零位标定的编译期整定，按装配变体选择。 |
| [motor/motor_state_types.h](../../firmware/motor/motor_state_types.h) | 运行值契约 | INIT~FATAL八个主状态的只读值枚举。 |
| [motor/motor_task_result.h](../../firmware/motor/motor_task_result.h) | 运行值契约 | 控制/标定单拍任务结果和故障事实。 |
| [motor/motor_types.h](../../firmware/motor/motor_types.h) | 运行值契约 | 故障码和易失MotorControlState；无配置指针。 |
| [motor/position/position_cascade_config.h](../../firmware/motor/position/position_cascade_config.h) | 派生量/时基/保护契约 | 位置串级固定增益/范围的限值契约。 |
| [motor/protection/bus_voltage_profile.h](../../firmware/motor/protection/bus_voltage_profile.h) | 派生量/时基/保护契约 | 母线过欠压及确认周期等保护常量。 |
| [parameters/application_profile_defaults.h](../../firmware/parameters/application_profile_defaults.h) | 默认输入与profile选择 | 应用速度/电流包络、外环与关节默认值。 |
| [parameters/device_profile_defaults.h](../../firmware/parameters/device_profile_defaults.h) | 默认输入与profile选择 | 单台设备节点及默认校准初值。 |
| [app/config/control_config.h](../../firmware/app/config/control_config.h) | 派生量/时基/保护契约 | 20k/2k调度时基、分频及采样周期派生量。 |
| [app/config/mcu_temperature.h](../../firmware/app/config/mcu_temperature.h) | 平台能力接口 | MCU温度换算内联能力契约。 |

## Loader配套APP的文件差异

两个APP工程共用上述业务源码。独立APP使用明确拒绝Loader能力的实现，配套APP换成真实交接端口并加入镜像契约。

| 配套工程源文件 | 功能 | 差异 |
| --- | --- | --- |
| [image_mailbox.c](../../shared/boot/image_mailbox.c) | 编解码APP进入Loader的一次性32字节邮箱消息，并校验magic、版本、节点和CRC32。 | 仅配套APP登记。 |
| [vector_mini_st_g431_app_descriptor.c](../../shared/targets/vector_mini_st_g431_app_descriptor.c) | 为配套APP在固定节放置产品、布局、版本、参数ABI及提交标记的镜像描述符。 | 仅配套APP登记。 |

## 本轮已落实的命名与归属

| 旧项 | 当前文件或归属 | 保持的职责边界 |
| --- | --- | --- |
| motor_execution | app/runtime/motor_command | 校验命令，主状态机决定准入。 |
| motor_control_execution | motor/foc/motor_control | 执行已经准入的控制模式和外环。 |
| foc_sensorless | motor/foc/flux_observer | 根据电压电流估计磁链、角度和速度。 |
| foc_sensorless_run | motor/foc/sensorless_startup | 推进标定支持的对准、开环和角度交接。 |
| foc_speed、foc_errhandle | speed_control、motor_fault | 分别负责速度控制及故障事实。 |
| foc_param、parameter_value_service | parameters/parameter_record、parameter_values | 记录转换与标量校验，不操作真实Flash。 |
| board_config | app/composition/app_startup | 启动、产品绑定和Loader交接组装。 |
| foc_task与bsp_task | app/composition/app_tasks | 保留原快慢拍入口及调用顺序，合并源文件。 |
| services/parameters | firmware/parameters，Keil 20_Parameters | 参数独立归属，不拥有运行生命周期。 |
| 停机服务、状态邮箱、请求值头 | app/interfaces，源文件归Keil 00_App | 通信只用窄接口，不引用APP内部实现。 |
| yg_protocol_transfer | communication/can/yg_protocol_canfd | 固定帧队列，容量与并发约定不变。 |

主状态机动作与纯规则、标定采集与纯LUT计算、热模型与有状态保护、平台接口与芯片实现继续保持各自边界。
yg_protocol_link负责装配和链路调度；控制重试记录与待发反馈归电机协议模块，升级票据归升级协议模块。
旧阻尼前馈宏、轨迹/摩擦诊断枚举及保留的UART厂商源不代表退役业务已启用。

## 从功能反查文件

| 要看什么 | 从哪里开始 |
| --- | --- |
| 上电顺序、通信/Loader绑定 | `main → Bsp_Init/App_Init/Bsp_Start/App_Start → motor_axis.c` |
| 使能、停止、清错和主状态跳转 | `motor_state_machine.c`与`motor_state_rules.c` |
| 速度命令如何变成电流 | `motor_control.c → speed_control.c → foc_algorithm.c` |
| 位置串级如何计算 | `motor_control.c → position_cascade.c → foc_algorithm.c` |
| LUT/零位标定 | `motor_maintenance.c → encoder_calibration_task.c → encoder_calibration_core.c`，无感支持见`sensorless_startup.c` |
| 四类配置入口及保存 | 默认profile头 → `motor_parameter_transaction.c → parameter_record.c → BspFlash_Write` |
| 原始ADC/PWM/编码器SPI | `bsp_adc.c`、`bsp_pwm.c`和各MCU的`encoder_spi.c` |
| Watch运行对象 | `motor_axis.c`；状态机上下文在`motor_state_machine.c`，均使用匹配AXF观察。 |

依赖总则见[架构地图](../../ARCHITECTURE.md)，目录规则见[代码结构](code_structure.md)，
对象写入与参数分类见[参数所有权](parameter_ownership_v1.md)。
