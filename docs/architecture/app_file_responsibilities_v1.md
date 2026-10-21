# APP文件职责与分层对照

更新：2026-10-21。适用当前`YG_ESC_APP`主工程及`Vector_Mini_ST_LoaderApp`配套工程。
主工程树包含106个源文件，下面按Keil现有七组及原排列顺序逐项说明；文件链接指向实际物理位置。
源文件有对应头文件时一并链接，头文件负责类型、函数契约和单位，实现仍属于同一模块。
另列37个没有同名`.c`的自有头文件，区分配置、值类型、平台能力和私有规则。

目录、Keil树和模块名已经同步。下面直接使用当前文件名，旧名到新名的对应见末尾迁移表。

## 七个工程分组分别负责什么

Keil分组是源码导航，数字顺序不是调用顺序、线程优先级或必须逐层经过的路径。
VS Code显示物理目录，Keil显示源文件清单；二者通过下表和逐文件路径对应。

| Keil分组 | 实际物理目录 | 核心职责 |
| --- | --- | --- |
| 00_App | `firmware/app/{composition,runtime,motor_state_machine,interfaces}` | 启动组装、运行所有权、调度和主状态机。 |
| 10_Motor | `firmware/motor/{foc,feedback,position,calibration,protection,math}` | 控制、反馈、单项标定与保护计算。 |
| 20_Parameters | `firmware/parameters` | 配置默认值、标量访问和参数记录转换；真实Flash归平台。 |
| 30_Communication | `firmware/communication/{can,protocol}` | CAN承载、消息分派、业务字段和单位转换。 |
| 40_Platform | `firmware/platform/stm32g4/{ports,bsp,cubemx}` | 硬件接口实现、初始化和中断接线。 |
| 50_Shared | `shared/{protocol,checksum,boot}` | 真正跨APP/Loader共用的格式和纯计算。 |
| 90_Vendor | `platform/.../cubemx/Drivers`及CMSIS系统文件 | 厂商实现。 |

```text
请求：CAN中断 → 接收队列 → 前台消息解析/路由 → APP所有者接口 → 主状态机
控制：ADC完整采样中断 → APP快拍调度 → 反馈/保护/控制计算 → 平台PWM端口
监督：TIM7中断 → APP慢拍调度 → 速度/位置外环 → 主状态推进
保存：APP后台事务 → 参数记录转换 → 平台Flash字节存储
```

功率启停许可归主状态机；算法给计算结果，通信给请求，平台兑现硬件动作。
`app/composition`是组装边界，可以绑定这些模块；控制算法不反向调用APP。

## 与Keil树逐文件对应

### 00_App 应用组织

决定何时初始化、受理请求、启停、维护和保存；拥有单轴运行对象。

| 文件与物理目录 | 核心功能 | 归属和边界 |
| --- | --- | --- |
| [app_startup.c](../../firmware/app/composition/app_startup.c) / [.h](../../firmware/app/composition/app_startup.h)<br>`firmware/app/composition` | 装配板级启动、生产身份、CAN业务绑定和APP确认/进入Loader交接。 | 仅组合所有者与平台能力；不实现FOC、协议codec或直接HAL操作。 |
| [motor_axis.c](../../firmware/app/runtime/motor_axis.c) / [.h](../../firmware/app/runtime/motor_axis.h)<br>`firmware/app/runtime` | 持有单份配置及Watch运行对象，编排20kHz快环、2kHz监督和后台保存。 | 按顺序借用显式对象并转递结果；控制计算、采样换算和热算法归motor，启停许可归主状态机。 |
| [motor_parameter_transaction.c](../../firmware/app/runtime/motor_parameter_transaction.c) / [.h](../../firmware/app/runtime/motor_parameter_transaction.h)<br>`firmware/app/runtime` | 拥有参数读写去重、活动revision及异步保存票据，并装载/保存配置记录。 | 只在既有准入下提交配置；不拥有主状态、功率使能或运行对象副本。 |
| [motor_state_machine.c](../../firmware/app/motor_state_machine/motor_state_machine.c) / [.h](../../firmware/app/motor_state_machine/motor_state_machine.h)<br>`firmware/app/motor_state_machine` | 独占INIT至FATAL生命周期，处理启停、维护、故障、请求身份和后台完成。 | 消费真实采样/资源事实并执行转换动作；不计算控制律，CLEAR成功仅回待机。 |
| [motor_state_rules.c](../../firmware/app/motor_state_machine/motor_state_rules.c)<br>`firmware/app/motor_state_machine` | 依据状态、事件和条件计算主状态转换及所需动作。 | 纯规则核心；不访问硬件、不执行控制或存储，实际动作由主状态机兑现。 |
| [motor_command.c](../../firmware/app/runtime/motor_command.c) / [.h](../../firmware/app/runtime/motor_command.h)<br>`firmware/app/runtime` | 校验控制命令未使用字段、有限性及当前电流/速度/位置/电压目标范围。 | 只校验目标，不授予启动许可、不修改运行状态、不执行控制算法。 |
| [motor_maintenance.c](../../firmware/app/runtime/motor_maintenance.c) / [.h](../../firmware/app/runtime/motor_maintenance.h)<br>`firmware/app/runtime` | 对单项LUT、零位、默认值和保存进行准入、任务分发、取消及资源归还。 | 不拥有主生命周期或标定内部阶段；保存由后台真实存储提交，任务不能自行启相。 |
| [app_tasks.c](../../firmware/app/composition/app_tasks.c) / [.h](../../firmware/app/composition/app_tasks.h)<br>`firmware/app/composition` | 提供20kHz快环、2kHz监督和分频LED/RGB任务入口，保持原调用顺序。 | 合并原两个任务源；不持有控制对象，调用MotorAxis与指示器能力。 |
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
| [yg_protocol_canfd.c](../../firmware/communication/can/yg_protocol_canfd.c) / [.h](../../firmware/communication/can/yg_protocol_canfd.h)<br>`firmware/communication/can` | CAN FD帧承载适配：应用帧编码/解码、DLC补零及校验，将CommHwCanFrame与队列帧互转，发送成功后才出队。 | communication/can；使用shared纯codec和comm_hw能力，不配置位速率/过滤器，不执行电机命令，不与IRQ竞争读取FIFO。 |
| [yg_protocol_canfd_irq.c](../../firmware/communication/can/yg_protocol_canfd_irq.c) / [.h](../../firmware/communication/can/yg_protocol_canfd_irq.h)<br>`firmware/communication/can` | RX中断入口：每次最多取3帧，经comm_hw取值后交给Link入队。 | communication/can；只搬帧且工作有界，不在中断做CRC、业务路由、参数存储或控制计算。 |
| [yg_protocol_endpoint.c](../../firmware/communication/can/yg_protocol_endpoint.c) / [.h](../../firmware/communication/can/yg_protocol_endpoint.h)<br>`firmware/communication/can` | 串行消息端点：帧解码、分片收拢、完整消息路由和应答入队；TX满时保留待发应答。 | communication/can；拥有通用端点/分片进度，借用显式队列与router，不拥有外设和电机业务规则。 |
| [yg_protocol_link.c](../../firmware/communication/can/yg_protocol_link.c) / [.h](../../firmware/communication/can/yg_protocol_link.h)<br>`firmware/communication/can` | APP通信会话所有者：组装8帧RX/TX队列、注册表/路由/端点和后端绑定；维护控制序号重放、116终态关联124反馈及56回复发送完成票据；前台每次最多处理/发送各8帧。 | communication/can；产品级接线与链路进度，不是纯编解码器；通过注入服务和快照访问APP，不直接读写电机全局、启动功率或执行Flash。此文件仍较集中，不能把其职责全部标成router。 |
| [yg_protocol_runtime.c](../../firmware/communication/can/yg_protocol_runtime.c) / [.h](../../firmware/communication/can/yg_protocol_runtime.h)<br>`firmware/communication/can` | 通信启动与周期服务入口：一次初始化Link、启动CAN，并在前台每1秒请求bus-off维护后推进Link。 | communication/can；只调度comm_hw能力与Link，速率/过滤/寄存器由平台实现，不拥有电机生命周期。 |
| [yg_protocol_boot.c](../../firmware/communication/protocol/yg_protocol_boot.c) / [.h](../../firmware/communication/protocol/yg_protocol_boot.h)<br>`firmware/communication/protocol` | Type56进入Loader请求的值转换：解析8B boot_epoch、来源/序号，编码4B准入结果并校验READY票据一致性。 | communication/protocol；不写boot邮箱、不复位CPU、不传输镜像；真实静默证明、邮箱/复位由APP/boot端口负责。 |
| [yg_protocol_job.c](../../firmware/communication/protocol/yg_protocol_job.c) / [.h](../../firmware/communication/protocol/yg_protocol_job.h)<br>`firmware/communication/protocol` | Type160/161/162维护任务服务边界：解码任务种类或job_id，调用注入后端，编码12B票据/阶段/提交效果。 | communication/protocol；只表示任务请求和真实进展，不执行LUT/零位、不分配工作区、不将ACCEPTED/COMPLETED冒充Flash COMMITTED；退役3~5编号不代表实现。 |
| [yg_protocol_message_registry.c](../../firmware/communication/protocol/yg_protocol_message_registry.c) / [.h](../../firmware/communication/protocol/yg_protocol_message_registry.h)<br>`firmware/communication/protocol` | 消息元数据静态目录：登记产品Type及是否允许分片/周期发送，校验重号并提供查找。 | communication/protocol；只描述消息，不绑定执行函数；Type登记不等于开放路由或实现对应功能。 |
| [yg_protocol_motor.c](../../firmware/communication/protocol/yg_protocol_motor.c) / [.h](../../firmware/communication/protocol/yg_protocol_motor.h)<br>`firmware/communication/protocol` | 电机线路值契约：解码101使能、116目标、117启动请求，调用注入motor服务并编码4B结果。 | communication/protocol；检查长度/保留位/模式字段，不判真实状态/限位/功率许可，不读取运行对象。 |
| [yg_protocol_motor_adapter.c](../../firmware/communication/protocol/yg_protocol_motor_adapter.c) / [.h](../../firmware/communication/protocol/yg_protocol_motor_adapter.h)<br>`firmware/communication/protocol` | 线路motor请求到业务接口的适配：STOP/禁用交停机服务，启动/清错交生命周期回调；116模式映射与mrad/mA/mV到SI转换后提交命令。 | communication/protocol；不同于motor.c的字节解码，也不同于APP owner_adapter的实际绑定；只调用注入端口，拒绝未实现MIT/制动模式，不直接关相、清故障或写控制变量。 |
| [yg_protocol_motor_status.c](../../firmware/communication/protocol/yg_protocol_motor_status.c) / [.h](../../firmware/communication/protocol/yg_protocol_motor_status.h)<br>`firmware/communication/protocol` | 电机快照到Type123状态查询/Type124运动反馈的单向转换：缩放与舍入、有效位和无效哨兵、45B反馈编码。 | communication/protocol；只消费状态源值，不采ADC/编码器、不改变目标或发送节拍；实际快照采集在APP status_adapter。 |
| [yg_protocol_parameter.c](../../firmware/communication/protocol/yg_protocol_parameter.c) / [.h](../../firmware/communication/protocol/yg_protocol_parameter.h)<br>`firmware/communication/protocol` | Type120读、121写、122保存的线路解析/结果编码，保留参数类型、权限、expected_revision及异步保存结果，调用显式参数后端。 | communication/protocol；不持有活动配置、不实现写入去重/修订或Flash；这些分别由Axis参数事务和存储层负责。 |
| [yg_protocol_readonly.c](../../firmware/communication/protocol/yg_protocol_readonly.c) / [.h](../../firmware/communication/protocol/yg_protocol_readonly.h)<br>`firmware/communication/protocol` | 只读查询服务分派：按配置选择身份/能力/电机状态provider，检查单播ACK元数据并限制响应payload长度。 | communication/protocol；选择已注入数据源，不采集硬件、不组织产品路由，不负责具体payload字节字段。 |
| [yg_protocol_readonly_payload.c](../../firmware/communication/protocol/yg_protocol_readonly_payload.c) / [.h](../../firmware/communication/protocol/yg_protocol_readonly_payload.h)<br>`firmware/communication/protocol` | 只读值payload编解码：设备APP/Loader状态及boot_epoch、分页查询、身份page0、能力page0显式小端编码。 | communication/protocol；纯字段转换，与readonly.c的provider选择分离；输入快照由APP提供，不直接读取UID/版本寄存器或状态。 |
| [yg_protocol_router.c](../../firmware/communication/protocol/yg_protocol_router.c) / [.h](../../firmware/communication/protocol/yg_protocol_router.h)<br>`firmware/communication/protocol` | 完整消息Type到handler/context的有限分派，校验路由重复及注册表匹配，并从服务结果构造应答消息元数据。 | communication/protocol；不同于registry元数据目录；不拥有队列、分片、重试历史或业务状态，不访问硬件。 |
| [yg_protocol_frame_queue.c](../../firmware/communication/can/yg_protocol_frame_queue.c) / [.h](../../firmware/communication/can/yg_protocol_frame_queue.h)<br>`firmware/communication/can` | 固定容量帧环形队列：push/pop/peek/count及满队列丢弃计数。 | communication/can；固定容量帧队列，不是镜像升级或分片业务；并发串行化由调用者负责。 |

### 40_Platform 硬件适配与生成入口

实现硬件能力，集中HAL/寄存器/引脚；CubeMX入口负责连接中断与APP任务。

| 文件与物理目录 | 核心功能 | 归属和边界 |
| --- | --- | --- |
| [board_startup_stm32g4.c](../../firmware/platform/stm32g4/ports/board/board_startup_stm32g4.c)<br>`firmware/platform/stm32g4/ports/board` | 执行板级ADC校准、PWM采样触发和监督中断启动，并读取MCU UID及生产身份原始页。 | 仅实现STM32G4板级能力和启动顺序，不解析身份业务、不计算控制算法。 |
| [boot_hw_unavailable.c](../../firmware/platform/stm32g4/ports/boot/boot_hw_unavailable.c)<br>`firmware/platform/stm32g4/ports/boot` | 为normal独立APP明确返回Loader进入、启动确认等能力不可用。 | normal能力拒绝桩，不设置Loader邮箱，不提供已实现的升级交接或继承看门狗。 |
| [comm_control_stm32g4.c](../../firmware/platform/stm32g4/ports/comm/comm_control_stm32g4.c)<br>`firmware/platform/stm32g4/ports/comm` | 配置FDCAN全局帧过滤策略，启动控制器并打开RX FIFO0新消息通知。 | 只处理CAN硬件启动，不解析应用ID、CRC或电机命令。 |
| [comm_status_stm32g4.c](../../firmware/platform/stm32g4/ports/comm/comm_status_stm32g4.c)<br>`firmware/platform/stm32g4/ports/comm` | 适配CAN FD硬件帧收发、健康状态、带票据发送完成及bus-off恢复。 | 管理硬件发送槽和完成证据，不解释业务载荷、不直接改变电机状态。 |
| [encoder_spi_stm32g4.c](../../firmware/platform/stm32g4/ports/motor/encoder_spi_stm32g4.c)<br>`firmware/platform/stm32g4/ports/motor` | 实现编码器SPI2命令和回读事务、片选/MOSI方向切换及有界寄存器等待。 | 封装STM32 SPI与引脚细节，不进行LUT校正、零位或速度估计。 |
| [encoder_tle5012b.c](../../firmware/platform/stm32g4/ports/motor/encoder_tle5012b.c)<br>`firmware/platform/stm32g4/ports/motor` | 实现TLE5012B角度请求、只读配置采集及归一化角度/总线错误输出。 | 型号帧格式留在驱动，传输调用encoder_spi；当前角度帧不读取安全字CRC，不能声称完整CRC诊断。 |
| [indicator_stm32g4.c](../../firmware/platform/stm32g4/ports/board/indicator_stm32g4.c)<br>`firmware/platform/stm32g4/ports/board` | 将统一颜色和闪烁通道接口映射到板级LED/RGB驱动。 | 只转换指示能力，模式配色及故障编号策略由APP提供。 |
| [motor_pwm_stm32g4.c](../../firmware/platform/stm32g4/ports/motor/motor_pwm_stm32g4.c)<br>`firmware/platform/stm32g4/ports/motor` | 把三相或单相归一化占空比转换为TIM1比较寄存器数值。 | 只更新占空比，不改变功率级使能；输入有限性和控制许可由上层保证。 |
| [motor_sensing_stm32g4.c](../../firmware/platform/stm32g4/ports/motor/motor_sensing_stm32g4.c)<br>`firmware/platform/stm32g4/ports/motor` | 读取三相与母线ADC原始数据，推进温度/VREFINT转换并输出换算结果。 | ADC寄存器与通道映射只在端口；不拥有控制目标、生命周期或电机配置。 |
| [param_store_stm32g4.c](../../firmware/platform/stm32g4/ports/board/param_store_stm32g4.c)<br>`firmware/platform/stm32g4/ports/board` | 将平台参数字节存储接口转接到板级Flash读写。 | 不解释schema、参数ID或电机对象，不自行决定保存请求是否受理。 |
| [power_stage_stm32g4.c](../../firmware/platform/stm32g4/ports/motor/power_stage_stm32g4.c)<br>`firmware/platform/stm32g4/ports/motor` | 按固定顺序启停六路TIM1 PWM输出，并读取六路使能位是否全关。 | 不计算占空比，不决定生命周期准入；寄存器关断证据不等于栅极和相电压的实物测量。 |
| [time_stm32g4.c](../../firmware/platform/stm32g4/ports/time/time_stm32g4.c)<br>`firmware/platform/stm32g4/ports/time` | 将HAL毫秒节拍提供为统一单调时间接口。 | 仅适配时间来源，不承担调度、控制或超时策略。 |
| [critical.c](../../firmware/platform/stm32g4/bsp/critical.c)<br>`firmware/platform/stm32g4/bsp` | 保存PRIMASK、关闭可屏蔽中断，并按原标记恢复。 | 芯片中断屏蔽原语；调用者负责成对使用和短临界区，不隐含线程锁或存储操作。 |
| [delay.c](../../firmware/platform/stm32g4/bsp/delay.c) / [.h](../../firmware/platform/stm32g4/bsp/delay.h)<br>`firmware/platform/stm32g4/bsp` | 根据系统频率和SysTick计数提供微秒/毫秒忙等待。 | 阻塞并占用CPU的板级时序能力，不用于替代非阻塞快环调度。 |
| [flash.c](../../firmware/platform/stm32g4/bsp/flash.c) / [.h](../../firmware/platform/stm32g4/bsp/flash.h)<br>`firmware/platform/stm32g4/bsp` | 实现G431参数区有界读取、关相门禁、页擦除、双字编程与读回，失败立即结束。 | 只处理目标Flash字节；忙超时锁止后续写入至复位，轮询界限不是同bank取指停顿的墙钟保证。 |
| [led.c](../../firmware/platform/stm32g4/bsp/led.c) / [.h](../../firmware/platform/stm32g4/bsp/led.h)<br>`firmware/platform/stm32g4/bsp` | 根据模式/故障闪烁计数更新板上红绿LED引脚。 | 仅拥有指示灯节奏和GPIO，不识别电机状态机或改变控制许可。 |
| [rgb.c](../../firmware/platform/stm32g4/bsp/rgb.c) / [.h](../../firmware/platform/stm32g4/bsp/rgb.h)<br>`firmware/platform/stm32g4/bsp` | 将RGB颜色和呼吸亮度编码为TIM2 PWM-DMA缓冲并发送。 | 仅拥有灯带缓冲和DMA发送数据，不承担电机PWM或故障保护。 |
| [adc.c](../../firmware/platform/stm32g4/cubemx/Core/Src/adc.c) / [.h](../../firmware/platform/stm32g4/cubemx/Core/Inc/adc.h)<br>`firmware/platform/stm32g4/cubemx/Core/Src` | 配置ADC1温度/VREFINT及ADC2三相/母线注入序列、过采样、触发源与ADC引脚中断。 | CubeMX初始化及USER CODE采样接线，不执行FOC或把逐rank事件当成完整快环帧。 |
| [dma.c](../../firmware/platform/stm32g4/cubemx/Core/Src/dma.c) / [.h](../../firmware/platform/stm32g4/cubemx/Core/Inc/dma.h)<br>`firmware/platform/stm32g4/cubemx/Core/Src` | 打开DMA/DMAMUX时钟并配置DMA1通道中断入口。 | CubeMX DMA基础初始化，具体TIM2发送通道在定时器MSP配置。 |
| [fdcan.c](../../firmware/platform/stm32g4/cubemx/Core/Src/fdcan.c) / [.h](../../firmware/platform/stm32g4/cubemx/Core/Inc/fdcan.h)<br>`firmware/platform/stm32g4/cubemx/Core/Src` | 配置FDCAN1帧格式、1M/5M位时序、引脚和中断优先级。 | CubeMX外设配置与USER CODE覆写；收发队列在端口，业务解析在communication。 |
| [gpio.c](../../firmware/platform/stm32g4/cubemx/Core/Src/gpio.c) / [.h](../../firmware/platform/stm32g4/cubemx/Core/Inc/gpio.h)<br>`firmware/platform/stm32g4/cubemx/Core/Src` | 初始化板级GPIO时钟、默认输出电平及片选/指示等引脚模式。 | 板级引脚配置，不包含运行模式、控制目标或功率准入策略。 |
| [main.c](../../firmware/platform/stm32g4/cubemx/Core/Src/main.c) / [.h](../../firmware/platform/stm32g4/cubemx/Core/Inc/main.h)<br>`firmware/platform/stm32g4/cubemx/Core/Src` | 处理启动能力、HAL/系统时钟与外设初始化，启动Board，并循环调度后台维护、协议和启动服务。 | CubeMX入口和USER CODE组合边界，不在主循环重建电机算法或第二套状态机。 |
| [spi.c](../../firmware/platform/stm32g4/cubemx/Core/Src/spi.c) / [.h](../../firmware/platform/stm32g4/cubemx/Core/Inc/spi.h)<br>`firmware/platform/stm32g4/cubemx/Core/Src` | 配置SPI1/SPI2模式、字宽、速率以及GPIO复用和外设时钟。 | 平台外设初始化；编码器事务与传感器帧解析分别属于端口和型号驱动。 |
| [startup_stm32g431xx.s](../../firmware/platform/stm32g4/cubemx/MDK-ARM/startup_stm32g431xx.s)<br>`firmware/platform/stm32g4/cubemx/MDK-ARM` | 声明Cortex-M4向量表、栈及链接运行区，执行Reset_Handler并进入C运行库。 | 芯片/工具链启动文件，保留汇编中的运行库heap预留不等于恢复了APP堆分配器。 |
| [stm32g4xx_hal_msp.c](../../firmware/platform/stm32g4/cubemx/Core/Src/stm32g4xx_hal_msp.c)<br>`firmware/platform/stm32g4/cubemx/Core/Src` | 执行HAL全局MSP初始化，打开SYSCFG/PWR并配置芯片级基础项。 | CubeMX系统底座，不持有业务状态或控制算法。 |
| [stm32g4xx_it.c](../../firmware/platform/stm32g4/cubemx/Core/Src/stm32g4xx_it.c) / [.h](../../firmware/platform/stm32g4/cubemx/Core/Inc/stm32g4xx_it.h)<br>`firmware/platform/stm32g4/cubemx/Core/Src` | 连接ADC完整JEOS快环、TIM7监督、CAN接收及DMA等中断，并提供异常/SysTick入口。 | 硬件IRQ和APP任务的组合边界；电机控制由任务派发，异常处理不被描述为已验证的自动恢复。 |
| [tim.c](../../firmware/platform/stm32g4/cubemx/Core/Src/tim.c) / [.h](../../firmware/platform/stm32g4/cubemx/Core/Inc/tim.h)<br>`firmware/platform/stm32g4/cubemx/Core/Src` | 配置TIM1互补PWM/ADC触发、TIM2 RGB DMA及时基为2kHz的TIM7监督中断。 | CubeMX定时器、引脚和DMA初始化，不执行控制律或主状态迁移。 |

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
| [stm32g4xx_hal.c](../../firmware/platform/stm32g4/cubemx/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c)<br>`firmware/platform/stm32g4/cubemx/Drivers/STM32G4xx_HAL_Driver/Src` | HAL全局初始化、默认毫秒节拍及通用支持。 | 厂商实现；经platform使用，保持上游源码。 |
| [stm32g4xx_hal_adc.c](../../firmware/platform/stm32g4/cubemx/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc.c)<br>`firmware/platform/stm32g4/cubemx/Drivers/STM32G4xx_HAL_Driver/Src` | ADC基础配置、启动、读数和中断处理。 | 厂商实现；经platform使用，保持上游源码。 |
| [stm32g4xx_hal_adc_ex.c](../../firmware/platform/stm32g4/cubemx/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc_ex.c)<br>`firmware/platform/stm32g4/cubemx/Drivers/STM32G4xx_HAL_Driver/Src` | ADC注入组、校准、过采样等扩展。 | 厂商实现；经platform使用，保持上游源码。 |
| [stm32g4xx_hal_cortex.c](../../firmware/platform/stm32g4/cubemx/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cortex.c)<br>`firmware/platform/stm32g4/cubemx/Drivers/STM32G4xx_HAL_Driver/Src` | NVIC优先级、中断使能和内核配置。 | 厂商实现；经platform使用，保持上游源码。 |
| [stm32g4xx_hal_dma.c](../../firmware/platform/stm32g4/cubemx/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma.c)<br>`firmware/platform/stm32g4/cubemx/Drivers/STM32G4xx_HAL_Driver/Src` | DMA基础传输及中断处理。 | 厂商实现；经platform使用，保持上游源码。 |
| [stm32g4xx_hal_dma_ex.c](../../firmware/platform/stm32g4/cubemx/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma_ex.c)<br>`firmware/platform/stm32g4/cubemx/Drivers/STM32G4xx_HAL_Driver/Src` | DMAMUX等DMA扩展能力。 | 厂商实现；经platform使用，保持上游源码。 |
| [stm32g4xx_hal_exti.c](../../firmware/platform/stm32g4/cubemx/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_exti.c)<br>`firmware/platform/stm32g4/cubemx/Drivers/STM32G4xx_HAL_Driver/Src` | 外部中断线配置和回调派发。 | 厂商实现；经platform使用，保持上游源码。 |
| [stm32g4xx_hal_fdcan.c](../../firmware/platform/stm32g4/cubemx/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_fdcan.c)<br>`firmware/platform/stm32g4/cubemx/Drivers/STM32G4xx_HAL_Driver/Src` | FDCAN配置、收发、过滤、通知及状态查询。 | 厂商实现；经platform使用，保持上游源码。 |
| [stm32g4xx_hal_flash.c](../../firmware/platform/stm32g4/cubemx/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash.c)<br>`firmware/platform/stm32g4/cubemx/Drivers/STM32G4xx_HAL_Driver/Src` | Flash解锁、编程、状态和错误处理。 | 厂商实现；经platform使用，保持上游源码。 |
| [stm32g4xx_hal_flash_ex.c](../../firmware/platform/stm32g4/cubemx/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ex.c)<br>`firmware/platform/stm32g4/cubemx/Drivers/STM32G4xx_HAL_Driver/Src` | Flash擦除及Option Bytes等扩展。 | 厂商实现；经platform使用，保持上游源码。 |
| [stm32g4xx_hal_flash_ramfunc.c](../../firmware/platform/stm32g4/cubemx/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ramfunc.c)<br>`firmware/platform/stm32g4/cubemx/Drivers/STM32G4xx_HAL_Driver/Src` | 需从RAM执行的Flash相关电源/控制辅助例程。 | 厂商实现；经platform使用，保持上游源码。 |
| [stm32g4xx_hal_gpio.c](../../firmware/platform/stm32g4/cubemx/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_gpio.c)<br>`firmware/platform/stm32g4/cubemx/Drivers/STM32G4xx_HAL_Driver/Src` | GPIO模式、引脚读写和外部中断支持。 | 厂商实现；经platform使用，保持上游源码。 |
| [stm32g4xx_hal_pwr.c](../../firmware/platform/stm32g4/cubemx/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr.c)<br>`firmware/platform/stm32g4/cubemx/Drivers/STM32G4xx_HAL_Driver/Src` | 电源控制器基本配置。 | 厂商实现；经platform使用，保持上游源码。 |
| [stm32g4xx_hal_pwr_ex.c](../../firmware/platform/stm32g4/cubemx/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr_ex.c)<br>`firmware/platform/stm32g4/cubemx/Drivers/STM32G4xx_HAL_Driver/Src` | 稳压器及低功耗等电源扩展。 | 厂商实现；经platform使用，保持上游源码。 |
| [stm32g4xx_hal_rcc.c](../../firmware/platform/stm32g4/cubemx/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc.c)<br>`firmware/platform/stm32g4/cubemx/Drivers/STM32G4xx_HAL_Driver/Src` | 振荡源、PLL和总线时钟配置。 | 厂商实现；经platform使用，保持上游源码。 |
| [stm32g4xx_hal_rcc_ex.c](../../firmware/platform/stm32g4/cubemx/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc_ex.c)<br>`firmware/platform/stm32g4/cubemx/Drivers/STM32G4xx_HAL_Driver/Src` | 外设时钟选择等RCC扩展。 | 厂商实现；经platform使用，保持上游源码。 |
| [stm32g4xx_hal_spi.c](../../firmware/platform/stm32g4/cubemx/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_spi.c)<br>`firmware/platform/stm32g4/cubemx/Drivers/STM32G4xx_HAL_Driver/Src` | SPI传输、DMA及中断等厂商实现。 | 厂商实现；经platform使用，保持上游源码。 |
| [stm32g4xx_hal_spi_ex.c](../../firmware/platform/stm32g4/cubemx/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_spi_ex.c)<br>`firmware/platform/stm32g4/cubemx/Drivers/STM32G4xx_HAL_Driver/Src` | SPI FIFO等扩展操作。 | 厂商实现；经platform使用，保持上游源码。 |
| [stm32g4xx_hal_tim.c](../../firmware/platform/stm32g4/cubemx/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim.c)<br>`firmware/platform/stm32g4/cubemx/Drivers/STM32G4xx_HAL_Driver/Src` | 定时器、PWM、输入捕获和基础时基。 | 厂商实现；经platform使用，保持上游源码。 |
| [stm32g4xx_hal_tim_ex.c](../../firmware/platform/stm32g4/cubemx/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim_ex.c)<br>`firmware/platform/stm32g4/cubemx/Drivers/STM32G4xx_HAL_Driver/Src` | 互补PWM、死区和同步等高级定时器扩展。 | 厂商实现；经platform使用，保持上游源码。 |
| [stm32g4xx_hal_uart.c](../../firmware/platform/stm32g4/cubemx/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart.c)<br>`firmware/platform/stm32g4/cubemx/Drivers/STM32G4xx_HAL_Driver/Src` | UART厂商驱动，源清单保留不代表APP已启用USART业务。 | 厂商实现；经platform使用，保持上游源码。 |
| [stm32g4xx_hal_uart_ex.c](../../firmware/platform/stm32g4/cubemx/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_uart_ex.c)<br>`firmware/platform/stm32g4/cubemx/Drivers/STM32G4xx_HAL_Driver/Src` | UART扩展驱动，当前APP没有对应USART业务入口。 | 厂商实现；经platform使用，保持上游源码。 |
| [stm32g4xx_ll_adc.c](../../firmware/platform/stm32g4/cubemx/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_adc.c)<br>`firmware/platform/stm32g4/cubemx/Drivers/STM32G4xx_HAL_Driver/Src` | ADC低层寄存器配置及辅助能力。 | 厂商实现；经platform使用，保持上游源码。 |
| [system_stm32g4xx.c](../../firmware/platform/stm32g4/cubemx/Core/Src/system_stm32g4xx.c)<br>`firmware/platform/stm32g4/cubemx/Core/Src` | CMSIS系统启动钩子、SystemCoreClock及系统时钟状态更新。 | 厂商实现；经platform使用，保持上游源码。 |

## 没有同名实现文件的头文件

下表覆盖APP自有目录中没有同名`.c`的37个头文件。平台能力的实现在`ports`或`bsp`，
值契约和默认宏通常无需独立实现文件。生成的CubeMX/HAL头文件仍由对应工具和厂商维护。

| 实际头文件 | 类型 | 功能和归属 |
| --- | --- | --- |
| [app/interfaces/motor_control_command.h](../../firmware/app/interfaces/motor_control_command.h) | 请求与只读进展契约 | SI目标、START身份、受理/应用进度及submit端口。 |
| [app/interfaces/motor_lifecycle_request.h](../../firmware/app/interfaces/motor_lifecycle_request.h) | 请求与只读进展契约 | 维护任务类别、请求身份、票据与阶段/提交效果。 |
| [app/interfaces/motor_readiness.h](../../firmware/app/interfaces/motor_readiness.h) | 请求与只读进展契约 | 启动阻断原因、自检事实及可启动模式快照。 |
| [app/motor_state_machine/motor_state_internal.h](../../firmware/app/motor_state_machine/motor_state_internal.h) | 模块私有规则契约 | 主状态机事件/动作/guards及规则状态，不是公共访问入口。 |
| [communication/protocol/yg_protocol_service.h](../../firmware/communication/protocol/yg_protocol_service.h) | 通信内部结果值契约 | 内部服务状态/reply及显式线路结果码映射。 |
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
| [platform/api/board_hw.h](../../firmware/platform/api/board_hw.h) | 平台能力接口 | 板级初始化组合所需的能力声明。 |
| [platform/api/board_profile_defaults.h](../../firmware/platform/api/board_profile_defaults.h) | 默认输入与profile选择 | 板级ADC/采样电阻/放大/分压等基础硬件输入。 |
| [platform/api/boot_hw.h](../../firmware/platform/api/boot_hw.h) | 平台能力接口 | Loader跳转/确认与启动邮箱相关硬件能力。 |
| [platform/api/comm_hw.h](../../firmware/platform/api/comm_hw.h) | 平台能力接口 | CAN帧值、收发、发送完成跟踪和bus-off等硬件能力。 |
| [platform/api/control_config.h](../../firmware/platform/api/control_config.h) | 派生量/时基/保护契约 | 20k/2k调度时基、分频及采样周期派生量。 |
| [platform/api/critical_hw.h](../../firmware/platform/api/critical_hw.h) | 平台能力接口 | 保存/恢复中断屏蔽状态的临界区能力。 |
| [platform/api/current_sense_profile.h](../../firmware/platform/api/current_sense_profile.h) | 派生量/时基/保护契约 | 电流采样编译期派生量及既有相电流阈值。 |
| [platform/api/encoder_sensor.h](../../firmware/platform/api/encoder_sensor.h) | 平台能力接口 | 传感器归一化角度/状态及分阶段采样契约，当前仅TLE5012B驱动。 |
| [platform/api/encoder_spi.h](../../firmware/platform/api/encoder_spi.h) | 平台能力接口 | 编码器总线事务能力，不向上暴露SPI句柄。 |
| [platform/api/fast_loop_profile.h](../../firmware/platform/api/fast_loop_profile.h) | 可选诊断契约 | 编译期开关的快环阶段钩子，默认空操作。 |
| [platform/api/indicator_hw.h](../../firmware/platform/api/indicator_hw.h) | 平台能力接口 | 状态指示灯能力。 |
| [platform/api/mcu_temperature.h](../../firmware/platform/api/mcu_temperature.h) | 平台能力接口 | MCU温度换算内联能力契约。 |
| [platform/api/motor_hw.h](../../firmware/platform/api/motor_hw.h) | 平台能力接口 | 三相PWM占空比输出能力。 |
| [platform/api/motor_sensing.h](../../firmware/platform/api/motor_sensing.h) | 平台能力接口 | 原始电流/母线采样与单位换算及温度轮询值。 |
| [platform/api/param_store.h](../../firmware/platform/api/param_store.h) | 平台能力接口 | 参数存储读写/擦除/错误能力。 |
| [platform/api/power_stage_hw.h](../../firmware/platform/api/power_stage_hw.h) | 平台能力接口 | 功率启停与实际六路输出使能读回能力。 |
| [platform/api/time_hw.h](../../firmware/platform/api/time_hw.h) | 平台能力接口 | 单调时钟能力。 |
| [platform/stm32g4/bsp/hw_conf.h](../../firmware/platform/stm32g4/bsp/hw_conf.h) | 板级实际接线 | TIM1时钟/PWM周期、ADC实例与相通道、编码器型号选择；包含CubeMX main.h。 |

## Loader配套APP的文件差异

两个APP工程共用上述业务源码。独立APP使用明确拒绝Loader能力的实现，配套APP换成真实交接端口并加入镜像契约。

| 配套工程源文件 | 功能 | 差异 |
| --- | --- | --- |
| [boot_hw_stm32g4.c](../../firmware/platform/stm32g4/ports/boot/boot_hw_stm32g4.c) | 为Loader配套APP适配向量重定位、进入Loader邮箱、继承看门狗及试启动确认。 | 替换normal的boot_hw_unavailable.c。 |
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
| yg_protocol_transfer | communication/can/yg_protocol_frame_queue | 固定帧队列，容量与并发约定不变。 |

主状态机动作与纯规则、标定采集与纯LUT计算、热模型与有状态保护、平台接口与芯片实现继续保持各自边界。
yg_protocol_link仍是单份APP通信会话所有者，集中维护重放和终态反馈，不将其描述成纯路由。
旧阻尼前馈宏、轨迹/摩擦诊断枚举及保留的UART厂商源不代表退役业务已启用。

## 从功能反查文件

| 要看什么 | 从哪里开始 |
| --- | --- |
| 上电顺序、通信/Loader绑定 | `main.c → app_startup.c → motor_axis.c` |
| 使能、停止、清错和主状态跳转 | `motor_state_machine.c`与`motor_state_rules.c` |
| 速度命令如何变成电流 | `motor_control.c → speed_control.c → foc_algorithm.c` |
| 位置串级如何计算 | `motor_control.c → position_cascade.c → foc_algorithm.c` |
| LUT/零位标定 | `motor_maintenance.c → encoder_calibration_task.c → encoder_calibration_core.c`，无感支持见`sensorless_startup.c` |
| 四类配置入口及保存 | 默认profile头 → `motor_parameter_transaction.c → parameter_record.c → param_store_stm32g4.c → flash.c` |
| 原始ADC/PWM/编码器SPI | `motor_sensing_stm32g4.c`、`motor_pwm_stm32g4.c`、`encoder_spi_stm32g4.c` |
| Watch运行对象 | `motor_axis.c`；状态机上下文在`motor_state_machine.c`，均使用匹配AXF观察。 |

依赖总则见[架构地图](../../ARCHITECTURE.md)，目录规则见[代码结构](code_structure.md)，
对象写入与参数分类见[参数所有权](parameter_ownership_v1.md)。
