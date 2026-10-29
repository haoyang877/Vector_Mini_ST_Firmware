# 代码框架与目录职责

更新：2026-10-26。此页描述当前目录，不用历史报告代替当前实现和验证结果。
产品依赖规则见根目录 [ARCHITECTURE.md](../../ARCHITECTURE.md)。
对照Keil阅读每个源文件时，使用[APP文件职责与分层对照](app_file_responsibilities_v1.md)：
按当前七组说明主要源文件、对应头文件、物理路径与独立配置头。

## 独立产品与共享契约

| 目录 | 职责 |
| --- | --- |
| [firmware](../../firmware/README.md) | 电机驱动 APP，负责控制、运行管理、通信业务与硬件适配 |
| [loader](../../loader/README.md) | 独立升级与启动产品，不把核心实现链接进 APP |
| [host_app](../../host_app/README.md) | 独立主机应用边界，外部主机项目不复制进 APP |
| [shared](../../shared/README.md) | APP/Loader 真正共用的协议 codec、CRC、镜像、启动邮箱和目标契约 |
| tests / tools | 离线夹具、构建与实机工具；硬件操作不进入默认验证 |
| outputs | 忽略的生成产物和证据，旧记录不能作为本轮 PASS |

## APP 实际文件归属

| 目录 | 内容与边界 |
| --- | --- |
| app/composition | app.h：应用入口；app_startup：App_Init初始化和App_Process后台编排，Loader交接为私有实现；app_tasks：App_FastTick和App_SupervisorTick；yg_protocol_*_adapter：业务所有者与协议之间的组装 |
| app/runtime | motor_axis：唯一电机对象组装及调度；motor_parameter_transaction：请求去重、修订及保存事务；motor_command：命令格式与范围校验；motor_maintenance：维护协调；maintenance_workspace：唯一固定维护工作区 |
| app/interfaces | motor_control_command、motor_lifecycle_request、motor_readiness：窄值契约；motor_stop_request：停机请求与真实关闭确认；motor_status_mailbox：只读快照传递；不引用runtime或私有状态机 |
| app/motor_state_machine | 唯一主状态所有者、实际动作与内部转换规则；内部头不作为其他模块接口 |
| motor/foc | motor_control：控制分发与外环；foc_algorithm：控制计算与PI配置绑定；foc_sensing：消费BSP物理帧及上电零偏累计；flux_observer和sensorless_startup：标定所需估计/启动；speed_control：速度控制 |
| motor/feedback | angle_feedback：编码器采样、角度校正、多圈与速度估计；所有模式共同使用 |
| motor/position | position_cascade：直接位置→速度→电流控制 |
| motor/math | control_math：既有数学、角度与滤波计算，无硬件或 APP 依赖 |
| motor/calibration | encoder_calibration_core：LUT/零位计算；encoder_calibration_task：采集/应用流程 |
| motor/protection | 电机故障、母线保护与绕组热保护；motor_thermal_control独占热历史，通过显式电流、时间及功率事实推进 |
| motor 根目录头文件 | 电机域值类型、模式、状态、任务结果及现有模型/启动 profile，不放应用运行实例 |
| parameters | parameter_record：schema14装载/导出及12/13兼容读取；parameter_values：标量类型/权限/范围及缩放；motor_axis_profile：轴记录；应用/设备默认值；不拥有通信节点变量 |
| communication/can | CAN 接入、队列、中断接入、链路及端点调度 |
| communication/telemetry | 按目标选择的2kHz只读RTT波形，控制输入只读，不含电机命令 |
| communication/commands | APP 业务 Type 的解析、回复与服务适配；纯帧 codec 归 shared/protocol |
| platform/bsp/include | 唯一公共硬件契约，保持单位、失败和执行上下文 |
| platform/config | 板型、支持与使能校验、接线与采样比例；应用准入不在此处 |
| platform/bsp/mcu/stm32g4 | STM32外设后端及私有生成Core，原厂Drivers在SDK目录 |
| platform/bsp/mcu/at32f456 | AT32外设后端、DMA/SPI物理事务及存储协调 |
| platform/bsp根目录 | 两板共用的采样/PWM/编码器/CAN流程及软件状态；器件协议在devices，必要私有声明在bsp_internal.h |
| platform/sdk | 厂商原目录，逐文件保持；不统一为同样的目录格式 |
| app/config | 应用必需功能、构建使能、控制周期、软件限流和测试输出许可 |
| projects | 完整APP目标装配、Keil工程、启动汇编、scatter和CubeMX元数据 |
| tests/bench/at32f456 | 五个独立外设/电机台架的源代码 |

源码与对应头文件相邻。目录按职责分组，不为单个函数新建目录，不创建空的未来配置目录。
数据流和所有权见 [APP 命名与所有权](software_architecture.md)、
[主状态机契约](motor_state_machine_v1.md)。

各模块核心输入、结果、状态所有者和实际删减见[模块核心职责](module_core_responsibilities.md)。

## 参数与运行状态

板级物理输入归platform/config/boards，配置选择、比例与时基头归config；
实现归属见[双板对应表](platform_interface_implementations_v1.md)。
电机模型默认值归motor，应用/设备默认值归parameters。
MotorAxis 独立持有 `MotorConfiguration`（model/application/device）和 `MotorControlState`。
设备配置内的 `EncoderCalibration` 单份保存LUT/零位；`EncoderState`只含采样、位置/速度和诊断。
运行对象不包含配置或配置指针，命令目标、模式、故障与测量留在易失状态中。
算法接收const配置视图与可写运行对象，参数事务和校准提交通过明确入口修改配置。
零偏观测实例仍由Axis持有，foc_sensing只通过显式参数累计；窗口满后由Axis监督任务发布零偏并增加修订，下一快拍换算电流。
runtime不解析ADC计数、不实现位置/速度外环或热模型，仍通过平台能力查询时间及真实关相事实。
按用户的 Watch 观测要求，MotorAxis 运行对象与主状态机上下文使用非静态全局符号；
定义和运行写入仍集中在所属模块，其他模块通过既有命令、快照和显式对象参数交互。
保留既有 s_ 名称属于本次调试兼容例外，详见[全局观测对象决策](../archive/plans/2026-09-29-watch-global-objects-v1.md)。
schema14保持现有存储布局并兼容读取12/13，由装载/导出边界映射，不能当作控制对象使用。
参数所有者给启动组合层传节点值，通信不再作为参数节点的存储所有者。
参数记录和编码器标定复用一份 6 KiB 固定联合工作区，APP 管理占用与归还。
编码器任务显式借用采样对象；LUT验证和停转后返回FINALIZE，仍保留候选工作区。
主状态机确认关相后由Axis后台复核会话、STOP和故障，原子发布正式LUT及角度元数据。
发布成功或取消才清除借用；实际归还后APP才允许独立SAVE复用工作区。
没有 heap 分配器、空闲链表或三份常驻大数组。
具体读写规则与持久层尚未拆开的兼容限制见[参数所有权](parameter_ownership_v1.md)。

## 工程与工具接线

双板完整APP共享APP/Motor/Parameters/Communication实现，按功能使能选择母线电流和诊断模块，
通过相同BSP功能接口装配。板级资源、采样比例及能力归`platform/config/boards/`；
电机物理模型与型号标定速度归`motor/profiles/`；速度估计公共默认值归
`motor/profiles/encoder_feedback_defaults.h`。
AT32两个测试目标分别预包含`app/config/targets/r40_app_off.h`和
`r40_app_commissioning.h`，由公共目标头集中提供组合选择及临时热数据。
这些头只有编译期输入，不复制算法、不新增运行时选择、也不改变BSP契约。
旧板维持既有默认选择；新板限时目标的800Hz PLL和速度PI仍是测试配置。
具体迁移、对照及未验收项见[双板配置收敛计划](../plans/active/2026-10-24-dual-board-config-v1.md)。

统一管理入口为仓库根目录`YG-ESC.uvmpw`，仅引用旧板APP及新板Off/Commissioning工程。
三个子工程独立构建，公共代码保持引用同一份；默认活动工程为新板Off，初始不勾选批量构建。
使用方法见[Keil工作区](../guides/keil_workspace.md)。
旧板主工程为 projects/stm32g4/MDK-ARM/ESC-APP-STM32G431.uvprojx。
Loader 配套 APP 为同目录 ESC-APP-STM32G431-LoaderApp.uvprojx；两者复用同一业务源码。
两板各自`projects/<board>/application/main.c`定义真实`main()`，模板逐字节一致。
入口固定调用`Bsp_Init → App_Init → Bsp_Start → App_Start`，前台循环调用`App_Process`。
MCU的system.c负责硬件初始化和IRQ接线，app_startup.c负责共用对象、协议绑定和后台业务。
不存在相邻platform_app.c或app_platform.h转发层；各板的具体职责见[BSP架构总图](bsp_architecture_v1.md)。
旧板普通APP与Loader配套APP选择同一份板级入口；四个完整目标各只链接本板一个main。
两板application源码均手工维护。IOC的MainLocation保持`Core/Src`，
生成的Core/Src/main.c不能加入完整APP工程；再生成后核对源清单，避免双入口。
本轮不执行CubeMX再生成，不能以静态配置检查代替生成器实测。
实施和验证见[双板入口分离](../plans/active/2026-10-24-board-application-entry-v1.md)。
普通 APP 独立从 Flash 起始地址链接；只有配套工程使用 Loader APP 槽，不能混称两者布局。

双板完整APP的Keil工程保持同名七组：`00_App`、`10_Motor`、`20_Parameters`、
`30_Communication`、`40_Platform`、`50_Shared`、`90_Vendor`。
物理子目录不扩展为更多Keil分组，工程树只登记源文件，不登记.h。
新板沿用旧板ARMCC 5.06、C99及公共逐文件优化；公共组的源文件与顺序一致。
新板入口和采集归Platform，可选只读遥测归Communication，SEGGER RTT归Vendor。
设备包、存储区、启动文件、SDK及目标配置预包含仍由各板独立选择；
新板保持禁止自动下载及关相/限时约束。CubeMX Core/Drivers内部相对位置不变。
本轮编译器切换后的新板镜像尚未实机验收，不能继承先前ARMClang镜像的快环测量。
对照与验收见[Keil工程对齐](../plans/active/2026-10-24-keil-project-alignment-v1.md)。
VS Code 按实际目录浏览；原生测试 include 路径统一由 tools/project_paths.py 提供。

目录迁移映射见 [path_migration.json](path_migration.json)，
本轮目标、回退和证据见[目录职责收口记录](../plans/completed/2026-09-29-directory-ownership-v1.md)。
历史路径转换夹具和 git show 引用保持其原版本路径，当前测试输入更新为实际目录。

## 校验与历史信息

目录整理核对源文件内容不变、实际工程路径、架构方向与 normal HEX；不替代实机功能验收。
历史目录迁移、USB 移除、全局状态收口和去重证据分别保存在 docs/plans 与 docs/reports。
当前默认验证不会烧录或操作电机；实机测试由专用会话在明确台架场景下执行。
