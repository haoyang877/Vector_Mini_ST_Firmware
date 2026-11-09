# 单轴电机 APP 层重构与维护计划 v1

> 已被 [v2](2026-11-07-app-refactor-v2.md) 取代；本文件仅保留历史设计，不再作为实施口径。

日期 2026-11-07。适用于 `the-system-refactor` 的 STM32G431、AT32F456 产品 APP，以及复用这些源码的工程。设计基线为 `ed18e5a`。此版本为历史方案，实施结果见 v2 的验收记录。当前工作树另有 Keil 工作区和两个目标工程的本地改动；实施时记录其差异，分开处理工程输入。

范围说明：`app_rtt.c` 是测试辅助文件，不纳入生产 APP 的目录调整、接口设计、命名整改或四项验收。双板构建使用现有工程配置。

## 四项验收要求

这里记录的是 `ed18e5a` 的现状与计划验收条件。目标结构和接口原型属于设计；四项“通过”须有实施后同一版代码的证据。

| 要求 | 当前判断与证据 | 怎样才算完成 |
| --- | --- | --- |
| 命名规范 | 部分符合。文件名多数符合 snake_case，但 `s_motor_configuration` 带文件私有前缀，却有外部链接 | 按 [STYLE.md](../../../STYLE.md) 审查每个改动的公共头、定义和调用方。文件、类型、函数前缀、单位与存储可见性符合约定；中断向量/第三方 ABI 的固定名称登记为例外。format、lint、两板编译与符号消费者检查均有结果 |
| 可重入性 | 当前不满足。生命周期、位置控制、标定及协议路径仍含唯一可变状态。仅在公共函数上加实例参数不足以隔离这些路径 | 在真实控制、标定和协议代码中，从 A 未返回的调用嵌套进入 B 的同一入口；两者各自的状态、回复、PWM 和 Flash 目标与分别运行的参考一致。同轴在外环提交前注入 STOP/故障，确认旧结果不能写回；重复输入、重初始化、资源冲突按逐接口契约处理 |
| 解耦性 | 部分符合。纯五态迁移已有独立测试，但外环计算、运行准入和提交集中在 Lifecycle，静态协议回调缺实例路由 | 板级绑定实例与硬件，Axis 调度已有算法，Lifecycle 持有许可及最终提交权，协议适配只传命令和值快照。正向测试跑通真实路径；依赖检查可检出反向依赖，并能用两组模拟外设查出错误路由。展平路径时同步检查规则 |
| 简洁易读 | 部分符合。启动/主循环顺序清楚，但读标定收尾需在一处追踪停稳、拟合、存储和状态结束，文档还保留旧行为 | 评审者从三个入口分别追踪一拍控制、一次 STOP、一次标定保存，指出写入者、执行上下文和失效点。每个新增公开接口有真实调用方与明确结果；审查删除了哪些转发、重复状态及旧入口，同时以行为回归确认必要的临界区与保护仍有效 |

首个生产改动按“速度目标 → 外环计算 → STOP 抢占 → 尝试提交旧结果”纵向贯通。现场产品仍只分配一个轴实例；第二个实例用于离线验证真实重入。P0–P5 每阶段提交列出四项的通过、失败或未验证状态。失败项保留触发用例并修复后重跑；门禁仅对其实际覆盖的范围给出结论。

## 当前 APP 功能清单

按基线源码中的调用与条件编译整理。表中“编排”表示 APP 调用 motor、communication、parameters 或 BSP 完成相应工作。具体协议格式以[协议契约](../../protocols/README.md)为准。

| 功能 | APP 当前实现 | 主要入口 / 适用条件 |
| --- | --- | --- |
| 1. 系统启动与设备身份 | 初始化单轴运行对象、协议节点和设备能力，读取 UID 与目标布局，启动通信和健康监督。初始化或启动失败走既有关相路径 | `App_Init / App_Start`，两板板级 main |
| 2. 参数装载与轴角色选择 | 读取持久记录，装配电机、应用和设备参数，恢复编码器/Hall 标定数据；AT32 根据有效记录选择左右轮、ROLL、PITCH、YAW 中的一种角色。无效产品配置影响运行准入 | `MotorAxis_Init` 调用 parameters 与 BSP；一块板控制一根轴 |
| 3. 周期调度与前台任务 | 派发快环、外环、监督及前台；前台处理位置规划、标定发布、协议、Loader 和看门狗 | 两板 ADC 快环 20 kHz、监督 2 kHz。外环 STM32 为 2 kHz，AT32 为 4 kHz |
| 4. 采样与反馈编排 | 组织电流零偏、母线与相电流采样、编码器更新、按模式运行磁链观测；有 Hall 的轴还处理输出轴定位与角度跟踪 | `MotorAxis_SampleFast / OuterTick`；Hall 和温度路径取决于板级能力 |
| 5. 状态迁移与功率启停 | 从 IDLE 接受启动或标定请求，管理启相许可、停机意图、会话失效和实际关相确认。SELF_TEST 枚举当前为占位 | `MotorStateMachine_Step`、`MotorLifecycle_RequestEnable / BeginFastTick / RequestRemoteStop`、STOP 服务 |
| 6. 模式、目标与运动控制 | 接受电流、速度、位置、Vq、开环电压、开环电流六种模式。运行中校验同模式目标、限额及行程；调用位置规划、外环与 FOC，提交仍有效的输出 | 命令 117 启动、116 改目标；`MotorLifecycle_SetTarget`、`MotorAxis_ControlFast / ProcessPositionPlan` |
| 7. 故障保护与恢复 | 检查与运行许可相关的采样、零偏、参数、母线及编码器条件，处理运行故障、外环超时关相和受控清错。清错后回到可重新请求启动的状态 | `MotorLifecycle_SupervisorTick / RequestClear`、`MotorAxis_OuterOverrun`；按板型及模式适用 |
| 8. 标定与持久化 | 接受编码器线性化 LUT、电气零位、机械零位请求；按轴类型执行 Hall 联合采集、停稳、候选发布、原会话复核和关相保存。写入或拟合失败进入相应故障路径 | 命令 160 的 kind 1、2、6；`run_active_calibration_cycle`、`MotorAxis_ProcessCalibrationPublication` |
| 9. CAN 业务与状态反馈 | 将命令映射为启停、目标、清错和标定请求，处理目标序号/重复请求；将运行状态、故障及有效测量转为线路单位。108 查询状态，124 是 116 关联反馈 | `AppProtocol_Refresh`、`Protocol_Motor / Protocol_Jobs`；中断搬帧与帧编解码由通信层承担 |
| 10. 运行健康监督 | 根据快慢环进展检查应用健康；按板级支持情况喂狗，失败时沿用关相处理 | `start_watchdog`、`service_watchdog` |
| 11. Loader 交接与镜像确认 | 支持的镜像登记并锁定停机、确认关相、生成票据，回复物理发送成功后复核并进入 Loader；健康窗口满足条件后可确认试运行镜像 | 56/57，仅配套 G431 LoaderApp；`handle_entry / handle_tx_complete / confirm_image` |

对外能力边界：在线参数读、写、保存目前未开放；标定进度查询和取消命令目前未开放。STOP 可以中断运动。APP 的标定收尾可按既有流程保存参数。独立 Loader 的镜像擦写、协议层的帧编解码、BSP 的寄存器驱动、motor 的控制算法分别由对应模块实现。

### 当前源码核对（HEAD `ed18e5a`，2026-11-07）

独立核对确认以下事实，作为阶段 0 的基线。其中多项与现行文档不一致，必须先统一再开始结构迁移：

- **标定会写 Flash**：[motor_axis.c](../../../firmware/app/motor_axis.c) 的 `MotorAxis_ProcessCalibrationPublication` 在关相前台的临界区内执行 `BspFlash_Write(&s_axis_record, sizeof(s_axis_record))`，三类标定共用该收尾。[APP README](../../../firmware/app/README.md)、[firmware README](../../../firmware/README.md)、[文件职责](../../architecture/app_file_responsibilities_v1.md)、[软件架构](../../architecture/software_architecture.md) 和[协议说明](../../protocols/README.md) 仍写“只改 RAM/不自动保存”，与代码及[标定指南](../../guides/calibration_process.md)矛盾。协议文档是兼容契约，须在 P0 明确以哪一侧为准；若确认代码行为正确，应同步修订协议说明、黄金向量和测试断言。
- **外环频率分板**：[config.h](../../../firmware/config/config.h) 中 AT32 的 `OUTER_LOOP_DIVIDER=5`，由 ADC 每五个有效快拍经 PendSV 执行 4 kHz；STM32 为 `SUPERVISOR_DIVIDER=10`，在 2 kHz 监督拍内执行。[软件架构](../../architecture/software_architecture.md) 的“位置/速度外环在 2 kHz 生成 Iq”只对 STM32 成立。
- **测量字段有有效位**：`AppProtocol_Refresh` 在 `measurements_valid` 时按字段写 108/124 的 `valid_bits`；[软件架构](../../architecture/software_architecture.md) 的“测量字段不宣称有效”与当前实现不符。
- **前台顺序**：`App_Process` 依次执行位置规划、标定发布、状态刷新、协议、Loader、看门狗；位置规划确实在前台运行。
- **Flash 窗口**：标定保存在 `BspSystem_EnterCritical` 内完成；AT32 在写入期间把 WDT 重载从 511 提升到 4095（具体超时取决于 LICK 分频）。该窗口的实际耗时及对应用看门狗和快慢拍进展的影响应在 P2/P5 用同板同工况测量，不能只凭源码推断。

## 1. 目标和现状

验收目标是让维护者能从一个执行入口追完一次采样、一次启停、一次标定保存或一次 Loader 交接，明确每一步由谁执行、写哪个对象、什么条件会让结果失效。整理范围是 APP 及与它直接相连的接口和工程输入。产品控制律、整定值、总线协议、持久格式、Flash 布局和板级动作时序均以基线为兼容契约。

| 现有位置 | 具体问题或值得保留的设计 | 本轮决定 |
| --- | --- | --- |
| [app.c](../../../firmware/app/app.c) | 启动和主循环可顺序阅读；同一文件还实现设备身份、看门狗、Loader 握手与镜像确认 | 保留主入口；有独立事务状态的 Loader 交接单独组织 |
| [motor_axis.c](../../../firmware/app/motor_axis.c) | 单轴运行对象集中，但启动装载、快环分派、外环、Hall 标定、参数写入共处；标定收尾跨多个上下文 | 保留轴对象和时钟入口；提取 APP 专用标定流程 |
| [motor_lifecycle.c](../../../firmware/app/motor_state_machine.c) | 启停准入、关相确认和会话编号有明确作用；外环副本计算、输出字段提交又与速度和位置控制细节交织 | 集中运行许可及失效处理，将外环执行顺序交给 Axis |
| [motor_state_machine.c](../../../firmware/app/motor_state_machine.c) | 纯状态迁移可单独测试；SELF_TEST 是现有占位 | 纯迁移函数移入生命周期编译单元，保留五态值语义和逐状态测试 |
| [app_protocol.c](../../../firmware/app/app_protocol.c) 与 [STOP 服务](../../../firmware/app/app_protocol.c) | 协议适配已有静态绑定、线缆单位换算、序号去重和实际关相确认 | 保留协议边界，收紧读写权限；沿用 ACCEPTED、OK 和失败结果 |
| [共享接口](../../../firmware/app/motor_lifecycle.h) | Watch 和 CAN 的读者与一致性要求不同；轴反馈只读指针本身不能保证同拍快照 | 分清值快照、即时只读观测和运行时私有状态 |

优先级最高的两处是标定提交和外环发布。先贯通有双板基线的速度外环切片，再用取消、会话替换和失败传播回归迁移标定。10 月的 [APP 核心精简](../active/2026-10-30-app-core-v2.md) 是历史基线。当前前台还运行位置规划，联合标定可以写 Flash。应在阶段 0 修正 [APP README](../../../firmware/app/README.md)、[文件职责](../../architecture/app_file_responsibilities_v1.md) 和 [架构说明](../../architecture/software_architecture.md) 的相应描述。

**P1 的代码评审切口。** 现有 `MotorLifecycle_OuterTick` 在一处完成快照、计算、复核和写回。迁移后由 Axis 调用 Lifecycle 获取本次计算输入，在允许抢占的上下文运行已有外环，再把候选结果交给 Lifecycle 的单一提交入口。输入副本、临时结果与调用绑定，包含原实例、epoch、模式及提交所需字段；实例 A 的结果交给 B 必须被拒绝。提交入口在与基线等价的保护范围内完成最终复核及允许字段的写回。严禁形成“先查询允许 → 离开保护区 → 稍后裸写目标”的窗口。对同 epoch 内新来的目标单独比较原行为，避免把导出的参考值写回到前台命令字段。

## 2. 目标结构与依赖

建议 APP 使用单层 `firmware/app/`。按保留现有 CAN 指令、实时控制和标定保存的产品范围，核心由五个 C 文件组成。头文件与实现同目录，只向真实调用方公开相应声明。

```text
firmware/app/
  app.c                实例初始化、主循环、调度入口、健康监督
  motor_axis.c         单轴状态、采样与反馈、位置规划、快环和外环编排
  motor_lifecycle.c    五态迁移、启停与目标的准入、故障恢复、功率许可
  motor_calibration.c   标定工作步骤、候选结果、停稳与保存
  app_protocol.c       CAN 命令转换、状态反馈、STOP 回复与 token
```

G431 LoaderApp 在同目录额外编入 `app_loader.c`，负责交接与镜像确认。算法、报文编解码和 Flash 驱动分别归 motor、communication 和 BSP。五态迁移仍为纯函数；当前唯一生产调用方是 Lifecycle，可放在同一编译单元，保留[逐状态测试](../../../tests/unit/native/five_state_lifecycle_test.c)。原 interfaces 中的状态、结果、目标和快照归生命周期的头文件，STOP 回复与 token 归协议端点。直接请求生命周期停机后读取实际关相状态，保持原 ACCEPTED/OK 判定和 token 复用规则。

### 可重入性契约

**实例隔离与同轴抢占分别验收。** 不同实例允许在 A 尚未返回时进入 B 的同一函数。同一轴的 ADC 可以抢占外环，STOP、故障或会话变更可以作废尚未提交的结果。无副作用的计算可自由嵌套；同轴每个采样拍只推进一次，共用 PWM 通道和 Flash 控制器的硬件提交须串行。对每个有副作用的入口标注允许的调用上下文、可抢占点及冲突行为，依照已有契约选择延后、忙或 overrun；ISR 不等待被抢占者释放锁。对这些串行入口只承诺规定交错下的安全性。

**状态绑定。** 板级静态分配一个 AppContext，启动时绑定轴、协议端点和真实硬件资源。组装入口接收应用实例，计算函数接收所需的轴或会话小对象。轴持有配置、FOC、PI、反馈和生命周期；标定会话持有工作区、候选和停稳计时；协议端点持有队列、去重、快照和 STOP token。看门狗与 Loader 事务按所属应用或物理设备绑定。Watch 直接观察真实实例，ISR 依据中断源取得对应实例。单轴产品只静态分配一套工作区，多实例测试分别分配，并对比 map、栈与 ROM/RAM。

**必须追到下层。** 当前 [MotorAxis](../../../firmware/app/motor_axis.c) 共享轴对象、标定暂存、局部静态位置规划缓冲和停稳计时；[Lifecycle](../../../firmware/app/motor_state_machine.c) 使用唯一的 `g_motor_context`。调用链还包括[位置控制](../../../firmware/motor/control/position_control.c)的 `s_state`、[编码器标定](../../../firmware/motor/calibration/encoder_calibration.c)的 `s_lut/s_zero`、[CAN 协议](../../../firmware/communication/yg_protocol.c)的静态队列和节点，以及 [FOC 输出](../../../firmware/motor/control/foc_output.c)直接提交 PWM 的调用。逐条登记可变静态量和回调，迁为调用内临时量、实例成员或独占设备状态。只读常量可共享。一条物理 CAN 总线由一个驱动搬帧，再按节点分派；每轴明确绑定 PWM 端点和参数槽，避免把第二实例的输出写到第一实例。

**提交与失效。** 外环使用调用自己的快照和临时结果，在短临界区复核实例、原 epoch、模式、STOP 与故障，再提交限定字段。同代际目标变更按现有协议及命令序号语义单独核对。标定先准备候选，最终复核与写入沿用关相事务的临界区；写入开始前已受理的 STOP 使旧候选失效。Flash 擦写已经开始后，后续请求按硬件实际完成顺序处理。保留原有中断优先级、屏蔽级别、超时关断和看门狗健康检查。

**完成判据。** 两组独立实例与模拟外设先各自运行，建立参考输出；再在 A 的调用途中嵌套调用 B 的相同入口，比较 PI、状态、token、队列、标定结果、PWM 写入和保存目的地。对同一轴在快照与提交之间注入抢占、STOP 或故障，断言旧结果无法提交、输出不会重新开启；验证重复快拍、外环超时、同一设备争用和 Flash 故障。覆盖实际领域算法与协议对象；使用总是成功的桩不能证明这些下层实现可重入。保留双板编译、RAM 预算和快环时序对照。

### 命名和编程规范

实现沿用仓库的 [STYLE.md](../../../STYLE.md) 和 [ARCHITECTURE.md](../../../ARCHITECTURE.md)。下面是它们在 APP 的具体用法。文件用 snake_case，同一模块的源文件和头文件同名。公共函数用 `Module_ActionObject`，每个模块一个前缀；模块内部用 `static` 小写下划线函数。类型用 PascalCase，枚举常量和宏用 UPPER_SNAKE_CASE，参数和成员用 snake_case。例：`MotorLifecycle_RequestEnable`、`MotorControlResult`、`MOTOR_CONTROL_ACCEPTED`、`speed_target_rad_s`。采用既有的 Foc、Pi、Pwm、Adc、Lut 及完整的 Calibration、Current 等词形。

Init 表示一次性建立实例，Configure 表示应用配置，Reset 明确复位范围；Request 表示受理意图，Step/Tick 对应一次执行，Publish/Commit 对应提交，Read/Get/Is 对应读取。返回结果区别“已受理、已关相、已保存、已失败”。数值名称带单位或表示法（如 `position_rad`、`current_a`、`now_ms`）；序号、控制拍计数和 epoch 分开命名。

使用四空格、Allman 大括号和仓库格式工具。公共头自包含并写中文 Doxygen，给出 @brief、每个 @param、非 void 的 @return，通过 @note 交代上下文、阻塞、并发及硬件效果。常量表可用文件私有 `static const s_...`；可变工作数据放进对应实例或由调用者提供。中断向量与第三方回调遵守原 ABI，在板级边界绑定实例。上下文借用指针必须覆盖对应回调生命周期，重新初始化前先停机并解绑事件源。避免在调用间共享临时缓冲、分配堆内存、用大块关中断包住业务计算或把 Watch 符号作为写入口。当前可观察符号与目标实例字段的映射随迁移登记；保留协议、存储与硬件接口的外部名称。

### 逐文件接口草案

以下是目标 API，不是当前头文件中的声明。每一对 .h/.c 对应一个模块：头文件给出调用契约，源文件拥有实现和私有辅助函数。签名中 const 指针表示该入口通过此指针只读；未标出完整类型的名称用于列举操作。所有实例由调用者静态提供，同一对象的资源从初始化到停机保持有效。

| 源文件 / 头文件 / 前缀 | 建议接口（首参均为对应实例指针） | 调用与返回契约 |
| --- | --- | --- |
| app.c / app.h / App_ | `bool App_Init(AppContext *app, const AppBindings *bindings)`；`bool App_Start(AppContext *app)`；`void App_Process(AppContext *app)`；`void App_SupervisorTick(AppContext *app)` | main 在 BSP 准备后 Init、BSP 开放采样后 Start；失败保持关相。已启动实例重复初始化应拒绝且不改动运行对象。Process 仅由前台推进，SupervisorTick 仅由 2 kHz 回调驱动 |
| motor_axis.c / motor_axis.h / MotorAxis_ | `MotorAxis_Init(axis, bindings)`；`MotorAxis_SampleFast(axis)`、`MotorAxis_ControlFast(axis)`；`MotorAxis_SupervisorTick(axis)`、`MotorAxis_OuterTick(axis)`、`MotorAxis_OuterOverrun(axis)`；`MotorAxis_ProcessPositionPlan(axis)`、`MotorAxis_NodeId(axis)` | Init 建立单份配置和运行对象，无效配置沿用锁存故障；Sample/Control 在同一 ADC 拍依次执行。前台规划可抢占；外环提交受 Lifecycle 复核。Tick 为 void，故障经实例传播，NodeId 返回 uint8_t |
| motor_lifecycle.c / motor_lifecycle.h / MotorLifecycle_ | `MotorLifecycle_Init(life, bindings)`；`RequestEnable(life, mode)`、`SetTarget(life, const MotorTarget *target)`、`RequestCalibration(life, kind)`、`RequestClear(life)` 返回 MotorControlResult；`RequestRemoteStop(life)` 返回受理 bool；`ReadStatus(life)` 返回 MotorStatus 快照 | 前台命令校验范围、状态和代际，读取不推进状态。协议 STOP 的 OK 仍以实际关相为准。快拍入口 `BeginFastTick(life)`、`PostOutcome(life, outcome, epoch)` 和监督入口 `SupervisorTick(life)` 保持优先级、动作和故障顺序。此行短名均使用 MotorLifecycle_ 前缀 |
| motor_calibration.c / motor_calibration.h / MotorCalibration_ | `MotorCalibration_Init(session, bindings)`；`MotorTaskResult MotorCalibration_StepFast(MotorCalibration *session, MotorCalibrationType kind)`；`MotorCalibration_SampleOuter(session, sample)`；`MotorCalibration_ProcessPublication(session, progress, progress_context)`；`MotorCalibration_Cancel(session)` | Axis 在有效标定会话内推进快拍，外环只在约定阶段提交 Hall 样本；任务结果由 Lifecycle 按调用前捕获的 epoch 判定。前台发布还需复核原 epoch 和关相；progress 使用所属应用的回调上下文；Cancel 清理本次工作区，功率与主状态仍由 Lifecycle 处理 |
| app_protocol.c / app_protocol.h / AppProtocol_ | `bool AppProtocol_Init(AppProtocol *adapter, ProtocolEndpoint *endpoint, MotorLifecycle *life)`；`void AppProtocol_Refresh(AppProtocol *adapter)` | Init 在上电时绑定端点及回调实例；Refresh 在前台取得快照、缩放并设置有效位。命令回调 `start_motor`、`set_motor_target`、`stop_motor` 等用 static；每次通过端点传入所属实例。帧解析、收发队列和每轮预算仍由 communication 管理 |
| app_loader.c / app_loader.h / AppLoader_ | `AppLoader_Init(loader, bindings)`；`AppLoader_Process(loader, now_ms)`；`AppLoader_OnTxComplete(loader, ticket, success)`；交接和镜像确认作为绑定到实例的私有回调 | G431 LoaderApp 前台处理超时、健康窗口和停机锁。发送结算使用原 64 位票据。重复、失败、超时或实例不符不触发复位；普通 APP 的能力响应保持基线 |

Lifecycle 的其余调用契约按真实消费者分组。在标定路径，`GetCalibrationType`、`IsCalibrationRunnable`、`ReadPendingCalibrationPublication` 是读取操作，`ContinueElectricalZero` 和 `CompleteCalibrationPublication` 只允许有效会话推进。在外环路径，Axis 调用已有 MotorOuterLoop 计算；Lifecycle 提供取得当前许可与条件提交所需的内部操作，最终提交时再次验证原实例及 epoch。Loader 依赖 `LockStop`、`IsStopped`、`ReadBootHealth`；所有这些调用都显式带所属实例，区分请求、只读查询和实际硬件结果。五态与事件采用值传递，纯状态迁移不读取实例全局量。

**类型与依赖。** AppContext 拥有或绑定一个 MotorAxis 和一个协议端点，按 G431 镜像能力绑定 Loader。MotorAxis 持有 MotorLifecycle、MotorCalibration 及轴数据；生命周期与标定只借用所需对象或参数。bindings 固定 PWM/ADC、Flash 区、时基及回调所属实例；涉及同一物理设备时在设备层仲裁。底层模块不反向持有整个 AppContext。需要运行期间服务看门狗的回调传入显式 context；既有无上下文的 ProtocolMotorHandler 需迁移端点及全部调用方。对跨层 C 内部接口的签名调整，同一阶段同步生产、Keil 和测试，线路报文保持原格式。

**有副作用接口的类型与返回值。** 下面列出上述简写的签名草案，供评审核对头文件。`MotorAxisBindings`、`MotorLifecycleBindings` 等是初始化时借用的资源集合；各模块仅包含自己的真实依赖。资源和回调上下文在对应实例使用期间保持有效。

```c
/* motor_axis.h：板级及组装入口；快拍的两个调用顺序固定。 */
bool MotorAxis_Init(MotorAxis *axis, const MotorAxisBindings *bindings);
void MotorAxis_SampleFast(MotorAxis *axis);
void MotorAxis_ControlFast(MotorAxis *axis);
void MotorAxis_SupervisorTick(MotorAxis *axis);
void MotorAxis_OuterTick(MotorAxis *axis);
void MotorAxis_OuterOverrun(MotorAxis *axis);
void MotorAxis_ProcessPositionPlan(MotorAxis *axis);
uint8_t MotorAxis_NodeId(const MotorAxis *axis);

/* motor_lifecycle.h：值契约和事务裁决；取结果时区分受理与实际完成。 */
void MotorLifecycle_Init(MotorLifecycle *life, const MotorLifecycleBindings *bindings);
MotorControlResult MotorLifecycle_RequestEnable(MotorLifecycle *life, MotorControlMode mode);
MotorControlResult MotorLifecycle_SetTarget(MotorLifecycle *life, const MotorTarget *target);
MotorControlResult MotorLifecycle_RequestCalibration(MotorLifecycle *life,
                                                    MotorCalibrationType kind);
MotorControlResult MotorLifecycle_RequestClear(MotorLifecycle *life);
bool MotorLifecycle_RequestRemoteStop(MotorLifecycle *life);
MotorStatus MotorLifecycle_ReadStatus(const MotorLifecycle *life);
void MotorLifecycle_BeginFastTick(MotorLifecycle *life);
void MotorLifecycle_PostOutcome(MotorLifecycle *life, MotorTaskResult outcome, uint32_t epoch);
void MotorLifecycle_SupervisorTick(MotorLifecycle *life);

/* motor_calibration.h：标定会话工作；回调明确所属应用。 */
typedef void (*MotorCalibrationProgress)(void *context);
void MotorCalibration_Init(MotorCalibration *session, const MotorCalibrationBindings *bindings);
MotorTaskResult MotorCalibration_StepFast(MotorCalibration *session, MotorCalibrationType kind);
void MotorCalibration_SampleOuter(MotorCalibration *session,
                                  const OutputEncoderCalibSample *sample);
void MotorCalibration_ProcessPublication(MotorCalibration *session,
                                         MotorCalibrationProgress progress,
                                         void *progress_context);
void MotorCalibration_Cancel(MotorCalibration *session);

/* app_loader.h：G431 LoaderApp 交接与镜像确认。 */
bool AppLoader_Init(AppLoader *loader, const AppLoaderBindings *bindings);
void AppLoader_Process(AppLoader *loader, uint32_t now_ms);
void AppLoader_OnTxComplete(AppLoader *loader, uint64_t ticket, bool success);
```

Init 的 bool 表示软件对象和资源能否完成绑定；合法回退到默认参数并锁存运行故障是既有启动结果。内部 void Init 的前提由组装入口一次核对，仅在采样开始前初始化。AppLoader 的初始化与回复回调均在绑定的前台协议上下文执行。校准回调可在本实例拟合时服务其看门狗，不得递归推进同一会话。对于确有同步调用者的查询才声明公开原型；参数编解码、协议回复处理函数、回滚辅助和纯迁移实现按其消费者确定私有可见性。

**头文件示例（拟议）**
```c
/**
 * @brief 请求本实例以指定模式启动。
 * @param life 已初始化且在本次调用中有效的生命周期对象。
 * @param mode 请求的电机控制模式。
 * @return ACCEPTED 表示已登记启动意图，其他值表示拒绝原因。
 * @note 前台调用；ADC 在实际启相前复核会话、停止和故障条件。
 */
MotorControlResult MotorLifecycle_RequestEnable(MotorLifecycle *life, MotorControlMode mode);
```

## 3. 执行上下文与数据写入

| 上下文 | 入口与频率 | 允许的工作及交接 |
| --- | --- | --- |
| 上电 | 板级 main | BSP 准备关相硬件 → APP 读取记录、绑定单轴与通信 → BSP 开放采样 → APP 启动通信与健康监督。保留 AT32 周期计数器及两板失败出口 |
| ADC 快中断 | 两板 20 kHz，周期 50 µs | SampleFast 完成采样与估计 → ControlFast 复核运行/标定许可 → 执行互斥的控制或标定任务 → 发布结果、处理故障、推进快拍计数。AT32 的外环派发保持在这条链之后 |
| 外环 | AT32 每五个有效快拍由 PendSV 执行，4 kHz；STM32 在 2 kHz 监督拍内执行 | 获取受保护的输入副本 → 在可抢占区计算 → 按原屏蔽级别复核代际、模式、故障和停机 → 提交限定字段。AT32 上个外环未完成时沿用现有 overrun 关断路径 |
| 监督中断 | 两板 2 kHz | 执行温度维护、停机消费、故障恢复、标定结束和健康计数；板级能力决定哪些采样实际启用 |
| CAN 接收中断 | 硬件事件 | 有界搬帧入队，每次最多 3 帧。命令解析和业务调用在前台执行 |
| 前台 | App_Process | 按当前次序处理位置规划、标定发布、状态刷新、协议、Loader、看门狗。可抢占的拟合与 CRC 在临界区外，最终写入遵守平台关相、停机前台及回读契约 |

评审任何可变字段时注明**归属、允许写入的上下文、读取方式、失效条件**。例如速度 PI 由轴持有，外环计算副本，仅有效会话可写回；停机会同时清除或作废相关状态。Hall 原始采样由快拍写，外环读取一组一致数据时沿用现有短临界区。标定工作区从请求开始独占到发布或取消。协议状态用一次值快照形成。

**重点状态如何管理**

| 状态或资源 | 权威来源与写入时机 | 调用方拿到什么 |
| --- | --- | --- |
| 主状态、epoch、待启停、功率许可 | Lifecycle 在对应的前台、监督及 ADC 入口按原临界区更新；纯迁移函数只计算下一状态 | 命令方拿受理结果；计算方拿本次 epoch；发出功率许可与 ADC 消费许可分别记录 |
| 位置/速度命令与 Id/Iq 参考 | 前台经 Lifecycle 准入后写目标；外环对其负责的字段作条件提交；停机复位会使旧参考失效 | 各模式沿用各自字段和单位；命令值、规划值、测量值分开读取 |
| FOC、速度 PI、编码器与观测器 | Axis 持有一套对象；快环更新 FOC/反馈，速度 PI 副本由外环提交，停机按现行顺序复位 | 控制算法借用类型明确的状态；跨中断的一致读取使用已有临界区或值快照 |
| 活动配置与持久记录 | Axis 在启动解码；标定会话可更新指定字段；关相前台串行使用整轴记录暂存区 | parameters 提供转换与校验；存储成功、失败以及 RAM 已发布的时点分别可观察 |
| 标定工作区、Hall 三元组、候选结果 | 标定阶段独占工作区；ADC 写三元组，外环按原临界区读；前台准备候选时携带原会话编号 | 生命周期接收任务结果或保存结果；丢弃旧会话不能影响新会话的候选数据 |
| CAN 状态快照、Loader 票据 | 前台适配层从生命周期读取快照；Loader 事务独占票据并在发送结算后消费 | CAN 用既有有效位及无效值；升级回复入队与物理发送完成各自有明确语义 |

**三个特别容易误改的顺序**

1. 启动受理时登记意图；外环准备完成后登记本次启相许可；快拍再次检查并启相；确认数字相通道开启后才登记 RUNNING。STOP 到达要撤销待启动意图，关相结果决定 STOP 回复。清错后需重新发起启动。
2. 标定流程按类型分支。带 Hall 的 LUT 会话先收集数据、关相等停稳、发布 LUT，再沿原会话进入电零位并准备保存。直驱轴、单独电气零位和机械零位沿各自现有路径。准备或拟合期间的 STOP、故障、会话替换使旧结果失效；最终复核、活动 RAM 更新、Flash 写入/回读、故障或完成通知的先后按当前实现固定。完整扫圈后 Hall 候选不合格的失败记录行为也要保留，单独验证它与中途取消的区别。
3. Loader 的停止受理、实际关相、带票据回复、回复物理发出、复核并进 Loader 是不同事件。保留重复请求、超时、发送失败、试运行镜像健康确认语义。看门狗仍按快慢环进展判断，保留停机拟合期间的健康服务回调。

## 4. 迁移步骤和交付物

每阶段可单独评审、验证和回退。下表中的文件是主要改动范围；涉及增加或移动 C 源文件时，在同一阶段同步实际引用它的 Keil 工程、生成工具、原生测试构建清单及文档链接。

P0 记录每条链上的共享可变状态、端点和 IRQ 绑定。按单一提交先展平 APP 的物理路径，更新 Keil 的 FilePath、IncludePath、生成器、原生测试构建清单、harness.toml 的目录依赖及文档链接。核对两板编译与原项目等价后，再迁移实例数据及接口。P1 保留纯迁移函数的直接覆盖，P4 合并 STOP 回调包装及类型头。

| 阶段 | 实施与交付物 | 进入下一阶段的条件 |
| --- | --- | --- |
| P0 固定基线 | 记录 HEAD、已有工作区差异、两板产品工程及 LoaderApp 的源文件和宏；形成当前启动、快环、外环、标定、协议、Loader 的调用顺序。修正文档中外环频率、标定 Flash 保存、状态反馈有效位和位置规划的描述；对协议文档与代码不一致处先做兼容决策 | 留存同一源码的 quick、PR、双板构建、map 和现有失败清单；识别已有测量和缺失的时序数据。工作区中的个人 Keil 设置可追溯 |
| P1 外环与生命周期 | 先由板级静态分配真实实例、贯通速度目标调用与 ADC/外环入口，显式绑定轴、生命周期、位置级联与规划状态，在 Axis 侧清楚展示快照、已有外环算法、复核和提交。生命周期保留准入、STOP/故障、会话编号与发出启相许可的权力；使用私有明确的调用保持计算和最终提交分离 | 先对照 A/B 嵌套运行与各自独立执行，再对照相同输入下的 PI、Id/Iq、位置/速度轨迹和状态；在外环计算期间插入 STOP、故障或新会话，验证旧结果不会生效；AT32 超时行为、双板调度频率及屏蔽区与基线对照 |
| P2 标定收尾 | 先整理 MotorAxis_ProcessCalibrationPublication 内部阶段，再移动到标定编排单元。实例化等待时间、Hall 扫描、LUT/零位会话、拟合及保存暂存区；直驱/Hall 共用最终提交，按原位置复核关相与 epoch | LUT→电零位→停机保存、单独机械零位、Hall 失败记录、取消、故障、代际改变、Flash 写入失败和重启恢复与基线一致；A 标定中重入 B 后两实例与各自参考一致；记录关相 Flash 窗口的实际耗时、最长临界区及对应用看门狗/快慢拍进展的影响 |
| P3 组合层与 Loader | 将板级中断及硬件资源绑定到实例，抽出 Loader 事务，绑定各应用的身份、健康计数与回复票据；把身份更新、应用健康与 Loader 确认之间的依赖保持显式。main 和 App_Process 可直接读出启动、调度和失败顺序 | 普通 STM32、AT32 与支持交接的 G431 LoaderApp 分别构建；空能力、超时、票据重放、发送失败、停机竞态及健康窗口回归通过 |
| P4 外部接口与状态反馈 | 按实例迁移协议节点、队列、服务回调及去重状态，合并 STOP 包装和值类型，核对 Watch 的实际消费者；公共头暴露值或只读视图。把模式映射、缩放、无效值、去重放在协议适配端。为运行态写入列出唯一责任入口 | 比对协议黄金帧、错误码、STOP token、重复目标处理、有效位、反馈字段顺序及 Watch 既有符号。核对 A/B 相同序号时各自的状态和回复互不串用，保持原参数能力 |
| P5 收口与交接 | 更新 APP 阅读地图、架构职责、调度表、工程清单及检查覆盖。按兴趣点审查剩余大文件；只有确实形成独立职责才再拆。记录每阶段差异、剩余缺口与回滚点 | 双板编译、PR 检查及受影响变体完成；资源与行为差异有解释。时序或实机项按实际测量单列状态；阶段结果归档，长期约定回到唯一文档入口 |

若 P1 外环迁移改变屏蔽顺序，或 P2 标定拆分引入大型 RAM 副本，应在原编译单元内完成职责整理并保留该阶段的对照结果，再决定后续迁移。每个阶段使用其前一已验证提交作回滚点，按依赖逆序撤回代码、工程输入和文档；保留用户 Keil 改动、已有参数记录与测试证据。

## 5. 验证与完成标准

| 关注点 | 复用的证据或测试 | 如何判断 |
| --- | --- | --- |
| 启停、保护、状态 | [生命周期原生用例](../../../tests/unit/native/five_state_lifecycle_test.c)、[调度测试](../../../tests/unit/native/test_app_orchestration.py) | 五态、SELF_TEST 占位、拒绝优先级、故障锁存、重复请求、关相确认、清错后无旧目标与基线一致 |
| 采样、外环与调度 | [ADC 派发](../../../tests/unit/native/test_adc_fast_dispatch.py)、[外环运行](../../../tests/unit/native/test_outer_loop_runtime.py) | 对比两板的调用次数、顺序、参考输出和中断边界；对快照、STOP 和 epoch 的交错另做行为核验 |
| 参数及标定 | [保存回归](../../../tests/unit/native/test_calibration_save.py)、[记录定义](../../../firmware/parameters/parameter_record.h)、[五轴配置](../../guides/at32f456_five_axes.md) | 覆盖五种节点、直驱和减速、有效/损坏/旧记录、存储失败；对照 schema14 基础记录和 OEN3 输出轴扩展字节、CRC 与复位恢复。分别检查取消无写入和完整失败记录 |
| 协议、遥测、升级 | [电机协议测试](../../../tests/unit/native/test_yg_protocol_motor_wire.py)、[Loader 运行测试](../../../tests/unit/native/test_loader_runtime.py) | 序号和去重、CAN 应答、状态有效位、Loader 票据和复位条件与基线一致 |
| 可重入性 | 两个实际软件实例、独立模拟外设；调用途中嵌套进入同一函数、同轴抢占注入 | 检查可变状态、过期结果、PWM/Flash 目标及单拍执行次数，记录不允许重入的硬件提交入口及其争用行为 |
| 工程与资源 | 两个产品工程、受影响的台架工程、独立 G431 APP 与 LoaderApp 的真实编译输入、map、镜像分区 | 解释 ROM、静态 RAM、栈及关键对象大小的变化；使用同工具链、同目标、同优化选项比较。单轴产品仅配置一套轴状态和标定工作区，记录实例字节数 |
| 实时行为 | 原有时序采样方法、同板同工况的前后对照 | 分别记录快环峰值、超 50 µs 次数、外环积压/overrun、最长临界区；测量覆盖启停、正常闭环和标定。关相后的 Flash 窗口单列。缺板或缺测量时标记未测 |

实现阶段先用 `uv sync --locked --extra dev` 建立仓库约定的环境，并用 `uv run python tools/run.py verify --profile pr` 做固件变更门禁。文档阶段使用 `verify --profile quick` 和文档检查。使用 `build_firmware --target normal` 与 `build_firmware --target at32f456-app-off`（通过 `tools/run.py`）分别检验产品构建；涉及 G431 LoaderApp 时同步其真实工程。触及链接、CubeMX、发布或实机验证时按 [AGENTS.md](../../../AGENTS.md) 选择 release 和明确的台架场景。将日志、map 与各 profile 的 summary.json 放在对应运行的 outputs 下。既有测试如果检查旧文件布局，更新定位同时保留原行为断言；缺少关键竞态覆盖时才补针对性用例。

**方案编写时的本机验证记录（2026-11-07）。** Quick 运行 `20261107T041618248331Z-ed18e5a4` 中，doctor、format、lint、工程布局及 16 项集成测试通过；124 项单元测试有 4 项失败，分别涉及 AT32 工程分组、RTT 厂商文件哈希、历史输出链接及 platform 根目录中的 stm32g4。全仓文档检查报告 104 个缺失的 outputs 链接，本计划及其索引有 0 个链接问题。此记录用于 P0 复核基线，本次交付为计划和功能清单。

## 6. APP 层日常管理

**需求归属。** 新增电机模式时，状态与启停资格归生命周期，控制律归 motor，命令编号与单位映射归通信和协议适配；新增持久字段归 parameters 并审查 ABI；新增 ADC 通道归 BSP 并在 Axis 的采样入口接入。新诊断字段先确定读者、节拍、单位和一致性要求，按需接入 CAN 状态快照或既有只读观测入口。新增板级能力先核对两板的工程选择和降级语义。

**评审责任。** 建议指定一位 APP 维护人管理运行顺序、接口边界及阅读地图，由控制、BSP、协议/存储维护人评审各自契约；小团队可兼任。变更跨层时，提交说明写清触发场景、状态写入者、执行上下文、失败后的状态、验证结果和回滚点。实际分派按团队成员决定。

**新增文件和 API 的判断。** 相同所有者、相同节拍、相同失败语义优先在原模块实现。跨模块新增接口时列出现有调用方，明确输入单位、运行上下文、可否阻塞、谁负责提交、失败怎样传播。中断临界区改动需给出抢占顺序和允许失效的交错。外层查询接口默认只读，命令的受理与动作完成各自有可验证的条件。

**每次 PR 如何管理。** 先选一个行为链及相应阶段，列出兼容项和影响目标。实现、Keil 输入、测试和现行文档一起评审。以阶段性行为和证据判断完成，文件数和行数只用于定位阅读负担。将发现的功能修复、参数整定和新增产品能力单列事项，便于结构变更回退。当前 harness 对 interfaces 设有独立层，其他 APP 路径共用 APP 层。展平后同步规则，通信继续只能接触其契约允许的对象；内部所有权首先由评审核对，若反复出现同一类违规，再为可判定的依赖增加规则并执行 PR 档验证。

**长期事实来源。** APP 入口及阅读顺序归 [APP README](../../../firmware/app/README.md)，稳定的跨层边界归 [ARCHITECTURE.md](../../../ARCHITECTURE.md)，协议与参数版本归 [协议契约](../../protocols/README.md) 和 [参数记录](../../../firmware/parameters/README.md)，编程与拆分标准归 [STYLE.md](../../../STYLE.md)。每阶段在本计划登记状态、源码版本、通过项、失败项和未测项；完成后更新索引并移入 completed。只有持续有效的规则进入长期文档。
