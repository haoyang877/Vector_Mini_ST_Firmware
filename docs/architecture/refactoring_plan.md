# Vector Mini ST 固件渐进重构计划

## 1. 基线与约束

本计划以 2026-09-04 的 `c747b93` 为代码基线，并保留工作树中已有的用户改动。当前已识别的用户改动包括 `.gitignore`、`Core/Src/main.c` 的换行修复、Keil 用户选项、示波器配置和未跟踪的 `data_recoder/`；各阶段不得覆盖、删除或回退这些内容。

当前工程为 Keil MDK 工程，目标 STM32G431，20 kHz FOC 由 ADC 注入转换回调驱动，1 kHz 监督任务由 TIM7 驱动。初始审计发现：

- TIM1 输出由初始化、FOC 算法、标定和故障模块多点控制，没有唯一所有者；
- 快速保护在控制计算之后汇总，单枚举故障可能被后续检测覆盖；
- `ModeNow_TypeDef` 混入正常控制、标定、保存、恢复默认和清故障；
- `MotorControl_TypeDef` 混入命令、配置、运行状态、结果和遥测；
- CAN/USB 可直接写控制内部变量，USB 还长期保存内部字段地址；
- 参数保存单槽、无 CRC/提交记录、忽略编程错误并使用动态内存；
- `foc_algorithm.c` 和其他控制文件直接访问 TIM1/HAL/聚合头文件；
- Keil ARMCC 5.06u7 位于 `C:\Keil_v5`，目标工程可通过 UV4 命令行执行可重复全量构建。

每个阶段都采用“建立边界 -> 接入一条路径 -> 验证 -> 扩大覆盖”的方式。不得为目录整齐一次性移动所有文件。

## 2. 每次迭代的固定流程

1. 记录 `git status --short`，确认用户改动边界。
2. 为待迁移行为补充基线说明、静态检查或测试。
3. 一次只引入一个可解释的边界或迁移一条调用链。
4. 执行目标构建；若工具不可用，则执行工程 XML、预处理/语法、依赖和禁用 API 静态验证，并明确标记“未完成目标编译”。
5. 审查功率输出、故障、ISR 时间和持久化影响。
6. 记录修改文件、验证结果、遗留风险和下一迭代。
7. 回滚只撤销本次迭代新增内容，不触碰用户改动或已验证的独立迭代。

## 3. Phase 0：行为基线和构建验证

### 范围

- 固化现有启动顺序、ISR 频率、模式编号、协议参数 ID、Flash 地址/Schema 和 PWM 极性。
- 建立可脚本化的工程源文件检查、依赖扫描和实时路径禁用 API 扫描。
- 查找 MDK 构建工具并保存可复现命令；不修改 CubeMX 生成行为。

### 迁移映射

- `MDK-ARM/Vector_Mini_ST.uvprojx` 保持为初期权威目标工程。
- 旧 API 暂不更名；测试/检查先围绕 `FOC20kHzIRQHandler`、`BSP1kHzIRQHandler` 和 `Board_Init` 建立。

### 风险与验证

- 风险：本机无命令行编译器导致只完成静态验证；IDE 私有文件导致不可重复构建。
- 验证：工程 XML 可解析、列出的源文件存在、重复枚举值报告、HAL/寄存器访问清单、动态内存/Flash/格式化调用清单。

### 完成标准

- 有可重复运行的基线检查；目标构建成功，或明确记录工具阻塞和等价静态证据。

### 回滚

- 删除新增检查脚本/报告即可，不改变产品行为。

## 4. Phase 1：PowerStage 唯一所有权、先保护后控制、多故障集合

### 范围与迭代顺序

1. 新建硬件无关 `FaultManager`，用活动/锁存位图保存多个故障，并保留旧单故障投影。
2. 新建 `PowerStage` Context/Port；STM32G431 TIM1 Adapter 成为启停和 CCR 的唯一底层访问者。
3. 先迁移启停路径，再迁移安全占空比和三相 duty 写入。
4. 将 20 kHz 调度改为采样后立即评估保护；若有阻断故障，同周期禁止输出并跳过控制。
5. 接入硬件 Break 状态和 NaN/范围保护；清故障不自动使能。

### 迁移映射

- `Start_PWM_Generate/Stop_PWM_Generate` -> `PowerStage_RequestEnable/PowerStage_ForceDisable`。
- `Set_A/B/C_Duty` -> 一次性 `PowerStage_ApplyDutyCycle`。
- `PWM_TurnOnHighSides/LowSides` -> 明确命名的安全/测试输出请求，仅 ServiceProcedure 可用。
- `ErrorNow_TypeDef` -> `FaultCode` 到 bit 的映射；`ErrorNow` 暂作为协议兼容视图。
- `Vbus_Update/Current_Cal/Temperature_Update` 中的故障写入 -> 保护评估结果集合。

### 风险与验证

- 风险：现有“高边全开”实际电气含义容易误判；PWM Start/Stop 时序变化；保护阈值抖动；ISR 增时。
- 验证：静态确认 TIM1 功率输出只有 Adapter 访问；故障注入同周期关断；多个故障同时保留；disable 幂等；clear 后保持关闭；占空比 NaN/越界拒绝。

### 完成标准

- TIM1 三相功率输出有唯一所有者；所有阻断故障先于控制执行；对外能读取完整故障集合和兼容主故障。

### 回滚

- 每个旧 API 在迁移期间仅作为薄适配器，单次可切回旧调用；Adapter 和 FaultManager 可独立回滚。

## 5. Phase 2：数据模型与 ParameterManager

### 范围

- 拆分 `MotorCommand`、`MotorConfiguration`、`MotorRuntimeState`、`MotorTelemetry`。
- 建立单写者、命令双缓冲和遥测快照。
- 引入 ParameterManager：typed ID、范围/状态/跨字段校验、候选配置和安全点 Apply。
- 引入可靠 ParameterStore Port 和 STM32 Flash A/B 双槽 Adapter，移除保存路径动态内存。

### 迁移映射

- `MotorControl_TypeDef` 字段按职责迁入四类对象，旧结构作为短期 Facade。
- `Param_Return_Default` -> `ParameterManager_LoadDefaults`。
- `Param_Upload/Param_Download` -> `ParameterRecord_Encode/Decode` 与 `ParameterManager_ApplyCandidate`。
- `flash_read/write_param` -> `ParameterStore_LoadNewest/SaveAtomic`。
- `Save_Param/Default_Param` -> Application Service 请求，不再是 MotorControlMode。

### 风险与验证

- 风险：协议/Flash 向后兼容；复合快照撕裂；错误参数短暂进入实时控制；Flash 掉电损坏。
- 验证：Schema v4-v8 迁移样例、CRC 损坏、半写、序号回绕、掉电点测试；命令/遥测并发测试；所有字段范围和单位测试。

### 完成标准

- 通信不再依赖内部地址；实时任务只读已验证配置快照；任意保存中断后至少一个槽可恢复。

### 回滚

- 先保持旧记录只读迁移器；新记录写入独立页。必要时固件可回到旧加载器，且不擦除旧页。

## 6. Phase 3：提取硬件无关控制域

### 范围与顺序

1. measurement：ADC 原始值到 SI 单位及滤波；
2. rotor feedback：传感器质量、角度展开、速度和电角度；
3. current control：变换、PI、解耦和电压限幅；
4. modulation：SVPWM 纯计算；
5. motion control：电流/速度/位置/阻抗策略。

### 迁移映射

- `foc_sensing.*` -> `domain/measurement` + ADC Port Adapter。
- `Bsp/encoder.*` -> `domain/rotor_feedback` + TLE5012B Adapter。
- `foc_algorithm.*` -> `domain/current_control` 与 `domain/modulation`。
- `foc_run.*`、`position_*` -> `domain/motion_control`。
- `foc_pid.*`、`foc_traptraj.*` 保留算法后更名归档。

### 风险与验证

- 风险：浮点舍入和调用顺序改变；单位混淆；状态未完整迁移；实时预算超限。
- 验证：旧/新算法回放同一采样数据进行差分；边界/NaN 测试；主机单元测试；目标周期最大耗时。

### 完成标准

- Domain 不含 HAL/寄存器和隐藏全局；控制输出在容差内匹配基线；20 kHz 最坏耗时满足预算。

### 回滚

- 每个子模块通过 Adapter 接到旧调度器，可逐个切回旧实现。

## 7. Phase 4：DeviceLifecycle 与独立服务过程

### 范围

- 建立 `DeviceState`、`MotorControlMode`、`ServiceProcedure` 三个模型。
- 所有转换集中在 `DeviceLifecycle`，显式定义 guard、entry、exit 和超时。
- 将电流偏置、编码器、电角度、观测器和相电阻流程拆为独立有界 step。

### 迁移映射

- `ModeNow_TypeDef` -> 三类枚举和兼容映射表。
- `ModeSwitch_Handle` -> `DeviceLifecycle_RequestTransition`。
- `foc_calibration.*` -> 多个 `application/*_service` 与 `domain/calibration`。
- `foc_phase_resistance.*` -> IdentificationService Adapter；`phase_resistance.*` 保持 Domain 核心。

### 风险与验证

- 风险：旧模式数字协议兼容；服务取消遗漏清理；位置模式进入时保持当前位置行为丢失。
- 验证：全状态转换表测试；每个过程成功/失败/取消/超时；任意退出均禁止输出并复位控制状态。

### 完成标准

- 保存/默认/清错/标定不再属于 MotorControlMode；任何状态变化都有唯一入口和可审计原因。

### 回滚

- 保留旧模式到新请求的协议翻译，不让旧枚举继续驱动内部 switch。

## 8. Phase 5：通信分层

### 范围

- CAN/USB 拆为 Transport、Protocol、Router、Service。
- ISR 仅进入静态队列；后台解析和响应。
- 统一 typed 命令、参数和遥测 API，保留旧帧兼容 Codec。

### 迁移映射

- `interface_can.*` -> `can_fdcan1_transport` + `legacy_can_protocol` + Router。
- `interface_usb.*` -> `usb_cdc_transport` + `legacy_usb_protocol` + Router。
- `VarInfo.addr` -> Telemetry ID getter / Parameter ID setter。
- 直接 `MotorControl.*` 赋值 -> Service 调用并返回结果码。

### 风险与验证

- 风险：现有工具兼容、队列溢出、命令延迟、CAN 心跳语义改变。
- 验证：黄金帧测试、模糊输入、队列满策略、并发 CAN/USB 仲裁、无内部地址/直接赋值静态扫描。

### 完成标准

- Transport 和 Protocol 可独立测试；CAN/USB 不能绕过 Service 修改控制或生命周期状态。

### 回滚

- 旧协议 Codec 与新 Service 并存；可按 Transport 逐个切换。

## 9. Phase 6：Bootloader 契约、Manifest 与量产诊断

### 范围

- 定义版本化 ImageManifest、启动邮箱、链接布局和签名/哈希边界。
- 建立 ProductManifest、构建版本、硬件/Profile 兼容检查。
- 加入只读量产诊断、复位原因、故障快照、控制周期水位和升级结果。
- 准备但不在未评审时自行启用生产密钥或不可逆安全选项。

### 迁移映射

- 零散宏版本 -> 生成的 BuildInfo + 常量 Manifest。
- 参数 magic/schema -> Product/Parameter 兼容矩阵。
- 诊断字符串 -> typed DiagnosticService，Transport 决定编码。

### 风险与验证

- 风险：链接地址/向量表、错误硬件刷写、升级掉电、密钥治理和防回滚策略。
- 验证：Manifest 解析、错误产品拒绝、哈希/签名失败、所有掉电点、试运行失败回滚、Application 健康确认。

### 完成标准

- Application 可安全请求升级并报告版本；Bootloader 契约不依赖业务内部；量产工具能获取完整可追溯诊断。

### 回滚

- 契约先只读发布、旧启动路径保持；Bootloader 经独立硬件验证后才切换链接布局。不可逆 Option Byte/密钥操作需单独发布批准。

## 10. 首批建议迭代

### Iteration 0A：基线检查

- 增加工程源文件/依赖/实时禁用 API 的静态检查。
- 保存可用构建工具结论，不改变运行行为。

### Iteration 1A：FaultManager 兼容接入

- 新增纯 C 多故障集合和测试。
- `Set_ErrorNow` 改为 raise，旧 `ErrorNow` 成为主故障投影；`No_Error` 仅通过明确 clear 操作处理。
- 保持旧协议错误数值不变。

### Iteration 1B：PowerStage 启停唯一所有权

- 新增 Context/Port 和 TIM1 Adapter。
- 先迁移 Start/Stop，板初始化改为初始化后禁止输出。
- 保留旧函数为薄兼容入口并加迁移注释。

### Iteration 1C：先保护后控制

- 把快速故障评估提到模式 switch 前；阻断故障立即禁止并跳过控制。
- 加入 duty 有限值/范围检查和一次性三相提交。

每一小步通过验证后才进入下一步；若硬件 PWM 极性或“高边全开”的安全含义无法从原理图/目标实测确定，则停止相关语义更改，只完成不改变波形的所有权迁移并请求硬件决策。

## 11. 本轮完成状态（2026-09-04）

- Phase 0-5 的软件边界重构已接入目标工程并通过全量 ARMCC 构建；旧 `Bsp/`、`Foc/`、`System/` 均已删除，`Firmware/Application`、`Firmware/Domain`、`Firmware/Ports`、`Firmware/Product`、`Firmware/Runtime` 不含 HAL 依赖。
- 功率级、ADC 测量、TLE5012B、Flash、FDCAN、USB CDC 和指示灯均由 STM32G431 Adapter 实现，并在 `FirmwareComposition_Initialize` 静态注入。
- 参数 A/B 原子记录、故障集合、生命周期投影、命令/参数/遥测/转子标定服务、ISR 静态队列和纯 Domain 算法均已启用。
- CAN/USB 已拆成 Interfaces、Transport、版本化 Protocol、Command Router，并通过 Application Service + Port 访问配置和响应队列；共享 `CANMsg`/`USBMsg` 状态已移除。
- Board/Motor/Encoder typed profile 已由组合根注入测量、参数、编码器和相电阻辨识路径；只读 `DiagnosticService` 汇总 Manifest、故障事件统计和一致性遥测快照。
- ADC、1 kHz、CAN 和 USB 中断统一进入 `FirmwareComposition_*`，周期 IRQ 只在所有 Context 与 Port 初始化成功后启用；旧 FOC/BSP API、类型和产品宏已从参与构建的源码中移除。
- `MotorCommand`、`MotorControlTargets`、`MotorConfiguration`、`MotorRuntimeState` 和不可变遥测快照已拆分；速度环、位置环和标定不再回写外部命令。
- `tools/verify_architecture.ps1` 已成为依赖、实时禁用 API、控制算法回写命令、工程漏编源文件、Router/Protocol/Transport 反向依赖和废弃基础设施的硬门禁。
- Phase 6 的 Manifest/ABI、`UpdateService`/`UpdateControlPort`、复位原因和控制周期水位已完成；Bootloader 链接布局、签名、邮箱物理地址和不可逆安全配置明确保持未启用，等待硬件与发布安全输入。
