# 重构验证记录

## 2026-09-05：显式 Context 与应用端点收敛

- Application、Communication、Runtime 中的活动实例指针已全部移除；`MotorStateContext`、`MotorControlRuntimeContext`、Supervisor、CAN/USB Interface、Router、参数快照/持久化以及各 Application Service 均由调用链显式传入。
- CAN/USB Router 通过同一个 `ApplicationEndpoints` 访问命令、参数、遥测、CAN 配置、转子标定和摩擦辨识用例；CAN 响应端口仍保持为 CAN Router 私有依赖。
- `tools/verify_architecture.ps1` 新增隐藏可变文件状态门禁，并加入双 `TelemetryServiceContext` 隔离测试源文件检查。
- Keil ARMCC 5.06u7 对 126 个工程源文件执行全量重建：0 error、0 warning；镜像大小为 Code 104700、RO-data 6072、RW-data 456、ZI-data 29240 bytes。
- 10 个 `tests/host/*.c` 已使用 ARMCC 完成独立语法编译；本机没有可用的 Windows 原生 C 测试运行器，因此未把该项描述为主机运行通过。
- J-Link 已完成 112640-byte 镜像 Program/Verify；复位后 USB 只读检查为 mode=0、error=0、Vbus=27.81 V、TLE5012B Online、CAN node=0、1000 kbit/s、heartbeat=500 ms。
- CANalyst-II Classic CAN 只读冒烟连续查询 20 次全部通过，接收/发送错误计数均为 0；测试未发送电流、速度或位置使能命令。
- CANalyst-II 全协议回归 106/106 通过，覆盖零电流、零速、当前位置保持、参数写回、节点/波特率切换、异常帧拒绝和心跳保护；结束后回到 mode 0/error 0。
- 20 kHz 快环运行计数持续增长；实测最大 4357/8500 cycles（51.3% 周期预算）、最新 4191 cycles，deadline 超限计数为 0。

未关闭项：非零速度 USB 回归未形成有效闭环证据。第一次运行因之前 CAN 会话留下的 500 ms 心跳租约触发 `MOTOR_FAULT_CAN_DISCONNECTED`；关闭 CAN 心跳后再次运行时，USB CDC 在带转矩状态下停止应答。已立即通过 J-Link 复位恢复，最终复核 mode 0、error 0、Vbus 27.80 V，电流限值 6 A、速度加减速 50 r/s2、CAN 心跳 500 ms 均恢复。需单独定位控制权/心跳归属以及带载时 USB 抗扰或 CDC 活性，不能把该项记为闭环通过。

## Phase 0：2026-09-04 基线

- 基线提交：`c747b93`；保留已有用户工作树改动。
- 工程：Keil MDK，ARMCC 5.06u7 build 960，STM32G431。
- 首次 `UV4 -b -j0`：业务源文件均完成编译，但 24 个驱动/中间件源文件因并行 `.__i` 文件访问失败，目标未生成。
- `-j1` 在当前 UV4 实例上不退出，已只终止本次启动的进程；未影响用户原有 UV4 进程。
- 结论：并行构建存在工具链/实例状态风险，后续每次构建需检查完整日志，不能只信进程退出方式。

## Phase 1 / Iteration A：2026-09-04

范围：多故障集合、PowerStage 所有权、保护优先调度、实时路径移除动态分配。

验证结果：

- Keil 增量目标构建成功：0 errors，0 warnings。
- 中间构建点镜像：Code 78040 B，RO-data 4928 B，RW-data 776 B，ZI-data 31096 B。
- 工程 XML 可解析，80 个工程条目均存在。
- TIM1 三相 CCR 写入只存在于 `Firmware/Platform/Stm32G431/power_stage_tim1.c`。
- `Foc/` 无 `HAL_` 和 TIM1/ADC/FDCAN/SPI 寄存器直接引用。
- 多次 `Set_ErrorNow` 现在累加到位图，旧 `ErrorNow` 使用固定最低编号主故障投影，不再后写覆盖。
- 20 kHz 采样/传感器故障评估后、模式控制 switch 前增加阻断门；活动故障使输出同周期禁止并跳过控制。
- CAN 恢复只清除 `CAN_DisConnect`，不会清掉同时存在的其他故障。
- PowerStage 初始化默认禁止；非法/NaN/越界 duty 被拒绝并触发关断；清故障不会自动请求使能。
- 编码器标定采样区改为静态存储，20 kHz 调用图不再调用 `HEAP_malloc/HEAP_free`。
- 自制 16 KiB Heap 退出目标工程，参数传输改为静态单实例；静态标定区接入后的链接结果为 Code 77580 B，RO-data 4928 B，RW-data 752 B，ZI-data 25096 B。

遗留风险：

- TIM1 Break 输入当前在 CubeMX 配置中禁用，软件保护不能替代硬件过流关断。
- `PWM_TurnOnHighSides` 是兼容旧代码的含糊名称；本迭代仅保持原占空比极性，待示波器/原理图确认后重命名并限制为服务过程 API。
- 旧通信仍可直接写控制内部变量，静态检查暂以 warning 报告，Phase 5 转为硬失败。
- Flash 参数仍为单槽、无 CRC/提交标志且使用动态内存；保存位于后台但掉电可靠性不足，Phase 2 处理。
- 故障清除条件目前保持旧行为；逐故障去抖、锁存策略与恢复条件尚未建模。

## Phase 2 / Iteration A-B：2026-09-04

范围：ParameterManager、可靠持久化边界和运行命令 Service。

验证结果：

- 新增硬件无关 `ParameterManager` 与 `ParameterStorePort`，STM32 Flash 细节仍在 Adapter。
- 参数记录使用 page 56-59 与 page 60-63 两个 8 KiB 槽；包含格式版本、payload 长度、序号、CRC32、产品 ID、硬件 Profile、电机 Profile、参数 Schema 和最后提交双字。
- 保存顺序为擦除非活动槽、写 Header 前缀、写 payload、分块读回 CRC 校验、最后写提交标志；任一步失败都不切换活动槽。
- 旧 page 56 格式仅作只读迁移；没有新记录时首次保存写 slot 1，保留旧记录。
- Flash 编程返回值不再忽略；保存失败 raises `ParameterStore_Error`。
- `Param_Upload/Download/Return_Default` 已替换为 `ParameterSnapshot_Capture/Apply/LoadDefaults`。
- CAN/USB 的模式、电流、速度、位置写入改走 `MotorCommandService`；直接访问静态检查数从 103 降到 95。
- 每一小步 Keil 构建均为 0 errors / 0 warnings。

遗留风险：

- 尚未做真实掉电注入与 page 擦写寿命测试；A/B 算法目前只有编译和静态审查证据。
- `ParameterSnapshot` 仍是兼容 DTO，配置、标定结果和通信设置尚未拆成各自模型。
- 其余 CAN/USB 参数仍直接访问内部字段，将在后续 typed ParameterService/TelemetrySnapshot 迭代迁移。

## Phase 3 / Iteration A：2026-09-04

范围：SVPWM 纯 Domain 提取。

验证结果：

- `Firmware/Domain/Modulation/svpwm.*` 不依赖 HAL、寄存器或 PowerStage。
- FOC 三条实时调制路径均使用新模块；输入 NaN、输出 NaN 或 duty 越界会拒绝输出并锁存 PowerStage 故障。
- 旧 `SVM_SectorJudge` 暂不删除，但已退出实时调用链。
- 完整 Keil 重编译成功：0 errors / 0 warnings。

遗留风险：

- 需要用采样回放比较旧/新 SVPWM 全六扇区和边界结果后，才能删除旧实现。
- measurement、rotor feedback、current control 和其余 motion control 仍待按相同方式提取。

## Phase 4 / Iteration A：2026-09-04

范围：生命周期模型及旧模式兼容投影。

验证结果：

- 新增独立 `DeviceState`、`MotorControlMode`、`ServiceProcedure` 和纯 C 转换规则。
- 正常控制映射 `ACTIVE`，标定/辨识/保存映射 `SERVICING`，故障映射 `FAULTED`；全部故障清除后才回 `STANDBY`。
- 旧 `ModeNow` 编号继续服务现有协议，常用赋值已收口到 `Set_ModeNow` 同步兼容投影。
- Keil 构建成功：0 errors / 0 warnings。

遗留风险：

- `ModeNow_TypeDef` 仍包含服务动作，内部 switch 尚未完全改为以 DeviceLifecycle 为唯一权威状态。
- 每个标定过程仍需独立 Context、结果对象和统一取消/退出路径。

## Phase 6 / Iteration A：2026-09-04

范围：产品 Manifest 和 Bootloader ABI 准备。

验证结果：

- 新增固定宽度 `BootImageManifest` 与 `BootRequestMailbox` 契约定义，不链接 Bootloader 实现。
- 新增只读 ProductManifest：产品 ID `VMST`、STM32G431、当前硬件/电机 Profile、参数 Schema 和版本字段。
- 未配置发布信息时版本保持 `0.0.0`、build 0、development，避免被误判为量产镜像。
- 最终完整 Keil 重编译成功：0 errors / 0 warnings；Code 79276 B，RO-data 4960 B，RW-data 752 B，ZI-data 25136 B。
- `tools/verify_architecture.ps1`：84 个工程条目全部存在，0 failure；唯一 warning 为 95 个待迁移通信内部访问。

进入硬件/发布相关实现前需要确认：TIM1 Break 信号来源与有效极性、PCB revision 编码、正式版本策略、Bootloader 分区/升级 Transport、签名算法和密钥治理。未确认前不改 Option Bytes、链接起始地址或生产安全配置。

## Phase 5 / Iteration A-C：2026-09-04

范围：typed 参数服务、一致性遥测快照、通信 ISR 有界化和功率模式白名单。

验证结果：

- 新增 `ParameterService`，集中 21 项电机配置/模型参数的有限值、范围和跨字段校验；CAN/USB 只保留协议单位换算。
- 限流参数更新会在同一服务内夹紧电流给定；速度上限更新会同步夹紧位置最大速度，消除两个协议实现漂移。
- 新增 `TelemetryService` 双缓冲快照，1 kHz 发布模式、故障位图、命令、测量、编码器状态和配置；USB 五通道打印一次读取同一快照。
- CAN/USB 不再直接读写 `MotorControl`/`FOC`，也不保存其字段地址；Phase 5 通信静态门禁由 95 项 warning 收口为默认硬门禁且 0 failure。
- USB CDC RX ISR 只写 255-byte 有效容量的静态 SPSC 环形缓冲；解析和 `sprintf` 移到主循环。修正了旧实现写满 256 bytes 时读写索引相等、被误判为空的问题。
- FDCAN RX ISR 校验标准数据帧和精确 4-byte DLC，只向 8 槽静态 SPSC 队列投递；Service 调用和故障恢复移到主循环，队列满时累计溢出计数。
- 功率级启动改为显式模式白名单。电流偏置、保存、恢复默认、清故障和设置零位不再因“非禁用枚举”误启动 PWM；从有功率模式进入保存/其他无功率服务时先关断输出。
- Flash 保存前增加第二道 `PowerStage_ForceDisable` 兼容入口，确保全局关中断和 Flash stall 前无法继续产生日志外的扭矩输出。
- 恢复默认参数及 2 KiB 编码器 LUT 清零退出 20 kHz ISR，改在功率级已禁止且控制 IRQ 暂停的后台事务中应用并保存。
- 最终 `tools/verify_architecture.ps1`：86 个工程条目，0 failure，0 warning。
- 最终 Keil ARMCC 5.06u7 全量构建：0 errors，0 warnings；Code 82812 B，RO-data 4960 B，RW-data 764 B，ZI-data 25540 B。

遗留风险：

- CAN/USB Transport、Protocol、Router 仍在单文件中，虽已执行边界隔离，但尚未拆分成可独立主机测试模块。
- USB 编码器 LUT 导出仍读取编码器校准数组；需要不可变 CalibrationSnapshot 或流式只读 Port 后才能完全去除通信到编码器对象的依赖。
- 队列溢出和协议兼容仅有静态审查与目标编译证据，仍需主机黄金帧/模糊测试及板上突发流量测试。
- 1 kHz 遥测保持单生产者模型；本轮已补有界 sequence-lock 读校验。若引入多核或第二生产者，仍需升级为跨执行域同步策略。

## 最终架构收口：2026-09-04

范围：平台 Adapter 全覆盖、Application 依赖倒置、纯 Domain 迁移、目录所有权和最终门禁。

验证结果：

- ADC 原始快照、TLE5012B、Flash A/B Store、FDCAN、USB CDC、TIM1 功率级、板级启动和 LED/RGB 物理输出全部位于 `Firmware/Platform/Stm32G431`；业务层扫描无 HAL/CMSIS/外设寄存器访问。
- Application 命令、参数和转子标定服务通过 `MotorCommandPort`、`MotorConfigurationPort`、`RotorCalibrationPort` 静态注入，不再 `extern MotorControl` 或包含 FOC/BSP/Communication 头文件。
- USB/CAN 不再持有 `MotorControl`、`FOC` 或编码器对象；编码器方向修改和 LUT 流式导出通过 Service，LUT 导出仅允许在 `Motor_Disable`。
- 遥测双缓冲增加有界 sequence-lock 校验，避免 1 kHz ISR 发布打断后台复合快照复制时产生撕裂。
- 转子反馈迁入 `Firmware/Domain/RotorFeedback`，采样分频和采样周期由调用者注入；相电阻核心迁入 `Firmware/Domain/Identification`，两者均为纯 C 且无平台依赖。
- 删除聚合头 `common_inc.h`、未使用的 16 KiB Heap、delay 模块和整个旧 `Bsp/` 所有权层；Core 主循环收口为 `Board_RunBackground()`。
- 修复 CAN 波特率读取响应误用 `CAN_GET_CAN_HB` ID 的旧协议缺陷，并集中 node ID、波特率和心跳范围校验。
- 最终 `tools/verify_architecture.ps1`：94 个工程条目，0 failure，0 warning。
- 最终 Keil ARMCC 5.06u7 全量构建：0 errors，0 warnings；Code 85784 B，RO-data 4960 B，RW-data 780 B，ZI-data 25716 B。

硬件/发布门禁：

- 未擅自改变 PWM 极性、TIM1 Break 配置、Option Bytes 或链接起始地址。
- Bootloader 分区、升级 Transport、签名算法、密钥托管、PCB revision 和正式固件版本仍需明确输入及板级验证；当前 Manifest 继续标记 development `0.0.0`。
- 仍需在目标板执行过流/欠压/过压、编码器断链、CAN/USB 突发、Flash 掉电点和控制周期 WCET 测试，目标编译不能替代这些量产验证。

## 架构一致性复核与通信/Profile 收口：2026-09-04

范围：修正 Keil 工程视图与目标架构不一致、彻底拆分通信共享状态、实体化 typed Product Profile，并补强自动门禁。

验证结果：

- Keil 工程源文件按 `Firmware/Domain/Measurement`、`Firmware/Domain/Identification`、`Firmware/Domain/CurrentControl`、`Firmware/Domain/MotionControl` 与 `Firmware/Communication/Transport`、`Protocol`、`Router`、`Interfaces` 分组；工程视图与物理目录一致。
- USB Parser、Router、传输发送状态分别使用 `LegacyUsbCommand`、`LegacyUsbRouterResponse` 和私有 `UsbInterfaceContext`；旧 `USBMsg_TypeDef` 已删除。
- CAN 配置与响应通过 `CanConfigurationService`/`CanResponseService` 和 Port 注入；Router 不再包含或调用 `interface_can`，旧 `CANMsg_TypeDef` 已删除。
- USB 环形缓冲迁到 `Firmware/Communication/Transport/byte_ring_buffer.*`，容量与单写者状态显式归属接口层；Cube CDC 回调仅投递字节。
- CAN 心跳在非受控模式下正确停用，超时事件只上报一次，并由新帧显式恢复；USB 初始化显式清理接口与队列状态。
- `BoardProfile`、`MotorProfile`、`EncoderProfile` 已实体化并从 `Board_Init` 注入参数加载/迁移、参数范围、测量保护、编码器速度估计、位置控制限值和相电阻辨识；消费者不再选择 Product 宏。
- `FaultManager` 增加事件序号、每类故障发生次数及首次/最近事件序号；`DiagnosticService` 将这些信息与 ProductManifest、遥测快照统一为只读 DTO。
- `tools/verify_architecture.ps1 -StrictCommunication`：110 个工程条目，0 failure，0 warning；同时验证工程分组、漏编源文件、Domain 隐藏状态、Profile 宏泄漏与通信反向依赖。
- Keil ARMCC 5.06u7 全量构建：0 errors，0 warnings；Code 88904 B，RO-data 5236 B，RW-data 568 B，ZI-data 26544 B。
- 额外扫描：20 kHz 路径禁用操作 0 项；Platform/Core 外 HAL/外设寄存器访问 0 项；Communication 下层对 Router/Service/控制实现的禁止依赖 0 项。

验证边界：以上是编译、工程结构和静态门禁证据。尚未执行目标板波形、硬件保护注入、通信突发/模糊输入、Flash 掉电、WCET 和 Bootloader 升级/回滚验证；这些项目需要硬件、链接布局与发布安全参数后才能关闭。

## 旧架构物理删除与最终命名收口：2026-09-04

范围：删除过渡目录和兼容主体，使物理目录、Keil 分组、类型/API 名称与目标架构一致。

验证结果：

- 旧 `Foc/` 与 `System/` 目录已删除；`Bsp/` 继续保持删除状态。原实时实现迁入 `Firmware/Runtime/MotorControl`，1 kHz 监督任务迁入 `Firmware/Runtime/Supervisor`，组合根迁入 `Firmware/Composition`，快速数学迁入 `Firmware/Domain/Math`。
- `ModeNow_TypeDef`、`ErrorNow_TypeDef`、`FOC_TypeDef`、`MotorControl_TypeDef` 以及旧 FOC/Board/BSP 函数名已替换为动作、故障、上下文和运行时职责命名。
- CAN/USB 文件由 `legacy_*` 过渡命名收口为 `*_protocol_v1` 与 `*_command_router`；线上协议 ID、数值和响应格式未改变。
- `hw_conf.h` 已删除：控制节拍进入 `Firmware/Product/control_loop_config.h`，ADC/母线比例、保护阈值和逆变器死区进入只读 `BoardProfile`，TIM1 Adapter 使用定时器实际 ARR。
- 运行配置、命令、电流控制状态和持久化 DTO 的公共字段已改为带单位的职责名称；产品代码不再保留 `_TypeDef`/拼写错误类型。
- Keil 工程组已同步为 `Firmware/Composition`、`Firmware/Runtime/MotorControl`、`Firmware/Runtime/Supervisor`、`Firmware/Domain/Math`、`Firmware/Domain/RotorFeedback` 和 `Firmware/Application/Indicators`，工程 XML 可解析且所有源文件存在。
- `tools/verify_architecture.ps1 -StrictCommunication`：110 个工程条目，0 failure，0 warning；门禁会直接拒绝重新出现的 `Foc/`、`System/` 和旧通信文件。
- Keil ARMCC 5.06u7 全量构建：0 errors，0 warnings；Code 88980 B，RO-data 5240 B，RW-data 572 B，ZI-data 26540 B。

验证边界：尚未执行目标板 PWM 波形、硬件保护注入、通信黄金帧/模糊输入、Flash 掉电点、20 kHz WCET 或 Bootloader 升级/回滚验证。软件重构完成不等于量产硬件验证完成。

## 设计语义一致性收口：2026-09-04

范围：消除协议动作与运行状态混合、拆分外部命令与派生控制目标、服务结果候选化、组合根运行入口、量产诊断与升级 Application 边界。

验证结果：

- Runtime 已删除组合动作枚举；稳定的 v1 动作号只存在于 `MotorCommandService` 边界，并映射为独立的 `DeviceState`、`MotorControlMode` 和 `ServiceProcedure`。
- 生命周期请求与服务结果使用原子邮箱在 1 kHz 监督任务消费；20 kHz 仅执行确定性的控制/服务 step，RTT 写入和 Flash 事务均位于后台。
- `MotorCommand` 不再承载速度环、位置环、无感启动或标定派生输出；实时单写 `MotorControlTargets`，静态门禁会拒绝算法重新回写命令。
- 电流零偏和三相相电阻先形成结果对象，经 `CalibrationService`/`IdentificationService` 校验后发布完整配置候选，由 20 kHz 安全点一次应用；算法文件不直接改写 `MotorConfiguration`。
- 单项自动恢复只清活动故障；锁存集合、每类发生次数、时间和故障现场保留到显式清故障。诊断增加 STM32 96-bit UID、RCC 复位原因、DWT 最大周期与 deadline 超限计数。
- CubeMX ADC/TIM7/FDCAN/USB 回调统一进入 `FirmwareComposition_*`，周期 IRQ 在全部 ISR 可见 Context/Port 初始化成功后才启用。
- 新增硬件无关 `UpdateService`/`UpdateControlPort`：候选校验、参数事务空闲检查、功率级禁止、`UPDATING` 转换、邮箱失败回退和复位提交均有明确边界；未绑定未知的 Bootloader 分区/邮箱物理地址。
- `tools/verify_architecture.ps1 -StrictCommunication`：121 个工程条目，0 failure，0 warning；工程 XML 可解析，`git diff --check` 无空白错误。
- 5 个主机测试翻译单元使用 ARMCC `--c99` 独立语法编译：生命周期、故障锁存、参数事务、服务结果校验和升级服务均为 0 error。本机无可用原生 C 编译器，因此未把它们误报为已执行的主机二进制测试。
- Keil ARMCC 5.06u7 全量重建：0 errors，0 warnings；Code 93488 B，RO-data 5400 B，RW-data 504 B，ZI-data 28536 B。

## 2026-09-04：源码根目录收敛

- Application、Domain、Ports、Product、Runtime、Communication、Platform、Composition 全部迁入统一的 `Firmware/` 源码根；Bootloader 继续作为独立工程边界保留在一级目录。
- SEGGER RTT 迁入 `Middlewares/Third_Party/SEGGER/RTT`，README 图片迁入 `docs/assets`；CubeMX 的 Core、Drivers、Middlewares、USB_Device 目录保持生成器兼容结构。
- Keil 工程源文件路径、IncludePath 和逻辑分组全部同步；工程 XML 可解析，121 个工程条目全部存在，旧一级层目录和旧相对路径扫描均为 0 项。
- `tools/verify_architecture.ps1` 已改为校验 `Firmware/` 物理边界，并拒绝重新创建旧的一级 Application、Domain、Runtime 等目录。
- 架构检查：121 entries，0 failures，0 warnings；`git diff --check` 无空白错误。
- 5 个主机测试翻译单元使用新 IncludePath 通过 ARMCC `--c99` 独立语法编译，全部 0 error。
- Keil ARMCC 5.06u7 全量重建：0 errors，0 warnings；Code 93488 B，RO-data 5400 B，RW-data 504 B，ZI-data 28536 B。

硬件/发布验证边界：目标板 PWM 波形与极性、TIM1 Break 保护注入、通信突发/模糊输入、Flash 掉电点、实测 20 kHz WCET、Bootloader 签名/回滚和 Option Bytes 仍需在确定硬件、内存布局与密钥策略后执行；本轮没有猜测或修改这些不可逆配置。
