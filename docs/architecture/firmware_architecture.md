# Vector Mini ST 量产固件架构

## 1. 目的与适用范围

本文定义 Vector Mini ST 在 STM32G431、当前 PCB、TLE5012B 角度传感器和现有电机配置上的目标软件架构。近期实现只支持这套已验证硬件，但所有硬件与产品差异必须通过 Port、Adapter 和只读 Profile/Manifest 注入；控制域不得因新增 MCU、PCB、角度传感器或电机而修改算法核心。

架构目标按优先级排列：

1. 任何故障路径都先撤销扭矩和功率输出，再执行控制或服务逻辑。
2. 功率级只有一个软件所有者，其他模块只能提交输出请求。
3. 20 kHz 实时路径具有有界执行时间，不分配动态内存、不访问 Flash、不格式化字符串。
4. 配置、命令、运行状态、遥测和持久化镜像是不同的数据模型。
5. 控制算法是硬件无关、可在主机上测试的纯 C 模块。
6. 通信只解释和路由请求，不直接写控制器内部变量。
7. Application 与 Bootloader 通过固定镜像契约协作，彼此不链接内部实现。

## 2. 架构原则与依赖规则

目标依赖方向如下：

```text
Product Manifest / Profiles
            |
            v
Composition Root
       |
       +------> Application Services -----> Domain Algorithms
       |                 ^                        ^
       |                 |                        |
       +------> Runtime Orchestration ------------+
       |                 |
       +------> Ports <--+
                      ^
                      |
              Platform Adapters -----> STM32 HAL / CMSIS / Core

Communication: Transport -> Protocol -> Router -> Application Services
```

运行期对象关系由组合根显式建立，不允许模块通过文件级活动指针寻找“当前实例”：

```text
FirmwareComposition
  +-- MotorControlRuntimeContext
  |     +-- MotorStateContext
  |     +-- control/calibration/identification Contexts
  |     `-- ParameterSnapshotContext
  +-- Application Service Contexts
  |     `-- ApplicationEndpoints
  |           +-- CAN Router Context
  |           `-- USB Router Context
  +-- CAN/USB Interface Contexts
  `-- SupervisorTaskContext
        +-- MotorControlRuntimeContext
        +-- Telemetry/Indicator Contexts
        `-- CAN/USB Interface Contexts
```

`FirmwareComposition` 是唯一允许静态拥有上述可变对象的位置。Application、Communication
和 Runtime 的公共操作都显式接收所属 `Context`；同一模块可以在测试中创建多个互不干扰的实例。
Platform Adapter 可以为 HAL 回调保留硬件单例，但该状态不能向上泄漏为应用级活动对象。

硬性规则：

- `domain/` 不包含 STM32 HAL、CMSIS 外设寄存器、CubeMX 生成头文件、USB 或 CAN 头文件。
- Domain 函数只接收调用者持有的 `Context`、不可变配置、输入值和 Port；不读取隐藏全局变量。
- `application/` 编排状态机和用例，可以依赖 Domain 与 Port 接口，不依赖具体 HAL Adapter。
- `Firmware/Platform/Stm32G431/` 是允许依赖 HAL/CMSIS 和 TIM/ADC/SPI/FDCAN 句柄的唯一产品代码区域。
- `Firmware/Communication/Transport/` 只移动字节/帧；`Protocol/` 只做编解码；`Router/` 做鉴权、范围和状态校验；Service 调用 Application API。
- 中断入口只采集必要输入、执行确定性任务并投递事件；字符串和大报文生成留在后台任务。
- 所有可跨中断访问的数据都必须有明确的单写者规则、原子快照或临界区策略。
- 不新建含糊的 `System`、`Common`、`Run`、`Handle` 模块；名称表达所有者和动作。

## 3. 目标目录及职责

当前工程已完成目录级迁移，产品固件统一收口在 `Firmware/` 下，旧 `Bsp/`、`Foc/`、`System/` 已删除。运行时编排、纯算法、应用用例、平台访问和组合根均有独立所有者；除 `Firmware/Platform/Stm32G431/` 与 CubeMX `Core/` 外，任何产品代码都不得访问 HAL 或外设寄存器。

```text
Firmware/
  Application/
    device_lifecycle.*          设备生命周期与允许的转换
    motor_command_service.*     命令验证、仲裁和发布
    parameter_manager.*         参数校验、应用、保存请求
    fault_manager.*             多故障集合、锁存和清除策略
    power_stage.*               功率级唯一所有权和安全门控
    calibration_service.*       标定流程编排
    identification_service.*    电机辨识流程编排
    telemetry_service.*         一致性快照与降采样
    diagnostic_service.*        产品信息、故障统计与遥测的只读诊断快照
    update_service.*            安全升级准备、取消与复位交接
    rotor_calibration_service.* 转子方向配置与 LUT 只读服务
    can_configuration_service.* CAN 配置用例与 Port 调用
    can_response_service.*      CAN 响应投递用例与 Port 调用
    communication_watchdog_service.* 链路故障上报用例
    Indicators/                 LED/RGB 状态逻辑

  Domain/
    Math/                       无平台依赖的快速数学函数
    Measurement/                电流、电压、温度换算与滤波
    RotorFeedback/              角度、速度、方向与质量状态
    CurrentControl/             Clarke/Park、PI、电压限幅
    Modulation/                 SVPWM 占空比计算
    MotionControl/              轨迹、位置级联和阻抗控制
    Identification/             无硬件依赖的辨识算法

  Ports/
    power_stage_port.h           使能、禁止、安全占空比、三相占空比
    measurement_port.h          ADC 原始快照
    rotor_sensor_port.h         角度传感器原始帧
    parameter_store_port.h       双槽记录读写
    monotonic_clock_port.h       单调时钟
    diagnostic_transport_port.h 诊断输出
    execution_timer_port.h       控制周期计时
    reset_reason_port.h          复位原因读取
    device_identity_port.h       MCU 唯一标识读取
    update_control_port.h        Application 到 Bootloader 邮箱/复位边界
    motor_command_port.h        命令服务到实时控制的窄接口
    motor_configuration_port.h  参数服务到运行配置的窄接口
    rotor_calibration_port.h    转子标定数据受控读写接口
    can_configuration_port.h    CAN 配置服务到接口状态的窄接口
    can_response_port.h         CAN Router 到发送队列的窄接口

  Platform/Stm32G431/
    power_stage_tim1.*          TIM1 唯一寄存器/HAL 访问者
    measurement_adc12.*         ADC/DMA 适配
    rotor_sensor_tle5012b.*     SPI/TLE5012B 适配
    parameter_store_flash.*     STM32 Flash 适配
    execution_timer_stm32g431.* DWT 周期计数适配
    reset_reason_stm32g431.*    RCC 复位标志适配
    device_identity_stm32g431.* STM32 UID 适配
    can_fdcan1_transport.*      FDCAN 适配
    usb_cdc_transport.*         USB CDC 适配

  Product/
    product_manifest.*          产品、硬件、固件兼容性标识
    control_loop_config.h       固定控制节拍与有界流程编译期配置
    control_tuning_profile.*    传感器/观测器/标定调参
    board_profile.*             引脚、量程、极性和时序
    motor_profiles.*            电机电气/机械参数和安全上限
    mechanical_load_profiles.*  阻尼器/无阻尼器启动、标定、摩擦前馈与摩擦辨识配置
    encoder_profiles.*          传感器类型、方向、标定能力

  Composition/
    firmware_composition.*      唯一组合根；静态创建并注入所有 Port/Profile

  Runtime/
    MotorControl/
      motor_control_runtime.*    20 kHz 确定性编排入口
      motor_control_types.h      命令、派生目标、配置和运行状态模型
      measurement_runtime.*      测量 Port 到测量模型的适配
      current_control_runtime.*  电流环与调制编排
      control_mode_runtime.*     正常控制策略调度
      current_offset_calibration_runtime.* 电流零偏有界 step
      encoder_calibration_runtime.* 编码器线性化/观测器标定有界 step
      electrical_zero_calibration_runtime.* 电角零位有界 step
      phase_resistance_runtime.* 有界相电阻辨识 step
      motor_state_runtime.*      生命周期请求、故障与协议只读投影
      parameter_snapshot.*       运行参数快照与 Schema 迁移
      parameter_persistence_adapter.* 参数快照到 ParameterManager 的适配
      *_adapter.*                Application Port 到运行时状态的窄适配
    Supervisor/
      supervisor_task.*          1 kHz 遥测、指示灯和通信监督

  Communication/
    interface_can.*              CAN 队列、心跳和 Port Adapter
    interface_usb.*              USB 队列、发送状态和打印调度
    Transport/byte_ring_buffer.* USB SPSC 静态接收队列
    Protocol/*_protocol_v1.*     当前线协议 v1 的有界编解码
    Router/*_command_router.*    协议 ID 到 Application Service 的映射

Bootloader/
  image_contract.h            Application/Bootloader 共享 ABI
  （独立工程，后续引入）

tests/
  host/                       Domain/Application 主机测试
  target/                     板级冒烟、保护和升级测试

docs/
  product_configuration_quick_guide.md 新硬件与新电机快速适配指南
```

## 4. Application 与 Bootloader 边界

Bootloader 与 Application 仅共享版本化的 `ImageManifest`、启动邮箱和复位原因，不共享业务结构体、HAL 句柄或链接符号。

`ImageManifest` 至少包含：

- 魔数、契约版本、结构长度；
- 产品 ID、硬件兼容位图、目标 MCU；
- 语义版本、构建 ID、镜像长度、入口地址；
- 镜像哈希、签名算法与签名位置；
- 参数 Schema 最小/最大兼容版本；
- 安全回滚计数器和发布通道。

Bootloader 负责镜像接收、完整性/真实性检查、槽位选择、试运行计数、回滚和跳转。Application 的 `UpdateService` 负责候选兼容性检查、请求进入升级模式、确认功率级已禁止、写入请求并触发复位；具体邮箱地址、复位和 Bootloader 实现只通过 `UpdateControlPort` 接入。升级传输可复用协议定义，但 Bootloader 和 Application 必须各自拥有路由与服务实现。

## 5. 硬件抽象与产品配置

Port 使用窄接口和调用者提供的上下文：

```c
typedef struct {
    void *context;
    bool (*enable_outputs)(void *context);
    void (*disable_outputs)(void *context);
    void (*write_duty)(void *context, float phase_a, float phase_b, float phase_c);
} PowerStagePort;
```

Port 不拥有上层状态。Adapter 可以包含 HAL 句柄或寄存器基址，但不得反向调用控制域。初始化时由 composition root 把静态分配的 Context、Profile 和 Port 组装起来。

当前 `FirmwareComposition_Initialize` 获取只读 `BoardProfile`、`MotorProfile`、`EncoderProfile` 并注入参数快照、测量模型、参数校验、编码器速度估计和相电阻辨识路径；这些模块不再从散落宏中自行选择产品配置。编译期宏只存在于 Product profile 构造中，历史持久化格式由显式 Schema 迁移处理。

配置分三层：

- `BoardProfile`：ADC 比例、分流电阻、栅极逻辑、PWM 频率/死区、传感器总线和安全电压温度边界。
- `MotorProfile`：极对数、R/L/磁链、最大电流/速度、控制器默认带宽和辨识边界。
- `ProductManifest`：产品 ID、PCB 修订、MCU、Bootloader 契约和允许的 Profile 组合。

编译期选择确定硬件能力，运行期持久化参数只能在 Manifest 给出的安全范围内调整，不能把不兼容硬件伪装为另一 Profile。

## 6. 数据模型

不得继续用一个结构体承载所有数据：

- `MotorCommand`：外部期望，如启停、控制模式、目标电流/速度/位置；由通信服务产生，由控制任务单写消费。
- `MotorControlTargets`：由当前模式或服务过程派生的 dq 电流、速度、位置和电压目标；只由 20 kHz 编排/算法写，不反向覆盖外部命令。
- `MotorConfiguration`：经校验且当前生效的参数；运行中默认只读，切换采用完整快照。
- `MotorRuntimeState`：积分器、轨迹、观测器和状态机上下文，只由对应实时模块写。
- `MotorTelemetry`：从运行状态复制出的只读快照；通信不得持有运行变量地址。
- `ParameterRecord`：带 Header/CRC/序号的持久化 DTO；不直接 `memcpy` 内部结构体作为协议。
- `FaultSet`：活动故障与锁存故障位图、首次发生时间、主故障和计数器。

命令发布使用双缓冲或短临界区复制；遥测按固定频率生成一致性快照。对 32 位 Cortex-M4 的自然对齐 32 位标量可原子读写，但复合对象仍必须由序号锁、双缓冲或临界区保证一致。

## 7. 实时任务与执行预算

### 20 kHz 电流环（ADC 注入转换完成）

固定顺序：

1. 获取 ADC 与转子传感器快照；
2. 更新测量值与传感器质量；
3. 评估快速保护并将故障加入 `FaultSet`；
4. 若存在禁止输出的故障，立即 `PowerStage_ForceDisable`，跳过控制输出；
5. 消费已验证命令快照；
6. 执行当前控制或服务过程的一个有界 step；
7. 计算 SVPWM 占空比并向 PowerStage 提交；
8. 更新轻量运行状态/降采样计数。

此路径禁止 `malloc/free`、Flash 擦写/编程、`printf/sprintf`、USB/CAN 发送、无界循环和阻塞 HAL API。

### 1 kHz 监督任务

负责生命周期转换、通信超时、慢保护去抖、命令看门狗、LED/RGB 状态、遥测快照和标定/辨识监督。不能绕过 PowerStage 或直接写控制状态。

### 后台任务

负责 Transport TX、诊断文本格式化、参数保存事务、升级准备和非实时统计。Flash 保存仅在设备处于安全禁止输出状态、实时任务明确让出后执行。

## 8. 状态模型

三个正交概念必须分开：

`DeviceState`：

- `BOOTING`：初始化和自检，输出禁止；
- `STANDBY`：可接收配置/命令，输出禁止；
- `ACTIVE`：控制模式运行；
- `SERVICING`：执行一种标定或辨识过程；
- `FAULTED`：存在阻断故障，输出禁止；
- `UPDATING`：升级交接，输出禁止。

`MotorControlMode` 只包含正常控制策略：`CURRENT`、`SPEED`、`SENSORLESS_SPEED`、`POSITION_CASCADE`、`POSITION_IMPEDANCE`、`VOLTAGE_OPEN_LOOP`、`VQ`。保存参数、恢复默认值、清故障、设置机械零位和各种标定不属于控制模式。

`ServiceProcedure` 包含 `CURRENT_OFFSET_CALIBRATION`、`ENCODER_LINEARIZATION`、`ELECTRICAL_ZERO_CALIBRATION`、`OBSERVER_CALIBRATION`、`PHASE_RESISTANCE_IDENTIFICATION` 等。每项过程都有 `IDLE/PRECHECK/RUNNING/VERIFYING/COMPLETED/FAILED/CANCELLED` 子状态，进入时验证前置条件，退出时统一撤销输出并发布结果。

状态转换只能由 `DeviceLifecycle` 执行。通信、故障检测和服务过程提交事件，不直接赋值状态。

## 9. PowerStage 与安全保护

`PowerStage` 是 TIM1 三相功率输出的唯一软件所有者：

- 只有平台 Adapter 可写 TIM1 CCR/BDTR、启动或停止互补 PWM。
- Domain 只生成归一化占空比；Application 负责验证有限值和范围后提交。
- 上电初始化、任何复位/异常和未知状态都采用输出禁止。
- 使能需要同时满足：设备状态允许、命令有效、必要标定完成、传感器在线、`FaultSet` 无阻断位、占空比为安全值。
- 禁止是幂等操作，且优先于本周期任何控制计算。
- 故障清除不自动重新使能；必须经过新的显式启动转换。

保护分层：硬件 Break/栅极驱动器提供最快关断；20 kHz 快保护处理过流、母线越界、无效数值、传感器连续丢帧；1 kHz 慢保护处理温度、通信看门狗和一致性问题。软件故障集合不能替代硬件过流关断。

`FaultSet` 同时保存多个故障。当前实现保存活动/锁存位图、主故障、事件序号和每个故障的发生次数及首次/最近事件序号；墙钟时间和测量快照将在引入单调时钟 Port 后补齐。兼容旧协议时按固定优先级投影为一个 `primary_fault`，但不得丢失其他活动/锁存位。

## 10. 参数、标定和持久化

`ParameterManager` 工作流：

1. Service 接收 typed parameter ID/value，不接收内部地址；
2. 校验类型、有限值、范围、跨字段约束、当前 DeviceState 和 Profile 兼容性；
3. 写入候选配置并计算派生量；
4. 在安全同步点原子应用完整配置快照；
5. 若请求持久化，进入后台保存事务；
6. 读回并校验后才确认保存成功。

持久化采用 A/B 双槽或日志式记录。每条记录含魔数、Schema、长度、产品/硬件/Profile ID、单调序号、payload CRC 和提交标志。写入顺序为：擦除非活动槽、写 Header/Payload、校验读回、最后写提交标志。掉电时始终保留一个已提交旧槽。加载时选择兼容且 CRC 正确的最高序号；失败则加载安全默认值并记录参数故障，不在实时路径访问 Flash。

标定/辨识结果先进入结果对象，只有验证通过并经 ParameterManager 接受后才能成为配置；保存必须显式请求。取消、超时、故障和正常完成都走统一清理路径。

## 11. 通信、协议和升级流程

通信分层：

- `Transport`：CAN 帧、USB 字节流、收发队列和链路统计；
- `Protocol`：帧格式、版本、CRC、序列号、typed payload 编解码；
- `Router`：命令 ID 到 Service 的映射、权限/状态/范围初检；
- `Service`：调用 MotorCommandService、ParameterManager、CalibrationService、DiagnosticService 或 UpdateService；
- `Telemetry`：读取不可变快照并编码响应。

ISR 仅把定长帧放入静态环形队列。任何协议写操作都返回明确结果：accepted、busy、invalid-state、out-of-range、not-supported 或 internal-error。CAN 与 USB 可以共享 Service，不共享 Transport 状态，也不能保存 `MotorRuntimeState` 字段地址。

升级流程：收到请求后验证 Manifest 与产品兼容性；DeviceLifecycle 进入 `UPDATING`；PowerStage 强制禁止并确认；ParameterManager 完成或取消事务；写启动邮箱和候选镜像信息；系统复位；Bootloader 校验、试运行和必要时回滚；Application 健康自检后确认镜像。

## 12. 故障与诊断

故障记录最少包含：活动位图、锁存位图、主故障、首次/最近时间、发生次数、当时的关键测量快照和最近复位原因。量产诊断提供：

- 产品/硬件/固件/Bootloader/参数版本；
- 唯一设备标识与生产批次字段；
- 上电自检、ADC 偏置、传感器、Flash、CAN/USB 状态；
- 活动/锁存故障集合及计数；
- 最近升级和回滚结果；
- 只读的控制周期最大耗时与溢出计数。

诊断读取不得改变设备状态。具有副作用的测试必须作为 ServiceProcedure 运行。

## 13. 命名规范

- 类型：完整 PascalCase，如 `MotorCommand`、`ParameterStorePort`。
- 函数：`Module_VerbObject`，如 `FaultManager_Raise`、`PowerStage_ForceDisable`。
- 枚举值：`TYPE_VALUE`，如 `DEVICE_STATE_STANDBY`。
- 单位写入字段名：`bus_voltage_v`、`speed_rad_s`、`timeout_ms`。
- 布尔值使用 `is_`、`has_`、`can_`；动作函数使用动词。
- 用 `Read/Write` 表示 I/O，用 `Encode/Decode` 表示协议，用 `Load/Save` 表示持久化，用 `Apply` 表示运行配置切换。禁止用含糊的 `Upload/Download`。
- 禁止新增 `System`、`Common`、`Run`、`Handle` 作为领域名称；协议兼容必须使用显式版本号，不得重新引入 `legacy_*` 主体模块。
- 文件名与单一职责一致；一个公共头文件不能聚合所有工程头文件。

## 14. 验证与发布门禁

每次迭代至少执行：工程文件完整性检查、目标编译（工具可用时）、无 HAL 依赖扫描、实时路径禁用 API 扫描和相关主机测试。安全修改还需验证：上电禁止、故障同周期关断、多故障不丢失、清故障不自启、NaN/越界占空比拒绝、模式切换输出连续性。

量产发布要求可重复构建、版本 Manifest、链接布局检查、静态分析、单元/集成测试、板级保护注入、通信兼容测试、参数掉电测试、升级/回滚测试，以及记录所用 Product/Motor/Encoder Profile。

## 15. 当前落地状态

当前架构已作为唯一参与构建的实现：

- `ParameterService` 统一校验电机配置的有限值、范围、单位和跨字段约束；CAN/USB 不再各自维护一份规则。
- `TelemetryService` 在 1 kHz 监督节拍发布双缓冲不可变快照；CAN 查询和 USB 五通道打印不再读取或保存电机控制运行时字段地址。
- `CanConfigurationService` 和 `CanResponseService` 通过 Port 访问 CAN 接口私有状态；Router 不包含 `interface_can.h`，也不直接调用 Transport。
- CAN 与 USB Router 共同依赖一个由组合根注入的 `ApplicationEndpoints`，协议层不再分别维护一套应用服务定位关系。
- `FirmwareComposition` 是 ADC、1 kHz、CAN、USB 与后台任务的统一入口；周期中断只在全部 ISR 可见 Context 初始化完成后启用。
- Application、Communication 和 Runtime 已移除 `ActiveContext`、`ActiveRuntime`、`ActiveState` 一类隐式单例；生命周期、故障、参数、遥测、接口和指示灯调用链均显式携带 Context。
- 控制算法只写 `MotorControlTargets`，不再把速度环、位置环或标定派生的电流参考回写到 `MotorCommand`。
- 电流零偏和相电阻先形成独立结果，由 Application 校验，再经配置候选邮箱在 20 kHz 安全点整对象提交；过程算法不直接改写生效配置。
- `DiagnosticService` 发布复位原因、故障现场与 DWT 统计的 20 kHz 最大周期/超限次数；RTT 仅作为 Platform 后台 Transport。
- 单项故障恢复只清活动位，锁存位与每类发生次数保留到显式整机清故障，以支持追溯诊断。

USB CDC 与 FDCAN 接收中断只采集到静态有界队列，命令解析、格式化和 Service 调用在主循环执行。USB Protocol、Router 和接口发送状态使用独立 DTO/Context，不共享可变控制状态。现有线上协议以 `can_protocol_v1` 和 `usb_protocol_v1` 明确版本化，保留既有协议 ID、显示单位和响应文本。

`tools/verify_architecture.ps1` 会拒绝旧目录/文件、工程漏编源文件、HAL 越界、Product 宏越界、通信反向依赖、实时路径动态分配/Flash/格式化操作以及未经过 PowerStage 的 TIM1 三相输出访问。
