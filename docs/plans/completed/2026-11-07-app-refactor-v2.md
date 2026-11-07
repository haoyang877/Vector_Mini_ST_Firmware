# 单轴电机 APP 层重构与维护计划 v2

日期 2026-11-07。基线 `ed18e5a`。本版取代 [v1](2026-11-07-app-refactor-v1.md)，新增两条硬约束：

1. **可重入性**：APP 运行时模块不得依赖隐藏的可变文件级状态；调用方持有上下文。
2. **接口收敛**：每个接口只接收自己使用的参数，不相关的参数不得传入；输出只覆盖调用方真正需要的字段。

## 生产实现与离线验收

本次实现以 `ed18e5a` 为基线。六个生产模块直接位于同一 APP 目录，测试用 `composition/app_rtt.c` 维持原路径和内容。下面的方案章节记录历史设计；实际接口和执行上下文以 [APP 阅读地图](../../../firmware/app/README.md) 为准。

| 项目 | 验收证据 |
| --- | --- |
| 命名 | 生产模块使用小写下划线文件名及对应模块前缀，公开声明和注释经过接口检查。[命名与边界回归](../../../tests/integration/test_app_module_contract.py) |
| 可重入性 | 应用、轴、外环、位置级联、标定会话、协议端点均有独立状态。双板配置各执行隔离参考和 A 内嵌套 B 的生产入口，检查 FOC/PWM、标定写入目标与记录内容、STOP 令牌、CAN 队列、过期工作及 PI 提交。[双实例回归](../../../tests/unit/native/app_instance_test.c) |
| 解耦性 | 板级持有实例，轴绑定 PWM 和参数槽。协议端点通过回调调用所属适配器，标定的实际写入在关相和原会话复核后执行。硬件中断、CAN 与板级采样按照产品的单物理设备约定串行。[标定发布回归](../../../tests/unit/native/test_calibration_save.py) |
| 简洁与可读性 | 轴调度、生命周期、标定、协议、Loader 与组合根按职责分开。五态迁移与唯一生产调用者同单元，保留纯函数接口。前台、采样和交接顺序可直接从生产入口追踪。[文件职责](../../architecture/app_file_responsibilities_v1.md) |

**构建与回归。** STM32G431 普通 APP、STM32G431 LoaderApp、AT32F456 普通 APP 均完成 Keil 全量重建，零错误、零警告。基线与本版在两种板型、独立/Loader 两种编译配置下，启动、身份页、看门狗与交接的事件摘要逐组相同。45 项原生套件包含真实计算路径、协议黄金帧与 schema14 标定记录。独立实机板尚未测量快环耗时和栈水位。

| 普通镜像 | 基线静态 RAM（RW + ZI） | 本版静态 RAM | 差值 |
| --- | ---: | ---: | ---: |
| STM32G431 | 22496 B | 22704 B | +208 B |
| AT32F456 | 48424 B | 48640 B | +216 B |

Keil 调用图中可计算的最大栈深，STM32 普通 APP 为 2212 B，AT32 从 4720 B 到 4744 B。报告标记间接调用等未知栈深，这些数值仅用于静态对照。整仓 PR 门禁仍记录基线的 SEGGER RTT 两个厂商文件与清单哈希不一致、历史文档链接缺少本地证据。构建、测试和 PR 的实际日志位于当前 worktree 的 outputs 目录。

## 0. 目标与非目标

**目标**

- 维护者能从一个执行入口追完一次采样、启停、标定保存或 Loader 交接。
- `motor_axis`、`motor_lifecycle`、`motor_calibration` 达到实例级可重入，无隐藏可变全局。
- 每个公共接口的输入、输出和调用上下文可审查；不出现“传整个 AppContext 给所有函数”或“填一个大部分字段无关的输出结构”。
- 协议、参数 ABI、Flash 布局、控制律、整定值和时序语义不变。

**非目标**

- 不把 BSP、CAN、Flash、SEGGER RTT 做成多实例硬件抽象。
- 不引入堆、RTOS、设备工厂或注册表。
- 不为未来多轴产品添加通用上下文框架。
- 不夹带全仓命名重排。

## 1. 不变量

- 协议编号、线缆单位、108/124 字段与有效位、黄金帧不变。
- schema14 参数记录、CRC、Flash 布局不变。
- 20 kHz 快环、2 kHz 监督、AT32 4 kHz 外环、STM32 2 kHz 外环不变。
- STOP 受理与实际关相、启相许可、标定代际、Loader 票据语义不变。
- 静态分配、无堆、无 RTOS；每个有效事实只有一个所有者。

## 2. 可重入模型

| 级别 | 含义 | 适用 |
| --- | --- | --- |
| R0 | 不可重入，依赖隐藏可变 static/global | Flash、协议发送、BSP 寄存器单例 |
| R1 | 单实例可重入 / ISR 安全 | 通信端点、RTT、BSP、组合层回调 |
| R2 | 实例级可重入，调用方持有上下文 | `MotorAxis`、`MotorLifecycle`、`MotorCalibration`、`MotorStopService` |
| R3 | 完全可重入 / 线程安全 | 仅 `MotorStateMachine` |

落地目标：`motor_state_machine` R3；`motor_stop_request` R2；`motor_axis` R2；`motor_lifecycle` R2；`motor_calibration` R2；`app_protocol` R1/R2；`app_loader` R2；`app_rtt` R1；组合层 R2 per `AppContext`；BSP/通信 R1；Flash/发送 R0。

**边界声明**：R2 不等于任意两个中断可以同时调用同一函数。它表示没有隐藏状态、调用方持有上下文、跨上下文共享字段有保护。硬件寄存器、Flash、CAN 端点仍是单例。

## 3. 接口收敛规则

### 3.1 输入收敛

1. 只有组合根接收 `AppContext *`：`App_Init`、`App_Start`、`App_Process`。
2. 模块自己的顶层调度函数可以接收模块上下文，例如 `MotorAxis_SampleFast(MotorAxis *axis)`。
3. 叶子函数只接收该操作需要的子上下文或值视图，例如 `MotorCalibration_UpdateHall(MotorCalibrationLut *lut, ...)`、`AppRtt_Sampling(RttContext *rtt)`。
4. 不传递未使用参数；参数必须在函数体内被读取，或传给确实会读取它的回调。
5. 不用 `void *context` 传递业务状态；仅通用端口（STOP 所有者、协议回调）使用，并写明具体类型和串行约束。
6. 不把整个模块上下文传给只需要一个子对象的函数；标定按 LUT / 电零位 / 机械零位 / 保存拆分。
7. 回调声明精确参数；不需要上下文的回调不添加。

### 3.2 输出收敛

1. 小结果用返回值：`bool`、`enum`、小型值结构。
2. 输出指针只用于同一调用方需要的一组相关字段；不填“大而全”结构。
3. 每个输出结构对应一个真实读者：`MotorStatus` 只给 108/124；`MotorOuterResult` 只给 Lifecycle 提交；`BootEntryReply` 只给 Loader 协议；`MotorStopResult` 只给 STOP 调用方。
4. 不输出调用方已有或可推导的数据；不重复输出 epoch、mode、fault，除非调用方确实要用。
5. 只读查询用 `Get/Is/Has/Can`，不通过输出参数暗改状态。

### 3.3 审查清单

- 每个参数在函数体内是否被读取？未读取的删除。
- 是否有更窄的类型可以替代宽上下文？
- 输出字段是否都能对应到调用方的实际读取点？
- 调用上下文、是否阻塞、是否串行、失败后的状态是否写明？
- 是否引入新的隐藏 `static`/`global` 可变状态？

## 4. 当前问题

**不可重入的隐藏状态**

- `app.c`：`s_communication_ready`、`s_identity`、`s_supervisor_ticks`、`s_watchdog_*`、`g_app_fast_samples`。
- `app_protocol.c`：`s_stop_service`、`s_source`、`s_boot_source`、`s_target_valid`、`s_last_target`、`s_target_epoch`、`s_calibration_request_number`。
- `app_rtt.c`：`rtt_data`、`s_rtt_buffer`。
- `motor_axis.c`：全部 `s_motor_*`、`s_foc_*`、`s_rotor_*`、`s_axis_record`、`s_encoder_linearization_workspace`、`s_hall_sweep`、`s_output_calib`、`s_hall_sample`；函数内 `static MotorPositionPlan work` 和标定停稳计时器。
- `motor_lifecycle.c`：`g_motor_context`。

**接口过宽或无关参数**

- `MotorAxis_ProcessCalibrationPublication(progress)` 把标定发布和进度回调塞在 Axis 接口里；非 Hall 分支不读 `progress`。
- `MotorLifecycle_OuterTick` 把外环计算、PI 副本、提交、STOP 消费和运行条件检查混在一个接口。
- `MotorAxisFeedback` 把五个只读指针打包；RTT 只需要 `linear_hall`。
- `App_Process` 无显式上下文，所有子步骤通过文件级状态通信。
- `service_boot` 把入口轮询、健康窗口、身份刷新和看门狗混在一个文件内。
- `motor_stop_request.h` 类型使用 `_t` 后缀，不符合 `STYLE.md`。

**文档漂移**

- 标定实际执行 `BspFlash_Write`，但 APP README、firmware README、文件职责、软件架构和协议说明仍写“只改 RAM/不自动保存”。
- 软件架构仍写外环统一 2 kHz；实际 AT32 为 4 kHz。
- 软件架构仍写 RTT 无调用；实际 AT32 2 kHz 调用。
- 软件架构仍写测量字段不宣称有效；实际 `AppProtocol_Refresh` 会写 `valid_bits`。

P0 必须先统一这些契约，再开始结构迁移。

## 5. 目标文件架构

```text
firmware/app/
├─ README.md
├─ composition/
│  ├─ app.h / app.c                    启动、主循环、看门狗、身份
│  ├─ app_context.h                    新增：AppContext + g_app
│  ├─ app_protocol.h / app_protocol.c  协议适配
│  ├─ app_loader.h / app_loader.c      新增：Loader 事务
│  └─ app_rtt.h / app_rtt.c            新增头：AT32 RTT
├─ runtime/
│  ├─ motor_axis.h / motor_axis.c      单轴对象、采样/控制顺序、位置规划、外环编排
│  ├─ motor_lifecycle.h / .c           五态、准入、epoch、功率许可、故障恢复
│  └─ motor_calibration.h / .c         新增：标定阶段、工作区、候选、收尾
├─ motor_state_machine/
│  ├─ motor_state_machine.h / .c       纯五态迁移
└─ interfaces/
   ├─ motor_state_types.h              值类型
   ├─ motor_stop_request.h / .c        STOP 受理与实际关相确认
   └─ README.md
```

说明：

- `app_context.h` 是组合根头；只有 `main.c`、`app.c` 和需要 `Protocol_Motor` 静态绑定的 `app_protocol.c` 包含它。
- `app_rtt.h` 只暴露 `RttContext` 和两个入口；RTT 不接收 `MotorAxis` 或 `AppContext`。
- `app_loader.c` 在所有 APP 工程编入，函数体用 `YG_APP_LOADER_IMAGE` 保护；非 LoaderApp 编译为空实现。
- `app_rtt.c` 仍只编入 AT32 工程。

### 5.1 上下文所有权

```text
main.c
└─ AppContext g_app;
   ├─ MotorAxis axis
   │  ├─ MotorConfiguration / MotorControlState / FocState
   │  ├─ EncoderState / FluxObserverState / SensorlessStartupState
   │  ├─ LinearHall / OutputEncoder / CurrentOffsetCalibration
   │  ├─ MotorAxisRecord / OutputEncoderCalibSample hall_sample
   │  ├─ MotorPositionPlan position_plan_work
   │  ├─ MotorCalibration calibration
   │  └─ MotorLifecycle *lifecycle
   ├─ MotorLifecycle lifecycle
   ├─ AppProtocolContext protocol
   ├─ AppLoaderContext loader
   ├─ RttContext rtt
   ├─ DeviceIdentity identity
   ├─ AppWatchdog watchdog
   ├─ uint32_t fast_samples
   ├─ uint32_t supervisor_ticks
   └─ bool communication_ready
```

### 5.2 初始化顺序

1. `MotorAxis_Init(&app->axis, &app->lifecycle)`：装载配置、初始化对象、初始化内嵌标定；只保存 lifecycle 指针，不调用 Lifecycle。
2. `MotorLifecycle_Init(&app->lifecycle, &bindings)`：绑定已初始化的 Axis 对象；bindings 只含指针。
3. `AppProtocol_Init(&app->protocol, &app->lifecycle)`。
4. `AppLoader_Init(&app->loader, &app->lifecycle, BootHw_IsAvailable())`。
5. `AppRtt_Init(&app->rtt, &app->axis.linear_hall)`。
6. 设备身份、能力页、通信绑定。

`MotorLifecycle_Init` 的 bindings 在 Axis 初始化后填充；Lifecycle 初始化本身不读取控制对象。

## 6. 逐文件接口

### 6.1 `composition/app_context.h`

- `AppWatchdog { last_ms, last_fast, last_slow, enabled }`
- `AppContext { MotorAxis axis; MotorLifecycle lifecycle; AppProtocolContext protocol; AppLoaderContext loader; RttContext rtt; DeviceIdentity identity; AppWatchdog watchdog; uint32_t fast_samples; uint32_t supervisor_ticks; bool communication_ready; }`
- `extern AppContext g_app;`

约束：`AppContext` 只允许 `App_Init/App_Start/App_Process` 接收。

### 6.2 `composition/app.h`

| 接口 | 输入 | 输出/返回 | 上下文 |
| --- | --- | --- | --- |
| `void App_Init(AppContext *app);` | 完整组合根 | 无 | main，`Bsp_Init` 后 |
| `void App_Start(AppContext *app);` | 完整组合根 | 无 | main，`Bsp_Start` 后 |
| `void App_Process(AppContext *app);` | 完整组合根 | 无 | 主循环 |
| `void App_OnFastSample(MotorAxis *axis, uint32_t *fast_samples);` | 轴 + 计数 | 无 | ADC |
| `void App_SupervisorTick(MotorAxis *axis, uint32_t *supervisor_ticks);` | 轴 + 计数 | 无 | 监督 |
| `void AppWatchdog_Start(AppWatchdog *watchdog, uint32_t now_ms, uint32_t fast_samples, uint32_t supervisor_ticks);` | 看门狗 + 计数 | 无 | 启动 |
| `bool AppWatchdog_Process(AppWatchdog *watchdog, uint32_t now_ms, uint32_t fast_samples, uint32_t supervisor_ticks);` | 看门狗 + 计数 | 是否健康 | 前台 |

移除 `App_OnCanReceive`；`main.c` 直接调用 `Protocol_RxInterrupt()`。

### 6.3 `composition/app_protocol.h`

上下文：`AppProtocolContext { ProtocolMotorStatus source; ProtocolDeviceStatus boot_source; MotorStopService stop_service; ProtocolCommand last_target; uint32_t target_epoch; uint32_t calibration_request_number; bool target_valid; MotorLifecycle *lifecycle; }`

| 接口 | 输入 | 输出/返回 | 上下文 |
| --- | --- | --- | --- |
| `void AppProtocol_Init(AppProtocolContext *protocol, MotorLifecycle *lifecycle);` | 协议上下文 + Lifecycle | 无 | 启动 |
| `void AppProtocol_Refresh(AppProtocolContext *protocol, const MotorStatus *status, uint32_t epoch);` | 协议上下文 + 状态值 + epoch | 更新 `protocol->source` | 前台 |

`ProtocolMotor` 当前回调无 context。P3/P4 给 `ProtocolMotorHandler` 增加 `void *context`，`Protocol_Motor.context` 指向 `&g_app.protocol`；若通信层暂不允许改，则 `AppProtocol` 保持 R1，使用文件内注册的上下文并写明单例约束。

### 6.4 `composition/app_loader.h`

上下文：`AppLoaderContext { BootEntryRequest entry_request; BootEntryResult entry_result; uint32_t entry_started_ms; uint32_t health_checked_ms; uint32_t observed_supervisor_ticks; uint8_t healthy_intervals; bool entry_requested; bool boot_supported; MotorLifecycle *lifecycle; }`

| 接口 | 输入 | 输出/返回 | 上下文 |
| --- | --- | --- | --- |
| `void AppLoader_Init(AppLoaderContext *loader, MotorLifecycle *lifecycle, bool supported);` | Loader + Lifecycle + 能力 | 无 | 启动 |
| `void AppLoader_Process(AppLoaderContext *loader, uint32_t now_ms, bool communication_ready);` | Loader + 时间 + 通信就绪 | 无 | 前台 |
| `void AppLoader_RequestEntry(AppLoaderContext *loader, const BootEntryRequest *request, BootEntryReply *reply);` | Loader + 请求 | 填 reply | 协议回调 |
| `void AppLoader_CompleteTx(AppLoaderContext *loader, uint64_t ticket, bool success);` | Loader + 票据 + 成功 | 无 | 协议回调 |
| `bool AppLoader_ConfirmImage(AppLoaderContext *loader, uint32_t version, uint32_t crc32);` | Loader + 版本 + CRC | 是否确认 | 协议回调 |

`Protocol_Boot` 由 `app.c` 定义；包装函数在调用前检查 `app->communication_ready && Protocol_IsReady()`，`app_loader.c` 不依赖 `AppContext`。

### 6.5 `runtime/motor_axis.h`

上下文：`MotorAxis` 持有配置、控制状态、FOC、PI、编码器、观测器、无感启动、Hall、输出轴、零偏、记录暂存区、Hall 快照、位置规划工作区、`MotorCalibration` 和 lifecycle 指针。

| 接口 | 输入 | 输出/返回 | 上下文 |
| --- | --- | --- | --- |
| `void MotorAxis_Init(MotorAxis *axis, MotorLifecycle *lifecycle);` | 轴 + Lifecycle | 无 | 启动 |
| `void MotorAxis_SampleFast(MotorAxis *axis);` | 轴 | 无 | ADC |
| `void MotorAxis_ControlFast(MotorAxis *axis);` | 轴 | 无 | ADC |
| `void MotorAxis_SupervisorTick(MotorAxis *axis);` | 轴 | 无 | 监督 |
| `void MotorAxis_OuterTick(MotorAxis *axis);` | 轴 | 无 | 外环 |
| `void MotorAxis_OuterOverrun(MotorAxis *axis);` | 轴 | 无 | ADC |
| `void MotorAxis_ProcessPositionPlan(MotorAxis *axis);` | 轴 | 无 | 前台 |
| `uint8_t MotorAxis_GetNodeId(const MotorAxis *axis);` | 轴 | 节点号 | 只读 |

移除 `MotorAxis_ProcessCalibrationPublication`、`MotorAxis_Feedback`、`MotorAxis_GetFeedback`。RTT 只取 `const LinearHall *`。

### 6.6 `runtime/motor_lifecycle.h`

上下文：`MotorLifecycle` 持有五态、配置指针、控制对象指针、恢复状态、标定类型、epoch、监督计数、功率许可、STOP/完成/发布 pending、故障锁存和启动锁。

```c
typedef struct {
    const MotorConfiguration *configuration;
    MotorControlState *motor;
    FocState *foc;
    PiController *speed_pi;
    EncoderState *rotor_encoder;
    SensorlessStartupState *sensorless_startup;
} MotorLifecycleBindings;

typedef struct {
    MotorControlState motor;
    PiController speed_pi;
    uint32_t epoch;
} MotorOuterInput;

typedef struct {
    float id_target_a;
    float iq_target_a;
    bool uses_speed_ramp;
    float speed_ramped_rad_s;
    float position_reference_rad;
    bool position_target_reached;
    float position_velocity_filtered_rad_s;
    float position_trajectory_speed_rad_s;
    float position_trajectory_acceleration_rad_s2;
    PiController speed_pi;
} MotorOuterResult;
```

| 接口 | 输入 | 输出/返回 |
| --- | --- | --- |
| `void MotorLifecycle_Init(MotorLifecycle *lifecycle, const MotorLifecycleBindings *bindings);` | 上下文 + 绑定 | 无 |
| `void MotorLifecycle_BeginFastTick(MotorLifecycle *lifecycle);` | 上下文 | 无 |
| `void MotorLifecycle_ReportOutcome(MotorLifecycle *lifecycle, MotorTaskResult outcome, uint32_t epoch);` | 上下文 + 结果 + epoch | 无 |
| `MotorState MotorLifecycle_GetState(const MotorLifecycle *lifecycle);` | 上下文 | 五态 |
| `bool MotorLifecycle_RequestRemoteStop(MotorLifecycle *lifecycle);` | 上下文 | 受理 |
| `MotorControlResult MotorLifecycle_RequestEnable(MotorLifecycle *lifecycle, MotorControlMode mode);` | 上下文 + 模式 | 结果 |
| `MotorControlResult MotorLifecycle_SetTarget(MotorLifecycle *lifecycle, const MotorTarget *target);` | 上下文 + 目标 | 结果 |
| `MotorControlResult MotorLifecycle_RequestCalibration(MotorLifecycle *lifecycle, MotorCalibrationType type);` | 上下文 + 类型 | 结果 |
| `MotorCalibrationType MotorLifecycle_GetCalibrationType(const MotorLifecycle *lifecycle);` | 上下文 | 类型 |
| `bool MotorLifecycle_IsCalibrationRunnable(const MotorLifecycle *lifecycle);` | 上下文 | 可否 |
| `bool MotorLifecycle_GetPendingCalibrationPublication(const MotorLifecycle *lifecycle, uint32_t *epoch);` | 上下文 | bool + epoch |
| `void MotorLifecycle_ContinueCalibrationSession(MotorLifecycle *lifecycle, uint32_t epoch);` | 上下文 + epoch | 无 |
| `void MotorLifecycle_CompleteCalibrationPublication(MotorLifecycle *lifecycle, uint32_t epoch, bool ok);` | 上下文 + epoch + 结果 | 无 |
| `MotorControlResult MotorLifecycle_RequestClear(MotorLifecycle *lifecycle);` | 上下文 | 结果 |
| `bool MotorLifecycle_GetBootHealth(const MotorLifecycle *lifecycle, uint32_t *progress);` | 上下文 | bool + progress |
| `uint32_t MotorLifecycle_GetEpoch(const MotorLifecycle *lifecycle);` | 上下文 | epoch |
| `void MotorLifecycle_SupervisorTick(MotorLifecycle *lifecycle);` | 上下文 | 无 |
| `bool MotorLifecycle_BeginOuter(MotorLifecycle *lifecycle, MotorOuterInput *input);` | 上下文 | 是否允许 + 计算输入 |
| `void MotorLifecycle_CommitOuter(MotorLifecycle *lifecycle, uint32_t epoch, const MotorOuterResult *result);` | 上下文 + epoch + 结果 | 无 |
| `bool MotorLifecycle_LockStop(MotorLifecycle *lifecycle);` | 上下文 | 是否锁定 |
| `bool MotorLifecycle_IsStopped(const MotorLifecycle *lifecycle);` | 上下文 | 是否停止 |
| `MotorStatus MotorLifecycle_GetStatus(const MotorLifecycle *lifecycle);` | 上下文 | 状态值 |

接口收敛要点：

- `MotorLifecycle_Init` 用 bindings 代替 6 个裸指针。
- `BeginOuter` 只输出计算需要的副本和 epoch；`allowed` 用返回值。
- `CommitOuter` 只接收 epoch 和 `MotorOuterResult`，不接收整个控制状态。
- `GetStatus` 返回 `MotorStatus` 值，只有协议适配层调用；其他查询用窄返回值。
- 移除 `MotorLifecycle_OuterTick`。

### 6.7 `runtime/motor_calibration.h`

阶段子上下文：

- `MotorCalibrationLut`：config、encoder、observer、startup、linear_hall、output_encoder、LUT 工作区、Hall sweep、output calib、`hall_collect_active`。
- `MotorCalibrationElectricalZero`：config、motor、foc、encoder、observer、startup。
- `MotorCalibrationMechanicalZero`：config、encoder、output_encoder。
- `MotorCalibrationSave`：record、`commit_record`、lifecycle、停稳计时。
- `MotorCalibration`：type、epoch、上述四个子上下文。

```c
void MotorCalibration_Init(MotorCalibration *calibration,
                           const MotorCalibrationBindings *bindings);
MotorTaskResult MotorCalibration_Step(MotorCalibration *calibration,
                                      MotorCalibrationType type);
void MotorCalibration_UpdateHall(MotorCalibrationLut *lut,
                                 const OutputEncoderCalibSample *sample);
void MotorCalibration_ProcessPublication(MotorCalibrationPublishInput *input,
                                         uint32_t now_ms,
                                         MotorCalibrationProgress progress);
```

`MotorCalibrationPublishInput { MotorCalibrationLut *lut; MotorCalibrationSave *save; MotorCalibrationType type; uint32_t epoch; }`

`MotorCalibrationProgress { void (*fn)(void *context); void *context; }`

接口收敛要点：

- LUT、电零位、机械零位、保存各自持有自己的绑定子集；叶子函数只接收对应子上下文。
- `MotorCalibration_Step` 是模块顶层分派，接收整个 `MotorCalibration`；内部 `lut_step`、`electrical_zero_step`、`mechanical_zero_step` 只接收各自子上下文。
- `MotorCalibration_UpdateHall` 只接收 `MotorCalibrationLut *`。
- `MotorCalibration_ProcessPublication` 只接收发布所需的 `lut`、`save`、`type`、`epoch`。
- 停稳计时器在 `MotorCalibrationSave`，不再使用函数内 static。
- 进度回调带上下文，避免隐藏单例。

### 6.8 `composition/app_rtt.h`

```c
typedef struct {
    RTT_Data_struct data;
    char buffer[4096];
    const LinearHall *linear_hall;
} RttContext;

void AppRtt_Init(RttContext *rtt, const LinearHall *linear_hall);
void AppRtt_Sampling(RttContext *rtt);
```

RTT 不接收 `MotorAxis`、`AppContext` 或反馈大结构。

### 6.9 `motor_state_machine/motor_state_machine.h`

`MotorStateEvent` 六个事件；`MotorState MotorStateMachine_Step(MotorState state, MotorStateEvent event);`。无上下文、无输出指针，R3。

### 6.10 `interfaces/motor_state_types.h`

只保留 `MotorState`、`MotorControlResult`、`MotorCalibrationType`、`MotorStatus`、`MotorTarget`。无函数、无状态。

### 6.11 `interfaces/motor_stop_request.h`

```c
typedef enum {
    MOTOR_STOP_RESULT_OK,
    MOTOR_STOP_RESULT_ACCEPTED,
    MOTOR_STOP_RESULT_DENIED,
    MOTOR_STOP_RESULT_UNAVAILABLE,
    MOTOR_STOP_RESULT_FAILED
} MotorStopResult;

typedef struct {
    bool (*request_stop)(void *owner_context, uint8_t source_node, uint16_t sequence);
    bool (*is_power_disabled)(void *owner_context);
} MotorStopOwnerPort;

typedef struct {
    const MotorStopOwnerPort *port;
    void *owner_context;
    bool confirmed;
    uint32_t token;
} MotorStopService;

bool MotorStopService_Init(MotorStopService *service,
                           const MotorStopOwnerPort *port,
                           void *owner_context);
MotorStopResult MotorStopService_RequestStop(MotorStopService *service,
                                             uint8_t source_node,
                                             uint16_t sequence,
                                             uint32_t *token_out);
```

`owner_context` 是通用端口的类型擦除，必须在头文件写明具体所有者类型和串行约束；`token_out` 是唯一输出参数。

## 7. 执行上下文与数据所有权

| 上下文 | 入口 | 允许的工作 |
| --- | --- | --- |
| 上电 | 板级 main | BSP 关相 → `App_Init` → `Bsp_Start` → `App_Start` |
| ADC 快拍 | `App_OnFastSample` / AT32 ADC | `SampleFast` → `ControlFast` → `fast_samples++` |
| 外环 | AT32 PendSV / STM32 监督 | `MotorAxis_OuterTick`：快照、计算、`CommitOuter` |
| 监督 | `App_SupervisorTick` | 温度、STOP、故障、标定收尾、STM32 外环 |
| 前台 | `App_Process` | 位置规划、标定发布、状态刷新、协议、Loader、看门狗 |
| CAN RX | `Protocol_RxInterrupt` | 有界搬帧入队 |
| Flash/发送 | 仅前台 | 关相、单上下文、禁止 ISR |

所有权：

- Lifecycle 拥有五态、epoch、启停意图、功率许可、校准会话、故障恢复。
- Axis 拥有配置、控制状态、FOC、编码器、观测器、无感启动、Hall、输出轴、PI、记录暂存区。
- Calibration 拥有各阶段工作区、候选、停稳计时、发布准备。
- Protocol 拥有协议缓存、目标去重、STOP 服务。
- Loader 拥有交接事务、超时、票据、健康窗口。
- RTT 拥有诊断缓冲和 `const LinearHall *`。
- BSP/通信拥有硬件和协议栈单例。

## 8. 关键调用链

**上电**

```text
main
  Bsp_Init
  App_Init(&g_app)
    MotorAxis_Init(&g_app.axis, &g_app.lifecycle)
    MotorLifecycle_Init(&g_app.lifecycle, &bindings)
    AppProtocol_Init(&g_app.protocol, &g_app.lifecycle)
    AppLoader_Init(&g_app.loader, &g_app.lifecycle, BootHw_IsAvailable())
    AppRtt_Init(&g_app.rtt, &g_app.axis.linear_hall)
  Bsp_Start
  App_Start(&g_app)
  for (;;) App_Process(&g_app)
```

**快拍**

```text
App_OnFastSample(&g_app.axis, &g_app.fast_samples)
  MotorAxis_SampleFast(&g_app.axis)
  MotorAxis_ControlFast(&g_app.axis)
    MotorLifecycle_BeginFastTick(&g_app.lifecycle)
    type = MotorLifecycle_GetCalibrationType(&g_app.lifecycle)
    if runnable: MotorCalibration_Step(&g_app.axis.calibration, type)
    else if RUNNING: MotorControl_Step(...)
    MotorLifecycle_ReportOutcome(&g_app.lifecycle, outcome, epoch)
```

**外环**

```text
MotorAxis_OuterTick(&g_app.axis)
  编码器慢估计 / Hall 快照 / 输出轴
  if Hall: MotorCalibration_UpdateHall(&g_app.axis.calibration.lut, &g_app.axis.hall_sample)
  MotorOuterInput input;
  if (MotorLifecycle_BeginOuter(&g_app.lifecycle, &input)) {
      MotorOuterLoop_SlowTick(&g_app.axis.configuration, &input.motor, &input.speed_pi,
                              &g_app.axis.rotor_encoder, &result);
  }
  MotorLifecycle_CommitOuter(&g_app.lifecycle, input.epoch, &result);
```

**前台**

```text
App_Process(&g_app)
  MotorAxis_ProcessPositionPlan(&g_app.axis)
  MotorCalibration_ProcessPublication(&publish_input, now, progress)
  status = MotorLifecycle_GetStatus(&g_app.lifecycle)
  AppProtocol_Refresh(&g_app.protocol, &status, MotorLifecycle_GetEpoch(&g_app.lifecycle))
  Protocol_Process(now)
  AppLoader_Process(&g_app.loader, now, g_app.communication_ready)
  AppWatchdog_Process(&g_app.watchdog, now, g_app.fast_samples, g_app.supervisor_ticks)
```

## 9. 迁移阶段

| 阶段 | 内容 | 完成条件 |
| --- | --- | --- |
| P0 | 冻结基线；写死接口/所有权契约；修正文档漂移 | 标定 Flash、RTT、外环频率、测量有效位描述一致 |
| P1 | `MotorCalibration` 上下文化；阶段子上下文；停稳计时移入 save | 无函数内 static；取消/失败/代际测试通过 |
| P2 | `MotorAxis`/`MotorLifecycle` 上下文化；`BeginOuter`/`CommitOuter` | 无可变 `s_*`/`g_motor_context`；行为等价；外环交错测试通过 |
| P3 | `AppContext`、`AppWatchdog`、`AppProtocolContext`、`AppLoaderContext`、RTT 窄接口；BSP 回调移到 main.c | 只有 `App_Init/Start/Process` 接收 `AppContext *` |
| P4 | `ProtocolMotor` context（或 R1 文档化）；命名重命名；Watch 路径更新 | 旧名 grep 清零；双板/变体编译 |
| P5 | 物理文件拆分收口；Keil/测试/文档同步 | 双板构建、PR、接口检查通过 |
| P6 | 最终验证、资源/时序记录、归档 | 行为、资源、时序、未测项有证据 |

## 10. 验证

- `AppContext *` 只出现在 `app.h`、`app_context.h`、`app.c`、`main.c`。
- `runtime/` 下不存在可变 `static s_*` 对象；允许 `static const` 表。
- APP 公共头的 `void *` 只出现在 `MotorStopOwnerPort` 和协议回调，且注释写明具体类型。
- 每个公共接口的参数都在函数体或回调中被读取。
- 旧符号 `MotorLifecycle_OuterTick`、`MotorAxis_Feedback`、`RTT_Sampling`、`g_motor_context` 清零。
- 双实例测试：两个 `MotorAxis`/`MotorLifecycle`/`MotorCalibration` 上下文交替调用，结果互不影响。
- ISR 交错测试：ADC/外环/前台交错下 STOP、epoch、标定取消仍正确。
- 行为等价：协议黄金帧、标定保存、Loader 事务、故障恢复。
- 资源：ROM/RAM/栈对比；`MotorAxis` 内嵌 6 KiB 工作区，总量不增加。
- 时序：快环最坏执行时间、外环积压、Flash 窗口。
- 门禁：`check_interfaces`、`check_architecture`、`check_project_layout`、`format --check`、`lint`、`verify --profile pr`。

## 11. 命名与重命名

| 现名 | 目标名 | 阶段 | 原因 |
| --- | --- | --- | --- |
| `RTT_Init` | `AppRtt_Init` | P4 | 模块前缀 |
| `RTT_Sampling` | `AppRtt_Sampling` | P4 | 模块前缀 |
| `MotorAxis_NodeId` | `MotorAxis_GetNodeId` | P4 | `Get` 查询 |
| `MotorAxis_ProcessCalibrationPublication` | `MotorCalibration_ProcessPublication` | P1 | 职责迁移 |
| `MotorLifecycle_OuterTick` | `MotorLifecycle_BeginOuter` + `CommitOuter` | P2 | 计算/提交分离 |
| `MotorLifecycle_WorkEpoch` | `MotorLifecycle_GetEpoch` | P2 | `Get` 查询 |
| `MotorLifecycle_ReadStatus` | `MotorLifecycle_GetStatus` | P4 | `Get` 查询 |
| `MotorLifecycle_ReadBootHealth` | `MotorLifecycle_GetBootHealth` | P4 | `Get` 查询 |
| `MotorLifecycle_ReadPendingCalibrationPublication` | `MotorLifecycle_GetPendingCalibrationPublication` | P1/P4 | `Get` 查询 |
| `MotorLifecycle_ContinueElectricalZero` | `MotorLifecycle_ContinueCalibrationSession` | P1 | 会话语义 |
| `MotorLifecycle_PostOutcome` | `MotorLifecycle_ReportOutcome` | P2 | `Report` 语义 |
| `motor_stop_result_t` | `MotorStopResult` | P4 | PascalCase |
| `motor_stop_owner_port_t` | `MotorStopOwnerPort` | P4 | PascalCase |
| `motor_stop_service_t` | `MotorStopService` | P4 | PascalCase |

命名规则沿用 `STYLE.md`：文件 snake_case；公共函数 `Module_ActionObject`；私有 `static snake_case`；类型 PascalCase；文件私有对象 `static s_`；对外可变全局 `g_`；宏 UPPER_SNAKE；单位后缀 `_a/_v/_rad/_rad_s/_s/_ms/_counts/_q15`；`Calibration` 写全，APP 新代码不用 `Calib`。

## 12. 风险与回退

- `AppContext` 较大，且内嵌 6 KiB 标定工作区；只允许一个静态实例，RAM 总量不变，但 Keil 符号路径改变。
- Watch 路径从 `s_motor_configuration` 等改为 `g_app.axis.configuration`、`g_app.lifecycle`；需同步 `docs/guides/watch_motor_commands.md`。
- `ProtocolMotor` 增加 context 会触及通信层和测试；若延期，`app_protocol` 保持 R1，并在头文件写明单例约束。
- 标定阶段拆分会增加类型数量；每个子上下文只放指针和工作区，不复制数据。
- 物理文件拆分改变 Keil 源文件列表；每阶段同步双板、LoaderApp、测试构建清单和文档链接。
- 每阶段以前一已验证提交为回退点；行为、资源、时序差异必须解释；缺板或缺测量标记未测。
