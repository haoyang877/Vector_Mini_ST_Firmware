# foc_run 归位与瘦身 v1.0

日期：2026-09-19。状态：主体完成并提交（阶段 0/1.1–1.4、2.1/2.2，硬件解耦 H1–H4 由本轮与并行工作流共同完成，架构债 44 → 11）。余量：H1.2 轴 profile 决策、H5 反向依赖（interface_can 4 条 + 服务/协议 7 条）、2.3（可选）。

## 意图与验收

`foc_run.c` 拆解（见 [foc_run.c 拆解简化](2026-09-19-foc-run-decomposition.md)）完成后，
接口的层级归属仍有历史错位：app 文件承载纯电机算法、motor 模块反向依赖 app 头、
communication 用本地原型绕过依赖方向。本轮目标：**公共接口签名不变**的前提下把职责归位、
消除全部可消除的反向依赖，并让文件边界与仓库分层一致。验收条件：

1. 保留清单内的公共接口签名与行为不变；
2. 架构棘轮零新增违规，且反向依赖逐步减少（现有债务只减不增）；
3. 每阶段以 `tests/run.py` 全量、`verify --profile pr`、Keil 主工程 0 Error/0 Warning 验收；
4. 原生测试夹具随文件迁移同步更新，等价性测试价值不降低。

**明确保留（签名不动）**：`Task_Speed_Mode`（对外接口，用户确认）、`Task_Current_Mode`、
`Task_Position_Impedance_Mode`、`Task_Voltage_Mode`、`Task_Vq_Mode`、`MotorOuterLoop_*`
外环运行时、`PositionMode_*` 位置适配器。

## 阶段与步骤

| 阶段 | 步骤 | 改动 | 状态 |
| --- | --- | --- | --- |
| 0 | 0.1 清除 `interface_can` 隐藏依赖 | 新增 `MotorControl.pos_trajectory_speed_rad_s` 影子字段；`MotorOuterLoop_FastTick` 按遥测有效期逐 tick 镜像 planned 值（无效窗口 NaN）；CAN 删除本地原型与调用，改读影子字段；同步更新 `run_can_status_tests.py` | 完成 |
| 0 | 0.2 `foc_run.c` include 显式化 | 去掉 `common_inc.h`，改最小显式头（utils/foc_errhandle/hw_conf/motor_axis_profile/string 等）；Keil 构建暴露缺失的 `<string.h>` 并已补 | 完成 |
| 0 | 0.3 本计划入库 | 本文档 + plans 索引 | 完成 |
| 1 | 1.1 无感序列下沉 | 代码与区域标记迁至 `motor/foc/foc_sensorless.{c,h}`（复用该文件既有 hw_conf 依赖，零新增违规）；共享速度斜坡随迁并公开（`MotorControl_UpdateSpeedRamp`）；`foc_run.c` 保留薄包装 `Task_Sensorless_Speed_Mode`（`foc_run.h` 不变）；`foc_calibration.c` 改调 `SensorlessStartup_Run` 并删 `<foc_run.h>`；两个原生夹具同步迁移；`foc_sensorless.h` 7 条历史接口债还清并从 `interface_debt.json` 删除 | 完成 |
| 1 | 1.2 `Task_Position_Mode_Reset` 拆分 | `foc_errhandle` 两处改调 `PositionImpedance_Reset` 并删 `foc_run.h`（架构债 −1，基线条目已删除）；`foc_run_state.c` 首次进入故障时统一 `Task_Position_Mode_Reset()`（邮箱失效 + 状态复位收口）；bus_voltage/run_state 夹具同步；模式切换的邮箱失效沿用 FastTick 既有逻辑 | 完成 |
| 1 | 1.3 位置入口决策 | `Task_Position_Mode` 保留为模式 3 顺序入口（与 `Task_Speed_Mode` 对称的外部/调试接口），定义处已注明"生产由外环执行本适配器"；不删除，等价性夹具继续以其为被测入口 | 完成（保留并文档化） |
| 1 | 1.4 无感运行模块拆分 | 启动序列与速度模式从观测器实现拆出为 `motor/foc/foc_sensorless_run.{c,h}`（观测器文件只留 Fluxobserver/Observer）；`SensorlessStartup_Reset` 随迁；`EncoderCalibConfig` 归位 `foc_calibration.c`（其 profile 所有者，已含 hw_conf 债务）；控制时基与无感默认参数下沉 `platform/api/control_config.h`（跨层文档化契约，hw_conf 引用并派生 PWM 参数）；删除两个全仓库无消费者的派生宏；`foc_run.c`/`motor_state.c`/calibration/errhandle 补 include；两个原生夹具与两个 Keil 工程同步 | 完成 |
| 2 | 2.1 hw_conf 控制常量上移 | 控制时基（FOC/速度/位置/级联环）与无感启动默认参数移入 `platform/api/control_config.h`；`hw_conf.h` 改为引用并派生 `PWM_TIM_FREQ` | 完成（作为 1.4 前置） |
| 2 | 2.2 速度环核心下沉 | 新建 `motor/foc/foc_speed.{c,h}`：`MotorControl_UpdateSpeedRamp` 自运行模块迁入、`SpeedMode_UpdateControl` 自 `foc_run.c` 迁入（去 static）、新增 `SpeedMode_Run`（分频 + 更新 + 电流环，顺序任务核心）；`Task_Speed_Mode` 保留为 app 薄包装（签名不变，外部接口）；摩擦辨识改调 `SpeedMode_Run` 并删 `foc_run.h`（**最后一条 motor→app 反向依赖消除**，架构债 −1）；夹具/两工程/债务基线同步 | 完成 |
| 2 | 2.3（可选）文件级拆分 | `app/foc_run.c` → `foc_mode_tasks.c` / `foc_outer_loop.c` / `foc_position_adapter.c` | 待办 |

## 依赖矩阵（改动面 × 并行工作）

| 文件 | 阶段 0 | 阶段 1 | 被并行 2b 修改中 |
| --- | --- | --- | --- |
| `firmware/app/foc_run.c` / `.h` | 0.2（includes）、0.1（镜像） | 1.1/1.2/1.3 | 否（本轮拆解独占） |
| `firmware/motor/data_type.h` | 0.1（追加字段） | — | 否 |
| `firmware/communication/can/interface_can.c` | 0.1 | — | 否 |
| `firmware/motor/identification/foc_calibration.c/h` | — | 1.1 | **是（2b 收敛后再动）** |
| `firmware/motor/protection/foc_errhandle.c` | — | 1.2 | 否 |
| `firmware/app/foc_mode_dispatch.c/h` | — | 1.2/1.3 | **是（2b 收敛后再动）** |
| `firmware/motor/foc/foc_sensorless.c/h` | — | 1.1 | 否 |
| `firmware/platform/api/**` | — | — | 否（阶段 2 前置） |
| `tests/unit/native/run_can_status_tests.py` | 0.1 | — | 否 |
| `tests/unit/native/test_sensorless_transitions.py` | — | 1.1 | 否（已在本轮拆解时更新过） |
| `tests/unit/native/test_position_config_cache.py` | — | 1.3 | 否 |

## 夹具迁移清单

| 夹具 | 最终状态 |
| --- | --- |
| `run_can_status_tests.py` | 已删除 `MotorOuterLoop_GetTelemetry` stub 与 telemetry 类型，改影子字段断言（0.1） |
| `test_sensorless_transitions.py` | 切片源与 `SensorlessStartup_Reset` 提取改指 `foc_sensorless_run.c`，`MotorControl_UpdateSpeedRamp` 独立自 `foc_speed.c` 提取；PRELUDE 补两个模块头；标定配置经 calibration 切片提供（1.1→1.4→2.2） |
| `test_outer_loop_runtime.py` | 斜坡与 `SpeedMode_UpdateControl` 自 `foc_speed.c` 提取，其余仍取自 `foc_run.c`（1.1→1.4→2.2） |
| `test_position_config_cache.py` | 保持以 `Task_Position_Mode` 为 actual 入口（1.3 保留决策），无需修改 |
| `test_bus_voltage_protection.py` | `Task_Position_Mode_Reset` stub 改为 `PositionImpedance_Reset`（1.2） |
| `test_run_state.py` | 新增 `Task_Position_Mode_Reset` 静默 stub（1.2） |

## 关键决策

- `interface_can` 的 planned 值不引入新 services 模块：把 mode-3 规划量发布为
  `MotorControl` 影子字段（`posShadow` + 新增 `pos_trajectory_speed_rad_s`），由
  `MotorOuterLoop_FastTick` 按 `telemetry_valid` 逐 tick 镜像；无效窗口写 NaN，
  与旧的"遥测无效即 NaN"语义逐位一致。追加字段遵循 `data_type.h` 的"尾部追加保持偏移"约定。
- 阶段 1.1 不新增 motor→hw_conf 违规：目标文件 `foc_sensorless.c` 已有该依赖（记录在案）；
  任何其它 motor 新文件在下沉前必须先完成 2.1。
- 阶段 1/2 的 app/motor 改动在统一运行状态机（2b）收敛后执行，避免同区域并行冲突。
- 1.2 保持"快速侧不直接复位级联控制器"的约束：外环 worker 运行在 PendSV 且可被 20 kHz
  快中断抢占，快速侧直接调用 `PositionCascade_Reset()` 会与被抢占的 worker 竞争；因此
  快速侧只置故障/请求，邮箱失效由 app 故障提交点收口，motor 侧仅复位与调用方同上下文
  运行的阻抗控制器。
- 1.4 的拆分边界遵循"最窄所有者"：观测器算法留在 `foc_sensorless`；启动序列与速度模式
  组成独立的无感运行模块；`EncoderCalibConfig` 属标定 profile，放在 `foc_calibration.c`
  并由其直接初始化（避免 motor 新文件依赖 hw_conf）；控制时基与无感默认值作为跨层契约
  放 `platform/api/control_config.h`，`hw_conf.h` 引用并派生定时器参数（bsp→api 合法）。
- 1.4 顺带删除两个全仓库无消费者的派生宏（`SENSORLESS_ALIGN_TIME_S`、
  `SENSORLESS_STARTUP_ELEC_ACCEL_RAD_S2`），符合"删除不再需要的行为"。
- 已知边界：镜像在 `MotorOuterLoop_FastTick` 内执行，模式切换提交到下一 tick 之间
  （≤50 µs）影子字段仍可能显示上一次准备值而非 NaN；该窗口远小于 CAN 状态帧周期，
  且旧实现同样依赖 tick 边界，接受不另行加锁。

## 硬件解耦（H 阶段，按 AGENTS.md「平台层外禁止直接硬件访问」编排）

| 步骤 | 内容 | 状态 |
| --- | --- | --- |
| H1.1 | `data_type.h` 去 CubeMX `main.h`，显式 `<stdint.h>` | 完成 |
| H1.3 | `common/utils.h`、`common/heap.h` 去 `main.h`（heap.c 显式补 `<stdint.h>`；common 仅剩纯软件） | 完成 |
| H1.4 | `foc_sensing.h`、`foc_traptraj.h` 去 `main.h`；四个头文件补齐 27 条中文接口契约并还清接口债 | 完成 |
| H1.2 | 轴 profile 纯类型下沉 motor | 待决策：`foc_errhandle` 调 services 的 `MotorAxisProfile_AllowsPosition`，移类型需连带拆"类型+谓词"或把校验上移 app |
| H2.1 | `foc_sensorless.c` 用 `platform/api/control_config.h` | 完成（早前） |
| H2.2 | 其余 `hw_conf` 用户归位：`foc_cogging_calibration.c`/`foc_friction_identification.c` 只留时基改 `control_config.h`；`foc_traptraj.c` 删未用包含；`foc_traptraj.c` 借此从 GBK 转为 UTF-8；`position_impedance.c` 走感测契约（宏改全名）；`position_impedance_config.h` 改引用新契约；**新增 `platform/api/motor_hardware_profile.h`（阻尼环/前馈选择簇）**；`current_sense_profile.h` 由 bsp 迁入 `platform/api`；`foc_param.c` 改用 `CURRENT_SENSE_PROFILE_*` 全名 | 完成（`foc_sensing.c` 余量由并行会话的感测重构一并清除） |
| H4a | `foc_algorithm.c` 的 `TIM1->CCR` 直访 → `motor_hw` PWM 端口 | 完成（并行会话，提交 `23ef80ab`） |
| H4b | `foc_sensing.c` 的 `adc.h`、`foc_errhandle.c` 的 `tim.h` → 平台接口 | 完成（感测契约 + 功率级端口，并行批次提交） |
| H3 | `encoder.h` BSP 大结构 → `platform/api` 采样快照接口 | 完成（并行会话编码器解耦：`aabd03f4` + 在途批次） |
| H5 | 其余反向依赖：`interface_can.*` 的 BSP/HAL 引用（delay/hw_conf/fdcan/main）、`data_type.h→services/motor_axis_profile.h`、`foc_algorithm/position_cascade→fast_loop_profile.h`、`friction/phase_resistance→foc_param_profile.h`（`foc_param.c→common_inc.h`、`foc_param.h→main.h` 已于 2026-09-19 清除） | 部分完成（架构债余 9 条，见下） |

## 收尾与余量（2026-09-19）

架构债剩余 **9 条**（44 → 9；2026-09-19 又清 `foc_param.c→common_inc.h`、`foc_param.h→main.h`
两条，聚合头 `app/common_inc.h` 已删除）：

- **interface_can 批次（4 条）**：`delay.h`、`hw_conf.h`、`fdcan.h`、`main.h` —— 需要通信层接入
  `time_hw`/`comm_hw` 契约并按需提取 CAN 帧缓冲视图。
- **服务/协议反向（5 条）**：`data_type.h → motor_axis_profile.h`（轴 profile 纯类型下沉，
  需先决定 `MotorAxisProfile_AllowsPosition` 归属，即 H1.2）、`foc_algorithm/position_cascade
  → fast_loop_profile.h`（profiling 契约归位）、`friction/phase_resistance →
  foc_param_profile.h`（参数 profile 边界）。
- **可选**：2.3 `foc_run.c` 文件级拆分（纯可读性；动 Keil 工程与三个夹具）。
- 已由并行工作流关闭：编码器解耦（H3）、标定功能移除与后续清理、感知/功率级契约（H4b）。

## 验证证据

- 阶段 0（2026-09-19，主树含并行 2b 改动）：
  - Keil 双目标 `build_firmware --target all`：0 Error / 0 Warning
    （普通 Code=86232、ZI=31432；HIL Code=86752、ZI=31480；相对基线 RAM +8/+16 B 为影子字段对齐）；
  - `tests/run.py` 全量 17 项 PASS（含 `run_can_status_tests` 影子字段断言与
    `test_position_config_cache` 位级等价）；
  - `format --check` 0 新问题、`lint` 0 新问题、`check_interfaces` 0 新违规、
    `tests.unit.test_harness` 全部通过。
  - 日志：`outputs/phase0/suite/`、`outputs/build/logs/`。
- 阶段 1.1（2026-09-19，foc_run.c 1335 → 769 行；foc_sensorless.c 164 → 793 行）：
  - Keil 双目标 0 Error / 0 Warning（Code 86600/87120，RAM ZI 31432/31480 不变；
    较阶段 0 Code +368 B 为跨 TU 后入口与斜坡不再内联，属预期）；
  - `tests/run.py` 全量 17 项 PASS（无感交接夹具改切 `foc_sensorless.c`、
    外环夹具从新文件提取斜坡、模式 3 位级等价）；
  - `format --check` 0 新问题、`lint` 0 新问题、`check_interfaces` 0 新违规
    （7 条接口债还清并移除基线条目）、`test_harness` 全部通过；
  - 日志：`outputs/phase1/suite/`、`outputs/build/logs/`。
  - 遗留说明：下沉时按层约束保留了薄包装，因此存在一次跨 TU 调用；若后续
    速度环核心下沉（2.2）后统一评估，可考虑让 dispatcher 直接调用 motor 核。
- 阶段 1.2（2026-09-19）：
  - Keil 双目标 0 Error / 0 Warning；
  - `tests/run.py` 全量 17 项 PASS（含 `test_run_state` 差分 38,400 组、`test_bus_voltage_protection`）；
  - `format --check` / `lint` / `check_interfaces` / `check_architecture` / `test_harness` 全部通过；
    架构债 −1（`foc_errhandle.c -> foc_run.h` 已从 `architecture_debt.json` 删除）；
  - 日志：`outputs/phase1/suite12/`、`outputs/build/logs/`。
- 阶段 1.3/1.4（2026-09-19，模块拆分 + 契约头前置）：
  - Keil 主工程 `build_firmware --target normal`：0 Error / 0 Warning
    （Code=88876、ZI=31456，与拆分前 88912 基本持平）；
  - `tests/run.py` 全量 17 项 PASS（无感夹具改切 `foc_sensorless_run.c` 并从
    calibration 切片取标定配置；外环夹具从新模块提取斜坡；模式 3 位级等价）；
  - `format --check` / `lint` / `check_interfaces` / `check_architecture` / `test_harness`
    全部通过；`check_project_layout` 两工程 0 错误（新文件已登记）；
  - 日志：`outputs/phase1/suite_split2/`、`outputs/build/logs/`。
  - 说明：A/B 实验确认控制常量上移为尺寸中性（上移前后主工程目标逐字节一致）。
- 阶段 2.2（2026-09-19，速度环核心下沉）：
  - Keil 主工程 0 Error / 0 Warning（Code=89340、ZI=31460；较拆分前 +464 B 为跨 TU
    后速度环与斜坡不再内联，属预期）；
  - `tests/run.py` 全量 17 项 PASS（外环夹具改从 `foc_speed.c` 提取斜坡与速度环、
    无感夹具独立提取斜坡）；摩擦辨识切换后单独复跑两个相关夹具仍 PASS；
  - `lint` / `check_interfaces` / `check_architecture`（36 条已知、0 新增）/
    `test_harness` / `check_project_layout`（两工程 0 错误）通过；
    架构债 −1：`foc_friction_identification.c -> foc_run.h` 已删除，motor 层不再
    直接依赖 app 头（`foc_run.h` 仅 app 内部与统一聚合头引用）；
  - 日志：`outputs/phase1/suite_speed/`、`outputs/build/logs/`。
  - 并行备注：验证期间另一会话正在迁移 `mcu_temperature.h`（motor/foc → platform/api）
    并对测试追加 encoding 参数，仓库级 `format --check` 的个别失败来自这些在途改动；
    本轮所属文件经定向 `format --check` 验证为 0 问题。
- 硬件解耦 H1.1/H1.3/H1.4（2026-09-19）：
  - Keil 主工程 0 Error / 0 Warning（Code=89224，纯 include 变更尺寸不变）；
  - `tests/run.py` 全量 17 项 PASS；
  - 架构债 −4（四个头文件 → `main.h` 全部清理，基线 33 → 29）；
    接口债 −4 文件条目（补 27 条中文契约后 `check_interfaces` 0 新增、0 已解决）；
  - `format --check` / `lint` / `check_architecture` / `test_harness` 全部通过；
  - 日志：`outputs/phase1/suite_hw3/`、`outputs/build/logs/`。
- 硬件解耦 H2.2（2026-09-19）：
  - Keil 主工程 0 Error / 0 Warning（Code=89620；含并行会话新提交的功能增量）；
  - `tests/run.py` 全量 17 项 PASS（`test_wheel_speed_limits` 与 `test_position_config_cache`
    随契约链变化同步更新：前者改感测全名后自愈，后者夹具显式包含 stub `hw_conf.h`）；
  - 架构债 −6（`hw_conf` 五个消费方 + `current_sense_profile` 迁移；基线 28 → 22）；
  - `format --check` / `lint` / `check_interfaces` / `check_architecture` / `test_harness` 全部通过；
  - 修复并行提交 `195b7ae4` 引入的主工程编译缺陷：`foc_run_state.c` 使用 `CANMsg`
    但全仓库无 extern 声明，按 `foc_param.c` 既有局部 extern 模式补充（一行）。
  - 日志：`outputs/phase1/suite_h22c/`、`outputs/build/logs/`。
