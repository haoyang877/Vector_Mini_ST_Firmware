# foc_run 归位与瘦身 v1.0

日期：2026-09-19。状态：阶段 0/1.1/1.2 已完成；阶段 1.3 待决策；阶段 2 待办。

## 意图与验收

`foc_run.c` 拆解（见 [foc_run.c 拆解简化](2026-09-19-foc-run-decomposition.md)）完成后，
接口的层级归属仍有历史错位：app 文件承载纯电机算法、motor 模块反向依赖 app 头、
communication 用本地原型绕过依赖方向。本轮目标：**公共接口签名不变**的前提下把职责归位、
消除全部可消除的反向依赖，并让文件边界与仓库分层一致。验收条件：

1. 保留清单内的公共接口签名与行为不变；
2. 架构棘轮零新增违规，且反向依赖逐步减少（现有债务只减不增）；
3. 每阶段以 `tests/run.py` 全量、`verify --profile pr`、Keil 双目标 0 Error/0 Warning 验收；
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
| 1 | 1.3 位置入口决策 | `Task_Position_Mode` 无生产调用者但为模式 3 顺序入口；删除需重定向等价性测试入口（覆盖度略降），保留则与 `Task_Speed_Mode` 形成对称外部接口。**待决策**：删除 / 保留并文档化 / 接入显式调用方 | 待决策 |
| 2 | 2.1 hw_conf 控制常量上移 | `Current_Ts/Speed_Ts/SPEED_LOOP_DIVIDER/POSITION_*/CASCADE_*` 的派生机制移入 `platform/api` 板级参数契约 | 前置条件 |
| 2 | 2.2 速度环核心下沉 | `SpeedMode_UpdateControl`+斜坡迁入 motor；摩擦辨识改调核；`Task_Speed_Mode` 保留为 app 包装 | 依赖 2.1 |
| 2 | 2.3（可选）文件级拆分 | `app/foc_run.c` → `foc_mode_tasks.c` / `foc_outer_loop.c` / `foc_position_adapter.c` | 依赖 2b 收敛 |

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

| 夹具 | 阶段 0 | 阶段 1.1 | 阶段 1.2 | 阶段 1.3 |
| --- | --- | --- | --- | --- |
| `run_can_status_tests.py` | 删除 `MotorOuterLoop_GetTelemetry` stub 与 telemetry 类型，改影子字段断言 | — | — | — |
| `test_sensorless_transitions.py` | — | 切片源改为 `foc_sensorless.c`（区域标记随代码迁移） | — | — |
| `test_outer_loop_runtime.py` | — | — | 若邮箱失效收口点变化，补故障/清空时序断言 | — |
| `test_position_config_cache.py` | — | — | — | actual 入口改为适配器组合 |
| `test_bus_voltage_protection.py` | — | — | `Task_Position_Mode_Reset` stub 随 1.2 调整 | — |

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
- 已知边界：镜像在 `MotorOuterLoop_FastTick` 内执行，模式切换提交到下一 tick 之间
  （≤50 µs）影子字段仍可能显示上一次准备值而非 NaN；该窗口远小于 CAN 状态帧周期，
  且旧实现同样依赖 tick 边界，接受不另行加锁。

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
- 阶段 2 与 1.3：实施后补充。
