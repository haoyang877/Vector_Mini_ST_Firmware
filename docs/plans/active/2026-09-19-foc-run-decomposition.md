# foc_run.c 拆解简化 v1.0

日期：2026-09-19。状态：实施完成，随本轮离线验证提交。

## 意图与验收

`firmware/app/foc_run.c`（1046 行）承载全部运行模式任务入口、无感启动状态机、位置模式配置适配
与外环延迟运行时，其中 `Task_Sensorless_Speed_Mode` 单函数 304 行、5 个状态深度嵌套。本轮目标：
在不改变任何逐 tick 可观察行为的前提下拆解长函数、消除重复逻辑，并使文件满足仓库风格门禁
（clang-format、全花括号、中文注释）。验收条件：

1. 公共 API/ABI 不变：`foc_run.h` 不修改，所有 `Task_*` 与 `MotorOuterLoop_*` 符号、参数与语义不变；
2. 无感任务按状态拆分为单一职责静态函数，状态迁移、复位与置错时序逐 tick 等价；
3. 重复逻辑收敛：速度斜坡、位置限幅、摩擦/阻抗配置、失败收尾；
4. 现有原生等价性测试全部通过：无感交接电流连续性、外环 42 万 tick、模式 3 与冻结基准 25200 tick；
5. `verify --profile pr` 通过（format、lint、原生测试、架构、接口、文档、卫生）。

## 拆解映射

| 原结构 | 新结构 |
| --- | --- |
| `SpeedMode_UpdateControl` 与 `Sensorless_UpdateSpeedReference` 两份相同斜坡 | 共享 `MotorControl_UpdateSpeedRamp` |
| `Task_Sensorless_Speed_Mode` 5 状态 switch（304 行） | 任务只做准入校验与分发；新增 `Sensorless_RunAlign/_RunOpenLoop/_RunSpeedLock/_RunHandoff/_RunClosedLoop` |
| 交接捕获块（旋转 dq 电流、预置速度 PI） | `Sensorless_BeginHandoff` |
| 小角度 sin/cos 展开与 PI 积分器旋转 | `Sensorless_SmallAngleSinCos`、`Sensorless_RotateToObserverFrame/_RotateFromObserverFrame` |
| `Task_Sensorless_Speed_Mode` 8 个参数逐层传递 | 文件私有 `SensorlessStartupRun_TypeDef` 上下文（指针 + pole_pairs + direction） |
| `PositionMode_UpdateConfiguration` 摩擦配置块（36 行） | `PositionMode_ApplyFrictionConfiguration` |
| 两处重复的解耦/限速钳位 | `PositionMode_EffectiveDeceleration/_EffectiveMaximumSpeed` |
| `Task_Position_Mode` 两处失败收尾 | `PositionMode_RejectRequest` |
| `Task_Position_Impedance_Mode` 25 项配置赋值 | `PositionImpedance_BuildConfiguration` |
| 测试夹具按注释文本切片 | 显式 `SENSORLESS_RUNTIME_BEGIN/END` 区域标记（与既有 `OUTER_RUNTIME` 一致） |

## 关键决策

- 状态处理函数统一 `void` + 内部早退：原 switch 各分支 `return` 之后不存在任务级代码，
  故「处理函数早退」与「任务级 return」可观察行为一致（含置错后不再执行后续电流输出的顺序）。
- 旋转辅助保持原浮点运算顺序与操作数顺序，确保与冻结基准逐位一致（配置缓存测试按位比较）。
- `MotorControl_UpdateSpeedRamp` 由速度模式与无感模式共享：两份原文本逐 token 相同，
  单位、时序、上下文一致，符合 STYLE.md 复用前提。
- 失效风格债务随改动一并清理：文件进入风格门禁后必须通过 c-format、c-readability 与中文注释检查；
  同步更新三个原生测试的提取列表/切片标记，其 Python 文件按 Ruff 重新格式化。
- 不修改 `foc_run.h`：无签名变化，其接口契约债与哈希豁免保持原基线。

## 未覆盖路径（改动为机械等价，建议评审时对照 diff）

- 无夹具直接覆盖：`Task_Current_Mode`、`Task_Speed_Mode`、`Task_Voltage_Mode`、
  `Task_Vq_Mode`、`Task_Position_Mode_Reset`、`PositionImpedance_BuildConfiguration`
  与 `Task_Position_Impedance_Mode`；这些仅做语句搬迁与中文注释，控制流未变。
- 无感启动夹具从 SPEED_LOCK/HANDOFF/CLOSED_LOOP 预设状态进入，因此 ALIGN 与
  OPEN_LOOP 两条上升沿路径（含方向反转复位）未被断言覆盖，仅经人工 diff 核对
  与状态迁移时序（同 tick 迁移、先输出后置错）逐条比对。

## 验证证据

- 隔离快照全量离线套件（HEAD `baaa0338` + 本轮 4 个改动文件，排除并行 WIP）：
  `tests/run.py` 16 项全部 PASS，含无感交接电流连续性、外环 42 万 tick、模式 3
  与冻结基准 25200 tick 位级对比；日志归档于 `outputs/refactor_evidence/`。
- 主树定向门禁：`format --check` PASS（0 新问题）、`lint` PASS（0 新问题）、
  `check_interfaces` PASS（0 新违规）、`tests.unit.test_harness` 全部通过。
- 说明：主树当时处于并行"统一运行状态机"改造中途（`foc_calibration.c` 已改返回
  `MotorWorkOutcome_TypeDef`，而 `foc_calibration.h` 仍声明 `void`），该 WIP 收敛前
  主树无感夹具不可运行，与本轮改动无关；隔离快照即为排除该因素后的验证环境。
