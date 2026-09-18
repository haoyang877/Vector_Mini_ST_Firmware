# 齿槽补偿扩展到全部转矩闭环模式 v1.0

日期：2026-09-19。状态：待实施（阻塞条件：并行会话正在修改 `foc_task.c`，工作区未落定；
落定后再动代码）。

## 目标与范围

把运行补偿的适用模式从 `Current_Mode` 扩展到所有以 Iq 作为执行量的闭环模式：
`Speed_Mode`、`Position_Mode`、`Position_Impedance_Mode`。齿槽转矩是位置相关扰动，
位置前馈对这些模式的输出同样成立。

本轮只做模式扩展与接线，不改补偿表格式、参数 ABI、CAN 帧格式与渐变/钳位语义；
无感、标定、Vq/电压类模式继续排除。默认关闭、开关不持久化、`iqRef` 所有权语义不变。

## 模式适用矩阵

| 模式 | 结论 | 依据 |
| --- | --- | --- |
| `Motor_Disable`(0) | 允许预开启 | 与现行为一致；不产生输出 |
| `Current_Mode`(1) | 已启用 | 现有基线 |
| `Speed_Mode`(2) | 新增 | 速度环输出即转矩指令，齿槽表现为周期性速度纹波 |
| `Position_Mode`(3) | 新增 | 位置环→Iq，补偿降低周期性跟踪误差 |
| `Position_Impedance_Mode`(18) | 新增 | 同上；与已有摩擦前馈正交叠加 |
| `Sensorless_Speed_Mode`(16) | 排除 | 无编码器位置与身份，查表不可用 |
| 各 `Calib_*`（含 `Calib_Anticogging`、`Calib_Friction`） | 排除 | 补偿会污染辨识采样 |
| `Vq_Mode`(14)、`Voltage_OpenLoop`(11) | 排除 | 电压控制，无转矩闭环 |

## 设计（锁定）

### 1. 适配层模式判据单一来源

在 `foc_cogging_calibration.c` 新增两个静态谓词，调用方与核心之间只传布尔事实：

```c
/* 维持请求/准入的允许集合：停机与全部转矩闭环模式。 */
static bool CompensationModeHoldsRequest(ModeNow_TypeDef mode);
/* 本拍是否叠加补偿：仅转矩闭环模式，不含 Motor_Disable。 */
static bool CompensationModeAppliesTorque(ModeNow_TypeDef mode);
```

- `FocCogging_SetCompensation(true)`：`info.mode_accepts_request = CompensationModeHoldsRequest(ModeNow)`。
- `FocCogging_Apply`：`tick.mode_applies = CompensationModeAppliesTorque(m->ModeNow)`。
- `FocCogging_Service`：`if (!CompensationModeHoldsRequest(ModeNow)) Shutdown();`
  其余（`Disable` 分支 `ZeroBlend`、request/enabled 对账）保持不变。

### 2. 纯核心字段语义调整

`CoggingCompensationTick.mode_is_current` 更名 `mode_applies`（栈上局部结构，非 ABI/全局，
无调试符号影响）；`cogging_compensation.c` 资格判定同步更名。插值、渐变、限幅、
身份复核、CRC 缓存策略全部不变。

### 3. 注入点（三处，均为现有 `FOC_Current` 调用）

| 位置 | 现状 | 改为 |
| --- | --- | --- |
| `foc_task.c` `case Speed_Mode:`（:255-260） | `FOC_Current(...)` | `FOC_CurrentWithReference(&FOC, &MotorControl, θ, ω, FocCogging_Apply(&MotorControl, &OnBoard_Encoder))` |
| `foc_task.c` `case Position_Mode:`（:271-278，保留 `FAST_PROFILE` 标记） | `FOC_Current(...)` | 同上 |
| `foc_run.c` `Task_Position_Impedance_Mode`（:785） | `FOC_Current(...)` | 同上 |

`FOC_CurrentWithReference(..., iq_reference)` 已存在且为 `Current_Mode` 现用路径；
补偿值不写回 `motor->iqRef`。

### 4. 明确不触碰

- `Task_Speed_Mode`（foc_run.c）：`foc_friction_identification.c` 复用它，
  摩擦辨识绝不能叠加补偿；保留原样。
- `Task_Position_Mode`（foc_run.c）：位置配置缓存等价性测试的参照实现
  （`test_position_config_cache.py`）；保留原样。
- `Task_Sensorless_Speed_Mode`：无编码器，排除。

## 行为与安全约束

- 总量仍受 `current_limit`（含台架限流收窄）钳位；`applied_a/total_a/table_a` 遥测语义不变。
- 身份失配、表 CRC 变化、Encoder 掉线、无感标志：与现有每拍判据一致，立即退出且不渐变。
- 离开允许集合由前台 `FocCogging_Service` 立即 `Shutdown`；重启默认关闭。
- 标定模式（`Calib_Anticogging` 等）不经过注入点，且模式判据排除，双重隔离。

## 验证方案

1. **离线（PR profile）**：
   - 扩展 `tests/unit/native/test_cogging_calibration.py`：
     在模拟 tick 下断言 `Speed_Mode/Position_Mode/Position_Impedance_Mode` 叠加、
     `Calib_Friction/Sensorless_Speed_Mode/Vq_Mode` 不叠加；`FocCogging_Service` 在
     退出允许集合时 `Shutdown`；`Motor_Disable` 预开启仍可用。
   - `verify --profile pr` 全绿；`foc_task.c`/`foc_run.c` 触碰后完成格式与中文注释清理，
     对应 style 债务条目随之移除。
2. **台架（需显式授权，逐模式 A/B）**：
   - 速度模式：低速恒速指令，比较 OFF/ON 的速度纹波 RMS 与特征阶次幅值。
   - 位置模式：多点保持与小行程往复，比较跟随误差 RMS/峰值与整定时间。
   - 阻抗模式：同位置模式指标。
   - 安全项：总电流不超限、模式退出自动关闭、身份失配拒绝、STOP 行为不变。
   - 证据写入 `outputs/`，按 HIL 规范记录工作台、电机配置、镜像哈希与场景；
     PR 只能证明编译与回归，不能替代台架结论。
3. **文档**：
   - `docs/guides/cogging_calibration.md`：更新"仅接受 mode 0/1""速度、位置…不使用"
     "离开 mode 0/1 自动关闭"三处描述为新的允许集合与排除项。
   - CAN 0x26/0x27 行为描述同步；帧格式与参数 ID 不变。

## 验收标准

1. 补偿恰好适用于 {Current, Speed, Position, Impedance}，其余模式（含各 Calib）不适用；
   由原生断言与代码走查共同确认。
2. `iqRef` 所有权、总线协议、参数 ABI、RTT/J-Link 字段不变；调用路径其余行为不变。
3. PR profile 全绿并留证据；涉及文件的格式/接口/样式债务清理完成。
4. 每个新增模式的台架 A/B 证据齐备后才可宣称纹波/误差改善；无证据的模式如实标注未验证。
5. 任一模式出现不稳定或超限，可独立回退该模式的注入点，不影响其他模式。

## 实施顺序

1. 等待并行会话的 `foc_task.c` 改动提交或明确稳定（`git status` 干净）。
2. 适配层谓词 + 核心字段更名 + 原生用例（离线可验证部分）。
3. 三处注入点接线；`foc_task.c`/`foc_run.c` 触碰即做整文件格式与中文注释清理。
4. PR 验证 + 文档更新 + 计划证据。
5. 台架逐模式 A/B（需授权），回填证据。

## 进度

- [ ] 阻塞解除：并行 `foc_task.c` 改动落定
- [ ] 适配层谓词与核心字段更名
- [ ] 三处注入点接线与 legacy 清理
- [ ] 原生用例与 PR 验证
- [ ] 指南与协议文档更新
- [ ] 台架逐模式 A/B 与证据

## 决策记录

- 2026-09-19：扩展集合限定为四个转矩闭环模式；无感/标定/Vq 明确排除。
- 2026-09-19：模式策略只放适配层（两个谓词），纯核心继续与模式枚举解耦。
- 2026-09-19：不新增轴配置判据：表索引基于编码器线性化角，与行程约束无关，
  总量仍受运行限流约束；mode 6 本就只允许无边界轴生成表。
- 2026-09-19：不修改 `Task_Speed_Mode`/`Task_Position_Mode`：分别被摩擦辨识复用
  与位置配置等价性测试参照，注入点放在调度分支与阻抗任务内。

## 证据

（待实施后填写）
