# 对接 deepseek-e：供料包（任务 4/5/6/7）v1.0

日期：2026-09-19。状态：任务 4（复验+回灌包）、任务 6、任务 7 已成文；
任务 5（故障三口径）待 E 侧与 P0-A 探索结果合并。
来源计划：[对接 deepseek-e 收尾与供料清单](2026-09-19-e-framework-interface-prep.md)。
E 侧核对点：`codex/deepseek-e-framework` @ `b5504186`。本节点基线：见 §4 执行记录。

## 1. 任务 4：E 核心正本对齐与冻结

### 1.1 复验结果

- 方法：剥离注释（`/* */`、`//`）与行接续符（`\`+换行），token 化后**忽略花括号与空白**；
  以 token 序列逐位比较为唯一判定口径。
- 结果（2026-09-19）：
  - `app_lifecycle.c`：**2571 / 2571 token 完全一致**；
  - `app_lifecycle.h`：**383 / 383 token 完全一致**。
- 差异构成（E → 本节点）：中文契约注释 + clang-format 风格（花括号化、空格）——
  **零语义差异**。行级统计：`.c` -201/+469；`.h` -70/+133。

### 1.2 回灌包（**待审，未执行**）

- 文件：
  - `docs/plans/active/2026-09-19-e-lifecycle-backfill.c.patch`
  - `docs/plans/active/2026-09-19-e-lifecycle-backfill.h.patch`
- 属性：头部路径为 E 仓相对路径（`yg_esc_app/application/lifecycle/…`）、CRLF 行尾、
  已在 E 仓执行 `git apply --check` **通过**（2026-09-19）。
- 应用方式（在 E 仓根执行）：`git apply <patch>`；建议先 `--check` + 人工审阅。
- 构成：中文契约注释回灌 **+** 代码格式统一为本节点 clang-format 风格。
  若只回灌注释、保留 E 侧紧凑风格，需另行生成"注释-only"变体（按此包评审结论决定）。

### 1.3 单向同步规则（提案，待审）

1. **代码正本 = E 侧**；本节点 `firmware/services/lifecycle/app_lifecycle.{c,h}`
   不再单独演化语义。
2. **中文契约注释 = 本节点资产**；回灌 E 侧一次后冻结，此后注释变更亦以 E 侧为源回流。
3. 核心任何变更流程：E 侧先行提交，本节点随后同步并重跑本节的 token 复验。
4. 复验口径固定为本节 1.1 的 token 化规则；不接受"仅忽略注释"的更宽口径。

## 2. 任务 6：适配器语义对照表（差分锁定）

来源：`firmware/app/foc_run_state.c`（Diff-locked semantics，2026-09-19）。
差分基线：16 种结果载荷 × 11 模式 × 11 起始模式 × 2 故障 × 2 轴有效 × 2 就绪 × 3 拍
= **46,464 组 tick 全部与旧实现一致**（`tests/unit/native/test_run_state.py`）。

| # | 语义 | 本节点实现 | 差分覆盖 | E 侧对接点（待其 motor_state.c 比对） |
| --- | --- | --- | --- | --- |
| 1 | 首拍延迟使能：位置/速度先在功率关断下准备，外环就绪后下一拍启相 | `RunState_ModeNeedsPreparation`(L62)、`Tick` 步骤 4 ENABLE 挂起、`RunState_ConfirmStart`(L178) | 就绪位两态 × 位置/速度 | `STARTING` 态语义：功率未开、等待 `START_DONE` 确认 |
| 2 | 故障事件不动功率级；停机清理统一由停止路径**执行一次** | `Tick` 步骤 3 故障事件 `apply=false`；清理在步骤 4 停机路径 | 全部故障载荷 | 故障路径不得二次关断（E `fault()` 只置位/取消） |
| 3 | 停机清理授权 = `ModeLast != Disable`（含"模块已自行停相"场景） | `RunState_RequestStop`(L171)/`ApplyActions`(L115) 的 `apply_power_actions` | 起始模式 × 停机载荷 | 旧语义兼容点：清理按模式归属，不按功率实际状态 |
| 4 | `Save/Default` 期间故障容错：`ModeNow` 不被故障强制回 `Disable` | `Tick` 步骤 2 | 故障 × Save/Default 目标 | E 命令准入与故障并存时的模式投影规则 |
| 5 | 命令类模式不自动使能：Save/Default/Clear/Zero/Anticogging | `RunState_ModeIsAutoStartable`(L55) | 11 模式全覆盖 | 命令与模式分离后由 commands 层承接 |
| 6 | 启动确认：位置/速度需 `MotorOuterLoop_IsReady()`；其余同拍确认 | `guards.startup_confirmed`(L107) | 就绪位两态 | 启动旅程的 ready 来源 |
| 7 | 兼容提交：`STARTING` 期间 `ModeLast` 不提交（等价旧 `defer`），其余每拍提交 | `Tick` 步骤 5 | 全部组合 | 投影提交点的对齐 |
| 8 | 会话事件拍推迟开启下一会话一拍（完成记录可观测） | `ServiceOperations` 返回 `sent`；`Tick` 条件开启 | 会话场景夹具 | 维护子状态机的完成→下会话时序 |
| 9 | 陈旧完成不污染新会话 | `ServiceOperations` 顶部清理；核心 epoch 校验 | 会话场景夹具 | 完成通知的 epoch 校验规则 |
| 10 | 取消确认门槛：`CANCEL_DONE` 需 quiet（相已关+控制/操作空闲+维护释放）+ 健康（无活动故障）+ 操作匹配 | 核心 `app_lifecycle.c` + `RunState_BuildGuards`(L92) | 取消/释放场景 | E 维护子状态机的"释放证明"要件 |
| 11 | 清除准入门槛：源恢复+样本新鲜+释放；通信类（CAN）自动恢复；未知枚举拒绝 | `RunState_FaultSourceRecovered`(L221)、`Tick` 步骤 3 | 恢复矩阵 6 组用例 | 任务 5 对照表（E FaultLatch × P0-A） |
| 12 | 维护目标从运行态退出：先 STOP 再开会话 | `Tick` 步骤 4 + `BeginOperationIfRequested`(L366) | 运行中切命令类目标 | 命令准入的停止前置 |

## 3. 任务 7：Operation 会话语义规格

对标：E 侧"维护显式子状态机"（缺口清单 §2 第 1 条）。实现参考：
`foc_run_state.c` `RunState_ServiceOperations`(L300)/`RunState_BeginOperationIfRequested`(L366)。

### 3.1 模型

- 会话五要素：**begin / step / cancel / release / result**；
- 结果：`NO_RESULT → COMPLETED / FAILED / CANCEL_REQUESTED → CANCELLED`；
- 效果：`COMMITTED / UNCOMMITTED / UNKNOWN`（SAVE 成功必须 `COMMITTED`，
  `UNKNOWN/UNCOMMITTED` 不得宣称成功）；
- 释放证明 = quiet 四元组：`phases_disabled && control_idle && operation_idle && maintenance_released`；
- 会话期间每拍守卫：`operation_permitted && fault_sources_clear &&`
  （标定类操作要求 `samples_fresh`；其余要求 `phases_disabled`）。

### 3.2 逐操作规格

| 操作 | 触发 | 开启 | 执行者/拍点 | 完成条件与效果 | 取消路径 | 失败 |
| --- | --- | --- | --- | --- | --- | --- |
| SAVE | `ModeNow=Save_Param` | READY→MAINTENANCE(SAVE) | 前台 `main.c`：关中断 `flash_write_param()` | 前台 `FocRunState_SaveFinished(true)` → 下一快拍 DONE + `COMMITTED` | 故障/停止打断 → `CANCEL_REQUESTED` → `CANCEL_DONE(UNKNOWN)`；**取消不回滚 Flash** | `SaveFinished(false)` → FAILED + `UNKNOWN` + `MotorParam_Error` 锁存 |
| DEFAULTS | `ModeNow=Default_Param` | READY→MAINTENANCE(DEFAULTS) | dispatch 每拍 `Param_Return_Default()` | 进入会话后下一拍 DONE + `UNCOMMITTED`；**不隐式清故障**；同值持续不重复开启 | 同上（`CANCEL_DONE`） | 无（幂等） |
| ZERO | `ModeNow=Set_ZeroPosition` | READY→MAINTENANCE(ZERO) | dispatch `Task_SetMechanicalZero` | worker 切往 `Save_Param` → DONE(ZERO, `UNCOMMITTED`)，下一拍自动开 SAVE 会话 | 同上 | `Encoder_Error` 故障 |
| 齿槽标定 | `ModeNow=Calib_Anticogging` | READY→MAINTENANCE(CALIBRATION) | dispatch `FocCogging_Task`（**模块自管功率**） | 切往 `Save_Param` → DONE(`UNCOMMITTED`)，随后 SAVE 提交 | 退出/故障 → `FocCogging_Abort` 路径 → `CANCEL_DONE` | `CoggingCalibration_Error` |
| 相电阻标定 | `ModeNow=Calib_PhaseResistance` | READY→MAINTENANCE(CALIBRATION)；**会话开启时按旧入口条件启相**（`ModeLast==Disable && axis_profile_valid`） | dispatch `PhaseResistanceMode_Run` | worker 返回 STOP → DONE(`UNCOMMITTED`)；停止路径关相 | 同上 | 4 类状态映射（大电阻/参数/欠压/过压） |

### 3.3 关键语义条目（E 侧定义输入）

1. **完成与请求单拍分离**：任何完成/失败/取消确认上报的拍，不开启下一会话（保证结果可观测）。
2. **陈旧完成丢弃**：非当前操作的完成载荷一律清除，不污染新会话。
3. **取消不回滚外部效果**：取消只声明效果未知（`UNKNOWN`），不承诺 Flash/RAM 未提交。
4. **取消必须确认**：被停止/故障打断的会话在资源静默且健康后上报 `CANCEL_DONE`，
   否则 `CLEAR` 与 `READY` 被永久阻塞（E 核心既有语义）。
5. **SAVE 链**：ZERO/齿槽的 `UNCOMMITTED` 完成随后由 SAVE 会话以 `COMMITTED` 收口。
6. **过渡现状**：`foc_calibration` 五任务已整体删除（待重建）；friction 会话待接入。

## 4. 任务 5：故障三口径对照表（E FaultLatch × 本节点矩阵 × P0-A）

来源：本节点 `foc_run_state.c::RunState_FaultSourceRecovered`（矩阵 v1）；
E 侧 `yg_esc_app/services/safety_management/motor_state.c`
（`Set_ErrorNow` L214、`Lifecycle_RecoveryHealthy` L451、`Lifecycle_Guards` L469、
`FAULT_CLEARED` L511、`Clear_Error → APP_EVENT_CLEAR` L680、`MotorState_CommunicationRecovered` L539）；
P0-A 分支 `codex/fault-protection-p0a`（`fault_core.{c,h}`、`protection_types.h`、
`protection_limits.h`、`protection_runtime.{c,h}`、`legacy_fault_projection.c`）。

### 4.1 模型级对照

| 维度 | 本节点矩阵 v1 | E `motor_state.c` | P0-A `fault_core` |
| --- | --- | --- | --- |
| 锁存 | 核心 `first_fault`+`latched_faults`；`ErrorNow` 单值投影 | `ErrorNow` 首因优先（`No_Error`/`CAN_DisConnect` 可被覆盖）+ `pending_faults` 位图（`1<<error`） | `active/latched/warnings/unknown_monitors` 位图 + 逐项目 `FaultStateRecord`（first_seen/last_seen/occurrence） |
| 恢复判据 | **逐故障矩阵**（按故障枚举分支） | **全局合取** `Lifecycle_RecoveryHealthy()`（所有源同时健康）| 每项 `recovery_class`（R0 自动 / R1 明确清除 / R2 服务验证 / R3 维修记录）+ 五态证据 |
| 证据模型 | 无（直接读全局量） | 无（直接读全局量） | 五态：`TRIP/HEALTHY/WARNING/UNKNOWN/NOT_APPLICABLE`；仅 TRIP 产生新锁存、仅 HEALTHY 撤销活动 |
| 清除入口 | `ModeNow=Clear_Error` → 适配器聚合准入 → 核心 `CLEAR` | `Clear_Error`→`APP_EVENT_CLEAR`（L680）→ 核心 CLEAR → `FAULT_CLEARED` 投影 `ErrorNow=No_Error` | `Protection_TryClear(mask, request_id)`：**排队成功≠已清除**，仍成立的 trip/未知证据/需服务的恢复等级保持锁存 |
| 首因 | 核心取最低位；适配器投影单值 | 首因优先 + 位图累积 | `first_fault_id`（同刻取较小 ID）+ `primary_fault_id`（按 severity 排序，仅上报） |
| 能力缺失 | 未建模 | 未建模 | `Protection_Capabilities()` 平台能力交集；`mandatory_faults` 缺能力→拒绝启动（`unavailable_monitors`） |
| 关断动作 | 适配器执行（`DISABLE_POWER` 授权门控） | `Set_ErrorNow` 内立即关相 + 命令栅栏 + 目标撤销 | 动作位并集（WARN/DERATE/CONTROLLED_STOP/DISABLE_DRIVE/BLOCK_START/MAINTENANCE_LOCK）；`drive/torque/start_permitted` 互锁 |

### 4.2 逐故障恢复条件对照（阈值/驻留）

| 故障（P0-A 目录 ID） | 本节点矩阵 v1 | E `Lifecycle_RecoveryHealthy` | P0-A `protection_limits.h` |
| --- | --- | --- | --- |
| BUS_UNDER/OVER_VOLTAGE（1/2） | `Vbus_filt ∈ [25.6, 33.8] V`（enable 窗口） | `Vbus_filt ∈ [ENABLE_MIN, ENABLE_MAX]` 且 `Vbus < HARD_OV`（同窗口） | trip 24 V/100 ms、34 V/2 ms；**recover 未设置（UNSET）** |
| BUS_HARD_OVER_VOLTAGE（3） | 同上（共用窗口判据） | `Vbus < HARD_OV` | 34.5 V/3 拍；recover UNSET |
| PHASE_OVER_CURRENT（4） | `|I| < trip−10%` 持续 **2000 拍（100 ms）** | `|I| < trip`（瞬时） | confirm 5 拍；severe 60 A；**recover UNSET** |
| CURRENT_SENSE_INVALID / 零偏（5） | 过渡：允许（操作员确认） | 零偏 ∈ [1948, 2148] 计数 | 同窗口（1948–2148） |
| MCU_OVER_TEMPERATURE（6） | `temp < 80 °C`（90→80 滞回）+ 传感器有效 | `temp < 90 °C`（**无滞回**）+ 有效 | trip 90 / warn 75 / derate 85；**recover UNSET** |
| MCU_TEMPERATURE_SENSOR（7） | `McuTemperature.valid` | 采样超龄 100 ms（`Temperature_IsRecovered` 内的 valid） | 最大年龄 100 ms |
| ENCODER_FEEDBACK（8） | `bad_frame_streak == 0` | `Encoder_IsOnline && read_status == OK` | 离线判据 `max_bad_streak = 3`；年龄 UNSET |
| ENCODER_NOT_CALIBRATED（9） | 过渡：允许 | 未在全局门内检查标定标志 | 目录项 9（策略表定级） |
| SENSORLESS/POLE_PAIRS/PHASE_R/PHASE_L/MOTOR_PARAM（10–14） | 过渡：允许（参数/操作类） | `axis_profile_valid` + 零偏窗口（全局门） | 目录项 10–14（策略表 `actions/recovery_class`） |
| CONTROL_OVERRUN（15） | 过渡：允许（D2 待做 1 s 静默窗口） | —（未单列） | 目录项 15 |
| COGGING/FRICTION（16/17） | 过渡：允许（会话释放由核心把关） | `CANCEL_OPERATION` 时调用模块 `Cancel/Abort` | 目录项 16/17 |
| CAN_DISCONNECT（18） | `can_hb_set>0 && can_hb_count<can_hb_set`（**收到帧即自动恢复**） | `!(latched & (1<<CAN)) \|\| link_recovered`（`CommunicationRecovered` 置位） | 目录项 18；核心 `LINK_RECOVERED` **永不清锁存** |

### 4.3 冲突与待裁决清单（供 E DS-03 F2/F3 与框架裁决）

E 缺口清单（`E_DESIGN_GAP_INVENTORY.md` §2/§3）自认：**F2 连续正常窗口/恢复阈值未接线**、
**F3 首因顺序未接线**。本表逐项映射：

1. **恢复模型归属**：逐故障矩阵（本节点）× 全局合取（E）× 目录+恢复等级（P0-A）。
   建议：以 P0-A 目录/等级为骨架，本节点逐故障条件作为 R0/R1 判定实现；
   E 的全局合取降级为"启动前健康快照"，不再充当清除门槛。
2. **温度滞回**：本节点 90→80 °C；E 无滞回（<90）；P0-A recover UNSET。
   待裁决：滞回值与驻留（建议 80 °C；如引入驻留则与 M CU 热时间常数对齐）。
3. **过流恢复**：本节点 trip−10% + 100 ms；E 瞬时；P0-A 仅确认 5 拍、recover UNSET。
   待裁决：恢复裕度与驻留常数。
4. **CAN 自动恢复通道**：本节点与 E 均自动（链路证据）；P0-A 核心明确 LINK_RECOVERED
   不清锁存。待裁决：CAN 故障的 `recovery_class`（R0 自动 vs R1 明确清除）与自动恢复路径。
5. **参数/操作类故障无在线判据**：本节点过渡允许；E 依赖 `axis_profile_valid`+零偏窗口；
   P0-A 有 R2（服务验证）/R3（维修记录，**未实现**）。待裁决：等级归属与 R3 来源。
6. **首因顺序（F3）**：本节点核心取最低位；E 用位图首因优先；P0-A 同刻取较小 ID +
   primary 按 severity。待裁决：同刻优先级规则。
7. **清除入口语义**：本节点 `Clear_Error` 模式 → 适配器聚合；E 同模式 → 核心 CLEAR；
   P0-A `TryClear(mask, request_id)`，排队≠清除。待裁决：对外统一语义与兼容码投影口径。
8. **能力缺失**：P0-A `mandatory_faults` 缺能力→拒绝启动（本节点/E 均无）→ 迁移项。

## 5. 执行记录

- 2026-09-19：任务 4 复验完成（token 2571/2571、383/383；回灌包 `git apply --check` 通过）；
  任务 6/7 成文；本文件提交：`6941cada（本供料包提交）`。
- 2026-09-19（同日追加）：任务 5 完成——E 侧与 P0-A 由两次后台探索代理超时后改为
  定向读取（E：`motor_state.c` 恢复/守卫/清错入口；P0-A：`fault_core`/`protection_types`/
  `protection_limits`/`protection_runtime`）。三口径表与 8 项裁决清单见 §4；
  E 缺口清单确认 F2（连续正常窗口/恢复阈值）与 F3（首因顺序）为其未接线项，与 §4.3 对齐。
  本文件提交：`6941cada（本供料包提交）`。
