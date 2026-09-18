# 统一运行状态机设计 v1.0

日期：2026-09-19。状态：**设计待评审**（未实施）。

## 背景

当前"状态"由五套隐式机制分别维护，没有任何一处是单一权威：

| 隐式状态 | 载体 | 写点（2026-09-19 统计） |
| --- | --- | --- |
| 运行模式 | `MotorControl.ModeNow`（16 值，含 4 个伪命令） | 约 17 处：`ModeSwitch_Handle` 内部直接赋值、`foc_calibration.c` ×7、`foc_cogging_calibration.c` ×4、`foc_friction_identification.c`、`main.c`、各标定完成/失败路径 |
| 故障 | `MotorControl.ErrorNow` | 30+ 处 `Set_ErrorNow`；清除路径散落（`Clear_Error` 模式、CAN 恢复时自动清 `No_Error`） |
| 功率级 | TIM1 PWM/OCN 开关 | `Stop_PWM_Generate` 被辨识模块直接调用 4 处；`Start_PWM_Generate` 被 cogging 模块直接调用 1 处；`foc_run_state` 2 处 |
| 使能准备 | `position_start_prepared`（`foc_run_state` 静态变量） | 1 处，无对外可见状态 |
| 标定阶段 | `CalibStep`、`SensorlessStartup.state`、cogging/friction 内部状态 | 各模块自持 |

`motor_state.{c,h}` 是状态**存储**层（实例归属与初始化）；`foc_run_state.{c,h}` 是迁移**策略**层，
两者职责不重叠。真正缺失的是把"模式 / 功率级 / 故障反应 / 使能准备"统一为一个显式状态机：
现状下迁移只能靠 `ModeLast` 与 `ModeNow` 差分推断，任何一个 `Task_*` 都能直接改模式或开关功率级，
异常路径（例如 PREPARING 中触发欠压）无法集中验证。

## 目标与验收

1. 引入唯一权威的显式运行状态机（命名 `RunState`），状态与迁移条件集中在一个函数内；
2. 单一写者：`ModeNow`、功率级开关（`Start/Stop_PWM_Generate`、`Clear_RunningData`、零值预载）
   只由状态机写；外部请求改为投递事件；
3. 标定/控制任务不再自行改模式或开关功率级，改为返回统一结果上报边界；
4. 阶段 1 行为等价：ISR 调用顺序与功率级调用序列与现状逐 token/夹具一致；
5. 迁移表可由原生夹具穷举验证（stub 驱动，无硬件）。

## 设计

### 状态集（4 态）

| 状态 | 含义 | 功率级 | 允许的事件 |
| --- | --- | --- | --- |
| `RUN_DISABLED` | 空闲；接受模式请求 | 关闭 | `REQ_MODE`、`REQ_STOP`、`EV_FAULT` |
| `RUN_PREPARING` | 已准入使能：预载零矢量、控制器校验、等待就绪 | 关闭（保持预载占空比） | `EV_READY`、`REQ_STOP`、`EV_FAULT` |
| `RUN_ENABLED` | 执行模式 worker | 开启 | `WORKER_RESULT`、`REQ_STOP`、`EV_FAULT` |
| `RUN_FAULT` | 故障锁存；功率级关闭、故障指示 | 关闭 | `REQ_CLEAR` |

### 事件集

| 事件 | 来源 | 说明 |
| --- | --- | --- |
| `REQ_MODE(mode)` | CAN 命令、内部请求 | 先经准入校验（现 `ModeSwitch_Handle` 拆为纯校验），失败原因为 `MotorParam_Error`/`Over_Voltage` 等 |
| `REQ_STOP` | CAN STOP、CAN 断开、内部请求 | 请求停机并清理运行数据 |
| `EV_FAULT(error)` | 保护模块 `Set_ErrorNow` | 状态机观察 `ErrorNow != No_Error` 即视为故障事件 |
| `EV_READY` | 预载完成 + `MotorOuterLoop_IsReady()` | 仅 `RUN_PREPARING` 使用 |
| `WORKER_RESULT` | 模式 worker 返回值 | `{RUNNING, SWITCH_MODE, STOP, FAULT}` |
| `REQ_CLEAR` | `Clear_Error` 命令、CAN 重连恢复 | 清除锁存；条件不满足时保持 `RUN_FAULT` |

### 迁移表（阶段 1 行为基线）

| 当前 | 事件 | 下一状态 | on-entry 动作 |
| --- | --- | --- | --- |
| `DISABLED` | `REQ_MODE`（Position/Speed/需准备类）| `PREPARING` | 记录待启动模式；保持零矢量预载 |
| `DISABLED` | `REQ_MODE`（其余可运行模式）| `ENABLED` | `Start_PWM_Generate`（沿用当前列表：除 Save/Default/Clear/SetZero/Calib_Anticogging 外） |
| `PREPARING` | `EV_READY` | `ENABLED` | 提交 `ModeNow`；`Start_PWM_Generate` |
| `PREPARING` | `REQ_STOP` | `DISABLED` | 清理准备标志；不写 `ModeNow`（保持停止语义） |
| `PREPARING` | `EV_FAULT` | `FAULT` | `ModeNow = Motor_Disable`（Save/Default 例外）；`Stop_PWM` |
| `ENABLED` | `REQ_STOP` / `WORKER_RESULT(STOP)` | `DISABLED` | `Clear_RunningData` + `Stop_PWM_Generate` |
| `ENABLED` | `WORKER_RESULT(SWITCH_MODE)` | `PREPARING`/`ENABLED` | 按新模式的使能策略选择 |
| `ENABLED` | `EV_FAULT` | `FAULT` | `Clear_RunningData` + `Stop_PWM` + 故障指示 |
| `FAULT` | `REQ_CLEAR`（条件允许）| `DISABLED` | 清 `ErrorNow`；指示复位 |
| 任意 | `EV_FAULT` | `FAULT` | 幂等：确保功率级关闭 |

说明：

- **保留现状的例外**：`Save_Param`/`Default_Param` 期间故障不强制停机（Flash 写入容错），
  在 `FAULT` 的 on-entry 中以显式分支保留；`Set_ZeroPosition`/`Calib_Anticogging` 不进入自动使能
  列表，同样在准入表中显式列出。
- **伪命令暂保留枚举**：`Save_Param`/`Default_Param`/`Clear_Error`/`Set_ZeroPosition` 仍占用
  `ModeNow` 值（CAN 兼容），内部先按"命令状态"处理，迁出枚举列为后续独立评估项。
- **`ModeLast`/`ErrorLast` 保留**：作为前台/诊断使用的影子副本，由状态机在提交点统一更新；
  差分不再作为迁移依据。
- **`Set_ErrorNow` 保持现状**：保护模块实时置位审计不变；清除路径收敛到 `REQ_CLEAR`。

### 单一写者规则

| 资源 | 唯一写者 | 其他模块的正确做法 |
| --- | --- | --- |
| `MotorControl.ModeNow` | 状态机提交点 | 投递 `REQ_MODE`/`REQ_STOP`/`WORKER_RESULT` |
| `Start/Stop_PWM_Generate`、`Clear_RunningData`、零矢量预载 | 状态机 on-entry | 不再直接调用（辨识模块现有 5 处移除） |
| `ErrorNow` | 置位：保护模块；清除：状态机 | 置位走 `Set_ErrorNow`，清除走 `REQ_CLEAR` |

### worker 结果协议（阶段 2）

```c
typedef enum
{
    FOC_WORK_RUNNING = 0,   /* 继续执行当前模式 */
    FOC_WORK_SWITCH_MODE,   /* 请求切换到 next_mode（标定完成/阶段推进） */
    FOC_WORK_STOP,          /* 请求停机 */
    FOC_WORK_FAULT          /* 上报故障，由状态机锁存并停机 */
} FocWorkResult_TypeDef;
```

- `Task_*`（`foc_run.c`）与辨识模块（`foc_calibration.c`、`foc_cogging_calibration.c`、
  `foc_friction_identification.c`）返回该结果，替换现有 12 处 `Set_ModeNow` 与 5 处直接 PWM 开关。
- 各标定内部阶段机（`CalibStep`、`SensorlessStartup.state` 等）保留，只在开始/完成/失败边界上报。

### 与现有文件的映射

| 文件 | 变化 |
| --- | --- |
| `foc_run_state.{c,h}` | 演进为状态机：新增 `RunState_TypeDef`/`RunEvent_TypeDef` 与迁移函数；现存三个函数降为内部动作或删除 |
| `motor_state.{c,h}` | 不变（纯存储）；可选后续改名 `motor_context` 以消除命名歧义 |
| `foc_mode_dispatch.{c,h}` | `ENABLED` 下的模式执行 + 结果收集；不再持有迁移判断 |
| `foc_errhandle.c` | `ModeSwitch_Handle` 拆为纯准入校验 + 模式准备动作；清除/停机路径改为事件 |
| `foc_task.c` | ISR 顺序不变，状态机调用点替换现 `FocRunState_*` 三个调用 |

## 分阶段实施

| 阶段 | 内容 | 验证 |
| --- | --- | --- |
| 1 | 显式化：引入 `RunState` 与迁移表，行为不变；`ModeLast` 差分降级为兼容断言 | token/调用序列等价 + 新增原生迁移夹具（穷举迁移表、功率级调用计数） |
| 2 | 收拢单一写者：worker 结果协议；移除辨识层 `Set_ModeNow` 与直接 PWM 开关 | 各标定完成/失败路径夹具 + 双变体编译 + PR |
| 3 | 命令与模式分离（伪命令迁出枚举）与 CAN 映射兼容处理 | 协议文档 + 上位机约定 + 兼容性说明；本阶段单独评审后启动 |

每阶段：`check_project_layout` 0 错误、普通/HIL 双变体交叉编译 0 错误、PR 离线套件通过。

## 风险与缓解

- **CAN 模式值 ABI**：`ModeNow` 数值出现在命令与状态帧中；阶段 1/2 不改数值，阶段 3 需版本化。
- **实时性**：迁移表为 O(1) 分支，无动态分配/回调；on-entry 动作与现状同为有界 HAL 调用。
- **Flash 保存流程**：`Save_Param` 特判在迁移表中显式保留，避免被故障分支打断。
- **并行工作**：cogging 补偿扩展到全模式的工作依赖模式接线点（`foc_mode_dispatch.c`），阶段 1 不改变该接口。

## 验证证据

待实施后补充：迁移夹具输出、token 等价报告、双变体编译日志、PR 运行目录。
