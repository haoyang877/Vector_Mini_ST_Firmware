# E 生命周期状态机：差距分析与迁移方案 v1.0

日期：2026-09-19。状态：**阶段 A/B 已完成；C/D 设计已就绪（见 §5/§6，恢复矩阵阈值待确认），实施待启动**。
来源：`D:/Work/Code/yg_esc_deepseek`（E 框架工作区，2026-09-18）：
`project_docs/architecture/e_app_lifecycle_design.md`、
`project_docs/handover/deepseek_e/E_PROTOTYPE_STATE_FLOW.md`、
`yg_esc_app/application/lifecycle/app_lifecycle.{c,h}`、
`project_docs/acceptance/app_lifecycle_core.md`（135 格转移表 / 2292 断言）。

## 1. 能力对照

| 维度 | E 框架（yg_esc，目标） | 本仓库现状（Vector_Mini_ST，阶段 1/2a/2b） | 差距 |
| --- | --- | --- | --- |
| 主状态 | 9 态：BOOT/SELF_TEST/READY/STARTING/RUNNING/MAINTENANCE/STOPPING/FAULT/FATAL | 4 态：DISABLED/PREPARING/ENABLED/FAULT（`foc_run_state.c`） | 缺 SELF_TEST/MAINTENANCE/STOPPING/FATAL；无 BOOT 独立态 |
| 独立状态域 | 生命周期 / 控制模式 / 维护操作（8 子态）/ 故障锁存，四域分离 | 模式（`ModeNow` 20 值含 4 个伪命令）/ 故障锁存（`ErrorNow`）| 维护操作未独立；伪命令占用模式空间 |
| 事件优先级 | `FATAL > FAULT > STOP > CANCEL > 完成 > 新请求`（完成+请求同拍报 `INTERRUPTED`） | 隐含：outcome → 故障 → stop/start 分支 | 无 CANCEL/完成/请求优先级的显式仲裁 |
| 代际与撤销 | `request_epoch`/`completion_epoch`；STOP/FAULT 使旧结果失效；`CANCEL_DONE` 需新 epoch+释放证明 | 无 epoch；停机靠 `ModeLast/ModeNow` 差分 | 缺撤销语义；旧队列目标无失效机制 |
| 健康 guards | `samples_fresh`/`fault_sources_clear`/`startup_confirmed`/`start_permitted` 等由适配器提供 | 内联判断（`axis_profile_valid`、`MotorOuterLoop_IsReady`、总线电压准入在 `ModeSwitch_Handle`） | guard 未集中；恢复条件散落 |
| 动作语义 | 核心只返回动作位（`DISABLE_POWER`/`REVOKE_REQUESTS`…），不是硬件证据；启相短临界区复核紧急禁止+epoch | 状态机直接调 `Start/Stop_PWM`、`Clear_RunningData`；`power_off` 为协议标志 | 缺"动作请求 vs 确认回读"分离 |
| 维护/保存 | Operation 域；SAVE 必须 `COMMITTED`；取消不回滚 Flash；效果分 `UNKNOWN/UNCOMMITTED/COMMITTED` | `Save_Param` 伪命令 + 前台 Flash 流程；无 operation 结果语义 | 保存成功语义未编程化 |
| 故障锁存 | FaultLatch 域：首故障+活动故障、恢复资格、通信恢复不清锁存 | `ErrorNow` 单值（首故障兼兼容投影）；CAN 恢复/默认参数有清错副作用 | 恢复条件未统一；存在隐式清错路径 |
| 验证 | 135 格转移表（9 态 × 15 事件）+ 2292 断言，`-UNDEBUG -Wall -Wextra -Werror` | 差分对拍 38,400 组 tick（旧实现 vs 新迁移）+ 逐路径夹具 | 状态转移矩阵未穷举；事件类型少 |
| 工程布局 | `yg_esc_app/` + `hardware_platform/...`，五个 Keil 工程；内部状态不占用 CAN 编号 | `firmware/` 单工程（普通/HIL 两目标） | 路径与构建挂载不同；CAN 模式编号是协议 ABI |

结论：本仓库现有 4 态是 E 目标的**简化子集**——DIAGNOSED 缺口不是"要不要状态机"，而是
**状态域划分、事件仲裁、撤销语义、操作会话、恢复条件**五项能力；E 的纯核心可直接复用。

## 2. 可直接采用 / 需适配 / 暂不适用

**可直接采用（整体移植，零依赖）**

- `app_lifecycle.{c,h}` 纯核心：无 HAL、无堆、无可写全局、无协议编号；C99。
- 验收方法：`test_app_lifecycle.py` + `app_lifecycle_test.c`（显式状态预期表，
  9 态 × 15 事件 = 135 格；`-UNDEBUG` 断言生效）。
- 事件/动作/拒绝原因/guards 的接口设计（`AppLifecycleEvent/Guards/Result`）。

**需适配**

- **模式投影**：`ModeNow`（0..19，含 4 伪命令）↔ `AppControlMode` + `AppOperation`
  的双向映射；**线上编号不变**（E 文档同样承诺不改 Mode/Error 编号）。
- **适配层**：E 用 `services/safety_management/motor_state.c`（32 KB，唯一 owner +
  撤销接缝）与 `motor_commands.c`；本仓库对应物是 `foc_run_state.c`（4 态）+
  `foc_mode_dispatch.c` + `foc_errhandle.c` + `interface_can.c`，需要按同一角色收敛。
- **失败闭合**：`Set_ErrorNow` 的 30+ 置位点保留为事件源；**清除权限**收归生命周期
  （对应阶段 3 的"命令与模式分离"与 CLEAR 恢复条件）。
- **Quick 差异**：本仓库无 IWDG/break 监督（E 文档亦说明硬件 fatal 由底层入口承担）；
  FATAL 初版仅覆盖 API 误用/epoch 耗尽，不宣称硬件急停。

**暂不适用**

- A/B 存储后端与掉电原子性声明；角色策略（normal/maintenance/HIL）在 E 中以
  `role_policy.h` 实现，本仓库是 `axis_profile` + Factory/HIL 目标，需另立映射表。
- D Loader 相关的交接与内存布局（两线各自登记中）。

## 3. 迁移方案（阶段 A–D）

| 阶段 | 内容 | 验收 | 依赖 |
| --- | --- | --- | --- |
| A ✅ | **移植纯核心**：`firmware/services/lifecycle/app_lifecycle.{c,h}`（逐 token 保真）；按裁决**暂不移植 135 格测试**、暂不登记 Keil（不接生产路径） | 与上游逐 token 一致（.c 2570 / .h 380 tokens）；门禁全过；编译冒烟 0 错误 | 无 |
| B ✅ | **适配器骨架**：`foc_run_state.c` 改为 AppLifecycle 适配器（事件合成 + guards 供给 + 动作执行 + 兼容投影），保持 `FocRunState_Tick(outcome)` 接口与 CAN 编号；核心 `app_lifecycle.c` 已登记两个 Keil 工程并新增 include 路径 | 差分夹具：旧三函数 vs 适配器 **38,400 组 tick 全部一致**；双变体编译 0 错误 | 无（foc_run.c 拆解已落定） |
| C | **Operation 会话（全量，含 6 个标定会话）**：Save/Default/Zero/标定完成路径挂入 MAINTENANCE/Operation（Begin/Step/Cancel/IsReleased/Result）；继续 2b 剩余路径但目标形态从"结果协议"升级为 operation 协议。模块清单与缺口见 §5 | 逐路径夹具（含取消/失败/释放确认）；SAVE 成功需 `COMMITTED` | 阶段 B；设计已就绪（§5） |
| D | **FaultLatch + CLEAR 恢复（直接定义恢复矩阵）**：活动故障/首故障分离；CLEAR 要求源恢复+样本新鲜+资源释放；迁移 CAN 重连/默认参数的隐式清错副作用。逐故障条件见 §6 | 故障锁存与恢复条件夹具；通信恢复不清错；幂等 CLEAR | 阶段 C；设计已就绪（§6），阈值待确认 |

每阶段：`check_project_layout` 0 错误、普通/HIL 双变体编译 0 错误、PR 全绿、
新增测试登记 hygiene 清单。

## 4. 风险与约束

- **双写者期**：B 阶段起新旧状态机并存，必须按 E 文档要求"每个运行组合只有一个有效
  写入者"；切换点以夹具锁定，不允许长期双真值。
- **CAN/参数 ABI**：`ModeNow`/`ErrorNow` 编号是线上协议的一部分；投影层必须双向且
  可回退；schema11/参数布局不受本迁移影响。
- **并行冲突**：`foc_run.c` 拆解（用户实施中）与 B/C 阶段共用该文件——顺序上先拆解后接入，
  或由拆解方同步适配接口。
- **不宣称硬件能力**：核心动作是请求；启相短临界区复核、实际关相回读、IWDG 等仍属
  硬件保护能力评审，不在本迁移中默认启用。

## 评审裁决（2026-09-19）

| 问题 | 裁决 | 影响 |
| --- | --- | --- |
| 核心落点 | `firmware/services/lifecycle/` | 已按此落地；纯核心零依赖，services 层合法 |
| 适配器 | 仅取核心 + 本仓库自研适配 | 不引入 `yg_esc` 的 `services/safety_management` 布局依赖 |
| 本仓库与 yg_esc 关系 | **最终合路 YG-ESC** | 核心保持与上游 API/语义逐 token 一致；投影层与角色策略的对齐在阶段 B 按 YG-ESC 约定设计 |
| 135 格测试 | **先不做** | 移植保真度改用逐 token 比对证明；测试移植登记为后续项，接生产路径前补齐 |

第二轮裁决（C/D 启动，2026-09-19）：

| 问题 | 裁决 | 影响 |
| --- | --- | --- |
| 阶段 C 范围 | **全量：含 6 个标定会话**（Begin/Step/Cancel/IsReleased/Result） | 各标定模块需补公开取消/释放/结果接口（见 §5） |
| 阶段 D 路径 | **直接定义恢复矩阵**（逐故障恢复条件，非先兼容映射） | 需先确认 §6 阈值项，再实施 |
| 提交节奏 | **先提交再继续**：代码批次已提交 `6c3dcf9` | C/D 基于该 SHA 推进 |
| 135 格测试 | **继续暂缓** | 回归依赖差分夹具与逐路径夹具 |

## 5. 阶段 C 设计：Operation 会话映射

目标协议（E 语义）：核心进入 `MAINTENANCE` 时由适配器开启操作会话，提供
`Begin/Step/Cancel/IsReleased/Result` 五项；`Result` 报告
`COMMITTED/FAILED/CANCELED(原因)`；取消不回滚 Flash；SAVE 成功必须 `COMMITTED`。

| 操作（核心） | 现有入口 | 现有能力 | 阶段 C 缺口 |
| --- | --- | --- | --- |
| SAVE | `ModeNow=Save_Param` 伪命令；前台 `main.c` 主循环处理（关中断 `flash_write_param()`） | 成功布尔；失败置 `MotorParam_Error`；`FocCogging_SaveResult(saved)` 既有消费者 | `Result(COMMITTED/FAILED)` 上报核心；前台步骤纳入 operation `Step`；`IsReleased` 关联写入完成 |
| DEFAULTS | `foc_mode_dispatch.c` 的 `Default_Param` 分支 | 一次性；**副作用 `Set_ErrorNow(No_Error)`** | 移除无条件清错（受 §6 约束）；`Result` 上报 |
| ZERO | `foc_mode_dispatch.c` 的 `Set_ZeroPosition` 分支（编码器机械零位写入） | 一次性 | `Result`/`IsReleased` 标准化 |
| CALIB_CURRENT_OFFSET | `Task_Calib_CurrentOffset`（已返回 `MotorWorkOutcome_TypeDef`） | 结果协议（2b 已迁移） | 公开 `Cancel`、`IsReleased`、`Result` 映射 |
| CALIB_R_L_FLUX | `Task_Calib_R_L_Flux` | 完成以切 `Save_Param` 表达 | 同上 |
| CALIB_ENCODER_OFFSET | `Task_Calib_EncoderOffset` | 同上 | 同上 |
| CALIB_ELE_ANGEL_OFFSET | `Task_Calib_EleAngelOffset` | 同上 | 同上 |
| CALIB_ENCODER_OBSERVER | `Task_Calib_EncoderObserver`（返回 outcome；取消为模块内 `static` 实现） | 结果有、取消未公开 | 取消公开化、`IsReleased`、`Result` 枚举 |
| CALIB_PHASE_RESISTANCE | `PhaseResistanceMode_Run/Cancel` + 状态返回 | 取消已有 | `IsReleased`/`Result` 包装 |
| CALIB_FRICTION | `FocFrictionIdentification_Start/Abort/GetState/GetReason` | 取消 + 原因已有 | `IsReleased`（由 `GetState` 导出）、`Result` 由 `GetReason` 映射 |
| CALIB_ANTICOGGING | `FocCogging_CanStart/Abort/GetState/SaveResult` | 取消 + 结果（经 `SaveResult`）已有 | `Result(COMMITTED)` 承接、`IsReleased` |

适配器侧（`foc_run_state.c`）：事件合成新增 `OPERATION_BEGIN/STEP/CANCEL/RELEASE`；
guards `operation_idle`/`maintenance_released` 由占位改为真实信号；经典模式编号仍经
`ModeNow` 投影（线上编号不变）。

范围调整（2026-09-19，用户裁决）：`foc_calibration.{c,h}` 将拆解并删除，其 5 个标定任务
（R_L_Flux/EncoderOffset/EncoderObserver/EleAngelOffset/CurrentOffset）**不在原地补会话接口**，
标定会话改为在拆解后的新模块结构上接入。当前实施范围 = SAVE/DEFAULTS/ZERO 会话 +
适配器 OPERATION 事件；cogging/friction/相电阻包装随后按同构接入。

实施进度（2026-09-19）：SAVE/DEFAULTS/ZERO 会话已接入 `foc_run_state.c`——
`FocRunState_SaveFinished()` 上报前台保存结果（COMMITTED/FAILED，失败按操作失败锁存）；
ZERO 以"切往 Save_Param"派生完成（UNCOMMITTED），随后一拍开启 SAVE 会话；
DEFAULTS 在进入会话后的下一拍派生完成（UNCOMMITTED）；被故障/停止打断的会话经
`CANCEL_DONE` 确认释放（要求在 ErrorNow 清除后，符合健康 guard 语义）。
夹具新增会话用例 5 组（SAVE 成功/失败、ZERO→SAVE 链、DEFAULTS、取消释放），
差分等价 38,400 组 tick 保持通过。

## 6. 阶段 D 设计：故障恢复矩阵（阈值已确认，实施中）

阈值裁决（2026-09-19，第二轮）：全部按建议默认执行——温度 90→80 °C 滞回；过流
trip−10% 持续 100 ms；过/欠压用 enable 窗口 25.6–33.8 V；控制超时静默 1 s；
编码器 streak 清零即恢复；CAN 断连收到帧即恢复。

实施切分：

- **D1（本批）**：清除权限收口——`Clear_Error` 直写路径（`foc_errhandle.c`）与
  `Default_Param` 清错副作用（`foc_mode_dispatch.c`）移除；清除统一由适配器评估
  （源恢复 + 样本新鲜 + 资源释放）后置 `ErrorNow=No_Error` 并走核心 CLEAR；
  CAN 断连恢复改为通知适配器（`FocRunState_LinkRecovered`）后由适配器清除；
  恢复条件 v1：CAN=链路恢复标志、编码器=坏帧 streak 清零、温度传感器=采样恢复、
  高温=温度 < 80 °C、过/欠压=enable 窗口 + 100 ms 驻留、过流=trip−10% + 100 ms 驻留、
  操作类=会话已释放；无在线判据的参数类故障保留操作员确认语义（文档注明过渡）。
- **D2（后续）**：FaultLatch 多故障投影完善与逐故障夹具扩充；控制超时静默窗口；
  与故障保护解耦计划（`2026-09-18-fault-protection-v1`）的检测接口协同。

D1 证据（2026-09-19）：

- `foc_errhandle.c`：Clear 只登记 `ModeNow=Clear_Error`（不再直写 `ErrorNow`）；
  `foc_mode_dispatch.c`：`Default_Param` 不再隐式清错、`Clear_Error` 变为请求标记；
  `interface_can.c`：接收恢复不再隐式清 `CAN_DisConnect`。
- `foc_run_state.c`：恢复矩阵 v1（编码器 streak 清零 / 温度 80 °C 滞回 + 传感器有效 /
  过欠压 enable 窗口 / 过流 trip−10% 驻留 2000 拍 / CAN 计数回落即自动恢复 /
  参数与操作类保留操作员确认（过渡）/ 未知枚举拒绝）；清除经核心 `CLEAR` + 自检回 `READY`。
- 夹具：`test_run_state.py` 新增恢复矩阵 6 组用例（含 2100 拍驻留验证），
  差分等价 **38,400 组 tick 保持**；`test_bus_voltage_protection.py` 更新为
  "登记请求、由状态机准入"契约；`run_can_status_tests.py` 移除已无调用的清错桩。
- PR 全量九项通过：`outputs/runs/20260919T024822024336Z-27f5b3a0/summary.json`。

原则（E 语义）：CLEAR 只在**源恢复 + 样本新鲜 + 资源释放**同时成立时被接受；
清除不是硬件恢复证明；通信类故障允许自动恢复，其余不允许隐式清错。

| 故障 | 置位源 | 自动解除 | CLEAR 准入（草案） | 待确认 |
| --- | --- | --- | --- | --- |
| `CAN_DisConnect` | 心跳超时（`interface_can.c`） | **是（现状保留）** | 收到有效帧即恢复；改经核心恢复事件上报 | 窗口 = 现有心跳超时 |
| `Over_Voltage` | 采样（34.0 V/2 ms、34.5 V/3 拍）、模式切换、辨识 | 否 | `vbus < 33.8 V`（enable 上界）持续 100 ms + 样本新鲜 | 用 enable 窗口还是 trip−滞回 |
| `Under_Voltage` | 采样（24.0 V/100 ms）、模式切换、辨识 | 否 | `vbus >= 25.6 V`（enable 下界）持续 100 ms + 样本新鲜 | 同上 |
| `Over_Current` | 采样（18 A/40 A 档 trip） | 否 | 电流回落至 trip−10% 以下持续 100 ms + 功率已释放 | 裕度与窗口 |
| `High_Temprature` | 采样 ≥90 °C（仅在 `ErrorNow` 干净时锁存）、辨识 | 否 | 温度 < 80 °C（滞回）+ 传感器有效 | 滞回值与驻留时间 |
| `TemperatureSensor_Error` | 采样丢失 ≥100 ms | 否 | `missed_ms < timeout`（采样恢复）且新鲜 | — |
| `Encoder_Error` | 坏帧步进 ≥100（运行/标定多点） | 否 | 有效帧恢复（streak 清零）+ 样本新鲜 | streak 窗口 |
| `Encoder_NotCalibrated` | 启动检查、运行检查、辨识 | 否 | 编码器标定有效 | — |
| `PolePairs_Error` | 参数校验、标定 | 否 | 极对数参数有效 | — |
| `MotorParam_Error` | 保存失败、参数校验（多点） | 否 | 参数校验全过 | — |
| `Large_Phase_Resistance` | R/L/Flux 标定多点 | 否 | 相电阻参数在界内 | 界内判据 |
| `Large_Phase_Inductance` | 标定 | 否 | 电感参数在界内 | 同上 |
| `CurrentOffset_Error` | 采样自检、零偏标定 | 否 | 零偏值有效（需重跑标定） | — |
| `Sensorless_Error` | 无感观测器多点 | 否 | 无感会话释放 + 退出无感模式（重启动，不热恢复） | — |
| `ControlOverrun_Error` | 快环超时 | 否 | 静默窗口内无新超时 | 窗口（建议 1 s） |
| `FrictionIdentification_Error` | 辨识多点 | 否 | 会话释放 + 电压/编码器健康 | — |
| `CoggingCalibration_Error` | 齿槽标定、模式互斥守卫 | 否 | 会话释放 + 编码器健康 | — |

迁移动作（与矩阵配套）：

1. `Clear_Error` 直写路径（`foc_errhandle.c`）收归适配器：置 `APP_EVENT_CLEAR`，按矩阵聚合准入；
2. `interface_can.c` 的 CAN 恢复自动清错改经核心恢复事件（保持"收到帧即恢复"语义）；
3. `Default_Param` 的无条件 `Set_ErrorNow(No_Error)` 移除（默认参数不得清无关故障）；
4. FaultLatch：`first_fault` + 活动故障由核心维护；`ErrorNow` 保留为兼容投影
   （当前无多故障位域，矩阵按单值锁存实施）。

## 验证证据

阶段 A（2026-09-19）：

- **移植保真**：与上游 `app_lifecycle.{c,h}` 逐 token 比对一致（.c 2570 tokens、.h 380 tokens；
  忽略注释/空白/花括号/行接续符）——上游 135 格转移表与 ROOT 独立复核结论可直接传递。
- **门禁**：clang-format、readability、comment-language、接口契约检查全部通过；
  `zig cc -std=c99 -Wall -Wextra -Werror` 编译 0 错误。
- 按裁决：未移植测试（保真度改由逐 token 比对证明）；Keil 挂载在阶段 B 一并完成（见上）。

阶段 B（2026-09-19）：

- **适配器落地**：`foc_run_state.c` 重写为 `AppLifecycle` 适配器：
  每拍把 worker 结果与外部模式写入合成为核心事件（BOOT/SELF_TEST 链、FAULT/CLEAR、
  STOP、ENABLE、START_DONE），供给 guards（`phases_disabled = !power_on`、
  `start_permitted = axis_profile_valid`、`startup_confirmed = 就绪查询`），
  执行核心动作（START_CONTROL/DISABLE_POWER）并维护兼容投影（ModeNow/LED/提交）。
- **差分等价**：`test_run_state.py` 改为加载真实核心（`app_lifecycle.c`）+ 适配器，
  与旧三函数对拍 16 种结果载荷 × 800 输入组合 × 3 tick：
  **38,400 组 tick 全部一致**（动作序列与每 tick 的 ModeNow/ModeLast/ErrorNow/镜像）。
- **差分揭示的兼容语义**（已编码进适配器）：
  1. 旧停机清理按 `ModeLast != Disable` 授权，而不是按真实功率状态——适配器以
     `apply_power_actions` 参数显式复刻该条件（含"模块已自行停相"场景）；
  2. 故障事件本身不动功率级，停机清理统一由停止路径执行一次，避免双重 Clear+Stop；
  3. 位置/速度的首拍延迟使能映射为核心 `STARTING`；其余模式同拍 ENABLE→START_DONE；
     兼容提交的 `defer` 等价于核心处于 `STARTING`。
- **构建**：`app_lifecycle.c` 已挂载普通/HIL 两个 Keil 工程，新增
  `../../../../services/lifecycle` include 路径；`check_project_layout` 0 错误。
- **门禁**：clang-format/readability/comment-language/接口契约全部通过；
  普通与 `SERVO_HIL_ENABLE=1` 变体编译 `foc_run_state.c`/`motor_state.c`/
  `foc_task.c`/`foc_mode_dispatch.c` 均 0 错误。

- **移植保真**：与上游 `app_lifecycle.{c,h}` 逐 token 比对一致（.c 2570 tokens、.h 380 tokens；
  忽略注释/空白/花括号/行接续符）——上游 135 格转移表与 ROOT 独立复核结论可直接传递。
- **门禁**：clang-format、readability、comment-language、接口契约检查全部通过；
  `zig cc -std=c99 -Wall -Wextra -Werror` 编译 0 错误。
- 按裁决：未移植测试、未登记 Keil 源列表（阶段 B 接入适配器时一并挂载）。
