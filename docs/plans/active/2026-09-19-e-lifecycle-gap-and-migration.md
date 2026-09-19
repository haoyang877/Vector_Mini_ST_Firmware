# E 生命周期状态机：差距分析与迁移方案 v1.0

日期：2026-09-19。状态：**阶段 A/B 已完成**，阶段 C/D 待排期。
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
| C | **Operation 会话**：Save/Default/Zero/标定完成路径挂入 MAINTENANCE/Operation（Begin/Step/Cancel/IsReleased/Result）；继续 2b 剩余路径但目标形态从"结果协议"升级为 operation 协议 | 逐路径夹具（含取消/失败/释放确认）；SAVE 成功需 `COMMITTED` | 阶段 B；2b 已迁移的 friction/观测器标定 outcome 直接映射为 operation 结果 |
| D | **FaultLatch + CLEAR 恢复**：活动故障/首故障分离；CLEAR 要求源恢复+样本新鲜+资源释放；迁移 CAN 重连/默认参数的隐式清错副作用 | 故障锁存与恢复条件夹具；通信恢复不清错；幂等 CLEAR | 阶段 C |

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
