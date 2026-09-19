# 对接 deepseek-e 节点：当前节点收尾与供料清单 v1.0

日期：2026-09-19。状态：**待启动**——等当前在途功能优化（齿槽全模式接线、
`foc_calibration` 拆解、硬件解耦 H 阶段等）完成后，按本清单统一执行。

对接目标：`D:/Work/Code/yg_esc_deepseek`（分支 `codex/deepseek-e-framework`，
2026-09-19 核对点 `b5504186`）。执行原则：每项独立可交付、可回退；产出"离线证据 +
语义表/向量"；不依赖实机；不改变线上协议与编号（CAN ABI、Mode/Error 编号）。

## 1. 背景与判断

- deepseek-e 是 E 框架的已接线原型：唯一生命周期所有者（`app_lifecycle` +
  `motor_state`）、统一命令准入（`motor_commands`）、维护会话（`motor_maintenance`）、
  Watch 入口（`motor_debug_command`）、只读投影（`motor_status`）均已进生产工程；
  遗留见其交接包 `project_docs/handover/deepseek_e/E_DESIGN_GAP_INVENTORY.md`
  （DS-03 的 F2/F3、维护显式子状态机、物理目录迁移、五目标构建与 DS-05 组合回归、
  实机验收）。
- 本节点与目标节点的关系：E 纯核心已移植到 `firmware/services/lifecycle/app_lifecycle.{c,h}`
  （2026-09-19 实测：两侧同 token，字节差异仅为中文契约注释与格式）；外围是**适配器路线**
  （`foc_run_state.c` + 差分夹具），与 E 侧**所有者路线**（`motor_state.c` + 旅程测试）
  互为对照。相关背景计划：[E 生命周期差距与迁移](2026-09-19-e-lifecycle-gap-and-migration.md)、
  [统一运行状态机设计](2026-09-19-unified-run-state-machine.md)、
  [foc_run 归位与瘦身](2026-09-19-foc-run-placement.md)、
  [FOC 硬件解耦](2026-09-19-foc-hardware-decoupling.md)、
  [故障保护解耦设计](2026-09-18-fault-protection-v1.md)、
  [齿槽补偿扩展](2026-09-19-cogging-compensation-torque-modes.md)。
- 对接面 = E 六件套 + schema11 + 验证口径。**功能差集小**（E 尚无：齿槽力矩补偿、
  MCU 内温回读）；**重构差集大**（本节点 09-18/19 的适配器、恢复矩阵、解耦产物
  尚未成"可被 E 直接吸收"的形态）。
- 本清单只有一个目标：**让 E 节点剩下的工作只做"接线与验收"，不再做"语义推导与再设计"。**

### 对接面速查

| E 节点（对接面） | 本节点对应物 | 现状/断点 |
| --- | --- | --- |
| `app_lifecycle.{c,h}`（纯核心） | `firmware/services/lifecycle/app_lifecycle.{c,h}` | 同 token；字节因注释/格式不同；无单向同步规则（任务 4） |
| `motor_state.c`（唯一所有者/快环写者/撤销） | `firmware/app/foc_run_state.c`（适配器）+ `firmware/app/motor_state.c`（存储） | 语义已被差分锁定，但未成对照表（任务 6）；剩余直写点未收口（任务 1） |
| `motor_commands.c`（统一准入） | `foc_mode_dispatch.c` + CAN 入口 | 伪命令仍占模式枚举（任务 2） |
| `motor_maintenance.c`（会话） | `foc_run_state.c` 会话 + 各标定模块 | 语义已实现未成规格（任务 7）；拆解未收尾（任务 3） |
| `motor_debug_command`（Watch 入口） | 无 | 属 E 侧调试资产，无需移植 |
| `motor_status`（只读投影） | telemetry/影子字段 | 已对齐，无需动作 |
| schema11 + 事务化记录（`common_libraries/persistence`） | `foc_param.c`（`PARAM_SCHEMA_VERSION=11U`） | 两侧均为 11；口径冻结（任务 12） |
| 五目标构建 + `verification_tests` | 单 Keil 目标 + `tools/run.py` harness | 测试设施不迁移；迁向量（任务 8） |

## 2. 任务清单

### 一、必须做（不先收口，对接必然踩坑）

- [ ] **1. RunState 2b 剩余直写点收口**
  - 2026-09-19 状态：启动默认遗留已修（`2b4fee3`：`motor_state.c` 由 `Calib_CurrentOffset`
    改为 `Motor_Disable`）。实测剩余 = cogging `Set_ModeNow`×4 + `Stop/Start_PWM_Generate`×3
    （`foc_cogging_calibration.c` L104/108/150/155/218/374/382）、`foc_errhandle.c` 准入路径
    `ModeNow` 写入×4（L220/242/250/260，属请求登记、夹具已建模的过渡）、TorqueGuard 跳闸路径
    （经 `Task_Current_Mode` 签名锁定，需经模块标志 + dispatch 转结果协议）、`main.c` save 收尾
    `Set_ModeNow(Motor_Disable)`、dispatch `Motor_Disable` 分支零矢量预载。
  - 深水设计约束（cogging finalize）：现 `FocCogging_Service` 在临界区内写 `Save_Param` 作为
    "占位锁"（防 CAN 抢占长耗时 CRC）；迁移须由状态机承接——模块置请求句柄，快环 ≤1 拍内落位，
    或在计划中显式记录保留边界。**每路径先建夹具再迁移（对照 2a 做法），不得跳步。**
    ——2026-09-19 已迁移：friction 会话（`44e7aa5`）、齿槽结果协议 + TorqueGuard 挂起标志
    （`5a7e92c`）。保留为例外并已在代码注明：齿槽自管启相 / finalize 保存占位写 /
    `foc_errhandle` 准入登记写（阶段 3）/ `main.c` 前台保存握手 / dispatch 禁用态预载。
  - 范围：TorqueGuard 跳闸、cogging `Stop/Start/finalize` 临界区握手、标定模块剩余
    `Save_Param`/高侧路径、sensorless/position/impedance worker、dispatch 零矢量预载。
  - 产出：全部直写点经结果协议或会话边界上报；统一运行状态机 2b 收尾记录更新。
  - 验收：`check_architecture` 零新增；逐路径夹具 + 差分等价保持；单目标 0 错误 0 警告。
  - 理由：E 侧前提是"唯一所有者 + 单一写者"，每留一处即多一处双写者隐患与实机核对轮次。
- [ ] **2. 伪命令迁出模式枚举（Stage 3）+ CAN 兼容映射表**
  - 产出：命令/模式/操作分离后的迁移映射表（`ModeNow` 伪命令 ↔ 新投影，线上编号不变）
    与协议兼容说明。
  - 验收：映射表可由夹具逐值验证；启动前单独评审（沿用既有裁定）。
- [ ] **3. `foc_calibration` 拆解收尾 + 模块对标表**
  - 产出：拆解后模块 ↔ E 侧房间（`motor_control/identification/*` + `services/parameters`）
    一一对应表，避免出现第三套标定模块划分。
  - 验收：单目标编译 0/0；相关夹具迁移完成；标定会话语义不回归。
- [x] **4. E 核心正本对齐与冻结**（供料包 §1：token 2571/383 一致；回灌包 `git apply --check` 通过，待审后执行）
  - 产出：`app_lifecycle.{c,h}` 逐 token 复验记录（忽略注释/空白/花括号/行接续）；
    单向同步规则——**代码正本 = E 侧；中文契约注释 = 本节点资产，回灌一次后冻结**。
  - 验收：复验报告 + 规则条目入库；此后核心不在本节点单独演化。
- [x] **5. 故障语义统一口径（含 P0-A）**（供料包 §4：三口径模型/逐故障/8 项裁决；E 缺口清单 F2/F3 对齐）
  - 范围：本节点恢复矩阵 v1 × E 侧 `FaultLatch`（first/latched/active）×
    `codex/fault-protection-p0a` 的 P0-A 故障核心。
  - 产出：三口径对照表 + 待裁决清单（阈值/驻留/首因顺序/清除准入的归属），
    供 E 侧 DS-03（F2/F3）直接使用。
  - 验收：每个故障一行：置位源、恢复条件、阈值、驻留、清除准入、冲突项标注。

### 二、值得做（供料；直接删掉 E 侧设计/验证工作量）

- [x] **6. 适配器语义对照表**（供料包 §2：12 条差分锁定语义 × 实现行号 × E 侧对接点）：差分已锁定语义逐条成表（首拍延迟使能、故障事件不动功率级、
  停机清理授权条件、`Save/Default` 故障容错、命令类不自动使能等）→ E 侧 `motor_state.c`
  比对清单 + DS-05 旅程预期来源。
- [x] **7. Operation 会话语义规格**（供料包 §3：五要素模型 + 5 操作规格 + 6 条关键语义）：SAVE/DEFAULTS/ZERO/齿槽/相电阻会话的
  begin/step/cancel/release/result、`COMMITTED` 语义、取消不回滚 Flash、释放证明 →
  E 侧"维护显式子状态机"（缺口清单 §2 第 1 条）的定义输入。
- [ ] **8. 语义向量抽取**：从差分/路径夹具中抽取与目录无关的输入/预期向量集
  （不迁 harness 本体）→ DS-05 组合回归直接引用。

### 三、可以做（子功能回移准备）

- [ ] **9. 特性移植卡**：对 E 尚无/薄弱的功能逐项登记（齿槽全模式补偿、MCU 内温、
  Mode3 相位参考等）：E 侧落点、依赖的硬件接口/状态机写点/会话、前置解耦项、已有证据。
- [ ] **10. 齿槽收尾**：全转矩模式接线（等 `foc_task.c` 并行改动落定）、标定会话不直写
  模式/功率级、夹具固化——用户已裁决"后续合并"的第一个确定性对象。
- [x] **11. 硬件解耦 H 尾项**：H4b（`foc_sensing.c`/`foc_errhandle.c`）、H5
  （`interface_can` 等反向依赖）先做；**H3（encoder 大结构 → `platform/api` 快照契约）
  单独立项评审**（最高风险，涉 5 处公共签名）。
  ——2026-09-19 已由波次关闭：H3（编码器解耦 A/B/C/D + 实机只读证据）、H4b（感测/功率级
  契约）、H5 硬件类（通信分层 S1 = T3）；架构债硬件类清零（11→5，余 5 条为 motor→services
  方向项，并入任务 9 一并处理）。

### 四、冻结与禁止（防返工）

- [ ] **12. schema11 口径冻结**：以 E 侧 schema11 统一设计与 `common_libraries/persistence`
  为准；本节点已为 11（`PARAM_SCHEMA_VERSION=11U`），停止旧链（`LEGACY_*`）方向的新投资；
  遗留登记为迁移项。
- [ ] **13. 禁止清单**：不动本节点目录结构（物理迁移是 E 侧任务）；不新建会被 E 取代的
  大件（持久化事务/NV 记录/产品配置）；不改 CAN 编号与线上语义；核心不单独演化；
  不给接口另起命名体系（`AppLifecycle`/`MotorState`/`MotorCommands` 已定）。
- [x] **14. 对接通道与锚点**：deepseek-e 尚无远程分支；本分支截至 2026-09-19 领先其
  origin 对应分支 15 个提交（另有在途未提交改动）。切换节点前完成锚定（提交/标签 +
  证据目录 + 备份）；对接通道（补丁包 vs 推送远程）由用户决定。
  ——2026-09-19 锚定已执行：推送 `4d395bf2..2b4fee3a` + 标签 `sync-20260919` /
  `sync-20260919-latest` 已在 origin。通道按既定方案执行：补丁/交底包**单向回移**到
  deepseek-e（两树结构分叉，不做整树 merge）；如需改为推送远程另议。

## 3. 执行顺序（三批）

| 批次 | 内容 | 前置 |
| --- | --- | --- |
| 第一批（纯文档/导出，零风险，可与功能优化并行） | 4、5、6、7、12、14 | 无 |
| 第二批（收尾，按"先夹具后迁移"节奏） | 1 → 2 → 3 → 10 | 在途功能优化落定；10 另需 `foc_task.c` 并行改动落定 |
| 第三批（按风险/余力） | 8、9、11（H3 先立项） | 第二批主体收敛 |

之后切换 deepseek-e 节点，按其交接包推进：DS-03 → DS-04 → DS-05 → 物理目录迁移 →
实机验收（E 侧任务卡已就绪，本清单产出直接作为其输入）。

## 4. 总验收

- 所有项均产出"离线证据 + 语义表/向量"，不依赖实机；
- 文档类改动走 quick 档；涉及代码的按 [AGENTS.md](../../../AGENTS.md) 规则走 PR 档；
  每项独立回退；
- 完成标准：E 节点接手时，DS-03/DS-05 可直接消费本清单产出，且无需回本节点补做重构。

## 5. 待确认

1. 对接通道选型（补丁包 / 推送远程）——需用户决定；
2. 任务 2 启动需单独评审（Stage 3）；
3. 任务 11 的 H3 需立项评审。

## 6. 执行记录

- 2026-09-19（第一批 4/5/6/7）：产出《对接 deepseek-e：供料包（任务 4/5/6/7）v1.0》
  （`2026-09-19-e-handoff-pack.md`）：E 核心逐 token 复验与回灌包（待审）、适配器语义对照表、
  Operation 会话语义规格、故障三口径对照表 + 8 项裁决清单。
  提交：`<pending>`。
- 2026-09-19（波次收尾）：硬件边界 T1–T5、通信分层 S1–S4/S6、HIL 退役全部落地并提交
  （硬件类架构债清零，`check_architecture` = 5 known 全部为方向类）；任务 11、14 关闭。
- 2026-09-19（本轮）：启动默认遗留修复 `2b4fee3`（quick 档 PASS：
  `outputs/runs/20260919T131434088712Z-05d9ebc6`；Keil normal 0 Error / 0 Warning，Code=81964）；
  锚定推送 `4d395bf2..2b4fee3a` + 标签 `sync-20260919` / `sync-20260919-latest`。
- 待办：任务 1 剩余清单见条目注释（深水项按"先夹具后迁移"逐路径推进）；任务 2/3/8/9/12 未启动。
- 2026-09-19（第二批）：friction 会话 `44e7aa5`；齿槽结果协议 + TorqueGuard `5a7e92c`
  （quick 全绿 `outputs/runs/20260919T145921116298Z-44e7aa57`；Keil normal 0/0，Code=82080；
  差分 46,464 保持；齿槽夹具全 PASS）。任务 1 的主体直写点已收口，仅余已注明的过渡例外。
