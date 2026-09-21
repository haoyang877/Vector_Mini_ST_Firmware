# 参数描述符模型与 ID 分区设计 v1.0

日期：2026-09-20。状态：**设计稿（仅文档，不含代码实施）**。
上游依据：[通信分层说明（CAN 接入）](../../architecture/communication_layering.md)（协议约定层 =
`communication/protocol/can_parameter_wire.*`）与
[通信分层优化设计](2026-09-19-communication-layering-optimization.md)（S0–S4/S6 已落地）。

## 1. 意图

把当前"每个 CAN 参数一段 `switch` case"的实现方式，收敛为一个**单一来源的参数描述符表**，
并显式化三件今天只隐含在代码条件里的事情：

1. **分组**：参数按控制层级归属，扩展时有明确落点；
2. **生效时机（apply_policy）**：每个参数声明"何时可以写、写了何时生效"；
3. **ID 分区**：在剩余 ID 空间里预留分组块，避免"靠自增分配"。

同时冻结一条纪律：**未实现的参数不得提前登记到线协议枚举**，避免制造投机性 ABI。
本版只做设计；P0 之外不实施。

非目标（明确排除，见 §5）：改动现有线格式/编号、实施 MTPA/弱磁/陷波/模态辨识/自整定/多轴同步、
力矩传感器或减速箱模型、功率限制策略。

## 2. 现状与证据

| 事实 | 证据 |
| --- | --- |
| 参数持久化布局为 `InterfaceParam_TypeDef`，schema 版本 11，`magic_word` 单独校验 | `firmware/services/parameters/foc_param.h:13-84` |
| 已发布 schema 版本与追加规则（v4→v11，字段只追加、偏移不变） | `foc_param.h:20-27, 65-83` |
| 线协议格式版本 `CAN_PARAMETER_FORMAT_REVISION = 2` | `communication/protocol/can_parameter_format.h` |
| 线协议参数 ID：`node<<8 \| param`，param 只用低 8 位 → **每节点上限 256** | `can_parameter_wire.h:167-173` |
| 已定义 ID 数 = **106**，剩余 **150**（0x3A–0x3D/0x63/0x66 与整段 0x70–0xFF） | `can_parameter_wire.h:8-118`（本文件实测统计） |
| 写路径只改运行态，**不自动落 Flash** | `can_binding_commands.c:32-211`（全部只写 `MotorControl.*`/`CANMsg.*`） |
| 参数保存为显式会话，不在命令写入时触发 | `docs/architecture/communication_layering.md` §3「保存会话」 |
| 编译期默认值来自 profile 宏，可被 Flash 记录覆盖 | `foc_param_profile.h:22-135` |
| 轴身份已有"稳定 ID 永不复用、存 `uint32_t` 而非 enum"的先例 | `motor_axis_profile.h:14-23` |
| 编码器参数已有"禁能态才可写"的先例 | `can_binding_commands.c:90-97`（`CAN_SET_ENCODER_REVERSE` 要求 `Motor_Disable`） |
| 节点号变更带限速副作用；波特率变更重启外设 | `can_binding_commands.c:68-75, 196-201` |

**关键区别（本设计的立足点）**：ID `0x00–0x07`（`CAN_SET_MODE/CURRENT/SPEED/POS`）是**实时控制命令**，
不是参数；它们不应进入参数描述符表。参数描述符只覆盖"设定型 / 标定型 / 身份型"参数。

## 3. 结构问题（可度量）

| 问题 | 现状证据 | 目标 |
| --- | --- | --- |
| apply 语义分散在 `if` 条件里，无统一声明 | `can_binding_commands.c` 的每个 case 各自判断范围与模式门限 | 每个参数一行 `apply_policy` 元数据 |
| 无分组，扩展靠"加 enum + 加 case + 加黄金向量" | `can_parameter_wire.h:8-118` 线性追加 | 分组表 + 分区预留 |
| 命令与参数混在同一 ID 空间与同一 switch | `can_binding_commands.c:37-65`（命令）与 `:67-206`（参数）同函数 | 描述符仅覆盖参数；命令单列 |
| ID 分配无策略 | 现用 106 个分散在 0x00–0x6F | 0x70–0xFF 按分组预留 |
| 电流环增益在 Flash 有存储但**无 CAN 读写 ID** | `foc_param.h:46-49`（`id_kp/id_ki/iq_kp/iq_ki`）vs `can_parameter_wire.h` 无对应项 | 记为缺口（P0/P1 决策点 D2） |

## 4. 设计

### 4.1 分组（groups）

沿用"公共约束 + 各控制层 + 补偿/滤波"的组织方式，但**只作为描述符的分区标签**，不新增结构体：

| 组 | 内容 | 现有代表 |
| --- | --- | --- |
| `common` | 单位/方向/命令超时/快停/公共限幅 | `current_limit` 0x10、`speed_limit` 0x12 |
| `identity` | 节点、极对数、轴身份 | `node_id` 0x08、`pole_pairs` 0x0A、`axis_profile` |
| `motor_model` | Rs/Ld/Lq/磁链 | 0x44/0x46/0x48/0x4A |
| `current_loop` | d/q 增益与电流限制 | Flash `id_kp/iq_kp`（缺 CAN ID，D2） |
| `velocity_loop` | 速度 kp/ki、加减速 | 0x18/0x1A、0x14/0x16 |
| `position_loop` | 位置 kp/kd/ki、积分限、加减速、最大速度、cascade | 0x22/0x24/0x50/0x52、0x1C–0x20、0x54/0x56 |
| `trajectory` | 前馈、jerk、到位窗口（P1） | 暂无 |
| `encoder` | 方向、零位、标定、线性化 | 0x4E、0x0D、`encoder_*` |
| `compensation` | 摩擦、齿槽 | 0x58–0x62/0x26、0x68–0x6D |
| `protection` | 母线/过流/过温/降额 | 0x2C/0x2E、protection 模块 |
| `comm` | 波特率、心跳 | 0x28、0x2A |

### 4.2 参数描述符字段

单一来源表（未来代码化时，仅描述，不实施）：

```text
id            CAN_PARAM_ID（线协议 ID，永不复用）
group         §4.1 的组标签
name          参数名（与 Flash 字段/运行态字段对齐）
unit          物理单位（A / rad/s / rad/s^2 / A/rad / ms / Ohm / H / Wb / count / -）
wire          线路编码（float32 / milli-i32 / centi-i32 / milli-i16）
range         [min, max]，非法值直接拒收（保持现状语义）
access        RO（遥测/结果）/ RW（可写）/ WO（会话命令）
apply_policy  §4.3
persist       Flash（进 `InterfaceParam_TypeDef`）/ RAM-only / compile-time
since         FORMAT_REVISION 引入版本
implemented   是否已接线（未接线不得登记到 enum）
```

### 4.3 apply_policy

```c
typedef enum {
    PARAM_APPLY_IMMEDIATE,      /* 下一控制拍即生效（增益类） */
    PARAM_APPLY_NEXT_SESSION,   /* 会话/校准流程内生效（摩擦、齿槽） */
    PARAM_APPLY_WHEN_DISABLED,  /* 仅禁能态可写（编码器方向、极对数） */
    PARAM_APPLY_AFTER_RESTART,  /* 需重启外设/重新初始化（节点号、波特率） */
    PARAM_APPLY_READ_ONLY       /* 仅读（遥测与辨识结果） */
} ParamApplyPolicy;
```

**由现状反推的推荐分类**（`建议` = 设计主张；`现状` = 代码已如此）：

| 参数 | 现状行为 | 建议 apply_policy |
| --- | --- | --- |
| 电流/速度/位置增益与限幅 | 立即写运行态 | `IMMEDIATE` |
| `CAN_SET_ENCODER_REVERSE` 0x4E | 要求 `Motor_Disable` | `WHEN_DISABLED`（现状一致） |
| `CAN_SET_POLEPARIS` 0x0A | 立即生效 | `WHEN_DISABLED`（**收紧**，决策点 D1） |
| `CAN_SET_NODE_ID` 0x08 | 立即 + 限速副作用 | `AFTER_RESTART`（滤波在初始化时取持久化值） |
| `CAN_SET_CAN_BR` 0x28 | 重启外设 | `AFTER_RESTART`（现状即重启） |
| `CAN_SET_CAN_HB` 0x2A | 立即 | `IMMEDIATE` |
| 摩擦应用 0x58 / 齿槽 0x26 | 会话请求 | `NEXT_SESSION` |
| 辨识结果 0x59–0x62、遥测 | 只读 | `READ_ONLY` |

### 4.4 ID 分区（仅文档预留）

**不改动 0x00–0x6F 既有编号**（已发布，禁止重排；参见 `motor_axis_profile.h:14-15` 的稳定 ID 纪律）。
在空闲段 0x70–0xFF（144 槽）按组分块：

| 区段 | 组 | 槽位 |
| --- | --- | --- |
| 0x70–0x7F | `common`（公共约束/限幅） | 16 |
| 0x80–0x8F | `current_loop` / `motor_model` | 16 |
| 0x90–0x9F | `velocity_loop` / `position_loop` | 16 |
| 0xA0–0xAF | `trajectory` / 前馈 | 16 |
| 0xB0–0xBF | `compensation`（摩擦/齿槽扩展） | 16 |
| 0xC0–0xCF | `encoder` / 状态估算 | 16 |
| 0xD0–0xDF | `protection` / 热降额 | 16 |
| 0xE0–0xEF | 滤波 / 模态（P2/P3 预留，暂不登记） | 16 |
| 0xF0–0xFF | 系统 / 会话 / 厂商预留 | 16 |

规则：**预留不等于登记**——只有 `implemented = true` 的参数才写入 `CAN_PARAM_ID`。

### 4.5 版本策略

- **线协议**：新增已实现参数时递增 `CAN_PARAMETER_FORMAT_REVISION`，同步 `tools/bench/can_*.py`
  主机镜像与黄金向量；未实现参数不占版本。
- **Flash schema**：沿用 `InterfaceParam_TypeDef` **只追加、不改既有偏移**的既有规则
  （`foc_param.h:65-83`）；`PARAM_SCHEMA_VERSION` 递增。
- **大表**（齿槽/MTPA 等）：不逐点占 ID，走"表 ID + 分批访问"，沿用现有 0x6B/0x6C 的单点 Q15 模式。

## 5. 非目标

- 不实施 MTPA / 弱磁 / MTPV / 陷波 / 模态辨识 / 自整定 / 在线惯量 / 多轴同步；
- 不新增力矩传感器、减速箱效率、重力补偿参数；
- 不新增功率限制（机械/电/回馈）策略——需先有实现；
- 不改动既有 106 个 ID 的编号、单位、编码、哨兵；
- 不引入投机性结构体、工厂、注册表代码（`AGENTS.md`「Simplification, coupling, and reuse」）。

## 6. 路线图（仅规划）

| 等级 | 范围 | 是否新增线格式 |
| --- | --- | --- |
| P0 | 公共约束 + 位置/速度/力矩/电流必要项 + 通信项；**对既有参数补元数据** | 否（除 D2） |
| P1 | 轨迹/前馈/软限位、摩擦与齿槽扩展 | 是（需功能先落地） |
| P2 | MTPA / 弱磁 / 陷波 / 增益调度 | 是 |
| P3 | 自整定 / 自适应陷波 / 在线惯量 / 多轴同步 | 是 |

## 7. 验收（设计稿）

- 本文件纳入 `docs/plans/active/README.md` 索引；本地链接通过 `check_docs`；
- `uv run python tools/run.py verify --profile quick` 通过（本次仅文档变更）；
- **无代码、无协议、无黄金向量改动**；不产生 `outputs/` 之外的跟踪产物。

后续若进入 P0 实施：走 PR 档，需冻结现有 106 个 ID 行为、更新主机镜像与黄金向量（另行计划）。

## 8. 风险与决策点

- **D1 收紧 `CAN_SET_POLEPARIS` 为禁能态可写**：可能影响现有上位机在线改写行为，需确认后执行。
- **D2 电流环增益缺 CAN ID**（`foc_param.h:46-49` 有存储、线协议无项）：是补 ID 还是维持仅 Flash？
- **D3 描述符是否代码化**：本设计仅文档；若代码化，需评估 ROM 与逐 case 展开的权衡。
- **R1 ID 耗尽**：150 槽对 P0–P1 充足，P2/P3 需大表间接化；分区表防止碎片化。
- **R2 命名冲突**：既有 ID 名（`CAN_SET_*`）与分组名不同层，描述符只做映射，不改名。

## 9. 证据

- 本文件：`docs/plans/active/2026-09-20-parameter-descriptor-model.md`。
- ID 统计：`can_parameter_wire.h` 实测 106 定义 / 150 空闲。
- 现存事实引用行号见 §2 表格。
- 未改动任何代码；未运行固件构建（文档变更不需要）。
