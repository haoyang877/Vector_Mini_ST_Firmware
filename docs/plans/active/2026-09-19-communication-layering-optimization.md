# 通信分层优化设计 v1.0

日期：2026-09-19。状态：**S0–S4、S6 已实施并提交；S5 未执行（用户裁决为"需先出最坏时延分析
并授权实机 CAN 回归"）**。
依据：[通信分层说明（CAN 接入）](../../architecture/communication_layering.md)的三层划分
（协议约定 / 协议实现 / 耦合）。自评结论见 §13（按用户要求由主会话自行评审，未使用外部评审代理）。

## 1. 意图

把三层边界从"文档描述"变成"代码边界 + 机械门禁"，且**零行为变化**（S5 除外，
S5 是默认不执行的决策门）：

1. 线路约定、传输机制、电机耦合分文件承载，任一文件只承担一层职责；
2. 公共调用点签名与调用方零改动：`FDCAN1_Param_Init`（`board_config.c:52`）、
   `CAN_BaudRateSwitching`/`CAN_DisConnect_Handle`（`bsp_task.c:75,79`）、
   `CAN_SendMessage`（`main.c:121`）、`CANRxIRQHandler`（`stm32g4xx_it.c:343`）、
   `CAN_ReceiveMessage_Update`/`CAN_SendMessage_Update`（原生夹具切片）保持原签名原语义；
3. 线上 ABI 冻结：11 位 ID 编址（`node<<8|param`）、0x00–0x6F 命令、0x64/0x65 状态流、
   48 字节大端布局、格式版本 2、Mode 0..19 / Error 0..17、定点比例与哨兵逐位不变，
   黄金向量逐字节一致；
4. 耦合显式化：消除 `services/parameters/foc_param.c:11` 与 `app/foc_run_state.c:9`
   的隐藏 `extern CANMsg`；通信不再向服务/应用暴露可变全局；
5. 机械验证全绿：通信硬件债 4→0、隐藏跨层引用 2→0、不新增任何违规。

非目标：不改协议/编号/单位/schema 11；不改控制时序（S5 除外且需裁决）；不实施 E 侧统一准入
（只留接入点）；不改 CubeMX 生成区；不新建会被 E 侧取代的大件。

## 2. 结构问题与改进目标（可度量）

| 问题 | 现状证据 | 目标 |
| --- | --- | --- |
| 单文件四种职责 | `interface_can.c` 907 行：传输（165-276、756-772、866-907）+ 编解码（31-159、720-750）+ 派发（293-713）+ 快照（828-860） | 按职责分文件，单文件目标 ≤ 300 行 |
| 线路约定住在实现文件 | 编码等级与读/写映射在 `interface_can.c:31-105` | 迁 `protocol/can_parameter_wire.{c,h}`，唯一来源 |
| 定点转换重复 | `interface_can.c:112-159` 与 `can_motor_status.c:39-75` | 单一实现（milli32/centi32/milli16/centi16） |
| 状态流常量重复 | `can_motor_status.h:8-9` 的 0x64/0x65 无消费者 | 删除（全仓 grep 证据，评审确认） |
| 隐藏跨层引用 | `foc_param.c:11,16,56,86,105,115,157,208,244-245,353`；`foc_run_state.c:9,226` 经局部 `extern` 读写 `CANMsg`（include 检查器不可见） | 0 处；`node_id`/`can_hb_set` 归 `services/parameters`，心跳健康经通信只读接口 |
| ISR 内完整派发 | `interface_can.c:756-822` 在 RX 中断内解码并执行命令 | S5 决策门：默认维持现状；获批后有界队列 + 前台处理 |

## 3. 不变的外部行为

- **线协议**：ID 编址、命令表、编码分级（i16 毫安 / i32 毫弧度 / i32 百分一 / float32）、
  大端、哨兵拒收、回复长度、状态帧 48 字节布局与扩展版本 1 全部不变。
- **状态与故障**：心跳超时置 `CAN_DisConnect` 的判据与饱和计数、`can_rx_en`/`can_hb_count`
  复位时点、模式切换（`ModeSwitch_Handle`）与 `Set_ErrorNow` 的调用顺序不变。
- **控制输出**：每个命令的接受条件、限幅、副作用（如 `CAN_SET_NODE_ID` 同步降限速、
  `CAN_SET_CURRENT_LIMIT` 同步约束 `iqRef`）逐位不变。
- **时序**：RX 中断处理时延、TX 重试上限（5 次）、1 kHz 波特率切换与心跳判据、
  主循环顺序不变（S5 若执行则另行评估并实机回归）。
- **资源**：`CANMsg` RAM 占用与状态流缓冲不变；Code/ZI 变化必须记录
  （跨 TU 去内联预计 +0~500 B，前例 +368/+464 B）。
- **产物**：黄金向量、原生夹具、主机镜像（`tools/bench/can_*.py`）行为不变。

## 4. 基线（S0 执行前重测）

- `check_architecture`（2026-09-19 实测）：**PASS (11 known, 0 new, 0 resolved)**，其中
  communication 4 条（`interface_can.c → delay.h / hw_conf.h / fdcan.h`、`interface_can.h → main.h`）。
- Keil（2026-09-19 实测，`build_firmware --target normal --uv4 C:\Keil_v5\UV4\UV4.exe`）：
  **0 Error / 0 Warning**；Code=81888，RO-data=4888，RW-data=248，ZI-data=31352。
  （§3 中 "Code=89620 / ZI=31456" 为 HIL 退役前的历史记录，已由本次实测取代。）
- `verify --profile quick`：PASS（`outputs/runs/20260919T073520384790Z-dda0c780/summary.json`）。
- CAN 夹具：`tests/unit/native/run_can_status_tests.py`、`tests/unit/can_motor_status_test.c`、
  `tests/unit/test_can_motor_status.py`、`tests/unit/test_can_parameter_protocol.py`、
  `tests/unit/native/test_wheel_speed_limits.py`、`tests/unit/native/test_run_state.py`、
  `tests/integration/test_usb_removed.py`。
- 黄金向量：`can_motor_status_test.c:28-31`（48 B 帧）、`run_can_status_tests.py:255-273`
  （回复编码）、`:297-306`（状态流握手）、`:329-345`（心跳判据）。
- 冻结值：`CAN_PARAMETER_FORMAT_REVISION=2`、`PARAM_SCHEMA_VERSION=11`、
  Mode 0..19、Error 0..17、`CAN_MOTOR_STATUS_ID_BASE=0x7F0`。
- 夹具实测（2026-09-19，`--cc .venv/Lib/site-packages/ziglang/zig.exe`）：
  `run_can_status_tests` PASS（9 组：生产 ID/心跳/48B 黄金帧/状态源/RX 编解码/HAL 端口/
  发送调度/回复编码/命令适配）；`test_wheel_speed_limits` PASS（5 组）；
  `test_run_state` PASS（差分 46464 tick 一致）。单元/集成 `unittest discover` 含在
  quick 档 tests: PASS 内。

## 5. 目标结构

```text
firmware/communication/
├─ can/
│  ├─ interface_can.{c,h}          门面 + 心跳状态机（保留全部公共签名）
│  ├─ can_transport.{c,h}          传输机制：滤波/启停/中断使能/波特率/收发（仅 comm_hw + time_hw）
│  ├─ can_command_binding.{c,h}    命令入口：CAN_ReceiveMessage_Update 定义（头部握手 + 路由）
│  ├─ can_binding_commands.{c,h}   写路径：模式/设定值/限幅/参数/会话命令
│  ├─ can_binding_queries.{c,h}    读路径：状态/遥测/会话查询
│  └─ can_status_source.{c,h}      快照组装 + 心跳可见性判定（唯一读电机状态的遥测点）
└─ protocol/
   ├─ can_parameter_format.h       版本常量（已有）
   ├─ can_parameter_wire.{c,h}     新增：CAN_PARAM_ID + 编码等级 + 读写映射 + 定点转换/哨兵 + ID/长度
   └─ can_motor_status.{c,h}       状态帧编码/调度（改用 wire 转换）
```

允许的依赖边（其余组合禁止）：

| 模块 | 允许依赖 |
| --- | --- |
| `protocol/can_parameter_wire` | 标准库（纯逻辑） |
| `protocol/can_motor_status` | wire + `services/telemetry`（载荷结构） |
| `can/can_transport` | `platform/api`（comm_hw、time_hw）+ 标准库 |
| `can/can_command_binding`、`can_binding_*`、`can_status_source` | wire + `motor` + `services` + `platform/api` |
| `can/interface_can` | 以上全部（组合层） |

## 6. 阶段与步骤

| 阶段 | 内容 | 不变量 | 验收 | 回滚 |
| --- | --- | --- | --- | --- |
| S0 | 基线与冻结：重测 §4 全部数据并回填 | 不动代码 | 数据齐全且夹具全绿 | — |
| S1 | 硬件契约化 = 执行既有 T3（见 [硬件边界收口](2026-09-19-hardware-boundary-closure.md) §T3） | 编号/帧格式/速率/心跳语义不变；初始化失败/重连/队列满行为不变 | debt 11→9；双目标 0 错 0 警；CAN 夹具 PASS；`comm_hw.h` 补中文契约 | 单提交 revert |
| S2 | 协议约定抽取（见 §6.1） | 全部编码结果逐位不变 | 黄金向量与 CAN 夹具 PASS（不改期望）；`check_interfaces` 0 新增 | 单提交 revert |
| S3 | 实现分解（见 §6.2） | 公共签名、调用顺序、TX 优先级、ISR 行为不变 | 双目标 0/0；全套夹具 PASS；黄金向量逐字节一致；架构 0 新增 | 单提交 revert |
| S4 | 耦合显式化（见 §6.3） | 参数载入/保存/默认值次序不变；恢复判据表达式不变 | 隐藏 extern 归零；参数/限速/恢复矩阵夹具 PASS；双目标 0/0 | 单提交 revert |
| S5 | 决策门：RX 中断瘦身（默认不执行） | 若执行：需最坏时延分析 + 实机 CAN 回归 | 另行立项验收 | 独立 revert |
| S6 | 门禁与收尾：可选为 transport 单列 harness 层；更新分层文档与技术债 | 无行为变化 | `check_architecture` 通信 0 条；文档/接口/布局全绿 | 单提交 revert |

顺序与批次：S0 → S1（=T3）→ S2 → S3 → S4（与统一状态机 2b 收敛后）→ S5（裁决）→ S6。
每阶段一提交、独立回退；S1/S3/S4 结束跑 PR 档，其余 quick + 定向夹具。

### 6.1 S2 协议约定抽取（中）

- 新增 `firmware/communication/protocol/can_parameter_wire.{c,h}`：
  - 迁入 `CAN_PARAM_ID`（`interface_can.h:7-117`，值不变）；`interface_can.h` 改为 include
    新头，下游源码零改动；
  - 迁入 `CanValueEncoding` 与 `CAN_CommandEncoding`/`CAN_ReplyEncoding`（改名
    `CanParamWire_*`，逐行搬移，行 44-105）；
  - 定点转换单一实现 `CanParamWire_Milli32/Centi32/Milli16/Centi16`，合并
    `interface_can.c:112-159` 与 `can_motor_status.c:39-75` 两份副本
    （合并前先做语义差异比对，见 §8 风险 R1）；
  - ID/长度规则：`CanParamWire_Identifier(node, param)` 与编码→长度映射（i16=2，其余=4）。
- 删除 `CAN_MOTOR_STATUS_COMMAND/REPLY`（`can_motor_status.h:8-9`，无消费者）。
- 死引用清理（编译验证后删除）：`interface_can.c:26 extern ModeLast`、
  `CANMsg.rx_data_u8` 拷贝（若无外部消费者）、S1 审计后的 `hw_conf.h` 残留。
- 新增/变更公共头按 [接口契约计划](2026-09-18-interface-contracts.md) 补中文契约。

### 6.2 S3 实现分解（中大）

- 新文件与搬移映射（均为逐 token 搬移，签名不变）：

| 新文件 | 搬入内容（现位置） |
| --- | --- |
| `can_transport.{c,h}` | 滤波器/启动/中断使能的机制部分（165-202）、波特率切换机制（242-276）、TX 提交与重试（866-907 后半）、RX 取帧与帧级校验（756-772） |
| `can_command_binding.{c,h}` | `CAN_ReceiveMessage_Update` 定义（293-713）：前导握手（0x67/0x64/0x65/0x6B 校验）+ 路由 switch |
| `can_binding_commands.{c,h}` | switch 写路径（设置类 328-369、用户类 371-647 中的 SET、会话命令 550-553/594-596） |
| `can_binding_queries.{c,h}` | switch 读路径（GET 回复、状态类 650-708、会话查询 554-629） |
| `can_status_source.{c,h}` | `CAN_BuildMotorStatusSnapshot`（828-860）+ 心跳可见性判定（211-214 的电机读取） |
| `interface_can.{c,h}`（保留） | 门面：`CAN_SendMessage`（866-883）、`CAN_DisConnect_Handle`（207-228）状态机、回复暂存与 `CAN_SendMessage_Update`（720-750）、心跳状态（`can_hb_en/count/rx_en`） |

- Keil 工程（`MDK-ARM/Vector_Mini_ST.uvprojx`）与 `tools/project_paths.py` 登记新文件。
- 夹具迁移（§7）；搬移后做一次迁移等价核对（前例：逐 token 比对）。
- `CANMsg` 收敛为 `interface_can.c` 私有状态（跨文件所需字段经显式参数传递）。

### 6.3 S4 耦合显式化（中）

- 归属修订（单一写者）：

| 状态 | 现所有者 | 目标所有者 |
| --- | --- | --- |
| `node_id` | CANMsg（CAN 运行时 + 参数服务双写） | `services/parameters`（带显式 API） |
| `can_hb_set` | CANMsg（同上） | `services/parameters`（带显式 API） |
| `baudrate` | CANMsg | `can_transport` 私有 |
| `can_hb_en/count`、`can_rx_en` | CANMsg | `interface_can.c` 私有 |
| 收发暂存 `rx/tx_*` | CANMsg | 各模块私有 |

- `services/parameters` 新增：`Param_GetCanNodeId/Param_SetCanNodeId/Param_GetCanHeartbeatMs/`
  `Param_SetCanHeartbeatMs`（`foc_param.c` 私有静态，默认值 `PARAM_HW_CAN_NODE_ID` /
  `PARAM_HW_CAN_HEARTBEAT_MS`）；`Param_Upload/Download/Return_Default/SetSpeedLimit` 改经自身 API。
- `interface_can.c`：过滤器初始化、ID 组装、节点匹配、`CAN_SET/GET_NODE_ID`、
  `CAN_SET/GET_CAN_HB` 改经 `Param_*`；`CAN_SET_NODE_ID` 的限速副作用
  （`Param_SetSpeedLimit`）保持。
- `app/foc_run_state.c`：新增只读 `CAN_IsHeartbeatAlive()`，恢复判据等价替换第 226 行表达式。
- 等价论证：`flash_read_param()` 早于 `FDCAN1_Param_Init()`（`Board_Init` 顺序），
  过滤器仍取持久化节点；载入/保存/默认解析次序与限速上限全部保持。
- 该阶段与统一运行状态机 2b（`foc_run_state.c`）和 H1.2（`data_type.h`/轴 profile）
  存在并行面 → 排在 2b 收敛后执行。

### 6.4 自评补充发现（2026-09-19）

- `interface_can.h`：S1 删除 `main.h` 后必须显式补 `<stdbool.h>` / `<stdint.h>`
  （`CANMsg_TypeDef` 使用 `bool` / `uint8_t`）。
- 夹具轨迹：`run_can_status_tests.py` 以硬路径读取 `interface_can.c` 并按函数名切片
  （行 30/46/69/89/204/253/293/323）；RX 切片自带 `CAN_CommandEncoding` 与
  `CAN_ReceiveMessage_Update` stub（行 77/87），S2/S3 后按新文件路径取源，必要时改编译
  真实源文件而不是继续加 stub。
- `test_wheel_speed_limits.py`：经 `command_case()`（行 16-21）从 `interface_can.c` 取
  `CAN_ReceiveMessage_Update` 与 `CAN_SET_NODE_ID`/`CAN_SET_SPEED_LIMIT` case 块；
  S3 改路径，S4 把 `CANMsg.node_id` 断言（行 65/78/93-98）改 `Param_GetCanNodeId()`。
- 归属等价链（S4）：`flash_read_param`（`flash.c:117-128`）先 `Param_Return_Default` 或
  `Param_Download`，`flash_write_param` 经 `Param_Upload`（`flash.c:105,134`），运行期
  `foc_mode_dispatch.c:135` 也会调 `Param_Return_Default`——三条路径全部经 `Param_*` 存取，
  值与时序保持；`Board_Init` 中 `flash_read_param` 早于 `FDCAN1_Param_Init` 的顺序不变。

## 7. 夹具与工具迁移清单

| 夹具/工具 | 受影响阶段 | 需要的改动 |
| --- | --- | --- |
| `tests/unit/native/run_can_status_tests.py` | S2/S3/S4 | `function_source` 目标文件随函数迁移更新；`CANMsg` stub 形状随字段收敛更新；新增 `Param_*` stub |
| `tests/unit/native/test_wheel_speed_limits.py` | S3/S4 | 切片来源更新；`CANMsg.node_id` 断言改 `Param_GetCanNodeId()` |
| `tests/unit/native/test_run_state.py` | S4 | `CANMsg` stub 改 `CAN_IsHeartbeatAlive` stub |
| `tests/unit/can_motor_status_test.c`、`test_can_motor_status.py`、`test_can_parameter_protocol.py` | S2 | 期望值不变；仅编译/包含路径随新文件更新 |
| `tests/unit/native/test_position_config_cache.py` | S3 | 若引用 `interface_can` 切片则同步路径 |
| `tools/project_paths.py` | S6 | 目录级 include 已含 `communication/can` 与 `communication/protocol`（行 34-35），**无需登记**；仅当 S6 新建目录（如 `communication/transport`）才追加 `NATIVE_INCLUDE_DIRS` |
| `MDK-ARM/Vector_Mini_ST.uvprojx` | S2/S3 | 新增源文件入组；双目标编译 0/0 |
| `tools/bench/can_parameter_protocol.py`、`can_motor_status.py` | S2 | 无需改动（主机镜像作为外部对照，黄金向量保证同步） |
| `tests/integration/test_usb_removed.py` | — | 不动（门面调用点保持） |

## 8. 风险与决策点

- **R1 转换器合并的位等价**（S2，已核对，无风险）：`interface_can.c:112-159` 与
  `can_motor_status.c:39-75` 的 `milli32`/`centi32`/`milli16` 三份实现逐行等价
  （同乘数、同饱和边界 2^31/32767、同哨兵 `INT32_MIN`/`INT16_MIN`、同截断）；`centi16`
  仅状态帧需要（温度/母线电压）。合并为四件套是纯去重，黄金向量继续锁定。
- **R2 跨 TU 体积**：拆分后函数不再内联，Code 预计 +0~500 B（前例 +368/+464 B）；
  RAM 应持平。超预期时按函数评估内联/`static inline` 归属。
- **R3 并行冲突**：`foc_run_state.c`（2b）、`foc_param.c`（T1/参数链）、`data_type.h`（H1.2）
  都有在途改动 → S4 排后；S2/S3 只在 `communication/**` 内，冲突面小。
- **R4 夹具切片脆弱性**：原生夹具按函数名从源码切片，搬移会使夹具失败——这是**有意的**
  预期失败，用于证明迁移被感知；每阶段同步更新夹具，不得放宽断言。
- **决策点 D1**：`CAN_SET_ENCODER_STATE`(0x0C) 现为空操作（`interface_can.c:234-237`）。
  默认保留线槽位与空实现并加显式注释；若删除须单独评审（涉及主机兼容说明）。
- **决策点 D2**：`CAN_MOTOR_STATUS_COMMAND/REPLY` 删除 vs 改为引用枚举；默认删除。
- **决策点 D3（S5）**：RX 中断内派发是否外移。事实：flash 写期间 `__disable_irq()`
  （`main.c:127-132`）对 ISR 同样阻塞，差异 = 主循环最坏迭代时间；默认**维持现状**，
  执行需用户裁决 + 实机 CAN 回归授权。**自评建议：确认维持现状**——① 中断内工作量有界
  （单帧、≤4 字节解码、无等待/无分配）；② 模式/禁用类命令依赖最短时延，主循环含
  `FocCogging_Service` 与 Flash 保存长临界区（`main.c:124-139`），最坏时延显著劣于中断路径；
  ③ "ISR 只做有界采集"的规则张力以本计划记录为已接受的偏差，E 统一准入落地后如需再评估则另行立项。
- **决策点 D4**：S6 是否为 `communication/transport` 单列 harness 层允许面
  （把"传输层不依赖 motor/services/protocol"变成机械门禁）。

## 9. 与既有计划的关系

- [硬件边界收口](2026-09-19-hardware-boundary-closure.md)：S1 直接执行其 T3，不重复设计；
  本设计的 S2/S3/S4 在其硬件清零之后，避免同一文件交叠。
- [foc_run 归位与瘦身](2026-09-19-foc-run-placement.md) H5：`interface_can` 4 条反向依赖
  由 S1 清偿。
- [FOC 快速环重构](2026-09-19-foc-fast-loop-refactor.md)：状态快照"前台按需组装"的决策
  在本设计中原样保留（S3 只搬家不改语义）。
- [统一运行状态机](2026-09-19-unified-run-state-machine.md) 2b / [E 对接清单](2026-09-19-e-framework-interface-prep.md)
  任务 2：CAN 命令准入最终归 E 统一准入；S3 的 `can_command_binding` 是其唯一重定向点，
  本设计不改变线上编号（兼容映射表由任务 2 另出）。
- [接口契约补全](2026-09-18-interface-contracts.md)：S1/S2 修改的公共头必须补中文契约。

## 10. 回滚与临时路径

- 每阶段独立提交、独立 revert；不引入临时 shim（`delay` 的阻塞语义按 T3 决策点单独处理）。
- 门面 `interface_can.{c,h}` 长期保留（公共签名稳定），若后续评估删除须另行计划。
- 任何阶段无法通过验收时：先 revert 该阶段，保留上一阶段绿灯状态，再复盘。

## 11. 验收与证据（待填）

| 阶段 | 证据 | 状态 |
| --- | --- | --- |
| S0 | 基线表（debt 11/4 条、Keil Code=81888 ZI=31352 0/0、CAN 夹具 3 组 PASS） | ✅ 完成 |
| S1 | debt 11→9、双目标 0/0、CAN 夹具 | 待启动 |
| S2 | 黄金向量逐字节、`check_interfaces` | ✅ 完成 |
| S3 | 双目标 0/0、全套夹具、等价核对 | ✅ 完成 |
| S4 | 隐藏 extern 归零、恢复/参数夹具 | ✅ 完成 |
| S5 | 裁决记录（如执行） | 未决（默认不执行，待用户确认） |
| S6 | `check_architecture` 通信 0 条、文档更新 | ✅ 完成（D4 待裁决） |

## 12. 执行记录

### 提交记录（2026-09-19）

| 阶段 | 提交 | 说明 |
| --- | --- | --- |
| S1 (=T3) | `c39d0d25` | 随"retire HIL target and close hardware boundary to platform layer"提交（含本计划的 comm_hw 契约与端口实现）。 |
| S2 | `7de7b7c1` | `refactor: extract CAN wire conventions and share fixed-point codecs` |
| S3 | `b5eefc08` | `refactor: split CAN implementation into transport, binding and status modules` |
| S4 | `ed78c8fa` | `refactor: drop the hidden CANMsg extern from the run state machine` |
| S6 | `680ade8f` | `docs: record the communication layering refactor and refresh its boundary reference` |

提交后复验：PR 档全绿 `outputs/runs/20260919T114553882298Z-680ade8f/summary.json`；
Keil 双目标 0 Error / 0 Warning（Code=81976 / RO=4888 / RW=252 / ZI=31348）。

S3/S4 的提交边界说明：`interface_can.{c,h}` 在 S3 被整体重写（同时承载 S2 的去重结果），
故 S2 提交只含协议约定模块与 `can_motor_status` 迁移；`CAN_IsHeartbeatAlive()` 随 S3 提交落地，
其消费方 `foc_run_state.c` 在 S4 提交切换。逐阶段回退仍成立（每个提交都可独立构建）。

### S0 基线与冻结（2026-09-19 完成，未提交）

- 改动文件：仅本计划文档（§4 基线回填、§11 状态、本记录）；未动任何代码。
- 证据：
  - `check_architecture`：`architecture: PASS (11 known, 0 new, 0 resolved)`。
  - Keil normal：`0 Error(s), 0 Warning(s)`，Code=81888 / RO=4888 / RW=248 / ZI=31352。
  - quick 档：`outputs/runs/20260919T073520384790Z-dda0c780/summary.json`（PASS）。
  - 夹具：`outputs/tests/s0_can/`、`s0_wheel/`、`s0_runstate/`（均 PASS）。
- 说明：`uv` 不在 PATH，按 §14 使用 `.\.venv\Scripts\python.exe`；Keil 需显式
  `--uv4 C:\Keil_v5\UV4\UV4.exe`（`KEIL_UV4` 未设置）。

### S1 硬件契约化（=T3，2026-09-19 完成）

- 说明：本阶段由并行会话收尾并随 `c39d0d25` 提交；本会话产出了其中的契约与端口实现。
- 改动文件：`platform/api/comm_hw.h`（+`comm_hw_can_start`/`comm_hw_can_set_baudrate`/
  `comm_hw_can_try_send_reply` 三个中文契约能力）、新增
  `platform/stm32g4/ports/comm/comm_control_stm32g4.c`（滤波/启停/波特率/TX 头组装下沉）、
  `communication/can/interface_can.{c,h}`（删 `fdcan.h`/`delay.h`/`hw_conf.h`/`main.h`，
  改调 comm_hw 与 `time_hw`，补 7 个公共函数中文契约）、
  `MDK-ARM/Vector_Mini_ST.uvprojx`（登记新端口源）。
- 证据：`check_architecture` 通信 4 条 → 0（总债务 9 → 5）；Keil 双目标 0 Error / 0 Warning
  （Code=81980 / RO=4888 / RW=248 / ZI=31352）；CAN 夹具全绿。

### S2 协议约定抽取（2026-09-19 完成）

- 改动文件：新增 `communication/protocol/can_parameter_wire.{c,h}`（`CAN_PARAM_ID` 注册表、
  `CanValueEncoding`、`CanParamWire_CommandEncoding/ReplyEncoding`、
  定点四件套 `Milli32/Centi32/Milli16/Centi16`、`Identifier`/`Length`）；
  `interface_can.h` 改为 include 新头；`interface_can.c` 删除本地编码/转换副本、
  改用 `CanParamWire_Length` 做长度门、`CanParamWire_Identifier` 组装 ID，并清掉死引用
  `extern ModeLast` 与无人消费的 `rx_data_u8` 拷贝；`can_motor_status.c` 改用统一转换器；
  `can_motor_status.h` 删除 `CAN_MOTOR_STATUS_COMMAND/REPLY`（D2 默认值）；
  夹具 `run_can_status_tests.py` 改从新文件取真实函数源（不再用 stub）；uvprojx 登记新源。
- 证据：PR 档全绿 `outputs/runs/20260919T092320244690Z-70984926/summary.json`；
  Keil 双目标 0/0（Code=81692 / RO=4888 / RW=248 / ZI=31352，较 S1 −288 B 为去重收益）；
  `run_can_status_tests` 9 组 PASS（含 48 字节黄金帧逐字节一致）；
  `test_wheel_speed_limits`、`test_run_state`（46464 tick）、`test_position_config_cache`
  （25200 tick）、Python CAN 用例 8 项全 PASS。

### S3 实现分解（2026-09-19 完成）

- 新增文件（均为逐 token 搬移，公共签名与调用方零改动）：
  `can/can_transport.{c,h}`（波特率运行态与切换、帧级收帧过滤、应答 5 次重试、状态帧提交）、
  `can/can_command_binding.{c,h}`（`CANRxIRQHandler` + `CAN_ReceiveMessage_Update`
  前导握手与路由）、`can/can_binding_commands.{c,h}`（写路径 28 个 case）、
  `can/can_binding_queries.{c,h}`（读路径 60 个 case）、
  `can/can_status_source.{c,h}`（`CanStatus_BuildSnapshot` + `CanStatus_HeartbeatArmed`）。
  `interface_can.{c,h}` 收敛为门面：`CANMsg` 运行态、应答暂存 `CAN_SendMessage_Update`、
  心跳状态机 `CAN_DisConnect_Handle`、发送调度 `CAN_SendMessage`、`FDCAN1_Param_Init` 与
  `param_comm_bridge` 实现；`CANMsg.baudrate` 字段删除（所有权归 `can_transport`）。
- 拆分后体量（目标 ≤300 行）：interface_can.c 139、can_transport.c 68、
  can_command_binding.c 113、can_binding_commands.c 207、can_binding_queries.c 272、
  can_status_source.c 54（拆分前单文件 866 行）。
- 偏差记录：`can_transport` 只承载"波特率运行态 + 帧级过滤 + 发送重试"，
  滤波/启停/中断使能的机制在 S1 已下沉 `platform/stm32g4/ports/comm`，
  故未再引入一层 pass-through 的 HAL 包装。`CANMsg` 保持为通信层公开运行态结构
  （声明集中在 `interface_can.h`，通信层外无隐藏 `extern`）。
- 夹具迁移：`run_can_status_tests.py` 的切片目标改到新文件并编译真实
  `CanTransport_ReceiveFrame/SendReply/TrySendStatus`、`CanStatus_BuildSnapshot/HeartbeatArmed`；
  `test_wheel_speed_limits.py` 的 `command_case` 改从 `can_binding_commands.c` 取
  `CanBinding_ApplyCommand` 的 case 块；uvprojx 登记 5 个新源。
- 证据：PR 档全绿 `outputs/runs/20260919T092908988818Z-70984926/summary.json`；
  Keil 双目标 0/0（Code=81960 / RO=4888 / RW=252 / ZI=31348；RW+4/ZI−4 为波特率运行态
  从 ZI 迁到带初值的 RW）；`run_can_status_tests` 9 组 PASS（含 48 字节黄金帧逐字节）；
  `test_wheel_speed_limits`、`test_run_state`（46464 tick）、`test_position_config_cache`
  （25200 tick）、`run_position_servo_tests` 全 PASS。

### S4 耦合显式化（2026-09-19 完成）

- `foc_param.c` 的隐藏 `extern CANMsg` 已由 `services/parameters/param_comm_bridge.h`
  窄桥替换（节点身份与心跳超时读写经 `CAN_NodeId_Get/Set`、`CAN_HeartbeatMs_Get/Set`）。
- 本阶段新增只读 `CAN_IsHeartbeatAlive()`（`interface_can.{c,h}`），
  `app/foc_run_state.c` 删除局部 `extern CANMsg_TypeDef CANMsg` 并用它等价替换恢复判据
  （原式 `can_hb_set > 0 && can_hb_count < can_hb_set`）。
- 偏差记录：桥接口采用"服务声明、通信实现并持有存储"的方向（与 §6.3 的"存储迁往
  services"相反），因为 `CANMsg` 是 CAN 运行态且节点的线槽位、滤波与限速副作用都依赖它；
  两种方向都能满足本阶段验收"隐藏 extern 归零"。
- 证据：`git grep "extern CANMsg_TypeDef" -- firmware tests` 仅剩 `interface_can.h` 的
  正式声明（通信层外 0 处）；Keil 双目标 0/0（Code=81976）；`test_run_state`
  差分 46464 tick 一致、恢复矩阵夹具 PASS；
  PR 档全绿 `outputs/runs/20260919T092930298304Z-70984926/summary.json`。

### S6 门禁与收尾（2026-09-19 完成，D4 待裁决）

- `architecture_debt.json` 通信条目为 0（总债务 5 条，全部为 motor→services 方向项）。
- `docs/architecture/communication_layering.md` 更新为落地后的三层与文件归属；
  `docs/plans/tech-debt-tracker.md` 的 ARCH-001/ARCH-002 改写为当前真实债务并记录
  硬件域已收口。
- 未执行项：D4（是否为 `communication/transport` 单列 harness 允许面）按 §14 需用户裁决；
  S5 维持"默认不执行"。
- 证据：`check_architecture` = `PASS (5 known, 0 new, 0 resolved)`，通信条目 0；
  最终 PR 档全绿 `outputs/runs/20260919T093142547570Z-70984926/summary.json`
  （含 docs 链接检查、format、lint、project-layout、interfaces、hygiene、tests）。

## 13. 自评记录（2026-09-19）

按用户要求，本设计的评审由主会话自行完成（未使用外部评审代理）。结论：**设计成立，
无阻断项**。要点：

1. R1 无位等价风险（§8），S2 可直接执行；
2. S4 归属迁移的三条数据路径已核对，值/次序/副作用全部保持（§6.4）；
3. S5 建议维持 ISR 内派发（§8 D3），"默认不执行"不变；
4. 夹具与路径的爆炸半径已具体到行（§6.4、§7）；`project_paths.py` 确认无需改动；
5. 唯一新增的机械动作是 S1 中 `interface_can.h` 的基础类型显式化（§6.4 第 1 条）；
6. 已撤销外部评审任务，未引入任何代理结论。

## 14. 执行交接（给执行会话）

本节是自包含的执行须知。执行者只需按本文件 §1–§11 施工；遇到与下述冲突之处，以本节为准。

**开工前置**

1. 读 §1 意图、§3 不变行为、§5 目标结构、§6 阶段、§7 夹具清单、§8 决策点、§11 验收；
2. 读 [通信分层说明](../../architecture/communication_layering.md)与
   [硬件边界收口](2026-09-19-hardware-boundary-closure.md)（S1 = 其 T3，不得重新设计）；
3. 顺序 S0 → S1 → S2 → S3 → S4 → S6；**S5 不执行**。

**硬约束（违反即回退该阶段）**

- 冻结线上 ABI：ID 编址 `node<<8|param`、0x00–0x6F、0x64/0x65、48 字节大端、版本 2、
  Mode 0..19 / Error 0..17、schema 11；不改协议、不加命令、不改单位与哨兵；
- 公共签名不变：`FDCAN1_Param_Init`、`CAN_BaudRateSwitching`、`CAN_DisConnect_Handle`、
  `CAN_SendMessage`、`CANRxIRQHandler`、`CAN_ReceiveMessage_Update`、`CAN_SendMessage_Update`；
- `firmware/platform/stm32g4/**` 之外不得出现 HAL 类型/句柄；不动 `cubemx/**` 生成区；
- 工作区存在大量既存未提交改动（HIL 退役等）：**禁止** `git checkout/clean/reset`，
  只修改本计划范围内的文件；
- 未经用户明确要求**不要 git commit**（§6 的"每阶段一提交"需先征得同意）；
- 新增/变更公共头按 [接口契约补全](2026-09-18-interface-contracts.md)补中文 Doxygen 契约；
- S4 必须在统一运行状态机 2b 于 `foc_run_state.c` 收敛后执行。

**决策点**：S5、D4 必须先问用户；D1/D2 采用 §8 默认值；其余按 §8 说明。

**环境**：`uv` 不在 PATH，使用 `.\.venv\Scripts\python.exe tools\run.py verify --profile quick`
或 `--profile pr`（离线全量）。Keil 构建（`build_firmware --target normal`）不可用时如实记录，
不得声称已构建；HIL 目标已退役，不得恢复。

**汇报**：每阶段更新 §11/§12（改动文件、测试结果、证据路径），S1/S3/S4 结束跑 PR 档；
全部完成后产出交接报告（完成阶段、证据路径、偏差、未决决策点）。没有绿色证据不得声称完成。
