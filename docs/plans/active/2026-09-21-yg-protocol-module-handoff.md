# yg_protocol 模块开发交接单 v0.1

日期：2026-09-21。状态：任务划分与交接准备；本文件没有启动其他开发任务。
目标：让电机、参数、任务、升级开发者使用同一接口基线独立开发，由集成人员统一接线。

## 1. 先准备共同基线

必读：仓库 `AGENTS.md`、`STYLE.md`、
[核心接口契约](../../protocols/yg_protocol_core_interfaces_v0_1.md)、
[技术实现规范](../../architecture/communication_protocol_refactor_spec_v1.md)、
[迁移计划](2026-09-20-company-canfd-protocol-migration.md)。任务只需额外阅读自己负责的公共头。

当前核对结果：不少 `yg_protocol_*` 源码和测试仍是未跟踪文件，其余文档存在暂存/未暂存修改。
不要让其他工作树直接从当前 HEAD 开始，否则不会包含这些接口。开始独立开发前，由集成人员
审查并形成一次只包含本任务必要文件的基线提交，记录完整 commit；保留无关用户改动。
如果通过文件包交接，则包含源码、测试入口、文档和哈希清单，不能只发送头文件或普通 git diff
（普通 diff 不包含未跟踪文件）。本轮不自动创建提交或工作树。

接口 v0.1 是协作基线，不是公司正式发布协议，也不是全部语义已冻结。各任务开工先核对：

- motor 请求有 source/sequence/lease/execute_at，但尚未提供完整控制会话关联契约；不能从来源
  地址推断授权，也不能把外层帧序号当作控制命令序号。
- parameter 的 value 目前仅为 int32，revision 没有自动解决事务、权限或持久化失败语义。
- job/update 的 data 是借用字节视图，格式与最大长度需要任务明确；不能把任意 CAN payload
  直接转交业务所有者。reply 只有通用 token/value/detail，复杂结果需要明确的查询契约。
- ACK/去重、绝对超时、原子采样、ISR 同步及硬件收发尚未全部实现；接口存在不等于这些能力可用。

有缺口时先交付“缺少哪个字段/语义、真实调用者、建议变更和兼容影响”，由接口维护者统一修订。
任务可继续完成不依赖该决定的离线处理器测试，不能自行改公共头或伪造默认语义。

## 2. 按业务所有者分工，不按 CAN 命令各写一套

```text
通信负责人：CAN FD → codec → router → 协议/业务适配器
                                      ↓ 绑定 handler + context
                              业务模块公开的窄接口
                                      ↓
                       电机 / 参数 / Job / Loader 所有者
```

通信侧适配器实现 `yg_protocol_*_handler_t`，负责线路值对象到业务对象的转换、结果映射。
业务所有者负责状态校验、限位、执行与生命周期；它的公共头不依赖 `yg_protocol_*`、CAN 或 HAL。
在 app 组合层绑定适配器及其 context。不能让每个业务模块自行解析 CAN 帧，也不能为接入协议
再实现一套与现有控制状态机竞争的状态机。

| 开发任务 | 读取的通信接口 | 本任务交付 | 第一批边界 |
| --- | --- | --- | --- |
| M 电机处理器 | `yg_protocol_motor.h`、`yg_protocol_motor_status.h` | 复用现有控制所有者的窄入口、状态/故障映射、稳定快照契约及 native 测试 | 优先 STOP/DISABLE，再 ENABLE/MODE/TARGET；不直接改 FOC 全局量 |
| P 参数处理器 | `yg_protocol_parameter.h` | 既有参数的读取、单位/范围/生效策略、版本与保存接口分析、native 测试 | 首批只读；不把描述符设计稿当成已实现代码，不顺带重构整个参数系统 |
| J 任务处理器 | `yg_protocol_job.h` | 对一个已有标定或辨识任务提供 start/cancel/query/result 窄接口 | 有界推进、可取消、候选结果；不一次接入全部任务 |
| U 升级处理器 | `yg_protocol_update.h` | Loader 请求/存储边界分析和内存存储替身下的有界升级状态机 | loader 当前只有规划入口；先离线，不引入 MCU Flash 操作或声明可实机升级 |
| C 通信核心 | endpoint/router/registry/fragment/transfer | 分片与重传策略、请求准入、长响应、各业务适配器 | 公共协议字段统一维护，不能随单个处理器需求改线上编号 |
| I 集成与验证 | 全部核心契约 | 接线、测试登记、构建输入、兼容证据；后续才做实机验证 | 唯一修改公共组装入口的任务；最终合并后重新验证 |

建议第一批并行执行 M、P 和 C 的离线工作；J 在现有任务生命周期稳定后接入，U 在镜像/存储
边界确认后实现。I 持续做接口评审。第一轮已经按此边界启动，结果见第 7 节。

## 3. 文件所有权与集成方式

派发单必须明确到文件，不使用“可以修改整个 firmware”这样的授权范围。

- C/I 共同指定一名维护者，独占 `firmware/communication/**/yg_protocol_*` 公共头、注册表、
  route 表、CAN 端口与组装入口；同一文件同一时间只有一个修改者。
- M/P/J/U 首轮只修改任务中列出的模块内新增处理器文件、独立测试和自己的计划文件。新文件名
  由派发单确定；如果必须修改已有控制/参数/任务实现，先列出现有调用者与保持不变的行为。
- 协议适配器放在通信侧，业务接口放在所属业务模块；Loader 保持独立构建。
- 每个任务使用独立工作树/分支，从同一基线开始。不要共享目录并行写同一套源码。
- 各任务提供独立测试入口；`tests/run.py`、公共 native 构建清单、Keil 工程由 I 统一登记，
  避免所有任务同时修改。独立测试必须实际运行，不能只依赖尚未登记它的 PR 测试。
- 现有参数描述符计划仅为设计，另行实施须单独列入范围；不能把本交接当成对其全量实施的指令。

## 4. 可直接复制给其他模块/AI 的任务模板

```text
任务：M-1 电机通信处理器接口对接（离线）
基线：由集成人员填写已包含 yg_protocol 核心及测试的完整 commit，不使用缺文件的 HEAD。
必读：AGENTS.md、STYLE.md、核心接口契约 v0.1，以及 yg_protocol_motor.h。

目标：复用现有电机控制所有者，给通信适配器提供非阻塞的 STOP/DISABLE 窄入口，
并输出其他三个操作所需的契约缺口。首批不切换现有 CAN 运行入口。

允许新增（建议文件名，派发时确定）：
- firmware/services/motor/motor_command_service.h
- firmware/services/motor/motor_command_service.c
- tests/unit/motor_command_service_test.c
- tests/unit/native/test_motor_command_service.py
- docs/plans/active/<日期>-motor-command-service.md

公共接口、旧 CAN、参数 Flash、CubeMX、FOC 和 tests/run.py 由集成人员维护。
确需修改已有文件时先提交最小变更提案与调用者分析，不自行扩大范围。

交付要求：
1. 先检查现有控制入口和所有者，复用它，禁止新增第二个状态机或隐藏全局依赖。
2. 明确请求/结果类型、单位、合法状态、上下文、最大工作量及所有权。
3. 无真实处理器用替身验证；未实现返回 UNSUPPORTED，不以空操作返回 OK。
4. STOP/DISABLE 成功需处理器确认输出禁止；异步等待用 ACCEPTED+token，不能假报完成。
5. 测试合法/非法状态、重复停止、处理器拒绝、输出禁止无法确认，保留失败证据。
6. 运行独立测试和 PR 验证，提交改动清单、证据路径、剩余项和回滚说明。
7. 不刷写、不转动电机，不自动合并或部署。

遇到公共接口缺口：指出具体字段、调用者和建议，不自行修改 yg_protocol 公共头。
```

P/J/U 使用相同模板，替换为对应公共头、独占路径和用例。P 首轮可以仅交付既有参数读取接口
与范围表；没有事务处理器时 WRITE/SAVE 返回 UNSUPPORTED，不能用 ACCEPTED 假装已进入队列。
J 验证取消及完成后结果生命周期；U 验证长度/偏移、重复块和校验失败，掉电恢复以实际范围为准。

## 5. 模块验收与交回材料

统一交回：基线commit、改动commit或包含新增文件的完整补丁、接口缺口、调用示例、独立测试
命令/结果、PR summary路径、未实现项、资源/时序的已知限制、回滚方法。

| 等级 | 证明内容 | 不能代替 |
| --- | --- | --- |
| 接口通过 | 头文件自包含、业务不依赖CAN/HAL、契约和所有权明确 | 处理器功能完成 |
| 离线处理器通过 | 替身/真实纯逻辑调用及错误/边界路径通过 | 硬件已执行 |
| 组合通过 | 真实协议解析→适配→处理器→应答串联，未实现能力不对外宣告 | MCU资源与时序验收 |
| 实机通过 | 独立硬件方案、当前构建/哈希、授权场景与采集证据 | 其他轴数/频率也已验收 |

文档交接改动用 `uv run python tools/run.py verify --profile quick`。
固件逻辑/协议/测试用 PR；构建输入、CubeMX、链接/发布变更按仓库规则执行对应检查，涉及硬件
台架的验证另行取得明确场景授权。I 每次合并后跑集成检查；子任务通过不自动等于合并结果通过。

## 6. 完成与回滚

本交接准备完成标准：交接文档已索引、文件边界可执行、模板可直接派发、quick 检查通过。
每个实际任务独立维护版本计划及验收证据；回滚优先按任务文件清单逐项删除或反向补丁，不能
恢复整个工作树覆盖其他并行改动。

## 7. 第一轮 DeepSeek v4.1-flash 执行记录

执行批次：`outputs/parallel/yg-wave-20260921T122913Z`。三个任务使用独立工作树，模型为
`opencode-go/deepseek-v4.1-flash`，主工作区只按允许文件清单集成。

| 任务 | 结果 | 主要交付 |
| --- | --- | --- |
| M | 已集成 | STOP/DISABLE 所有者窄端口、异步 token、停机确认和拒绝语义；未接真实 FocRunState/CAN |
| P | 已集成 | 参数只读快照服务，速度/电流限值与配置版本映射；WRITE/SAVE/RESTORE 保持 UNSUPPORTED |
| C | 已审查并集成 | 46B 分片、seq0、元数据一致性、重复片、片间/绝对超时和 poll；无 ACK/整消息重传 |

集成后执行 `uv run python tools/run.py verify --profile pr`，当前结果为 PASS，证据位于
`outputs/runs/20260921T134946510005Z-8982922a/summary.json`。随后已接入只读 CAN FD 诊断端点：
`yg_protocol_link` 通过唯一 RX 入口和 2 kHz 服务循环完成 GET_INFO/GET_CAPS 响应，新增 native
闭环证据 `tests/unit/native/test_yg_protocol_link.py`。STM32G4 的 `.ioc` 与 CubeMX USER CODE
覆盖配置为 1 Mbit/s 仲裁段、5 Mbit/s 数据段。下一轮仍需处理真实业务接线、
请求响应线上编码、参数写入事务、Job/升级处理器和至少 5 轴 1 kHz 时序实测。
