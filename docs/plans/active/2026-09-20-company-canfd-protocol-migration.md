# 公司 CAN FD 通信协议框架重构计划 v1.0

日期：2026-09-20。状态：**规划阶段，未修改固件运行行为**。

本计划用于把公司通信协议的主体框架先搭稳，再按阶段迁移当前固件。它覆盖协议核心、CAN FD
传输、命令路由、电机控制、状态反馈、参数、升级和后续多轴同步。每一阶段都有独立边界、验收
证据和回滚点，后续可以把工作包交给不同子代理并行实施。

## 1. 目标与当前基线

目标是让电机控制服务只看到平台无关的命令和状态对象，让 CAN FD 只负责承载这些对象。协议
核心不能依赖 STM32 HAL、FDCAN 句柄或电机控制全局变量。

当前代码与设计稿的关系必须先固定：

- 固件当前实现的是 11 位标准 CAN、参数命令和 48 字节大端状态流；入口在
  `firmware/communication/can/` 与 `firmware/communication/protocol/`。
- 公司 CAN FD 设计稿是 29 位扩展 ID、公司头部、CRC8/CRC16、分片和升级流程，当前固件还未实现。
- 当前 `platform/api/comm_hw.h` 及 STM32 端口仍以旧协议的标准 ID 和发送接口为主。
- 现有旧协议必须在迁移完成前保持可回滚；新协议不能通过悄悄改变旧接口来接入。

依据：[通信分层说明](../../architecture/communication_layering.md) 和
[公司协议源代码回读](../../protocols/company_protocol_source_readback.md)。

## 2. 目标架构

```text
CAN FD BSP/port（HAL、过滤器、位时序、硬件错误）
        │  CommHwCanFrame：ID/FD/BRS/DLC/数据
        ▼
CAN FD transport（队列、DLC、收发、统计、超时）
        │  无硬件的物理帧对象
        ▼
yg_protocol_frame_codec（公司头部、CRC、分片/重组、长度/版本）
        │  ProtocolMessage
        ▼
yg_protocol_message_registry + yg_protocol_router（类型、方向、序号、错误、应答）
        │  平台无关命令/快照
        ▼
Command gateway / services
   ├─ 电机控制：STOP、DISABLE、ENABLE、模式、目标、看门狗
   ├─ 状态反馈：位置、速度、电流/力矩、模式、故障、温度、电压
   ├─ 参数与诊断：查询、写入、保存、故障和标定任务
   └─ 升级服务：会话、分片、校验、切换、回滚
        │
        ▼
Motor control / parameter store / loader
```

各层职责：

| 层 | 负责内容 | 明确禁止 |
| --- | --- | --- |
| 平台端口 | HAL 初始化、FDCAN 过滤器、位时序、硬件错误和实际收发 | 放入协议字段、命令语义或电机状态判断 |
| CAN FD transport | 扩展 ID、DLC 映射、队列、非阻塞收发、统计 | 直接调用 `MotorControl` 或解释 payload |
| 公司 wire codec | 小端序列化、头部、CRC、帧长、分片/重组 | 访问 HAL、全局电机状态或动态内存 |
| yg_protocol_router | 版本、类型、方向、src/dst、sequence、会话和错误路由 | 修改电机控制变量 |
| command gateway/services | 参数校验、权限、控制租约、模式切换、状态快照 | 依赖 STM32 头文件或直接操作 FDCAN |
| 电机控制/loader | 唯一的控制状态写者、Flash 和升级状态机 | 反向读取通信队列或解析线路帧 |

## 3. 必须冻结的协议不变量

以下内容进入协议契约和黄金向量，任何变化都必须升版本并补兼容说明：

1. 公司应用头部为 16 字节，含 magic `0xA55A`、version、flags、src、dst、type、sequence、
   payload length、reserved 和 CRC8；payload 后有 CRC16。
2. 线上数值统一显式小端序列化，不使用 C 的 packed struct 直接映射；CRC16 低字节先传。
3. CRC8：多项式 `0x9B`、初值 `0x00`；CRC16：多项式 `0xBAAD`、初值 `0xFFFF`。CRC 覆盖范围
   以公司协议最终裁决为准，并在向量中逐字节锁定。
4. CAN ID 只承担路由和优先级。当前设计采用 29 位扩展 ID、J1939 风格的 priority/R/DP/PF/dst/src
   布局，`PF=0xEF`；应用 `type`、`sequence` 和 payload 长度继续放在公司头部，不能用 CAN PF
   再定义一套冲突的 command ID。
5. 物理帧的 CAN/CAN FD 控制器 CRC 不重复放入应用 payload；应用 CRC 只校验公司应用帧。
6. 实时控制消息必须在单个 CAN FD 数据帧内完成，避免实时环路分片；管理、诊断和升级消息才允许
   分片和重组。重组缓存有固定上限、超时和错误计数。
7. 接收中断只做有界取帧和入队；解析、路由和服务调用在通信任务或明确的 1 kHz/2 kHz 调度点完成。
   禁止 ISR 动态分配、阻塞和 Flash 操作。
8. 控制命令必须带 sequence、时间/租约语义和目标模式；断连、序号回退、模式不匹配和看门狗超时
   都有明确的拒绝或安全停机结果。
9. Node ID 采用公司的节点注册表。现有源码读回的节点号属于公司约定；新 type 编号和优先级映射
   在公司登记前只能标为项目候选，不能写成“已批准”。

## 4. 内部数据契约

协议层和电机层之间使用独立对象，避免把线路帧直接传给控制器。第一版建议最少建立以下对象：

```text
ProtocolFrame       = transport metadata + company header + payload + CRC result
ProtocolMessage     = version/src/dst/type/sequence/flags + bounded payload view
MotorCommand        = source/sequence/mode/target/limits/lease/execute time
MotorStatusSnapshot = timestamp/state/mode/fault/position/speed/current/torque/temp/voltage
ServiceResult       = accepted/rejected + error code + applied sequence + optional detail
```

`MotorCommand` 和 `MotorStatusSnapshot` 不包含 HAL 类型、CAN ID 或指向可变全局变量的指针。控制服务
是命令的唯一写者；状态服务产生只读快照。线路单位、缩放和哨兵值在 codec/registry 中集中定义，
不能散落到电机算法文件。

## 5. 分阶段实施

### Stage 0：基线、决策和兼容矩阵

先记录旧协议行为、当前未提交改动和设计稿中的未决项，形成一张兼容矩阵：旧标准 CAN、公司 CAN FD
基础帧、实时控制帧、管理分片帧和升级帧分别列出 ID、长度、字节序、CRC、时序和失败行为。

必须在进入代码前裁决：公司头部 CRC8 的覆盖范围、payload CRC16 的覆盖范围、`len` 是否包含 CRC、
ACK/错误码位定义、节点注册表、type 登记、标称速率与数据速率、升级镜像格式和 loader 边界。

验收：只增加文档和黄金向量，不改变固件；旧协议基线测试通过；所有未决项有责任人和决策记录。

### Stage 1：协议核心框架（第一批实现）

新增纯逻辑的公司 codec，先不替换运行时旧路径：

- `yg_protocol_wire_types`：版本、flags、消息类型、错误码、受限长度和结果码。
- `yg_protocol_crc`：CRC8/CRC16，覆盖范围和边界测试固定。
- `yg_protocol_frame_codec`：头部、payload、CRC 的 encode/decode；拒绝短帧、超长、未知版本、坏 CRC、
  非法 src/dst 和整数溢出。
- `yg_protocol_can_id`：29 位 ID 的 priority/R/DP/PF/dst/src 编解码；不放入电机命令语义。
- `yg_protocol_fragment`：仅供管理和升级使用的固定缓存、序号、总长、超时和重组错误。
- 主机/native 黄金向量和负例测试；已有 `host_app/examples/company_canfd_position.c` 作为参考向量，
  最终应与正式 codec 共用契约，避免出现两份定义。

验收：codec 可在无 STM32 头文件的 native 环境独立构建；黄金向量逐字节一致；错误路径和边界路径
全覆盖；固件旧通信行为不变。

### Stage 2：CAN FD transport 和平台端口

在现有 `CommHwCanFrame` 基础上**增量**扩展能力，避免破坏旧调用方：

- 帧描述增加 `extended`、`fd`、`brs`、29 位 `identifier`、实际长度和远程帧语义；发送描述使用
  `uint32_t identifier` 和显式数据长度。
- STM32 端口负责 FDCAN FD 格式、BRS、DLC 映射、过滤器、启动停止、总线错误和队列提交；HAL 只在
  `platform/stm32g4/ports/comm` 内出现。
- 第一版按硬件验证结果配置标称速率和数据速率。当前设计稿为数据段 5 Mbit/s；标称段先保持现有
  可验证配置，不在没有示波器/板级证据时擅自改为 1 Mbit/s。
- transport 提供有界 RX/TX 队列、丢帧/CRC/过滤/总线错误计数和非阻塞失败码。

验收：native transport mock、板级 loopback 或授权台架能证明 29 位扩展 ID、FD、BRS、DLC、队列满和
总线恢复；未接入电机控制。

### Stage 3：消息路由器与只读服务

建立 `yg_protocol_message_registry` 和 `yg_protocol_router`，完成版本、方向、src/dst、sequence、长度、权限和
应答错误码检查。第一批只做无副作用消息：心跳、设备信息、能力、协议版本、读取电机状态。

路由器输出 `MotorStatusSnapshot` 或 `ServiceResult`，不能直接写 `MotorControl`。所有应答经过
统一 encoder，避免各命令自行拼 CAN 数据。

验收：主机发送坏版本、错节点、坏 CRC、重复 sequence、未知 type，设备均返回确定结果；状态快照
和旧状态源对照一致；控制输出不发生变化。

### Stage 4：最小电机控制闭环

按安全顺序接入控制命令：

1. `STOP` / `DISABLE`：任何允许状态都能安全执行，确认输出已关断。
2. `ENABLE`：只在无锁存故障、参数有效、控制租约有效时接受。
3. `SET_MODE`：模式枚举和允许转换由电机状态机校验。
4. `SET_TARGET`：统一位置/速度/力矩目标结构，按模式解释字段；不为每个模式复制整套命令。
5. watchdog/lease：序号、来源和到期时间绑定；超时进入定义的安全状态。

通信服务只调用现有电机状态机或新增窄网关接口，不能写 FOC 全局量。每条命令记录 accepted、
rejected、reason 和 applied sequence，方便主机判断是否真正生效。

验收：单轴仿真和台架覆盖模式切换、重复包、乱序包、断连、故障锁存、停止优先级和恢复；控制
周期、故障行为和输出边界有证据。

### Stage 5：反馈、参数和诊断

- 反馈帧统一携带位置 `int32`、速度 `int16`、电流/力矩 `int16` 及状态、模式、故障和时间戳；CAN FD
  数据区足够时合并为一帧，避免把一个快照拆成三条相互不同步的消息。
- 状态发布使用快照时间和序号，主机能判断丢帧和旧数据；反馈调度与控制调度分离但共享时间基准。
- 参数服务提供 GET、SET、SAVE、DEFAULT、版本和范围查询；Flash 写入只在服务/loader 的受控上下文
  完成，通信 ISR 永远不写 Flash。
- 诊断/标定采用有界 job 状态机，返回 job id、阶段、进度和错误，不把长任务塞进单次请求。

验收：位置/速度/电流单位、缩放、端序、饱和和无效值有黄金向量；掉电/写失败/参数越界行为可验证。

### Stage 6：升级与 loader

升级作为独立服务和状态机实现，复用 Stage 1 的 codec/fragment，但不复用电机控制路径：

`BEGIN → ERASE/READY → DATA(offset, sequence) → VERIFY(hash/signature) → COMMIT → REBOOT/ROLLBACK`。

必须定义镜像元数据、最大镜像、块大小、窗口/重传、断电恢复、版本回退、loader/app 地址边界和
完整性/真实性校验。通信层只提交受控的 loader API；不得在 RX/TX 回调中擦写 Flash。

验收：断包、乱序、重复、掉电、坏 hash、版本回退和空间不足均进入安全结果；升级失败不会启动
损坏镜像；旧版本兼容窗口和恢复路径有主机脚本与台架证据。

### Stage 7：5 轴、1 kHz 双向同步

这是容量和时序验收阶段，不能从“CAN FD 帧长足够”直接推导出可实现。先完成单轴和总线测量，再做
至少 5 轴：

- 控制端发送组位置/速度目标，携带未来执行时间或同步计数；各轴本地排队并按同一时间基准执行。
- 每个轴每 1 kHz 发布状态快照；反馈包含位置、速度、电流/力矩、状态、故障和时间戳。
- 测量标称仲裁段利用率、数据段利用率、最坏排队延迟、抖动、同步误差、丢帧和重传；仲裁段不能
  直接使用 5 Mbit/s，速率上限取决于所有节点收发器、布线、采样点和 CAN FD 物理层配置。
- 若单总线无法同时满足 5 轴双向 1 kHz，必须在架构评审中选择减少载荷、分组反馈、双总线或提高
  标称速率等方案，并以实测数据决定，不能在协议文档中先承诺。

验收：5 轴控制和每轴反馈均达到 1 kHz 目标；记录每轴端到端延迟、周期抖动和同步误差；停止、
故障和总线降级仍满足安全要求。

### Stage 8：兼容窗口与旧协议退役

新协议稳定后，保留明确的旧协议兼容窗口：主机工具切换、版本协商、日志和故障码迁移完成后，才
关闭旧 parser 和旧发送接口。退役前必须有兼容矩阵、现场升级/回滚方案和最终协议登记。

## 6. 工作包与后续委派边界

每个工作包拥有独立文件范围和验收，不把同一个公共头交给多个执行者同时修改：

| 工作包 | 范围 | 依赖 | 不负责 |
| --- | --- | --- | --- |
| WP-A 协议契约 | type/Node 注册、头部、CRC、错误码、黄金向量、文档 | Stage 0 | HAL、FOC、Flash |
| WP-B 纯 codec | encode/decode、分片、native 测试 | WP-A | 端口和电机状态 |
| WP-C CAN FD port | `comm_hw` 增量接口、STM32 FDCAN、mock、统计 | WP-A；可与 WP-B 并行 | 命令语义 |
| WP-D router/gateway | registry、路由、`MotorCommand`、只读服务 | WP-B、WP-C | 直接改 FOC |
| WP-E 电机适配 | 状态机接入、模式/目标/租约/反馈快照 | WP-D | 帧解析、HAL |
| WP-F loader/升级 | 升级状态机、存储接口、镜像校验和回滚 | WP-B；可与 WP-E 后段并行 | 电机实时控制 |
| WP-G 验证/工具 | 主机工具、回归、HIL/台架、容量和时序报告 | 持续跟随全部工作包 | 修改生产协议语义 |

建议的并行方式是 WP-A 先冻结契约；WP-B 与 WP-C 在接口冻结后并行；WP-D 等两者通过；WP-E 与
WP-F 分开施工；WP-G 从第一天开始维护向量和回归。后续若派发子代理，按此表分配文件所有权，
由主任务合并公共契约和验收结果。

## 7. 验收门槛

| 门 | 必须证明 | 允许进入下一门 |
| --- | --- | --- |
| G0 基线 | 旧协议、当前分支改动和设计稿差异已记录 | Stage 1 |
| G1 codec | 纯 native 构建、正负黄金向量、边界/CRC/版本测试 | Stage 2 |
| G2 transport | 扩展 ID、FD/BRS、DLC、队列、错误恢复和速率证据 | Stage 3 |
| G3 只读 | 版本/能力/状态读取及错误应答稳定 | Stage 4 |
| G4 控制 | 单轴安全控制、租约、序号、故障和停止行为稳定 | Stage 5 |
| G5 参数/反馈 | 快照、单位、参数事务和 Flash 失败路径稳定 | Stage 6 |
| G6 升级 | 完整性、断电、回滚和 loader 边界稳定 | Stage 7 |
| G7 多轴 | 5 轴双向 1 kHz 的延迟、抖动、同步误差和利用率有实测证据 | Stage 8/发布评审 |

协议/参数 ABI 改动必须同步更新版本化文档、黄金向量、兼容说明和测试。固件逻辑、协议、构建输入
或测试改动使用 PR 验证；仅计划文档改动使用 quick 验证。

## 8. 风险、取舍和回滚

- **旧协议并存风险**：新旧 parser、ID 过滤和发送队列可能互相影响。解决方式是显式版本/模式和独立
  路由，直到兼容窗口结束前不删除旧路径。
- **公司协议未决项**：CRC 覆盖、长度定义、type 登记和 Node ID 必须在 Stage 0 锁定；未锁定时只做
  codec 试验，不接入电机控制。
- **位时序风险**：数据段 5 Mbit/s 不代表仲裁段可用同速率。所有速率变更需要板级、收发器、布线
  和示波器/错误计数证据。
- **实时风险**：1 kHz 目标同时受总线利用率、主机调度、本地控制周期和时钟同步影响；Stage 7 前不
  承诺 5 轴性能。
- **边界风险**：`comm_hw` 当前接口是旧协议语义。采用新增能力或版本化接口，逐步迁移调用方，禁止
  直接改参数含义。
- **Flash 风险**：升级和参数写入必须通过 loader/storage 服务，有界、可中断、可恢复；通信路径不得
  直接擦写 Flash。

每个阶段独立提交、独立验证、独立回滚。任何门槛失败时回退该阶段，保留上一个绿色阶段；不得用放宽
断言或删除旧测试来“通过”。本计划当前不创建提交，不改现有固件行为，不访问硬件。

## 9. 当前进度与下一步

- [x] 记录当前旧协议与公司 CAN FD 设计稿的差异。
- [x] 固化目标分层、内部对象和工作包边界。
- [x] 固化 Stage 0–8 的实施顺序、验收门槛和回滚规则。
- [ ] Stage 0 决策记录：CRC 覆盖、len 语义、ACK/错误码、Node/type 注册、位时序和升级元数据。
- [ ] Stage 1：把公司 wire codec、CAN ID codec 和 native 黄金向量整理成正式公共契约。

推荐下一次实施从 Stage 1 开始，只新增纯协议模块和测试，不替换旧 CAN 路径；Stage 1 通过后再进入
CAN FD transport。这样可以先验证“公司协议帧能否正确编码/解码”，再把硬件和电机控制接上。

## 10. 证据记录

本计划建立时只做了仓库和文档审计，未运行硬件、未刷写固件、未修改电机控制行为。后续每个阶段在
本节追加验证命令、`outputs/runs/<run>/summary.json`、黄金向量版本、板级测试结果和已知偏差。
