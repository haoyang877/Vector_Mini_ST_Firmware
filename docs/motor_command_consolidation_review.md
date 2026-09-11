# 电机控制命令合并评审

版本：1.3 建议稿；基线：1.2.3；更新日期：2026-09-08。

当前已确认仅 CAN FD，仲裁 500 kbit/s、数据 5 Mbit/s、BRS=1，全部使用公司完整帧。基线有效消息 65 种（63 请求、2 上报），已采用 type=124 单帧合并反馈；本文件仅评审进一步精简，未替换[当前命令表](motor_command_catalog.md)。

## 1 简化建议与数量

| 当前入口 | 新入口 | 减少 |
| --- | --- | --- |
| 103/104/105/131 四种标量目标 | 190 SET_TARGET(mode,value) | 3 |
| 143/151/161/171 四种任务启动 | 115 JOB_START | 3 |
| 144/152/162/172 四种任务中止 | 116 JOB_ABORT | 3 |
| 145/153/163/173 四种任务状态 | 117 JOB_GET_STATE | 3 |
| 164/174 两种结果读取 | 118 JOB_READ_RESULT | 1 |
| 108/133/185/186 四种物理状态查询 | 108 保留基础查询并增加页面 | 3 |

| 阶段 | 请求 | 上报 | 主消息合计 |
| --- | --- | --- | --- |
| 当前 1.2.3 | 63 | 2 | 65 |
| 统一目标和任务 | 50 | 2 | 52 |
| 再合并只读状态（建议） | 47 | 2 | 49 |

历史多链路建议中的 50 项包含经典 CAN 心跳 123；该编号已经退出当前范围，因此相同简化在 CAN FD-only 范围为 49 项。若保留全部线上兼容入口，登记仍为 74 项：49 主消息 + 18 控制兼容 + 3 查询兼容 + 4 历史退出编号；历史 120～123 不实现、不发送、不复用。

## 2 统一任务命令定义

以下编号经 1.2 注册表检查未使用，作为拟分配值，正式发布前仍需公司注册。均使用公司完整帧、Q/R 公共前缀、小端、单播、ACK_REQ=1；仅使用 CAN FD 公司完整单帧或分片承载。

| type | 名称 | 请求 payload | 成功响应 |
| --- | --- | --- | --- |
| 115 / 0x0073 | JOB_START | Q + job_id:u32, kind:u8, reserved:byte[3]，共 16 B | R，ACCEPTED 表示开始，非已完成 |
| 116 / 0x0074 | JOB_ABORT | Q + job_id:u32, kind:u8, reserved:byte[3]，共 16 B | R，ACCEPTED 表示停止中，终态已停止可 OK |
| 117 / 0x0075 | JOB_GET_STATE | Q + job_id:u32, kind:u8, reserved:byte[3]，共 16 B | R + 1.2 公共任务状态 24 B，共 36 B |
| 118 / 0x0076 | JOB_READ_RESULT | Q + job_id:u32, kind:u8, reserved:u8, first_item:u16, count:u16, reserved2:u16，共 20 B | R + job_id:u32, kind:u8, reserved:u8, first_item:u16, count:u16, reserved2:u16 + entries，每项 12 B |

kind 保留 1=轨迹、2=回零、3=标定、4=辨识。写操作的 kind 必须与 job_id 绑定对象匹配，不能只凭当前 mode 猜测任务。START/ABORT 的 job_id 必须非零；GET_STATE 的 job_id=0 且 kind=0 查询当前或最近任务，明确编号查询时 kind 必须匹配。JOB_READ_RESULT 的 job_id 必须非零，kind=2/3/4；轨迹在此版没有候选参数结果，kind=1 返回 UNSUPPORTED。

JOB_START 仍要求当前控制者、有效 lease、匹配 boot_id、任务 ARMED、设备 ENABLED、保活新鲜、无安全故障。任务已绑定同步组时，拒绝普通 START，必须通过组 COMMIT 启动。合并命令不合并“准备、使能、执行”三个动作。

JOB_ABORT 只终止指定任务，采用原有停止策略，不提供任意 force 字段越过产品安全策略；STOPPING 期间可查询进度。对同一个请求的重复处理返回已有结果，不再次初始化任务或重复制动。终态对象保留期间的新 ABORT 请求可幂等确认；已经被新任务替换的旧对象返回 STALE_REQUEST。

GET_STATE、READ_RESULT 可 session=0，仅查询不刷新看门狗；沿用只读旁路，不覆盖有副作用请求的去重结果。结果读取 count 上限为 min(8,max_result_items)，并满足 24+12×count≤路径 max_message_payload；不能沿用旧结果页的 20 B 固定开销。读到末尾可返回更少条目，first_item 等于总数返回 count=0，大于总数返回 RANGE。

结果条目仍使用 1.2 的 item_id、value_type、valid、value、quality_permille 和 reserved，不重新编号。回零通过 kind=2 明确读取机械零偏，避免继续借用名为 READ_CALIB_RESULT 的专用接口。APPLY_TASK_RESULT 165 保持原样，用于显式应用结果和可选持久化。

## 3 标量电机目标合并

拟分配新 type=190 / 0x00BE，名称 SET_TARGET。它只覆盖旧 103、104、105、131 的标量目标，不覆盖完整阻抗 132。业务 payload 仍为 8 B，并显式带模式，避免同一个整数在模式改变后被误当作另一种单位。

| payload 偏移 | 类型 | 字段 | 规定 |
| --- | --- | --- | --- |
| 0 | u8 | target_mode | 1=Iq，2=速度，3=位置，4=基础阻抗位置 |
| 1 | u8 | command_seq | 沿用 8 位新鲜度规则 |
| 2 | u16 | lease_tag | 当前控制会话短标识 |
| 4 | i32 | target | 按 target_mode 解释单位 |

mode1 为 mA；mode2 为 0.001 rad/s；mode3/4 为 0.001 rad。mode4 仍使用已提交的阻抗配置和本地受限参考轨迹，前馈电流为 0。target_mode 必须与已通过 SET_MODE 设置的活动 mode 一致，字段仅标明含义，不能隐式切换模式。目标不隐式使能，范围检查、预装安全目标、看门狗和已应用序号规则全部沿用。

## 4 合并四个只读状态入口

保留 type=108 GET_MOTOR_STATE 作为状态查询入口，将 GET_IMPEDANCE_STATE(133)、GET_SYNC_STATE(185)、GET_CLOCK_STATE(186) 映射为扩展页面。四个查询均无副作用，允许 session=0，不刷新运动看门狗，不需要为不同物理状态重复定义请求分发器。JOB_GET_STATE(117) 保留在任务服务中，继续返回轨迹、回零、标定和辨识的统一生命周期，不在本轮再折叠为泛化的任意对象查询。

| 原查询 | 新入口 | 页面 | 返回数据 |
| --- | --- | --- | --- |
| 108 GET_MOTOR_STATE | 108 基础查询 | 隐含 page=0 | 原 32 B 电机状态体 |
| 133 GET_IMPEDANCE_STATE | 108 扩展查询 | page=1 | 原 20 B 阻抗状态体 |
| 185 GET_SYNC_STATE | 108 扩展查询 | page=2 | 原 24 B 同步组状态体 |
| 186 GET_CLOCK_STATE | 108 扩展查询 | page=3 | 原 28 B 时钟状态体 |

拟定的请求、响应布局：

- 基础查询仍为 payload=Q，8 B；OK 响应仍为 R+32 B，44 B。它隐含 page=0，不增加分页头；CAN FD 按需使用这个入口。
- 扩展查询为 Q+page:u16+schema:u16+selector:u32，共 16 B；只允许 page=1/2/3、schema=1。
- 扩展查询 OK 响应为 R+page:u16+schema:u16+该页固定状态体。page=1/2/3 分别为 36/40/44 B payload，均不超过 CAN FD 单帧 payload 的 46 B 上限。
- page=1/3 的 selector 必须为 0；page=2 的 selector 低 16 位为 group_id，高 16 位为 group_tag，对应原有两字段，不丢失对象匹配条件。
- 响应数据体、单位、无效值、版本和时钟误差语义沿用原定义。扩展页响应必须与请求的 type、request_id、page、schema 匹配，不能凭 payload 长度猜测页面。
- 不支持的 page/schema 返回 UNSUPPORTED，保留位或 selector 非法返回 RANGE；错误响应仍按公共规则仅返回 R。扩展形式不接受 page=0，避免同一基本状态有两种额外封装。
- 设备按已校验的完整 payload 长度 8 或 16 分流，其他长度拒绝。旧节点仅支持 8 B 请求；建议通过 GET_CAPS page3.advanced_features 位 11 宣告扩展状态页能力，未置位时使用旧独立查询，不能通过盲发扩展请求探测。

基础查询仍为 26 B 请求、62 B 公司完整响应，分别填充到 32/64 B CAN FD 数据区；type=124 仍为 8 B payload、一个 32 B FD 帧、200 Hz。分页不增加周期反馈负载。

兼容固件可保留 133/185/186 作为旧入口，但必须返回原响应布局，不添加分页头。它们和所有其他只读查询继续使用只读旁路，不覆盖写操作去重缓存，不刷新控制/任务看门狗。

## 5 能力、兼容与实现边界

拟在 GET_CAPS page3.advanced_features 中增加位 9=统一任务、10=统一标量目标、11=扩展状态页；不与 page0 能力位混淆。未报告能力时使用当前 CAN FD 命令，不通过发送运动命令探测版本。旧请求的可选兼容入口仍返回旧格式，不能在同一在途请求中切换 type 重试；所有查询仍使用只读旁路，不覆盖写事务去重缓存。

SET_TARGET 在公司帧头中使用 type=190，payload[0] 为 target_mode，不要求其等于 type；8 B payload 经公司头和 CRC 后为 26 B，再填充到一个 32 B CAN FD 帧。保留完整 8 位 command_seq、16 位 lease_tag、32 位 target。它不能隐式使能或切换模式；同会话旧目标族和新目标族不能混用，切换需释放控制权并重建会话。

不合并 MOTOR_STOP 与正常失能/任务中止；不合并模式、准备、使能、任务启动；不合并目标与任务保活；不合并参数读取/暂存/保存；不将读结果变为自动应用；不取消轨迹提交、镜像最终校验和激活边界。

MOTOR_ENABLE/DISABLE 可另做 2→1，单独采用时主消息 49→48；四种 CONFIG 可做带固定 schema 的 4→1，单独采用时 49→46，但仍保留各分支校验与算法，暂不推荐仅为减少编号而实施。升级 10 个命令与 GET_INFO/GET_CAPS 保持原定义。

通信仍按 comm_hw → CAN FD transport → 公司 protocol → command_router → mc_command_t/服务对象分层。复用命令处理函数不等于减少硬件控制或安全检查，不预先承诺 Flash 节省。每轴 200 Hz 目标 + 200 Hz 反馈帧数保持不变。

验收：新旧入口业务结果一致；模式/lease/序号/范围错误不执行；固定长度与错误响应规则明确；page=1/2/3 响应 payload 为 36/40/44 B，可 CAN FD 单帧承载；实际位时序和最坏延迟另行 HIL 验证。
