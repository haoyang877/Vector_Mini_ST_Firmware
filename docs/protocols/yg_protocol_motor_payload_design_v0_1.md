# yg_protocol 电机控制、参数与反馈 Payload 细化 v0.1

日期：2026-09-23。状态：**历史提案，电机启动/控制流程已由[简化版 v0.1](yg_protocol_simplified_motor_v0_1.md)替代；本文件仍可用于核对当时的现状字段**。Type 编号来自[业务 Type 分配提案 v0.3](yg_protocol_business_type_allocation_v0_3.md)。表中 **现状** 表示当前代码已经编码/解码的字节，**目标** 表示当时拟在业务重构时冻结的格式。飞书原文尚未登记这些电机具体类型。

## 1. 先分清三种 ID

```text
29 位 CAN ID：priority + PF=0xEF + dst_node + src_node  → 总线仲裁和寻址
公司帧头 type:u16（偏移 6～7）                   → 业务 CMD ID
业务 Payload 中 parameter_id:u16 / task_kind:u8 → 某个参数 / 标定任务
```

例如主控 `0x02` 发给电机 `0x03` 的普通管理帧可用 CAN ID `0x18EF0302`；帧头 `type=108` 的小端字节是 `6C 00`，表示查询状态；换为 `type=116` 的 `74 00` 就表示设置目标。CAN ID 不变时业务可不同。主控 `0x02` 是飞书原文给 RV1126B 的节点号；现有项目确认记录另写 `0x01`，须在上线前裁决。

所有多字节数按小端；帧为 `16B 公司头 + payload + 2B 公司 CRC16 + 0～N B CAN FD 零填充`。CAN FD 单帧 payload≤46B，CRC16 后的 DLC 填充不计入公司 `len` 和 CRC。应用 `type` 不是 CAN ID 的 PF；所有实时目标与反馈保持单帧。

## 2. 统一事务前缀与结果语义：目标格式

按需命令建议统一使用以下前缀。下表是**目标格式**，110/111 当前仍使用第 3.2 节的旧局部格式。

| 请求 Payload 偏移 | 字段 | 类型 | 约束 |
| ---: | --- | --- | --- |
| 0～3 | session_id | u32 | 只读、紧急 STOP 可为 0；受控写操作必须匹配已取得的会话 |
| 4～7 | request_id | u32 | 非零，同一会话内递增；重试保持不变 |

上述 8B 称 `Q`。单帧请求成功或业务拒绝都返回相同 `type`、相同请求头部 `seq_id`、相反 src/dst、RESPONSE 标志。若未来的管理命令使用公司分片，头部 `seq_id` 按分片规则变化，事务仍以 `request_id` 关联；本文件的实时命令均禁止分片。回复 Payload 前 12B 称 `R`：

| 回复 Payload 偏移 | 字段 | 类型 | 约束 |
| ---: | --- | --- | --- |
| 0～3 | session_id | u32 | 回显请求；ACQUIRE_CONTROL 的成功回复可返回新会话 |
| 4～7 | request_id | u32 | 回显请求，用于去重与关联 |
| 8～9 | result | u16 | 0 OK、1 ACCEPTED、2 UNSUPPORTED、3 BAD_LENGTH、4 BAD_FIELD、5 BAD_STATE、6 BUSY、7 DENIED、8 SESSION_EXPIRED、9 STALE_REQUEST、10 CONFLICT、11 RANGE、17 FAULT_ACTIVE；其余结果由对应业务申请 |
| 10～11 | detail | u16 | 0 无补充；字段错误时为出错 Payload 偏移 |

错误回复原则上只有 `R`；若需报告仍存在的故障等诊断字段，必须由该 type 显式规定。`ACCEPTED` 只表示已受理，后续通过状态/任务查询确认完成。相同 `(boot_id, src, session_id, type, request_id)` 重传必须返回缓存的首次受理结果，不得重复执行写参数、使能、标定或 Flash 操作；最终执行结果单独查询。CRC、地址或不足 8B 的请求不能可靠关联，不生成业务回复。

注意：当前 `yg_protocol_service_status_t` 是内部枚举，110/111 当前直接把其序号放进回复第 0 字节；**它不是上表的 `result:u16`**。接入新命令前须建立显式映射，不能在主机端混用两套数字。

## 3. 当前实际已有的字节格式

### 3.1 `108 / 0x006C GET_MOTOR_STATE`

请求恰好 8B：`Q(session_id=0, request_id≠0)`。成功回复为 `R` 12B 加下列 34B 状态，合计 46B payload；公司帧恰好 64B。业务错误只回复 12B `R`。现有代码返回 BAD_LENGTH=3、BAD_FIELD=4、BUSY=6。

| 状态附加区偏移 | 字段 | 线上类型与单位 | 无效表示 |
| ---: | --- | --- | --- |
| 0～3 | boot_id | u32 | 0 当前仅占位，不能当真实启动代次 |
| 4～7 | sample_counter | u32 | — |
| 8～11 | position | i32 mrad | `INT32_MIN` |
| 12～15 | speed | i32 mrad/s | `INT32_MIN` |
| 16～19 | iq | i32 mA | `INT32_MIN` |
| 20～23 | faults | u32 位图 | 0 表示当前映射无故障 |
| 24～25 | bus_voltage | u16 mV | `0xFFFF` |
| 26～27 | bus_current | i16 mA | `INT16_MIN` |
| 28～29 | temperature | i16，0.01 °C | `INT16_MIN`；目前来源是 MCU 温度 |
| 30 | state | u8 线上状态 | 0 启动/自检、1 就绪、2 启动中、3 运行、4 停机中、5 故障；6 保留 |
| 31 | mode | u8 线上模式 | 0 禁用、1 电流、2 速度、3 位置、4 阻抗、7/8 任务 |
| 32 | last_applied_sequence | u8 | 当前固定 0；只容纳 8 位，不足以独立确认 1 kHz 长运行序列 |
| 33 | valid_bits | u8 | bit0 位置、1 速度、2 Iq、3 已应用目标、4 母线电压、5 温度、6 母线电流 |

这些测量由同一次应用快照编码。`iq` 是 q 轴电流，不是 N·m 力矩；若需要实际扭矩，另定义有效位和标定模型。`GET_MOTOR_STATE` 是按需完整状态查询，不作为五轴每轴 1 kHz 的周期帧。

### 3.2 `110 MOTOR_STOP` / `111 MOTOR_DISABLE`

**现状**：请求 payload 为 0B；回复 payload 固定 16B：

| 偏移 | 字段 | 类型 | 当前来源 |
| ---: | --- | --- | --- |
| 0 | status | u8 | 内部 `yg_protocol_service_status_t` 序号 |
| 1 | reserved | u8 | 0 |
| 2～3 | detail | u16 | 电机适配器结果 |
| 4～7 | token | u32 | 异步停机 token |
| 8～11 | revision | u32 | 当前通常为 0 |
| 12～15 | value | i32 | 当前通常为 0 |

当前路由存在，但真实停机服务未绑定，所以不能把收到回复解读成已停机。**目标**：仍用 110/111，但请求改为 `Q` 8B，回复改为 `R` 12B 加 `token:u32`（总 16B）；只有电机所有者确认功率输出禁止才回 OK，否则回 ACCEPTED/token 或故障。STOP 可允许指定可信源以 `session_id=0` 发起；DISABLE 的会话规则须在接线前冻结。广播 STOP 的多轴回执由各轴状态证据关联，硬件 ACK 不算业务成功。此改动是线上 ABI 变化，必须与主机和黄金向量一次提交。

### 3.3 `124 / 0x007C MOTION_FEEDBACK`

现有纯编码函数输出 8B payload，**尚未接入周期发送**：

| 偏移 | 字段 | 类型与单位 |
| ---: | --- | --- |
| 0～3 | position | i32 mrad |
| 4～5 | speed | i16 mrad/s |
| 6～7 | iq | i16 mA |

无效/超范围用 `INT32_MIN` 或 `INT16_MIN`，不把溢出回绕或截断成有效值。公司完整帧 26B，CAN FD 物理数据区 32B，末尾 6B 为零填充。该 8B 中没有状态、故障、已应用序号或母线/温度；这些由 108/126 提供。124 的公司头 `seq_id` 是报告发送序号，**不是**已应用目标序号。

## 4. 控制命令目标 Payload（尚未实现）

下表列出成功请求的建议字段；除 110/111 外都以 `Q` 开头。回复均先带 `R`。所有 reserved 字节必须为 0；不支持的模式、状态、会话、范围要拒绝，不能静默更改成另一物理量。

| Type | 请求 Payload，按顺序排列 | 长度 | 成功回复附加字段 | 业务门槛 |
| ---: | --- | ---: | --- | --- |
| 101 MOTOR_ENABLE | Q + expected_mode:u8 + reserved[3] | 12B | state:u8、power_enabled:u8、reserved:u16 | 无故障、配置有效、当前目标安全且新鲜；不由目标命令隐式使能 |
| 106 SET_MODE | Q + mode:u8 + reserved[3] + expected_revision:u32 | 16B | current_mode:u8、state:u8、reserved:u16 | 参数版本必须匹配；只在允许状态切换；清理旧模式目标 |
| 107 CLEAR_FAULT | Q + clear_mask:u32 | 12B | remaining_faults:u32 | 物理故障仍在时回 FAULT_ACTIVE |
| 109 SET_REPORT | Q + period_us:u16 + feedback_type:u16 + event_enable:u8 + reserved[3] | 16B | accepted_period_us:u16、feedback_type:u16 | 普通反馈选 124；组反馈 125 由 GROUP_CONFIG 管理 |
| 112 GET_CONTROL_STATE | Q | 8B | owner_node:u8、state:u8、mode:u8、flags:u8、last_applied_seq:u16、target_age_ms:u16、session_id:u32 | 用完整 u16 序号补足 108 的 u8 不足 |
| 116 SET_TARGET | Q + target_kind:u8 + flags:u8 + lease_ms:u16 + position_mrad:i32 + speed_mrad_s:i32 + iq_mA:i32 + execute_at_us:u32 | 28B | accepted_seq:u16、mode:u8、applied:u8 | 详见下文；单轴管理控制，不代替 193 的五轴广播流 |

`116` 的 `target_kind` 候选：1=电流，仅使用 `iq_mA`；2=速度，仅使用 `speed_mrad_s`；3=位置，仅使用 `position_mrad`；4=阻抗，允许三个目标同时使用。未使用的目标字段必须为 0；**数值为 0 本身是合法目标**，由 `target_kind` 而非“非零字段”选择物理量。`execute_at_us=0` 表示下一允许控制周期应用；非 0 仅在设备声明定时目标能力后接受。`flags` 首版为 0。28B payload 加公司开销 18B 共 46B，CAN FD 数据区为 48B。

`116` 的完整请求偏移可直接用于主机编解码：

| 偏移 | 0～3 | 4～7 | 8 | 9 | 10～11 | 12～15 | 16～19 | 20～23 | 24～27 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 字段 | session_id | request_id | target_kind | flags | lease_ms | position_mrad | speed_mrad_s | iq_mA | execute_at_us |

例如位置 1.000 rad、租约 50 ms、立即在下一个允许周期应用，且 `session_id=0x11223344`、`request_id=3`，28B 请求 payload 是 `44 33 22 11 03 00 00 00 03 00 32 00 E8 03 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00`。回复 16B 为 `R + accepted_seq:u16 + mode:u8 + applied:u8`；`accepted_seq` 回显该请求公司头的 16 位 `seq_id`，`applied=0` 表示尚未进入控制周期，此时 `result=ACCEPTED`；`applied=1` 表示回复产生前已应用，此时 `result=OK`。业务拒绝只返回 12B `R`。

回复 `result=ACCEPTED` 表示命令已进入有界应用邮箱；实际进入控制周期后，`112.last_applied_seq` 才更新到相同的 `accepted_seq`。重复 `request_id` 或旧序号不重新应用，也不延长 `lease_ms`。`lease_ms` 范围和到期停机策略需要电机安全评审；不能拿管理事务的 1 s 超时当运动保护。普通单轴位置命令可逐条业务回复，五轴 1 kHz 应走 193/125。

## 5. 参数命令目标 Payload（尚未实现）

**业务 Type 与参数 ID 是两级编号**：`type=120` 永远表示“读参数”，payload 中的 `parameter_id=1` 才表示“最大速度”。当前离线只读服务只登记参数 0=活动配置 revision（u32）、1=最大速度（i32、0.001 rad/s）、2=最大电流（i32、mA）；它还没有绑定 120 运行路由。更大参数表需要单一描述符登记，不沿用已删除的标准 CAN 参数线编号解释。

| Type | 请求 Payload | 长度 | 成功回复附加字段 | 说明 |
| ---: | --- | ---: | --- | --- |
| 120 READ_PARAM | Q + parameter_id:u16 + reserved:u16 | 12B | parameter_id:u16、value_type:u8、reserved:u8、value_bits:u32、revision:u32 | `R+12B`，共 24B；值的单位/倍率由参数描述符确定 |
| 121 WRITE_PARAM | Q + parameter_id:u16 + value_type:u8 + reserved:u8 + value_bits:u32 + expected_revision:u32 | 20B | parameter_id:u16、reserved:u16、new_revision:u32 | 写运行态；版本不符拒绝；不会自动写 Flash |
| 122 SAVE_PARAM | Q + expected_revision:u32 | 12B | persisted_revision:u32 | 首版仅同步保存：停机且允许保存时写入并校验完成后回 OK；若无法在事务时限内完成则回 BUSY，不回无从查询的 ACCEPTED |
| 123 GET_PARAM_SCHEMA | Q + parameter_id:u16 + page:u16 | 12B | 下述固定 16B 描述符 | 首版 page 必须为 0；不支持的页回 BAD_FIELD |

`value_type` 首版建议 1=i32、2=u32；`value_bits` 是相应整数的小端二进制。浮点运行态转换到线上整数时，参数描述符必须声明物理单位和倍率；例如最大速度 1.25 rad/s 编码为 i32 `1250`，最大电流 3 A 编码为 i32 `3000`。不能把内部 float 的内存字节直接发给主机。标定参数如果可读，也用 READ_PARAM 的独立 `parameter_id`，而不是增设一个“GET_CALIB_PARAM”业务 Type。

`123` 成功回复是 `R` 12B 后接一个 16B 描述符，总 payload 28B；首版一次只查询一个 `parameter_id`，无需列表分页。描述符从附加区偏移 0 开始：`parameter_id:u16`（0～1）、`value_type:u8`（2）、`access:u8`（3）、`scale_exp10:i8`（4）、`unit_code:u8`（5）、`apply_policy:u8`（6）、`reserved:u8`（7）、`min_raw:i32`（8～11）、`max_raw:i32`（12～15）。`access` bit0 可读、bit1 可写、bit2 可保存；`raw = SI 值 × 10^scale_exp10`；`unit_code` 首版 0 无量纲、1 rad/s、2 A；`apply_policy` 首版 0 只读、1 即时、2 仅停机、3 下次使能、4 下次重启。参数 0 描述 revision（u32、无量纲、只读）；1 描述最大速度（i32、rad/s、倍率 10³）；2 描述最大电流（i32、A、倍率 10³）。具体最小/最大值来自当前产品参数约束，不允许用整数类型极限代替产品安全限值。

参数修改的完整流程建议为：主机 `123` 读取 schema 和权限 → `120` 取得当前值及 revision → 在电机允许的状态用 `121(expected_revision)` 更新运行态 → 再用 `120` 读回并核对新 revision → 需要掉电保留时执行 `122`。多个参数的原子更新首版没有定义；修改两项时若第二项失败，主机必须按读回结果处理，不能假定两项一起生效。

## 6. 五轴 1 kHz 同步反馈：125 与 193

`193 SET_GROUP_POSITION` 建议广播：`group_tag:u32 + cycle_seq:u16 + axis_count:u8 + flags:u8 + execute_at_us_low32:u32 + position_mrad:i32[N]`。5 轴 payload=32B，公司完整帧=50B，物理 CAN FD 为 64B；最多 8 轴可在 46B payload 限制内单帧承载。

每个电机以 `125 MOTION_FEEDBACK_SYNC` 发送 14B：

| Payload 偏移 | 字段 | 类型 |
| ---: | --- | --- |
| 0～3 | group_tag | u32 |
| 4 | state_mode | u8，高 4 位模式、低 4 位状态 |
| 5 | fault_summary | u8 故障摘要 |
| 6～9 | position | i32 mrad |
| 10～11 | speed | i16 mrad/s |
| 12～13 | iq | i16 mA |

**仅 125** 的公司头 `seq_id:u16` 回显该电机本次采样实际应用的 `cycle_seq`。完整公司帧恰好 32B。主控按 `(group_tag, node_id, cycle_seq)` 判断五轴是否都反馈，不能把广播的 CAN 硬件 ACK 当成五轴执行确认。运行时 125 替代 124，不同时发送两份周期反馈。故障摘要之外的故障、母线和温度按需用 108 查询或由 126 事件报告。

`fault_summary` 候选位：bit0 过流、bit1 过压、bit2 欠压、bit3 过温、bit4 传感器、bit5 通信/同步、bit6 软件停止锁定、bit7 其他；允许同时置位。`125` 是设备主动周期上报，不对上报帧再发业务回复；它承担 `193` 每轴每周期的执行回执。设备在拒绝某个组目标时还须立即给出事件或状态证据，不得发送伪造的“已应用周期”反馈。

当前 125/193 均未注册；单路 1M/5M 在 1 条 64B 组目标 + 5 条 32B 反馈时，估算周期占用约 84%，只能通过[五轴同步建议](canfd_1khz_sync_design_proposal.md)中的实机时序验收确认可行。

## 7. 一次位置控制交互示例

以主控 `0x02` → 电机 `0x03`，会话 `0x11223344` 为例；这是**字段示例，不是当前固件可执行命令或 CRC 黄金向量**。

1. `106 SET_MODE`，设位置模式 `3`：CAN ID `0x18EF0302`；公司头 `type=6A 00`；假定活动参数 revision=5，则 Payload `44 33 22 11 01 00 00 00 03 00 00 00 05 00 00 00`（Q + mode + reserved + expected_revision）。回复应为同 type、同请求号，并明确结果。
2. `101 MOTOR_ENABLE`：单独请求；成功回复与 108 状态都须证实已使能，不能由下一条目标暗中使能。
3. `116 SET_TARGET`，目标位置 1.000 rad：公司头 `type=74 00`；Payload 中 `target_kind=03`、`position_mrad=E8 03 00 00`，速度/Iq 字段为 0；其余字段按第 4 节填写。前台回复 ACCEPTED 只表示入有界邮箱。
4. 控制周期消费目标后，`112 GET_CONTROL_STATE` 的 `last_applied_seq` 对应目标序号；`108 GET_MOTOR_STATE` 给出同一快照的位置、速度、Iq、状态及有效位。目标 1.000 rad 与实际位置可能不同，不能把“目标已应用”理解为机械已经到位。

## 8. 定版前必须消除的四个差异

1. 当前 110/111 的 0B 请求与内部状态码回复，需要改成统一 Q/R 并生成新向量；不能让主机同时按两种格式解析相同 Type。
2. 当前 108 的已应用序号只有 u8 且固定 0；接入真实目标后必须从控制所有者发布，长运行关联优先使用 112 的 u16 或 125 的 `cycle_seq`。
3. 当前参数 READ 仅有离线服务，`value_type`、单位/倍率、revision 尚未形成完整线上描述符。先从已实现的参数 0/1/2 定版，再扩展标定和全部运行参数。
4. 1 kHz 目标使用 193/125 周期反馈作为回执的语义，以及 STOP/DISABLE 广播时各轴的确认规则，须与公司 ACK/RESPONSE 位定义一起评审；不能把硬件 ACK 充当业务成功。

本文件通过后再进入[业务重构实施方案](../plans/active/2026-09-23-yg-protocol-business-implementation.md)的契约冻结阶段；本轮只交付设计。
