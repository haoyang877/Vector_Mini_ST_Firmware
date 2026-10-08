# 电机运行与整计划标定线路 v1.4

日期：2026-10-08。`0x0076` 和 `0x00A6` 已在本项目登记并接入 APP 路由；公司外部 CMD ID 表仍待同步。当前独立 APP 尚无可信启动 epoch，因此新 START 返回“不支持”，能力位保持关闭；未开放功率输出。
本提案承接[整机流程 v1](../architecture/motor_platform_lifecycle_v1.md)和[业务接口 v1.3](yg_protocol_motor_lifecycle_v1_3.md)。公司应用帧仍为版本 `0x01`，头、CRC、29 位 CAN FD ID、45 B 反馈和参数 schema12 均不改变。

## 为什么需要两条新命令

| 候选 CMD ID | 建议名称 | 用途 | 编号现状 |
| --- | --- | --- | --- |
| `0x0076` | `RUN_SESSION` | 四种模式的启动或更换控制方式，以及按原启动身份续租 | 项目内已登记；公司外部表待同步 |
| `0x00A6` | `CALIBRATION_PLAN` | 启动、查询、取消一项多步骤标定计划 | 项目内已登记；公司外部表待同步 |

旧 `0x0075` 的 4 B 请求仍只表示速度模式零目标启动，没有 epoch 字段；旧 `0x00A0～0x00A2` 仍只操作单项标定的原始 ticket，也没有 epoch 字段。它们是现有调试兼容路径，不能声称具有跨复位保护。新命令不用旧单项任务的去重缓存。旧固件收到新编号可能超时而非回复“不支持”。

## `0x0076 RUN_SESSION`

仅主站 `0x02` 单播，应用层请求回复位有效；请求 **20 B**，数值小端。现有线上模式编号明确映射为 `1=CURRENT(A)`、`2=SPEED(rad/s)`、`3=POSITION(rad)`、`6=VQ(V)`；不发送内部 `MotorControlMode` 枚举值。Nm 力矩模式不提供。

| 偏移 | 字段 | 规则 |
| ---: | --- | --- |
| 0 | `schema:u8` | 固定 1 |
| 1 | `op:u8` | 1=START_OR_SWITCH；2=RENEW |
| 2 | `mode:u8` | op=1 时取 1/2/3/6；op=2 时固定 0 |
| 3 | `reserved:u8` | 0 |
| 4～7 | `timeout_ms:u32` | 主站明确给出，范围 1～`INT32_MAX`；不放宽现有超时上限 |
| 8～9 | `origin_seq:u16` | op=1 时为 0；op=2 时为**当前活动启动/切换请求**的公司帧头 seq |
| 10～11 | `reserved:u16` | 0 |
| 12～19 | `boot_epoch:u64` | 必须等于本次 `0x0003 GET_DEVICE_STATUS` 的有效非零启动身份 |

同 Type、同帧头 seq 回 **4 B** `result:u16, detail:u16`。首次 START_OR_SWITCH 返回 `result=1` 表示已受理，仍须查询 `0x006C` 核对 RUNNING 和目标模式；重发完全相同的原请求只读取原进展，不再次启动或延长租约。RENEW 成功返回 `result=0`，只刷新租约，不启动、切模式或清故障。续租请求的帧头 seq 每次递增；同 seq、同内容重发返回缓存结果，**不再延长租约**；同 seq、不同内容拒绝。过期后需处理 CAN_DISCONNECTED 故障、CLEAR，再用新 seq 启动。

适配器将上述值映射到 `MotorStateMachine_RequestIdentifiedRun`、`ReadEnableProgress` 与 `RefreshControlLease`。更换控制方式时，状态机先关相、预载中性 PWM，再确认是否可以重新启动；`0x0074` 仍只更新当前已运行模式的目标。主站 `0x02` 发出 STOP、设备接受失能请求，或成功清除故障后，此前的 RUN_SESSION START/RENEW 与 CALIBRATION_PLAN START 不能再进入业务所有者。其他节点的 STOP 仍执行停机，但其序号不用于判断主站请求的新旧。所有拒绝必须不改变租约、模式或功率状态。

**接入前置条件：**当前独立 APP 的 `0x0003` 报告启动 epoch 无效，不能靠 UID、`boot_id=0` 或主站随机数代替。未提供可信非零 epoch 时，`RUN_SESSION` 必须回复不支持；能力位保持 0。现有所有者对同来源/seq/模式但不同 `timeout_ms` 的重试会刷新租约，协议适配器必须先按完整 payload 去重，不能直接调用所有者。还需核对被拒绝的新启动不会留下来源绑定；见[决策记录](../plans/active/2026-10-08-motor-lifecycle-wire-review.md)。

## `0x00A6 CALIBRATION_PLAN`

仅主站 `0x02` 单播；START 请求 **20 B**，STATUS/CANCEL 请求 **12 B**，数值小端。

| 偏移 | 字段 | 规则 |
| ---: | --- | --- |
| 0 | `schema:u8` | 固定 1 |
| 1 | `op:u8` | 1=START；2=STATUS；3=CANCEL |
| 2～3 | `reserved:u16` | 0 |
| 4～7 | `steps_or_ticket:u32` | START 为步骤位图；STATUS/CANCEL 为非零父 ticket |
| 8～11 | `reserved:u32` | 0 |
| 12～19 | `boot_epoch:u64` | 仅 START 存在；须与本次 `0x0003` 的有效非零启动身份相同 |

线上步骤位与内部枚举**不同**：bit0 相电阻、bit1 编码器 LUT、bit2 电角零位、bit3 机械零位、bit4 摩擦、bit5 齿槽。bit6 电感、bit7 磁链暂列为**不支持**，不能返回虚假的标定成功；bit8～31 保留并拒绝。START 的步骤位须非零，依赖关系仍由唯一标定所有者检查。没有 Nm 换算或按 `motor_type` 自动选择配置。

回复 **24 B**：`result:u16, detail:u16, parent_ticket:u32, step_ticket:u32, requested_steps:u32, completed_steps:u32, step:u8, phase:u8, effects:u8, reserved:u8`。位图使用上面的线上定义；`step` 单独映射为 0=无、1=相电阻、2=LUT、3=电零位、4=机械零位、5=摩擦、6=齿槽、7=最终 SAVE，不发送内部枚举值。`phase` 为 0=无、1=进行中、2=完成、3=请求取消、4=已取消、5=失败；`effects` 为 0=未知、1=未持久提交、2=已提交。

START 的 `result=1` 只表示已取得父 ticket；STATUS 的 `result=0` 只表示查询成功，**计划完成还须 `phase=2` 且 `effects=2`**，即最终 SAVE 已回读验证。CANCEL 的受理不表示当前叶任务和后台资源已经归还；继续查询原父 ticket 直到终态。父 ticket、当前子步骤 ticket、所选步骤和已完成步骤不可混用。

适配器独立保存最近一项计划的完整 START 请求和原回复；相同请求重发返回同一父 ticket，不重复进入叶算法；相同 seq、不同内容返回冲突。不同新 seq 仍须由所有者判定是否忙。旧 `0x00A0` 的单项维护重试记录不得充当此缓存。STATUS/CANCEL 不要求 epoch，仍严格核对当前运行期的父 ticket；这不承诺跨复位保存旧 ticket 的查询历史。拒绝请求不触发标定、不修改现有计划。

## 能力、兼容和验收门槛

建议 `GET_CAPS.features` bit13 表示 `RUN_SESSION v1`、bit14 表示 `CALIBRATION_PLAN v1`；`supported_modes` 的 bit1/2/3/6 对应四个**实际验收通过**的线上模式。当前 `features=0x00001C00`、`supported_modes=0` 均不变。只有编号登记、可信启动 epoch、接线测试及相应整机安全验收后才逐位开放；代码存在不等于能力可宣告。

当前离线测试覆盖完整 CAN FD 请求/回复字节、无效 epoch、重复/冲突、续租去重、STOP/失能/清故障后旧帧以及计划查询/取消。后续仍需补足未知版本/保留位、故障与超时、真实父子 ticket、最终 SAVE 的所有者测试。离线检查不能代替真实关相、功率时序和五轴通信验收。

## 线路黄金向量

下面展示两个请求；[完整的四帧 JSON 向量](yg_protocol_motor_lifecycle_wire_v1_4_vectors.json)还包含对应回复、CAN ID、DLC 和全部填充字节。请求由主站 `0x02` 发给示例节点 `0x07`，29 位 CAN ID 为 `0x0CEF0702`，开启 FDF/BRS。DLC=14，对应 48 B 数据区；末尾补零不进入应用 CRC。测试在 `tests/unit/yg_protocol_link_test.c` 中逐字节核对。

**运行请求：**`RUN_SESSION`，seq=`0x002A`，START、位置模式、超时 1000 ms，epoch=`0x1122334455667788`。

```text
5A A5 01 20 02 07 76 00 2A 00 14 00 00 00 00 90
01 01 03 00 E8 03 00 00 00 00 00 00 88 77 66 55
44 33 22 11 59 F6 00 00 00 00 00 00 00 00 00 00
```

**整计划请求：**`CALIBRATION_PLAN`，seq=`0x002B`，START、相电阻与 LUT 两步骤，epoch 同上。

```text
5A A5 01 20 02 07 A6 00 2B 00 14 00 00 00 00 C3
01 01 00 00 03 00 00 00 00 00 00 00 88 77 66 55
44 33 22 11 D5 EE 00 00 00 00 00 00 00 00 00 00
```

运行请求的 `result=1, detail=0` 回复 payload 为 `01 00 00 00`；整计划的 `result=1`、父 ticket=`0x12345678`、当前相电阻步骤的 24 B 回复 payload 为 `01 00 00 00 78 56 34 12 00 00 00 00 03 00 00 00 00 00 00 00 01 01 00 00`。这两个结果都表示设备接受请求，不能据此断言启相或 SAVE 已完成。
