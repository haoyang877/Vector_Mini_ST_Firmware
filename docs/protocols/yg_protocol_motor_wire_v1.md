# 当前单轴命令与45B状态线路

核对日期：2026-10-21。依据当前APP消息注册表、motor服务、link及状态所有者适配器。
本页合并原单轴契约和逐命令反馈提案，区分已接线行为、公布能力与实机证据。

## 统一承载与命令

公司应用帧为16B头+payload+2B CRC16，小端；版本0x01。
头CRC8覆盖0～14，poly0x9B/init0；CRC16覆盖完整头和payload，poly0xBAAD/init0xFFFF，均不反射。
请求flags0x20、回复0x10，重试加bit2；29bit扩展CAN FD+BRS，1M/5M，主站2。
目的节点使用设备实际配置，不把向量地址或主站号当作设备地址。控制请求禁止分片。

| Type | 当前请求/响应 | 行为 |
| --- | --- | --- |
| 101 SET_ENABLE | 4B enable:u8+reserved[3]=0；回复4B result/detail | enable0走实际停机；enable1低侧制动未开放，不代替START |
| 107 CLEAR | 空；回复4B | 源恢复和完整健康满足后回IDLE，不启动 |
| 108 GET_MOTOR_STATE | 空；回复45B | correlated_seq=0；无新鲜快照result6、测量无效；非空请求result3 |
| 110 STOP | 空；回复4B | 受理和真实关相确认分开；不得以ACK推断机械静止 |
| 116 SET_CONTROL | 20B，见下文；回复4B，另发关联124 | 只更新RUNNING的同模式目标 |
| 117 START_CONTROL | 4B mode+reserved；回复4B | 见[普通启动v0.2](yg_protocol_ordinary_start_wire_v0_2.md) |
| 124 MOTOR_FEEDBACK | 45B主动反馈；无请求 | 关联116的受理/拒绝或实际应用终态 |

116布局：mode:u8、flags=0:u8、reserved=0:u16、position_mrad:i32、speed_mrad_s:i32、
iq_mA:i32、v_q_mV:i32。mode1/2/3/6分别为电流/速度/位置/q轴电压，仅对应字段非零。
mode0不作为控制目标，MIT4和低侧制动5不开放。线路编号必须显式映射内部枚举。
当前GET_CAPS只公布SPEED：features=0x1D00、supported_modes=0x0004；其他三种启动仍为候选实现。

线路结果：0执行完成、1已受理、2不支持、3长度错误、4字段/陈旧序号、5状态不允许、
6忙、7未授权源、8内部失败、11超范围、17故障未解除。detail由各业务定义。

## 45B字段与有效性

108和124共用下表，逐字段小端编码，应用帧63B补零到CAN FD64B。

| 偏移 | 字段 | 类型/单位 |
| ---: | --- | --- |
| 0 | result | u16，关联业务结果 |
| 2 | correlated_seq | u16，116原序号；108恒0 |
| 4 | fault_code | u32，状态适配器显式映射的故障位图 |
| 8/12 | reference/actual_position | 各i32，mrad |
| 16/20 | reference/actual_speed | 各i32，mrad/s |
| 24 | valid_bits | u16，仅低12位 |
| 26 | bus_voltage | i16，0.01V |
| 28 | bus_current | i16，mA |
| 30/32 | reference/actual_iq | 各i16，mA |
| 34/36 | mcu/motor_temperature | 各i16，0.01°C |
| 38/40 | v_q/v_d | 各i16，mV |
| 42 | node_id | u8，发送设备节点 |
| 43 | motor_state | u8，见[状态与恢复](yg_protocol_motor_state_v1_2.md) |
| 44 | control_mode | u8，实际模式 |

valid_bits bit0～11依次对应：参考位置、实际位置、参考速度、实际速度、母线电压、
母线电流、参考Iq、实际Iq、MCU温度、电机温度、Vq、Vd。高4位为0。
无来源、样本过期或缩放溢出的字段清位并用该整数类型最小值，不发送伪造有效值。
当前电机温度/Vq/Vd没有真实测量来源；MCU温度不能复制为电机温度。
状态、模式、故障、参考/实际量取同一所有者快照，不能跨时刻拼接。

## 目标终态、重试与队列

首次受理116先回复result1；唯一延迟124等真实控制周期终态：已应用0，
STOP撤销5、故障17、复核越限11。APPLIED不表示机械到位。
未产出终态不超时补造成功；主机监控通信并显式STOP。
等待终态期间处理STOP/CLEAR，新目标BUSY且不覆盖原请求。
同序号同内容重试重放原116结果；124报告待应用或已有终态并附当前快照，
不能重复执行。首次受理的延迟终态只发布一次；历史APPLIED不代表仍维持旧目标。

当前link RX/TX各8帧，控制历史32槽；窗口外陈旧请求拒绝，不承诺历史256槽范围。
提交目标前预留ACK与反馈两处发送容量；背压不能变成“目标执行而结果丢失”。
CAN入队、业务受理、目标应用、功率实际关闭和转子静止均是不同证据。

黄金字节仍由 `tests/unit/yg_protocol_contract_test.c` 固定，普通启动另有JSON向量。
旧500kbit/s编号表、111失能、带会话108、周期短反馈和五轴组控制不作为现行ABI。
