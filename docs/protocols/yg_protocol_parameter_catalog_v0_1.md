# 当前参数ID、来源、尺度与权限

核对日期：2026-10-21。依据 `parameter_values.c` 的真实登记和
`motor_parameter_transaction.c` 的准备资格诊断；本页取代含未来候选号的旧目录。
Type120/121/122与payload内u8参数ID是不同编号；所有整数小端。

R为可读，W-IDLE为停机写候选权限，C为由标定所有者写入。
通用写入仍受身份、状态、revision、类型、整体配置及耦合范围检查；有W位不表示任意数值可写。
SI×N表示raw=SI值×N并按实现的半值/边界规则编码；字段本身为A、V、rad、rad/s、H、Ω、Wb、s或无量纲热比例。

| 十进制/十六进制ID | 当前来源 | 类型与尺度 | 权限 |
| --- | --- | --- | --- |
| 0 / 0x00 | 活动revision | u32 | R |
| 1 / 0x01 | `application.speed_limit_rad_s` | i32，SI×1000 | R/W-IDLE |
| 2 / 0x02 | `application.current_limit_a` | i32，SI×1000 | R/W-IDLE |
| 3 / 0x03 | device.node_id | u32 | R |
| 4 / 0x04 | 电机类型编码（当前返回1） | u32 | R |
| 5 / 0x05 | model.motor_pole_pairs | u32 | R |
| 6 / 0x06 | `model.motor_flux_wb` | i32，SI×1000000000 | R |
| 7 / 0x07 | `model.motor_phase_resistance_ohm` | i32，SI×1000000 | R |
| 8 / 0x08 | `model.motor_d_inductance_h` | i32，SI×1000000000 | R |
| 9 / 0x09 | `model.motor_q_inductance_h` | i32，SI×1000000000 | R |
| 10 / 0x0A | (Ld+Lq)/2；类型i32、H×10⁹ | i32，H×10⁹ | R |
| 11 / 0x0B | `model.id_kp` | i32，SI×1000 | R |
| 12 / 0x0C | `model.id_ki` | i32，SI×1000 | R |
| 13 / 0x0D | `model.iq_kp` | i32，SI×1000 | R |
| 14 / 0x0E | `model.iq_ki` | i32，SI×1000 | R |
| 16 / 0x10 | `application.speed_kp` | i32，SI×1000000 | R |
| 17 / 0x11 | `application.speed_ki` | i32，SI×1000000 | R |
| 19 / 0x13 | `application.speed_acceleration_rad_s2` | i32，SI×1000 | R |
| 20 / 0x14 | `application.speed_deceleration_rad_s2` | i32，SI×1000 | R |
| 21 / 0x15 | `application.cascade_position_kp` | i32，SI×1000000 | R/W-IDLE；AXS2时只读 |
| 22 / 0x16 | `application.cascade_position_kd` | i32，SI×1000000 | R/W-IDLE；AXS2时只读 |
| 23 / 0x17 | `application.position_maximum_speed_rad_s` | i32，SI×1000 | R/W-IDLE；AXS2时只读 |
| 24 / 0x18 | `application.position_acceleration_rad_s2` | i32，SI×1000 | R |
| 25 / 0x19 | `application.position_deceleration_rad_s2` | i32，SI×1000 | R |
| 26 / 0x1A | `application.position_error_window_rad` | i32，SI×1000 | R |
| 64 / 0x40 | device.encoder.electrical_zero_q15 | u32 | R/C |
| 65 / 0x41 | device.encoder.mechanical_zero_q15 | u32 | R/C |
| 66 / 0x42 | device.encoder.reverse（0/1） | u32 | R/C |
| 67 / 0x43 | LUT与电零位两位均有效（0/1） | u32 | R/C |
| 68 / 0x44 | `device.phase_a_offset_counts` | i32，SI×1000 | R/C |
| 69 / 0x45 | `device.phase_b_offset_counts` | i32，SI×1000 | R/C |
| 70 / 0x46 | `device.phase_c_offset_counts` | i32，SI×1000 | R/C |
| 76 / 0x4C | 编译分流电阻，mΩ | u32 | R |
| 77 / 0x4D | configuration_confirmed（0/1） | u32 | R/W-IDLE |
| 78 / 0x4E | configuration_source | u32 | R |
| 79 / 0x4F | readiness.start_blockers | u32 | R |
| 80 / 0x50 | readiness.self_test_checks | u32 | R |
| 81 / 0x51 | readiness.startable_modes | u32 | R |
| 96 / 0x60 | `model.thermal.continuous_vector_a` | i32，SI×1000 | R/W-IDLE |
| 97 / 0x61 | `model.thermal.peak_a` | i32，SI×1000 | R/W-IDLE |
| 98 / 0x62 | `model.thermal.heating_tau_s` | i32，SI×1000 | R/W-IDLE |
| 99 / 0x63 | `model.thermal.derating_enter_heat` | i32，SI×1000000 | R/W-IDLE |
| 100 / 0x64 | `model.thermal.derating_exit_heat` | i32，SI×1000000 | R/W-IDLE |
| 101 / 0x65 | `model.thermal.trip_heat` | i32，SI×1000000 | R/W-IDLE |
| 102 / 0x66 | `model.thermal.clear_heat` | i32，SI×1000000 | R/W-IDLE |
| 104 / 0x68 | application.speed_feedforward.enabled（0/1） | u32 | R/W-IDLE |
| 105 / 0x69 | `application.speed_feedforward.gain_a_per_rad_s2` | i32，SI×1000000 | R/W-IDLE |
| 106 / 0x6A | `application.speed_feedforward.acceleration_limit_rad_s2` | i32，SI×1000 | R/W-IDLE |
| 107 / 0x6B | `application.speed_feedforward.jerk_limit_rad_s3` | i32，SI×1000 | R/W-IDLE |
| 108 / 0x6C | `application.speed_feedforward.current_limit_a` | i32，SI×1000 | R/W-IDLE |

0x4F～0x51由APP事务读取一致准备快照，未在纯标量表中重复保存。
0x43是完整标定资格布尔值，不是所有原始标定位的直通输出。
15、18、27～34、0x47～0x4B（旧摩擦字段）、0x67等未登记ID读取返回UNSUPPORTED；
不能根据旧候选表或兼容枚举推断存在真实来源。非登记ID写入由现有Apply错误映射拒绝，不能借此写入字段。
LUT不通过数千个标量ID或未实现163接口读取；当前没有SN通用写命令。

WRITE仅修改活动值，不自动保存；成功变更恰好推进一次revision，同身份重试不重复应用。
SAVE在真实Flash提交/读回后才报告成功。类型、字节、detail和重试见[参数线路](yg_protocol_parameter_wire_v1.md)。
热保护/速度前馈的范围及兼容见[schema13扩展](parameter_schema13_delivery_v1.md)，
当前持久记录为[schema14](parameter_schema14_storage_v1.md)。
