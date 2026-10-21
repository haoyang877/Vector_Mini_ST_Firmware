> 历史归档（2026-10-21）：重写前的历史/混合说明快照；当前同名文件按实际代码重建。 本文的“当前”指原始基线；现行入口见[当前说明](../../protocols/yg_protocol_parameter_catalog_v0_1.md)。归档不新增执行授权。

# 电调读写参数表 v0.1（项目编号与当前登记范围）

2026-10-17增量：热配置`0x60..0x66`及速度前馈`0x68..0x6C`已接入同一120/121/122；
单位、耦合条件和schema13兼容性见[交付功能参数扩展v1](../../protocols/parameter_schema13_delivery_v1.md)。
这些增量覆盖下文“其他未知编号”的旧范围说明，0x67仍不登记。

2026-09-29 更新：120/121/122 已接入通信路由、APP 参数所有者和真实存储提交。
逐字节结果、detail 及 revision 见[线路接入 v1](../../protocols/yg_protocol_parameter_wire_v1.md)。
旧离线只读适配器已删除，当前登记由 `parameter_value_service` 的实际来源决定：

- 可读：基础 ID `0～14、16、17、19～26`，标定 ID `0x40～0x4C`；准备资格 ID `0x4D～0x51`。
- 通用可写：`1、2、21、22、23、0x4D`；数值轴 profile 下 `21/22/23` 只读。
  写入还受停机、身份、revision、类型、产品范围和耦合约束限制。
- 未登记：`15、18、27～34` 及其他未知编号，返回 `UNSUPPORTED`。
- `122` 异步保存，真实存储提交成功后才回复 OK；活动 revision 不等于 schema 版本。

以下完整表仍包含候选权限和未来项，以以上实际登记范围为准，不代表全目录已实现。
本次删除旧实现、同步文档后未运行验证，不沿用旧适配器结果。

初版日期：2026-09-23。本文细化[CMD 功能契约](yg_protocol_cmd_function_contract_v0_1.md)的 `120 READ_PARAM`、`121 WRITE_PARAM`、`122 SAVE_PARAM`。编号、单位和权限是本项目约定；飞书[《电调通信协议》“生产相关”](https://wcnxj7iyqdkp.feishu.cn/wiki/M5KJwCZApilqfjk251XcktAPnxa)的原始清单不作为这些具体数值已获公司登记的证明。

## 1. 统一规则

- 参数命令的 `type` 仍是公司帧头中的 `120/121/122`，**参数 ID 是 payload 中的 `u8`**，两者不是同一编号。现有内部 `yg_protocol_parameter_request_t.parameter_id` 为 `u16`，接线时可以无损承载 `u8`，但不能据此把线上字段改成 `u16`。旧[电机 payload v0.1](yg_protocol_motor_payload_design_v0_1.md)的 `parameter_id:u16` 是已被本表替代的历史候选。
- 飞书“生产相关”写明只在 `idel` 状态进入；本表按 **IDLE 且功率输出失能** 处理 `120/121/122`。运行中身份/版本查询使用 `1 GET_INFO`，状态使用 `108/124`，不通过参数写入改控制目标。未实现或无可信来源的参数，读写都返回 `UNSUPPORTED`，不得填默认常数冒充当前值。
- 所有标量在线路上用小端 `i32/u32` 原始值。表中 `×10³` 表示 `raw = round(SI 值 × 1000)`；实际换算、范围和溢出必须先验证。字符串/SN 用长度明确的 `BYTES`，不含隐式 NUL。`READ_PARAM` 返回**活动运行值**，不是 Flash 原始记录；`SAVE_PARAM` 才持久化已确认的活动配置。读取持久值如确有需要再扩展，不混用同一回复。
- `R`=可读；`W-IDLE`=候选可写，仅 IDLE、授权主控、完整校验后应用；`C`=由标定流程拥有，通用 `121` 不可写；`—`=暂不注册。有固件字段不等于线上已实现，实际登记和权限见本文开头。

## 2. 参数 ID 与字段映射

保留现有离线只读 ID `0/1/2` 的含义，新增编号从 `3` 递增。空缺项先保留候选号，不在运行注册表/GET_CAPS 中声明。列出的“上限”是验证来源，不意味着当前已有逐参数写入处理器。
本表有 `0～34` 共 35 个基础候选号、`0x40～0x4C` 共 13 个标定结果候选号；它们不等于已经实现 48 个参数。

| ID | 飞书项目 / 线路名称 | 值与单位 | 建议权限 | 当前固件来源及必须验证的条件 |
| ---: | --- | --- | --- | --- |
| `0` | 活动配置修订号 `config_revision`（项目补充） | `u32` 次数 | R | APP 参数所有者提供；用于 `121` 乐观并发检查，与 Flash `schema_version` 不同。 |
| `1` | 最大速度 `speed_limit` | `i32` mrad/s | R，W-IDLE | `MotorControl.speed_limit`；必须为正，不能超过轴型、编码器与电机配置共同上限。 |
| `2` | 最大扭矩电流 `iq_max_limit` | `i32` mA | R，W-IDLE | `MotorControl.current_limit`；它是 Iq 限流值，**不是 N·m**。上限由电流采样/功率硬件与轴型配置共同决定。 |
| `3` | 通信节点 `node_id` | `u8` 节点号 | R；写仅生产配置流程 | 直接来自独立schema12字段，APP要求1～254的有限整数；AXS角色不再覆盖地址。端点启动冻结，不开放在线改址；兼容变化见[地址与应用包络v2](../../protocols/parameter_node_application_v2.md)。 |
| `4` | 通信波特率 `can_bitrate_profile` | `u8` 枚举；候选 `1=1M/5M CAN FD` | R | 当前固定 CAN FD 仲裁 1 Mbit/s、数据 5 Mbit/s；单个“波特率”不能表达两个速率，因此读一个完整时序 profile。首版不提供动态写入。 |
| `5` | 电机极对数 `pole_pairs` | `u32` 对 | R，W-IDLE 候选 | `MotorControl.motor_pole_pairs`；必须为正且适配编码器/产品。更改后使电角度、标定有效性重新评估。 |
| `6` | 电机磁链 `motor_flux` | `i32` nWb | R，W-IDLE 候选 | `MotorControl.motor_flux`，须正且有限；辨识结果通过标定流程提交，不绕过产品界限。 |
| `7` | 相电阻 `phase_resistance` | `i32` µΩ | R，W-IDLE 候选 | `MotorControl.motor_phase_resistance`，单位 Ω 的运行值；与三相测试向量、测量路径补偿和电流 PI 参数一致。 |
| `8` | d 轴电感 `L_d` | `i32` nH | R，W-IDLE 候选 | `MotorControl.motor_d_inductance`，须正且有限。 |
| `9` | q 轴电感 `L_q` | `i32` nH | R，W-IDLE 候选 | `MotorControl.motor_q_inductance`，须正且有限。 |
| `10` | 等效电感 `L_s` | `i32` nH | R | 现有无独立存储；无感观察器取 `(L_d+L_q)/2`。返回派生值，不允许与 8/9 分别写出相互矛盾的值。 |
| `11` | d 轴电流环 `Kp_d` | `i32`，系数 ×10³ | R，W-IDLE 候选 | `MotorControl.id_Kp`；当前默认由 `L_d × 带宽` 得出。写入前确认控制器允许独立整定。 |
| `12` | d 轴电流环 `Ki_d` | `i32`，系数 ×10³ | R，W-IDLE 候选 | `MotorControl.id_Ki`；当前默认由 `R_s × 带宽` 得出。 |
| `13` | q 轴电流环 `Kp_q` | `i32`，系数 ×10³ | R，W-IDLE 候选 | `MotorControl.iq_Kp`；与 q 轴电感和电流环稳定性联合校验。 |
| `14` | q 轴电流环 `Ki_q` | `i32`，系数 ×10³ | R，W-IDLE 候选 | `MotorControl.iq_Ki`；与相电阻和电流环稳定性联合校验。 |
| `15` | 电压极限环 / 调制比 `modulation_limit` | 拟 `u32`，万分比 | — | 飞书列名，当前未找到独立可配置运行字段；不能把 PWM 硬件限幅常量伪装成可写参数。 |
| `16` | 速度环 `Kp` | `i32`，内部系数 ×10⁶ | R，W-IDLE 候选 | `MotorControl.speed_Kp`；控制器输出按 `current_limit` 归一化，不能误称 A/(rad/s) 的固定物理增益。 |
| `17` | 速度环 `Ki` | `i32`，内部系数 ×10⁶ | R，W-IDLE 候选 | `MotorControl.speed_Ki`；与速度周期、限流值和稳定性联合校验。 |
| `18` | 速度环 `Kc` | 待定 | — | 飞书列名；当前速度 PI 未见独立 `Kc` 字段/算法，先不开放。 |
| `19` | 速度最大加速度 `speed_accel` | `i32` mrad/s² | R，W-IDLE 候选 | `MotorControl.speedAcc`；正数，不能超过轴型与电机允许加速度。 |
| `20` | 速度最大减速度 `speed_decel` | `i32` mrad/s² | R，W-IDLE 候选 | `MotorControl.speedDec`；正数，制动能力与母线回灌限制需一起验证。 |
| `21` | 位置环 `Kp` | `i32`，系数 ×10⁶ | R，W-IDLE 候选 | mode 3 串级位置环对应 `MotorControl.cascade_pos_Kp`，单位约 `1/s`；上限由 `CASCADE_POSITION_KP_MAX_PER_S` 与轴型配置决定。 |
| `22` | 位置环 `Kd` | `i32`，系数 ×10⁶ | R，W-IDLE 候选 | mode 3 串级位置环对应 `MotorControl.cascade_pos_Kd`，无量纲；不与 MIT/阻抗 `pos_Kd` 混用。 |
| `23` | 位置环最大速度 `position_max_speed` | `i32` mrad/s | R，W-IDLE 候选 | `MotorControl.pos_maxspeed`；不能超过 ID 1 和轴型 `maximum_speed_rad_s`。 |
| `24` | 位置环最大加速度 `position_accel` | `i32` mrad/s² | R，W-IDLE 候选 | `MotorControl.posAcc`；正数，须遵循轨迹/轴型上限。 |
| `25` | 位置环最大减速度 `position_decel` | `i32` mrad/s² | R，W-IDLE 候选 | `MotorControl.posDec`；正数，还需检查实际制动能力。 |
| `26` | 位置环误差窗口 `position_error_window` | `i32` mrad | R；写待持久化设计 | 当前 `MotorControl.pos_error_window` 初始化为 1 mrad，Flash 参数记录无此字段。写入前要定义失电行为与进入/退出滞回，不把它当限位。 |
| `27` | MIT `Kp` | 待定 | — | 飞书列名；不能直接把当前 `Position_Impedance_Mode.pos_Kp` 当已定义的 MIT 算法参数。 |
| `28` | MIT `Kd` | 待定 | — | 与 ID 27 一起定算法、单位、限幅与 mode 4 后再开放。 |
| `29` | 电机类型 `motor_type` | `u32` 产品枚举 | R 候选 | 当前编译期 `FOC_ACTIVE_MOTOR_PROFILE`，与轴关节类型 `MotorJointType` 不同；需定义稳定产品枚举。 |
| `30` | 电机 SN `motor_sn` | `BYTES`，≤32B | —，将来 R | 当前没有可信存储来源；生产写号不能通过普通 `121` 任意改写。 |
| `31` | 硬件 SN `hardware_sn` | `BYTES`，≤32B | —，将来 R | 当前没有可信存储来源；由制造流程提供，不能复制电机 SN。 |
| `32` | 硬件版本 `hardware_version` | `u32` 版本编码 | R 候选 | `GET_INFO.hardware_revision` 取共同目标清单的修订号 1；是编译目标声明，不是实际板卡自动识别。 |
| `33` | Loader 版本 `loader_version` | `u32` 版本编码 | —，将来 R | 当前 `GET_INFO` 填 0，不代表真实 Loader 版本；接入 Loader 元数据后开放。 |
| `34` | APP 版本 `app_version` | `u32` 版本编码 | R 候选 | 当前 `GET_INFO` 填 0，表示未提供可信发布版本；正式须与镜像元数据同源，不能两处独立维护。 |

飞书“电流环 Kp/Ki”“位置环 Kp/Kd”等是一组名称，并未给独立 ID；本表把 d/q 轴与串级环拆成独立标量 ID，是为了使 `READ_PARAM/WRITE_PARAM` 每次只操作一个值。`Rs/Ld/Lq/flux` 改写后不能静默保持旧电流环增益：要么在一个受控参数事务中重算并增加 revision，要么拒绝不完整的组合更改；方案在写路由前冻结。

## 3. 标定结果的可读参数

用户要求能获取标定参数。下表仅给当前固件**已有值来源**的候选 ID；标定执行仍走 `160/161/162`，大数组由条件保留的 `163 CALIBRATION_RESULT` 分页读取。通用 `121` 不直接写校准完成位、零位、三相电流偏置或摩擦拟合结果，否则会绕过标定有效性检查。

| ID | 名称 | 值与单位 | 权限 | 当前来源 |
| ---: | --- | --- | --- | --- |
| `0x40` | 编码器电角零位 | `u32`，Q15 圈内计数 | R/C | `OnBoard_Encoder.electrical_zero_q15` |
| `0x41` | 编码器机械零位 | `u32`，Q15 圈内计数 | R/C | `OnBoard_Encoder.mechanical_zero_q15` |
| `0x42` | 编码器方向 | `u32`，0/1 | R/C | `OnBoard_Encoder.reverse`；更改需重新验证零位与方向。 |
| `0x43` | 编码器标定有效标志 | `u32`，0/1 | R/C | `OnBoard_Encoder.calib_flag`；不得通过普通写命令强制置 1。 |
| `0x44`～`0x46` | A/B/C 相电流偏置 | 各 `i32`，千分之一 ADC count | R/C | `MotorControl.A_Offset/B_Offset/C_Offset`；读值要明确与采样比例的关系。 |
| `0x47`～`0x48` | 正/反向库仑摩擦 | 各 `i32` mA | R/C | `friction_coulomb_pos_a/neg_a`。 |
| `0x49`～`0x4A` | 正/反向粘性摩擦 | 各 `i32`，µA/(rad/s) | R/C | `friction_viscous_pos_a_per_rad_s/neg_a_per_rad_s`。 |
| `0x4B` | 摩擦模型有效标志 | `u32`，0/1 | R/C | `friction_model_valid`。 |
| `0x4C` | 电流采样分流电阻 | `u32` mΩ | R | 参数记录 `current_sense_shunt_milliohm`；与硬件 profile 不符时不能恢复旧限流。 |

编码器线性化 LUT 与齿槽转矩 1024 点表不拆成数千个 Param ID；使用 `163` 的分页结果及独立完整性校验。三相电阻诊断向量 `phase_resistance_vector` 是标定诊断结果而非持久运行参数，也由 `163` 提供。若 163 最终不启用，这些大数组就不能承诺通过 120 完整读取。

## 4. 请求/回复 payload 候选

公司统一外层 `type/seq/src/dst/CRC` 不变。本表取代旧草案的 `u16 parameter_id` 与固定 4B 值假设；所有请求和错误都有同 Type/同序号回复。结果码沿用待冻结的业务结果表，不直接序列化 C 枚举。静态表与版本化主机足以完成首版读写；条件保留的 `123 GET_PARAM_SCHEMA` 只在主机确实需要动态发现时实施。

| Type | 请求 payload | 成功回复 payload | 规则 |
| ---: | --- | --- | --- |
| `120 READ_PARAM` | `param_id:u8, reserved[3]=0`（4B） | `result:u16, detail:u16, param_id:u8, value_type:u8, value_len:u8, access:u8, revision:u32, value[0..32]`（12B+值） | `value_type=1 i32 / 2 u32 / 3 BYTES`；整数值长固定 4B，BYTES 长度明确；成功值从一次一致快照读取。 |
| `121 WRITE_PARAM` | `param_id:u8, value_type:u8, value_len:u8, reserved:u8, expected_revision:u32, value[4]`（标量 12B） | `result:u16, detail:u16, param_id:u8, value_type:u8, value_len:u8, reserved:u8, new_revision:u32, applied_value[4]`（16B） | 首版只允许表中 W-IDLE 标量；类型、长度、状态、权限、revision 与范围全部满足后原子应用。BYTES/SN 不通过此命令写。 |
| `122 SAVE_PARAM` | `expected_revision:u32`（4B） | `result:u16, detail:u16, persisted_revision:u32`（8B） | IDLE 且输出关闭；写入并校验持久记录后才回成功，失败保留可诊断状态。不能把 CAN ACK 当 Flash 成功。 |

错误回复统一为 `result:u16 + detail:u16`，其中至少区分 `UNSUPPORTED_ID`、`READ_ONLY`、`BAD_TYPE/LENGTH`、`OUT_OF_RANGE`、`WRONG_STATE`、`REVISION_CONFLICT`、`PERSIST_FAILED`。错误回复不得附伪造值。`121` 不自动保存；写成功后 `120` 可读到新活动值与新 revision。一次只写一个标量；跨参数一致性检查失败时整次拒绝、旧值不变。重试沿用公司头 `seq_id`，对同序号写入去重；新序号但旧 `expected_revision` 必须返回冲突。

例：在 IDLE 下把 ID `1` 最大速度改为 `1.250 rad/s`，线上原始值 `1250 = E2 04 00 00`，`121` payload（假定活动 revision=5）为 `01 01 04 00 05 00 00 00 E2 04 00 00`。身份和产品范围满足后才能应用；成功后读回 ID 1，并在需要掉电保留时发 `122`。示例不构成实机执行证据。

## 5. 实施门槛与兼容说明

当前调用链为 `yg_protocol_link` → `yg_protocol_owner_adapter` → APP 参数所有者 →
`parameter_value_service`；保存经维护流程提交真实参数存储。状态、身份、去重与 revision
由 APP 所有者负责，值服务只做确定性换算和约束应用。没有来源或产品范围未冻结的项
不得登记或声明能力；禁止恢复旧 `ParameterRead_Get` 或只读快照适配器兼容层。

旧草案中的 `u16 Param ID`、`Q/R` 固定格式、`1/2` 以外的设想，均不能与本表混用。正式上线若采用本表，主机和固件须一起切换协议版本；不要求保留旧候选兼容。具体安全范围不能用整数类型最大值充当产品界限，须由当前硬件 profile、轴型、标定状态与电机额定值共同确定。

## 准备资格增量契约 v1（2026-09-29）

0x4D 为配置确认 u32 0/1，停机可写；0x4E 为来源、0x4F 为电流模式启动受限位、
0x50 为基础自检事实位、0x51 为四种模式可启动位，均 u32 只读。原始 payload/序号/revision
不变；字段定义及来源见[基础自检与模式就绪](../../architecture/motor_readiness_v1.md)。
确认值必须显式写入，不因任一参数修改或 SAVE 自动变为 1；标定权限仍由各任务准入控制。
