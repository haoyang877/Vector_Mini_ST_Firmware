# 电机配置

板卡选择与电机选择独立。公共入口为`motor_profile_selection.h`，所有型号电流环带宽为1kHz。
20kHz控制频率不变；默认PI按`Kp=L×2π×1000`、`Ki=R×2π×1000`计算，Ki不预乘Ts。

| 电机/配置 | `FOC_ACTIVE_MOTOR_PROFILE` | `MOTOR_HAS_DAMPING_RING` | 参数文件 |
| --- | --- | --- | --- |
| HT8115-4，无阻尼环 | `FOC_MOTOR_PROFILE_HT8115_4`（1，默认） | 0（默认） | [ht8115_4.h](ht8115_4.h) |
| HT8115-4，有阻尼环 | `FOC_MOTOR_PROFILE_HT8115_4`（1） | 1 | 同上；复用现有启动/标定及前馈开关 |
| YQ-5708，8:1 | `FOC_MOTOR_PROFILE_YQ_5708`（2） | 0 | [yq_5708.h](yq_5708.h) |
| YQ-5740，40:1 | `FOC_MOTOR_PROFILE_YQ_5740`（3） | 0 | [yq_5740.h](yq_5740.h)，复用5708电气参数 |
| YQ-8116，暂定HT基线 | `FOC_MOTOR_PROFILE_YQ_8116`（4） | 0 | [yq_8116.h](yq_8116.h)，后续单独修订 |

Keil在目标的C/C++ Define中选择型号和阻尼环；例如
`FOC_ACTIVE_MOTOR_PROFILE=3U,MOTOR_HAS_DAMPING_RING=0U`选择YQ-5740。
型号选择对整目标保持一致，不根据板名、CAN地址或生产标签自动切换。
AT32 ClosedLoop独立台架未显式选择时使用YQ-5708，STM32完整APP默认HT无阻尼环。
AT32完整APP的关相/限时目标在Keil的Misc Controls预包含一个
`platform/config/targets/r40_app_*.h`配置头，C/C++ Define只保留SDK芯片选择。
目标头集中指定组合、调试策略和临时热数据；不要再次在各算法文件或工程Define覆盖数值。
共用速度估计默认值见`encoder_feedback_defaults.h`，默认90Hz；限时目标800Hz仍待完整速度验收。
各型号的无阻尼环标定速度也在型号文件中，公共启动配置只消费数值。
YQ-5740为保持行为仍保留原315rad/s电角目标、15rad/s机械目标，两者与14极对不一致，
未获准沿用5708的标定验收，后续应作为独立功能问题处理。

`motor_profile_defaults.h`是完整APP的兼容入口，只接受具有完整默认参数的型号。
YQ-5708/5740磁链已按电机轴Kt=0.0801N·m/A及14极对反算为0.003814286Wb，
转子惯量为0.0005kg·m²；热参数仍待实测。YQ-8116全部继承输入均为暂定。
`PARAM_MOTOR_TORQUE_CONSTANT_NM_PER_A`按电机轴转矩/Iq幅值定义，
`PARAM_MOTOR_ROTOR_INERTIA_KG_M2`为电机转子的SI惯量；各自配套`_VALID`标志。
HT转矩常数由既有磁链推导，非独立实测；两种HT装配的惯量未知，数值0仅为无效占位。
YQ-8116转矩常数及惯量均未确认。惯量先用于配置记录，不接入算法、协议或Flash布局。
三者允许编译电气配置和独立台架，当前禁止装配为可驱动的完整APP默认配置。
工作区独立关相APP集成目标暂用3600s热时间常数及3A持续/峰值热电流，
用于关相集成验证；另对用户明确授权的独立限时验证工程开放相同临时数据，
不改变正式可驱动目标的准入，也不代表热保护已验收。
这不限制YG_ESC_V1.0后续支持这些电机，补齐参数及对应APP适配后再开放。

减速比和电机额定电流只是配置输入，不自动改变台架限流、速度门限或输出轴反馈。
更换电机不能复用旧设备的编码器零位、LUT及未核对的Flash模型；
现有有效schema12/13/14的人工PI保持原装载行为，1kHz默认值不强制覆盖Flash。

新增型号时添加一个参数文件及一个选择分支，再增加型号矩阵测试；
HT两种阻尼环的机械参数后续在型号/装配配置中分别完善，不改控制算法与BSP接口。
