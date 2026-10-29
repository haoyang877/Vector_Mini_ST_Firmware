# 当前电机模型与单位

核对日期：2026-10-23。算法内部使用SI浮点量，通信在边界显式缩放。
配置与运行数据定义见[`motor_config.h`](../../firmware/motor/motor_config.h)、
[`motor_types.h`](../../firmware/motor/motor_types.h)及[`foc_algorithm.h`](../../firmware/motor/foc/foc_algorithm.h)。

| 量 | 算法单位 | 线上常用尺度 |
| --- | --- | --- |
| 机械位置/速度 | rad / rad/s | i32 mrad / mrad/s |
| 相电流、d/q电流 | A | 参数/目标mA；反馈i16 mA |
| 母线电压 | V | 反馈i16 cV，即0.01V |
| q轴电压目标 | V | i32 mV |
| 相电阻 | Ω | 参数raw为Ω×10⁶ |
| d/q电感 | H | 参数raw为H×10⁹ |
| 磁链 | Wb | 参数raw为Wb×10⁹ |
| 转矩常数（电机轴 / Iq幅值） | N·m/A | 仅型号配置，当前无线上字段 |
| 转子惯量（电机轴） | kg·m² | 仅型号配置，当前无线上字段 |
| 温度 | °C | 反馈i16为0.01°C；仅MCU实测来源 |
| 编码器环形角 | 65536 counts/转 | Q15历史命名不表示32768 counts/转 |

尺度和权限最终见[参数目录](../protocols/yg_protocol_parameter_catalog_v0_1.md)及
[45B状态字段](../protocols/yg_protocol_motor_wire_v1.md)。无效或溢出反馈必须清有效位并用哨兵，
不能饱和成看似有效的测量。Iq命令是电流，当前没有由Nm目标到电流的公开线路。

Id=0时，`Te=1.5×p×ψ×Iq`，因此`ψ=Kt/(1.5×p)`；此处Kt为电机轴总转矩/Iq幅值。
YQ-5708/5740的Kt=0.0801N·m/A、p=14，得到ψ=0.003814286Wb。
用户确认转子惯量为0.0005kg·m²，不按截图的e-4标签再次缩放，也不乘减速比。
惯量尚未接入控制算法；未知惯量以`PARAM_MOTOR_ROTOR_INERTIA_VALID=0`表示，
不能把占位0用于计算。公式依据见[输入记录](../hardware/r40_at32f456_configuration_v1.md)。

机械/电角速度通过极对数换算，rpm=rad/s×60/(2π)，机械角度deg=rad×180/π。
电流采样硬件输入与既有固定限值归
[`vector_mini_st.h`](../../firmware/platform/config/boards/stm32g4.h)，
经 `bsp_config.h` 选择，派生比例仍归
[`sampling_scale.h`](../../firmware/platform/config/sampling_scale.h)。
2mΩ/6mΩ是不同编译配置，ADC满量程、可命令限流和过流保护门限是不同量。
4×过采样累加先乘0.25恢复12bit count口径，再减本次零偏并换算A，见[采样说明](current_oversampling.md)。

PI的输入/输出单位取决于调用契约；位置串级速度增益仍乘配置标称电流上限。
热模型的heat是无量纲估计量，不是电机温度传感器读数；速度前馈增益单位A/(rad/s²)。
参数、控制器和遥测换算不能直接共享一个“通用比例”代替各自契约。
