# 电机域核心功能导航

本域提供电机控制、测量、反馈、标定和保护；APP拥有生命周期与调度，平台提供硬件能力。

| 目录 | 工作与边界 |
| --- | --- |
| 根目录 | 电机活动配置、运行类型、控制模式和任务结果等共享值契约 |
| control | 模式分发、位置和速度外环、PI、FOC及无感启动 |
| sensing | 相电流与上电零偏、母线电压、温度测量和关联故障 |
| feedback | 编码器标定数据、采样、角度与速度估计及磁链观测 |
| calibration | 编码器LUT、电零位任务和独立计算核心 |
| protection | 故障、绕组热模型、降额和恢复条件 |
| math | 确定性角度、滤波及数学运算 |

活动配置值保留在motor域；编译期应用、电机、板卡和目标配置统一放在`firmware/config`。
APP的MotorAxis调度20kHz快环和2kHz监督拍；状态机拥有启停与功率许可。
sensing和feedback通过BSP接口访问硬件；FOC计算与PWM提交分别位于`foc.c`和`foc_output.c`。
FOC计算仍会报告故障并参与快环计时。Axis持有`MotorMeasurement`供采样写入、
控制和保护读取；FOC保留坐标变换及控制估算值。

型号参数见[电机配置](../config/motors/README.md)，控制与维护流程分别见
[控制执行](../../docs/architecture/motor_execution_core_v3.md)和
[维护和标定](../../docs/architecture/motor_maintenance_core_v3.md)。

## 接口与所有权

- `motor_control.h`提供`MotorOuterLoop_SlowTick`和`MotorControl_Step`；APP决定调用许可。
- `motor_sensing.h`提供电流零偏、`MotorCurrent_Update`、`MotorVoltage_Update`和
  `McuTemperature_Update`；调用者提供快帧与测量对象，快环更新函数只搬运物理值。
- `encoder_feedback.h`和`flux_observer.h`提供反馈；调用者持有反馈对象并管理调度。
- `motor_fault.h`和`motor_thermal_control.h`提供故障和热限流；热历史由热控制模块持有。
- `encoder_calibration.h`提供标定任务入口；标定完成与Flash保存是独立事件。

快拍20kHz，监督拍2kHz；采样事务、关相检查和故障发布仍遵守原时序。
状态、单位、失败与中断上下文见各公开头文件。型号选择和完整APP默认值检查分开，
以保留台架与正式目标的不同准入；不根据板名自动切换电机型号。
