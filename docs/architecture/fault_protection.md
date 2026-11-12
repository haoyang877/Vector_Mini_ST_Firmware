# 当前故障、关断与恢复

核对基线：176f3c7e。本文按当前实现描述职责，取代旧故障保护设计稿中的拟议接口。

| 所有者 | 实际职责与入口 |
| --- | --- |
| [`motor_sensing`](../../firmware/motor/sensing/motor_current.c) | ADC偏置、相电流、母线和MCU温度判定及测量资格 |
| [`motor_fault`](../../firmware/motor/protection/motor_fault.c) | 故障锁存、过流恢复驻留和源恢复矩阵；不启相 |
| [`motor_thermal_control`](../../firmware/motor/protection/motor_thermal_control.h) | 保留热模型控制接口，当前 APP 未调度 |
| [`motor_axis`](../../firmware/app/motor_axis.c) | 调度采样与保护，执行快速故障关断，向监督提供一致事实 |
| [`motor_state_machine`](../../firmware/app/motor_state_machine.c) | 启停许可、故障锁存、清错及实际关相 |
| [`BspPwm`](../../firmware/platform/bsp/include/bsp_pwm.h) | 功率关闭、启用与实际数字回读；寄存器实现位于各板PWM BSP |

快环故障路径不等待前台或2kHz监督才关相。源恢复只是一项事实，
不能直接清锁存、恢复目标或重新出力。`MotorFault_Report`保留首个故障，
原CAN断连故障可被后续更具体故障替代；它不是多故障位图存储器。
协议故障位图由状态适配器显式映射，不直接发送内部枚举。

## CLEAR与停止

FAULT 中显式 CLEAR 要求关相读回、有效采样与配置、标定会话空闲、故障源恢复并通过启动/停机锁检查，
成功只回IDLE；旧目标和任务不恢复。过流恢复使用2kHz下2000拍驻留证据；
MCU温度恢复低于80°C；电压使用启动恢复区间。详细矩阵以 `MotorFault_SourceRecovered` 为准。
实际关相失败等情况会置 `fault_locked`，状态机拒绝普通 CLEAR。
停止受理、数字关相、资源归还和机械静止是不同事实，均按对应接口确认。

## 热保护边界

当前 APP 使用配置电流额度，绕组热模型的计算接口和存储字段保留。运行时 MCU 温度测量与绕组热模型是两种不同来源。采样、恢复计数及 CLEAR 的具体要求以 [MotorFault_SourceRecovered](../../firmware/motor/protection/motor_fault.c) 为准。

门限与测量细节见[母线保护](../guides/bus_voltage_8s.md)、[MCU温度](../guides/mcu_temperature.md)，
状态转换见[电机五态与功率交接](motor_state_machine_v1.md)。本页不声明新实机或硬件额定能力。
