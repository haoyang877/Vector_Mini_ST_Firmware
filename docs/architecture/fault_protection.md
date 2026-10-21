# 当前故障、关断与恢复

核对日期：2026-10-21。本文按当前实现描述职责，取代旧故障保护设计稿中的拟议接口。

| 所有者 | 实际职责与入口 |
| --- | --- |
| [`foc_sensing`](../../firmware/motor/foc/foc_sensing.c) | ADC偏置、相电流、母线和MCU温度判定及测量资格 |
| [`motor_fault`](../../firmware/motor/protection/motor_fault.c) | 故障锁存、过流恢复驻留和源恢复矩阵；不启相 |
| [`motor_thermal_control`](../../firmware/motor/protection/motor_thermal_control.h) | 快环热历史/动态电流额度，失效及策略边界；监督监控采样窗口 |
| [`motor_axis`](../../firmware/app/runtime/motor_axis.c) | 调度采样与保护，执行快速故障关断，向监督提供一致事实 |
| [`motor_state_machine`](../../firmware/app/motor_state_machine/motor_state_machine.c) | 唯一启停许可、FAULT/CLEAR/FATAL及实际资源释放 |
| [`power_stage_hw`](../../firmware/platform/api/power_stage_hw.h) | 功率关闭、启用与实际数字回读；寄存器实现位于STM32端口 |

快环故障路径不等待前台或2kHz监督才关相。源恢复只是一项事实，
不能直接清锁存、恢复目标或重新出力。`MotorFault_Report`保留首个故障，
原CAN断连故障可被后续更具体故障替代；它不是多故障位图存储器。
协议故障位图由状态适配器显式映射，不直接发送内部枚举。

## CLEAR与停止

FAULT中的明确CLEAR须满足源恢复、完整基础健康、输出关闭、取消确认及维护/前台资源已归还，
成功只回IDLE；旧目标和任务不恢复。过流恢复使用2kHz下2000拍驻留证据；
MCU温度恢复低于80°C；电压使用启动恢复区间。详细矩阵以 `MotorFault_SourceRecovered` 为准。
FATAL没有普通软件事件出口，不能以STOP、CLEAR或重连解除。
停止受理、数字关相、资源归还和机械静止是不同事实，均按对应接口确认。

## 热保护边界

热模型基于快环三相电流平方，每20拍更新；停机后保留热历史估计冷却，MCU复位不保留历史。
电流闭环按可用额度降额，开环电压/功率标定不允许用降额继续，触及策略边界则故障关断。
失效输入或采样看门狗失败禁止出力；不能通过CLEAR伪造新鲜样本。
热配置的停机重配保留并保守换算历史；普通参数变更不清空热量。

门限与测量细节见[母线保护](../guides/bus_voltage_8s.md)、[MCU温度](../guides/mcu_temperature.md)，
状态转换见[八状态契约](motor_state_machine_v1.md)。本页不声明新实机或硬件额定能力。
