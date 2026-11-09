# 双板PWM与功率输出链路 v1

更新：2026-11-09。公开契约见[bsp_pwm.h](../../firmware/platform/bsp/include/bsp_pwm.h)。

## 职责与调用

PWM初始化、占空比换算、寄存器写入和启停均位于各MCU的pwm.c；工程选择实现，应用共用同一接口。

- 初始化：main → Bsp_Init → MCU PWM初始化。
- 控制输出：ADC中断 → main直接调用（AT32）或HAL完成回调（STM32） → MotorAxis_SampleFast → MotorAxis_ControlFast → FOC → BspPwm_SetDuty。
- 启相：电机状态机完成准入后直接调用BspPwm_Enable。
- 停相：STOP或故障路径调用BspPwm_Disable。

一次性通电窗口、装载时长、快拍/毫秒双截止、窗口功率契约及专用调试变量已删除。
2026-11-03 已删除 Commissioning 工程；普通 APP 的 PWM 功率链路保持不变。

| 项目 | AT32F456 | STM32G4 |
| --- | --- | --- |
| 通道 | TMR1 CH2/3/4，CH1采样 | TIM1 CH1/2/3，CH4采样 |
| 时基 | 180MHz，TOP4500 | 170MHz，TOP4250 |
| 写入 | MCU直接换算和写比较寄存器 | MCU直接换算和写比较寄存器 |
| 启停 | MCU直接操作输出位 | MCU直接操作输出位 |

BSP不提供窗口状态、提交超时或启停结果封装。三相比较值为独立写入。
热模型调用也已从运行链路移除；外环和FOC采用配置电流上限。

## AT32F456 一个 PWM 周期内的事件

AT32F456 的 CH2/3/4 输出互补 PWM，CH1 提供双 ADC 触发。定时器内部比较信号、带死区的 MCU 引脚、
实际栅极电压和比较缓冲装载的关系，统一见[AT32F456 电机驱动说明](at32f456_pwm_adc_driver.md)及其中的单周期时序图。

## 验证范围

上述时刻由寄存器配置推算。当前带载时的开关沿、放大器建立时间和最后一次比较寄存器写入相对谷底的余量尚无本轮实测。
