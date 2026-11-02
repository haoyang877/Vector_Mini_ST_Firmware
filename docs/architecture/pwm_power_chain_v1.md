# 双板PWM与功率输出链路 v1

更新：2026-10-30。公开契约见[bsp_pwm.h](../../firmware/platform/bsp/include/bsp_pwm.h)。

## 职责与调用

PWM初始化、占空比换算、寄存器写入和启停均位于各MCU的pwm.c；工程选择实现，应用共用同一接口。

- 初始化：main → Bsp_Init → MCU PWM初始化。
- 控制输出：ADC中断 → main.c的BspApp_OnFastSample → App_FastTick → MotorAxis_FastTick → FOC → BspPwm_SetDuty。
- 启相：电机状态机完成准入后直接调用BspPwm_Enable。
- 停相：STOP或故障路径调用BspPwm_Disable。

一次性通电窗口、装载时长、快拍/毫秒双截止、窗口功率契约及专用调试变量已删除。
Commissioning工程保留名称和其它工程设置，不再具有该窗口时限。

| 项目 | AT32F456 | STM32G4 |
| --- | --- | --- |
| 通道 | TMR1 CH2/3/4，CH1采样 | TIM1 CH1/2/3，CH4采样 |
| 时基 | 180MHz，TOP4500 | 170MHz，TOP4250 |
| 写入 | MCU直接换算和写比较寄存器 | MCU直接换算和写比较寄存器 |
| 启停 | MCU直接操作输出位 | MCU直接操作输出位 |

BSP不提供窗口状态、提交超时或启停结果封装。三相比较值为独立写入。
热模型调用也已从运行链路移除；外环和FOC采用配置电流上限。

## 当前验证范围

本次仅完成源码、工程引用和链接清单静态核对；未编译、未运行测试、未操作硬件。
旧阶段的窗口或PWM保护验证不代表当前实现已验证。
