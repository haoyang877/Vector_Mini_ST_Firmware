# ADC应用中断入口显式位于main.c v1

## 目标

用户要求看到main函数，并要求ADC中断调用路径直接在主程序中体现。
两板项目入口分别为projects/at32f456/application/main.c与projects/stm32g4/application/main.c。
在各main.c显式定义BspApp_OnFastSample，直接调用App_FastTick。
硬件IRQ、标志确认和清除仍归MCU；不将中断轮询塞入main前台循环，不引入回调注册和状态。

## 实现

原app_tasks.c内BspApp_OnFastSample的完整应用处理改为App_FastTick实现，直接调用MotorAxis_FastTick。
删除原App_FastTick单行转发，调用层数不增加。功率窗口、采样计数、诊断和健康检查顺序不变。
两板main.c各自实现相同的BspApp_OnFastSample -> App_FastTick接线，main初始化顺序不变。
AT32两个APP的链接脚本将main.o中的BspApp_OnFastSample节放入原快速SRAM区。

## 核对与回退

仅作调用路径、定义唯一性、工程与链接选择的源码核对，不编译、不跑测试、不操作硬件。
保留原20kHz ADC中断执行上下文及两板原标志清除顺序。
修改前副本在outputs/adc-main-entry-20261030/before/，回退只撤销本轮片段。
当前状态：已实现，完成源码核对，未编译、未跑测试、未操作硬件。

## 核对记录

- 两板main.c各有一个BspApp_OnFastSample定义，均只调用App_FastTick。
- app_tasks.c仅有一个App_FastTick实现，原应用处理顺序保留，直接调用MotorAxis_FastTick。
- 两板MCU的adc.c没有修改，原IRQ、HAL回调及中断标志处理保留。
- 两个AT32工程和两个STM32工程仍分别选择本板main.c；AT32两份链接脚本新增对应快速代码节选择。
- 原生健康测试夹具补充被提取App_FastTick使用的采样计数变量，仅同步源码，未执行。
- 已格式化本次C/H修改并检查git diff --check；运行结果、时序及镜像资源仍未验证。
