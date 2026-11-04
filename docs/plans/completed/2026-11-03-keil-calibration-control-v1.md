# Keil全局标定控制 v1

## 目标与范围

扩展现有正式APP的Keil全局调试命令，支持选择编码器线性化、电气零位和机械零位。
按用户澄清，选择量为非static的volatile全局变量，不更改既有标定名称、算法、参数记录或CAN编号。
新增独立的标定类型全局量和命令4，沿用原有命令0/1/2/3及结果回写握手。

## 实施约束

由App_Process前台单次消费，并转交MotorLifecycle_RequestCalibration。
默认类型0为未选择，拒绝未选择或非法类型；不自动标定、不重试、不清错、不保存Flash。
启动检查、故障行为、标定执行和取消沿用现有生命周期；命令3继续请求停止或取消。
不改BSP、ISR、标定算法或现有全局控制变量的意义，不使用goto。

## 基线与验收

基线是已通过完整PR及四目标构建的keil-global-control-20261103。
修改前app.c、专项测试和指南备份至outputs/keil-calibration-control-20261103/。
专项测试覆盖三类映射、未选择及非法值、原结果码传播、一次性消费、
模式及目标参数隔离、正常停止路径，以及既有启停和目标行为。
运行完整PR及STM32正式/LoaderApp、AT32正式/Commissioning四个目标构建。
检查全局符号与资源变化；不烧录或执行任何实机动作。

## 回退

仅撤销标定类型全局量、命令4分支及对应测试和指南变更，保留此前已确认的全局启停功能。

## 完成结果（2026-11-03）

新增g_debug_calibration_type，非static且volatile，缺省为0（未选择）。
取1/2/3分别选择编码器线性化、电气零位或机械零位，再写g_debug_command=4提交。
未选择及非法值返回INVALID；所有请求只消费一次，原结果回写后清零命令。
命令3保留正常停止/取消路径，不依赖标定类型、运行模式或目标是否合法。

双板专项测试及完整PR通过：43组原生检查、140项单元和16项集成；
格式、lint、工程布局、架构、接口、文档及卫生检查均通过。
PR证据：outputs/runs/keil-calibration-control-20261103-pr/summary.json。

STM32正式/LoaderApp、AT32正式/Commissioning四个目标均0错误0警告。
四份MAP均确认七个g_debug_*全局符号，包括4字节的标定类型选择量。
相对上一轮keil-global-control-20261103，各目标资源变化如下（字节）：

| 目标 | Code增量 | RO增量 | RW增量 | ZI增量 |
| --- | --- | --- | --- | --- |
| STM32正式 | 44 | -4 | 4 | -4 |
| STM32 LoaderApp | 48 | 0 | 4 | -4 |
| AT32正式 | 40 | 0 | 4 | 4 |
| AT32 Commissioning | 40 | 0 | 4 | 4 |

标定选择量增加4字节RW；ZI变化来自对象对齐填充，已有对象大小不变。
构建及符号证据在outputs/keil-calibration-control-20261103/builds/，
资源对照在同轮resource_comparison.json。
仅完成离线验证，未烧录、未启相；标定的实机完成情况未在本轮验证。
