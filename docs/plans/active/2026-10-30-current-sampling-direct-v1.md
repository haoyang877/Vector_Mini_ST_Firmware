# 三相电流直接赋值 v1

本阶段已由[电流采样接口一致性v2](2026-10-30-current-sampling-consistency-v2.md)取代，
当前采样通过统一的更新接口完成，以下保留上一阶段记录。

## 目标与边界

MotorAxis_FastTick 直接将三个 BSP 电流通道组成 ABCVect 写入 FocState，
删除仅转发结构体赋值的 MotorCurrent_Update 及声明，并同步测试和接口导航。
保持零偏完成门槛、A/B/C 对应关系、采样次数、NaN 传播、母线滤波和快环调用顺序。
不增加状态、不改变 FocState 布局、协议或控制算法；不宣称未经测量的时序收益。

## 基线、验收与回退

修改前源码在 outputs/current_sampling_direct_v1/before/。
修改前两板上电零偏和快环调度测试通过，日志为该目录的 baseline_startup.log。
验收复用两板上电零偏和电流精度用例，检查接口无残留，执行格式、lint 与 PR profile。
只撤销本轮差异，保留其他在途修改；不连接或操作硬件。

## 结果

- 已直接写入三相向量，删除 MotorCurrent_Update 的定义、声明、调用和测试兼容别名。
- 两板上电零偏、快环调度和电流精度测试均通过，与修改前基线一致。
- 两板 MotorAxis 与 motor_current 共四项生产文件编译通过，格式及 lint 通过。
- PR profile 的 doctor、architecture、interfaces 通过；母线测量、采样调度等相关用例通过。
  全量仍有既有格式、迁移后工程/文档路径、测试清单及其他迁移中的测试问题；
  执行调度旧夹具仍寻找已移除的 maintenance_runnable 代码片段。
- 日志与编译产物位于 outputs/current_sampling_direct_v1/；全量记录位于
  outputs/runs/current-sampling-direct-v1/summary.json。未验证实机时序，未操作硬件。
