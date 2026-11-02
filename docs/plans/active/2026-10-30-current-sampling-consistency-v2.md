# 电流采样接口一致性 v2

三相与母线电流的独立更新入口已由[统一更新v3](2026-10-30-current-update-merged-v3.md)合并，
以下保留接口一致性阶段记录。

## 目标与边界

替代 current-sampling-direct-v1 的直接赋值方案。三相电流、母线电流和母线电压
统一由 MotorCurrent_Update、MotorBusCurrent_Update、MotorVoltage_Update 读取 BSP 通道。
调用只传入所属 FocState，采样细节收回 sensing，三相按 a/b/c 成员明确赋值。
保持校零完成门槛、通道对应、每拍读取次数、滤波算式、非有限值和调度行为；
不改变结构布局、控制算法或硬件驱动，不新增状态与通用抽象。

## 基线与验收

本轮修改前副本位于 outputs/current_sampling_consistency_v2/before/，
双板上电零偏与调度基线已通过，日志为 baseline_startup.log。
同步全部生产调用及测试桩，复用双板上电、精度、母线滤波和电压窗口测试。
完成相关生产文件编译、格式、lint、接口搜索与 PR profile。
仅撤销本轮差异可回退；保留其他在途工作，不操作硬件。

## 结果

- 三相、母线电流与电压的更新接口均只接收FocState，通道读取位于sensing。
- 双板上电零偏、快环调度和电流精度通过，与修改前基线一致。
- 母线滤波、正负电流、非有限值恢复，以及1018946组电压窗口检查通过。
- 两板三个生产文件共六项编译通过；本轮格式与lint通过，全部调用已使用新签名。
- PR profile的doctor、lint、architecture、interfaces通过，相关采样测试通过。
  全量仍受既有格式、工程/文档目录迁移、测试清单及其他迁移测试失败影响。
- 专项日志及编译产物位于outputs/current_sampling_consistency_v2/；
  全量报告位于outputs/runs/current-sampling-consistency-v2/summary.json。
  未操作硬件，未验证实机时序。
