# 三相与母线电流统一更新 v3

## 目标与不变行为

将MotorBusCurrent_Update合入MotorCurrent_Update(FocState *)，一次调用更新三相电流、
硬件母线电流和母线低通值。删除旧声明和调用，统一测试与当前接口文档。
读取顺序仍为A、B、C、母线；调用仍位于零偏完成门槛内，滤波算式、首次有效值、
非有限值传播及恢复、结构布局、控制与故障行为不变。不操作硬件或修改板级驱动。

## 基线、验收与回退

编辑前源码在outputs/current_update_merged_v3/before/；母线测量与双板电流精度基线通过。
上电调度基线因先前APP迁移移除motor_parameter_transaction.c而无法生成，记录在baseline_startup.log。
验收使用真实生产函数的合并采样、滤波、非有限恢复和读次数检查，复用双板电流精度，
编译受影响生产文件，完成格式、lint及PR profile。
仅反向应用本轮差异，保留其他在途修改。证据放在outputs/current_update_merged_v3/。

## 结果

- 已合并，MotorCurrent_Update只接收FocState，生产与测试代码无MotorBusCurrent_Update残留。
- 合并采样测试通过：每次调用四个电流通道各读一次，母线无效时仍更新三相。
  原有母线滤波、正负电流、NaN/Inf传播与恢复以及对象隔离断言均通过。
- 双板电流精度、当前上电零偏及快环调度测试通过。
  上电夹具在基线采集后已随并行APP迁移更新，最终可正常运行。
- 两板MotorAxis及motor_current共四项生产文件编译通过，本轮格式与lint通过。
- PR profile的doctor、lint、architecture、interfaces通过；相关电流与采样测试通过。
  全量仍有原有格式、devices目录引用、测试清单及其他迁移中的测试失败。
- 全量结果位于outputs/runs/current-update-merged-v3/summary.json，未操作硬件。
