# FOC 电压向量成员 v1

## 范围与不变行为

将 FocState 的 voltage_alpha_v/voltage_beta_v 合为 AlphaBetaVector voltage_ab_v，
将 vd_v/vq_v 合为 DqVector voltage_dq_v。同步读取、初始化和回归对照。
保持单位 V、成员顺序、内存布局、控制数学及线路帧不变，不新增电压计算。
控制命令、遥测帧与冻结参考中的同名字段保留各自既有契约。

## 验收和回退

编辑前快照位于 outputs/foc_voltage_vectors_v1/before/。
检查当前 FocState 不再声明四个独立标量；运行坐标、FOC 布局/状态对照、RTT、
双板编译以及格式/lint 和 PR profile。仅回退本任务差异，不覆盖其它在途修改。

## 结果

FocState已使用两组向量，不保留标量别名；遥测和当前测试读取均已更新。

- 48000拍与标量参考逐位对照通过，包括非零电压成员、PI、占空比和故障结果。
- 两组向量偏移与原标量首成员偏移一致，状态总大小不变。
- 非零d/q电压的真实RTT帧测试通过；两板共6项核心翻译单元编译通过。
- 修改文件格式检查和lint通过。PR profile证据位于
  `outputs/runs/foc-voltage-vectors-v1/summary.json`；全量仍受既有工程、测试和文档问题影响。
- 无硬件访问、无烧录。
