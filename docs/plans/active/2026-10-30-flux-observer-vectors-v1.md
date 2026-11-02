# 磁链观测器输入向量 v1

## 范围

FluxObserverState 的电压、电流分别使用 AlphaBetaVector voltage_ab_v/current_ab_a。
FluxObserver_Update 的四个标量输入改为两个只读向量指针，更新所有调用和夹具。
坐标类型移入无外部依赖的 math/coordinate_types.h，由 FOC 与观测器共用。

## 不变行为与验收

保持单位、分量次序、浮点运算、模型检查、状态复位与位置纪元语义。
保留输入赋值在模型检查之后的原执行位置，不增加算法或采样行为。
基线快照位于 outputs/flux_observer_vectors_v1/before/。
验收包括完整状态逐位对照、上电调度与观测器用例、两板编译、接口搜索及 PR profile。
只反向应用本任务差异，保留其余在途修改；不连接硬件或烧录。

## 结果

观测器已改为两个AlphaBetaVector成员，公开更新接口为模型、两组向量和输出状态。
共享坐标类型仅定义于coordinate_types.h，不再让反馈模块依赖FOC控制头。

- 160000拍与本轮修改前源码逐位对照通过，覆盖非有限输入/状态、模型变化与重置。
- 状态大小、字段偏移不变；两个输入向量在调用后保持不变。
- 两板共6项核心编译通过，上电采样调用和252032组坐标测试通过。
- 观测器/标定热路径用例在指定板卡下两种阻尼配置均通过。
- 本次文件格式和lint通过；PR profile的doctor、lint、architecture、interfaces通过。
  全量检查未通过：已有格式问题、已迁移目录的工程/文档引用、测试清单漂移及其他迁移中的测试失败。
  观测器原测试驱动缺少BSP_BOARD定义；补充板卡编译参数后的两种阻尼配置用例均通过。
  PR profile证据位于`outputs/runs/flux-observer-vectors-v1/summary.json`。
- 未接触硬件或烧录。
