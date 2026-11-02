# 标定准入去重 v5

保留启停、三类标定和全部必要保护，不向其他生产文件迁移实现。
本轮最终仅保留标定受理分支的扁平化与重复标志复位删除。

基线位于outputs/lifecycle-admission-v5-20261030/before。
原先尝试给输出/标定校验传显式参数，但ARMClang生成代码增大，因此未保留。
最终仍使用原验证函数，保存一次校验结果，合并成功和代际溢出分支，保持拒绝结果顺序。
标定完成/候选发布标志仅在CALIBRATION中置位，离开会话由stop_motor清零；
初始化也清零，因此IDLE受理新标定时无需再次清零。迟到结果受原代际和状态检查约束。

motor_lifecycle.c由695行减为690行。使用相同夹具分别链接前后版本，两板启停、
三类标定、关相失败、故障恢复、迟到结果和外环抢占测试均通过。
ARMClang O3代码/只读段STM32G4为4310→4302字节、AT32为4172→4164字节，
静态RAM均保持64字节。这是独立对象段数据，不是完整镜像或实机时序。

证据见outputs/lifecycle-admission-v5-20261030/comparison.json及before/after日志。
最终PR检查位于outputs/runs/lifecycle-admission-v5-final-20261030。
本轮格式、lint、架构和接口检查通过。整仓仍有既有格式、路径及测试夹具失败；
与v4相比无新增失败测试。最终哈希与结论见同证据目录final-evidence.json。
未烧录、未执行台架。
