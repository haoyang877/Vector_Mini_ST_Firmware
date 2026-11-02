# FOC 测量成员直接合并 v2

## 目标和边界

按用户要求取消 measurement 成员和 MotorMeasurement 类型，将采样字段直接合入
FocState。控制、采样、状态机、标定、保护和遥测共用同一对象；同时传入 FOC 与
measurement 的接口删除后者，不保留别名或过渡包装。

实测母线电流沿用 bus_current_a/bus_current_filtered_a，原控制估算量改名为
bus_current_estimated_a/bus_current_estimated_filtered_a，保持二者的算法和消费语义。
不改变采样顺序、滤波、PI、故障策略、PWM、协议或持久化格式。

## 基线和验收

编辑前快照与变更清单位于 outputs/foc_measurement_flat_v2/。
本轮开始时用户已经展开了头文件字段，存在重复电流字段和空的 MotorMeasurement，
因此以此前已验证的数值行为、冻结参考和当前专项用例共同验收。
要求旧类型/头文件及成员层级不再出现在生产代码，双板核心编译、专项回归、格式、
lint 与 PR profile 均执行；既有参数宏和旧 BSP 测试问题分别记录。

## 回退与结果

只反向应用本轮差异，不覆盖并行进行的用户修改。
已删除 MotorMeasurement 头文件、结构体成员层级及重复参数，状态机和无感上下文
只保留一个 FOC 指针。生产代码中无旧类型、旧头文件或 measurement 成员访问。

- 两板配置下共34个生产翻译单元编译通过；当前状态总存储大小保持不变。
- 48000拍与合并前实现对照：控制量、占空比、PI和故障结果逐位一致；采样值未被改写。
- 上电零偏、状态快照、母线测量、252032组坐标以及三角函数测试通过。
- 指定板卡的无感交接测试通过；RTT实帧测试使用无硬件屏障桩通过。
- 本任务文件格式检查及lint通过。PR profile已执行，结果位于
  `outputs/runs/foc-measurement-flat-v2/summary.json`；全量仍存在既有工程路径、
  参数宏、旧BSP夹具、文档链接和无关文件格式问题，未宣称完整工程构建通过。
- 无硬件访问、无烧录。
