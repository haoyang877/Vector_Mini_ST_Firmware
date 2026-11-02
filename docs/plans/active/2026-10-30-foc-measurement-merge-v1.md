# FOC 合并测量对象 v1

> 本方案已由 [FOC 测量成员直接合并 v2](2026-10-30-foc-measurement-flat-v2.md) 替代，
> 当前类型不再保留 measurement 成员。

## 目标

删除 Axis 的独立 `s_motor_measurement`，将它内嵌为 `FocState.measurement`。
采样、控制、维护和状态发布都引用 `s_foc_state.measurement`，只保留一份存储。
`MotorMeasurement` 继续作为采样函数的类型，区分硬件实测电流与 FOC 估算电流。

## 不变行为

- ADC/SPI 采样和快慢环的执行顺序、滤波系数、零偏标定、故障判定。
- FOC 控制算法、PI 复位边界、PWM 输出、协议及持久化布局。
- 测量接口仍只拿到测量成员指针，不新增采样数据复制或动态分配。
- 新成员追加在 FocState 尾部，保持已有控制成员的偏移。

## 基线、验收和回退

编辑前源文件存于 `outputs/foc_measurement_merge_v1/before/`。
上电电流标定、状态快照及母线测量三组原生测试均已通过。
验收：同三组测试、FOC 坐标测试、当前调用扫描、格式与 lint、PR profile。
只回退本任务差异，不覆盖已有未提交工作或其它任务的变更。

## 结果

- 固件中的独立对象已删除；所有当前采样、初始化、控制和状态快照均指向
  `s_foc_state.measurement`。历史源码适配测试保留条件化的旧符号映射。
- 上电偏置测试在两板配置下通过；新增检查确认 Foc_ResetCurrent 不清除内嵌测量。
- 状态快照、母线滤波和 252032 组坐标变换测试通过。
- MotorAxis 在 BSP_BOARD=1/2 下分别编译通过；本次修改文件格式检查通过，lint 通过。
- PR profile 已执行，证据为 `outputs/runs/foc-measurement-merge-v1/summary.json`。
  全量仍因既有格式、工程路径、文档链接、旧测试接口和参数宏问题失败。
  当前测量夹具的路径替换问题已修复并单独复测，剩余编译失败为既有参数宏未定义。
- 本次日志位于 `outputs/foc_measurement_merge_v1/`；未连接硬件或烧录。
