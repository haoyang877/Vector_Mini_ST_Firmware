# ADC辅助温度快照与单次VDDA换算 v1

## 目标
原码、序号、采样时间在一个私有快照中传递；MCU层只做纯换算，一次计算VDDA，分别返回温度及电压有效性。公共层拥有取样、去重、后续触发和发布，不新增转发层。

## 不变行为
公开BSP API和数据布局不变；温度与电压的有效范围、错误分类、超时及故障保护保持不变。STM32在换算后、发布前请求下一转换；AT32保持自动DMA采样。快环、PWM、NTC算法和应用保护不变。不烧录、不操作硬件。

## 基线
修改前源文件及SHA-256保存在 outputs/adc-temperature-snapshot-20261029/before.json。上一轮双板及PR证据位于 outputs/bsp-v1-implementation-20261026/adc-direct-results-final-20261029、outputs/runs/adc-direct-results-final-20261029。

## 验收
运行真实公共ADC与两板MCU实现；检查换算期间新帧到达不会混用序号或时间戳、发布时刻与读取时刻不同、去重、回绕、失效、独立温度/VDDA有效性、原码读取不换算。保留温度滤波、过温原始值跳闸、缺样超时、已有故障不覆盖等回归。AT32 Commissioning与STM32应用重建，运行完整PR验证。比较公开接口、应用保护源及构建资源。

## 回滚
仅恢复本轮before快照对应修改；保留用户及其他任务的变更，不重置仓库。最终证据与结果在完成后补充。

## 实现结果
- AdcAuxSample、AdcTemperatureSample各自携带sampled_ms，删除分散的s_aux_ms、s_temperature_ms。
- 删除AdcTemperaturePoll、AdcTemperature_Poll与McuAdc_ReferenceVoltage；以同帧纯换算McuAdc_ConvertTemperature返回两个独立结果。
- AT32温度去重从MCU可变状态移至公共层已有缓存；STM32也使用同一逻辑。没有新增生产文件或转发层。
- 温度原码错误、缺温度校准不使有效VDDA失效；NTC仍使用原分压/B值算法。
- AT32未声明温度校准时在编译期排除温度公式，不执行零斜率计算，仍单独验证内部参考校准。
- 采样时间保持原发布点语义：AT32是DMA完成发布，STM32是消费JEOS并发布；不是精确采样保持时间。

## 验证中已确认
- 冻结实现：outputs/bsp-v1-implementation-20261026/adc-temperature-final-20261029，提交628a8ab3cca175b82d119b70cef6c2e27683f2d9（仅证据目录内Git提交）。
- 两板公开BSP头文件、应用/电机保护源均与上轮验证版本一致。
- 两板Adc_CompleteFast、BspAdc_ReadRawCurrent、read_frame反汇编逐段一致。
- 原码读取不换算、元数据同帧、发布时刻早于处理时刻、新帧插入、序号回绕、去重、传感器失效、温度与VDDA独立状态均有离线覆盖。
- 新旧算法对照：AT32 77,824组、STM32 544,768组；覆盖完整12位温度原码、参考值边界及STM32多组有效/失效校准，合法输入的有效物理量逐位一致、错误分类一致。
- AT32 Commissioning与STM32应用均0错误/0警告。AT32 Code81976/RO3400/RW340/ZI22036；STM32 Code83560/RO3244/RW312/ZI18464。AT32工作区另有编码器变更，整体体积差异不全部归因于本轮。
- bsp_adc.c 522→528行；AT32 temperature.c 154→91行；STM32 temperature.c 74→50行；bsp_internal.h 426→417行。减少的是采集/轮询与换算之间的重复状态和转发。
- 比较证据：outputs/adc-temperature-snapshot-20261029/{invariants.json,numeric-equivalence.json,code-comparison.json}。编译证据见outputs/bsp-v1-implementation-20261026/adc-temperature-final-{at32,stm32}-build-20261029/builds.json。
- 最终完整PR通过：outputs/runs/adc-temperature-final-20261029/summary.json。doctor、format、lint、project-layout、全部测试、architecture、interfaces、docs、hygiene均PASS。
- 汇总验收及构建、源码SHA-256：outputs/adc-temperature-snapshot-20261029/validation.json。最终复核六份生产源与已构建快照逐字节一致；未连接硬件、未烧录。
