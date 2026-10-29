# FocState 测量边界方案 v1（离线实施与验证完成）

## 目标与范围

将采样模块直接写入的测量值从FOC控制状态分离，令控制、保护和状态机显式读取
同一个Axis持有的测量对象。先保持现有物理意义、时间顺序和故障行为；不在此轮改变
算法、配置、协议、Flash格式、PWM许可或测量来源。正式目标为STM32G431和AT32F456
两个APP工程。用户已确认实施；本计划不授予硬件操作授权。

## 当前所有权：逐字段

| 字段 | 当前写入位置、读者 | 建议归属 |
| --- | --- | --- |
| `bus_voltage_v`、`bus_voltage_filtered_v` | `MotorVoltage_Update` 20kHz写；FOC限幅、观测器、保护、维护准入、状态机及遥测读取 | `MotorMeasurement`；原始值与滤波值均为V |
| `phase_a_current_a`、`phase_b_current_a`、`phase_c_current_a` | 零偏初始化设NAN；`MotorCurrent_Update` 20kHz写；FOC、观测器、过流/热保护和自检读取 | `MotorMeasurement`，单位A |
| `temperature_c` | `McuTemperature_Update` 2kHz写；状态机、恢复判断和状态快照读取 | `MotorMeasurement`；明确是MCU温度，单位°C |
| `current_alpha_a`、`current_beta_a` | 观测器和FOC均可能在快拍各自执行Clarke并写入；模式和早退路径会影响最后写入者 | 暂留`FocState`；单写者化须单独验证时序和观测值 |
| `id_a`、`iq_a` | FOC的Park变换写；停机清理显式置零；控制和状态/遥测读取 | 留`FocState`，为特定控制角度下的变换反馈 |
| `id_filtered_a`、`iq_filtered_a` | `Foc_Complete`在PWM提交后更新 | 留`FocState`，保持滤波历史和更新条件 |
| `bus_current_a`、`bus_current_filtered_a`、`power_filtered_w` | `Foc_Complete`利用调制比和dq电流估算；状态快照读取滤波电流 | 留`FocState`，是计算估算而非ADC实测 |
| `id_pi`、`iq_pi` | FOC配置/运行/复位；无感交接旋转积分 | 留`FocState`，控制器状态 |
| `mod_d`、`mod_q`、`mod_alpha`、`mod_beta` | FOC计算；观测器在下一次FOC前读取α/β调制 | 留`FocState`，不得改成当前拍提前计算的值 |
| `duty`、`duty_a`、`duty_b`、`duty_c`、`sector` | FOC调制计算；`foc_output.c`提交abc占空比 | 留`FocState`，控制输出及诊断 |
| `voltage_alpha_v`、`voltage_beta_v`、`vd_v`、`vq_v`、`current_alpha_filtered_a`、`current_beta_filtered_a` | 在当前固件对`FocState`字段检索未找到运行写入；诊断遥测读取`vd_v`/`vq_v` | 原样保留，不视为有效测量、不补写或删除；单独评估现有诊断语义 |

上述检索针对当前源码树；Watch、编译配置、夹具和目标差异须在执行时再次核实。
`BspAdcFastSample.bus_current_a`是另一条物理母线电流输入；有对应硬件能力时，
`YgProtocolStatusAdapter_Refresh`以`BusCurrentFeedback_Read`的实测值及其有效位覆盖
状态反馈。没有硬件能力时保留FOC估算。不能混用或改变失败时的NAN/有效位行为。

## 现有调用和时间关系

1. 初始化：Axis装配配置、零偏初始化并使三相电流无效；初始化FOC/PI和状态机。
2. 快拍：编码器采样开始 → 读取一次ADC快帧 → 更新母线电压及保护 → 刷新PI电压
   限幅 → 更新相电流或累计零偏 → 编码器读回启动 → 热保护 → 非位置模式下观测器。
3. 观测器读取本拍相电流与本拍已滤波母线电压，以及前一次FOC留下的`mod_alpha/beta`；
   它当前也可能写FOC的α/β电流。之后完成编码器采样，检查故障/立即关相，再分发
   维护或普通控制。FOC再次变换电流、计算并提交PWM，提交后才更新滤波和PI跟踪。
4. 监督拍先处理零偏发布、热看门狗、编码器慢估计、MCU温度，再执行状态机慢拍。
   快拍和监督拍频率不同，不能把整个测量对象宣称为同一次ADC同步快照。
5. 状态快照沿用`MotorAxis_ReadStatus`既有版本核对和有限重试；协议适配器可按硬件
   能力覆盖母线电流。迁移不能添加快环等待或假设简单拷贝天然跨中断一致。
6. 故障、STOP、维护取消及重启路径会影响电流有效性和dq清零；无感启动和编码器
   标定经维护链复用FOC，不能只迁移普通运行入口。

## 提议接口

建议新增`firmware/motor/sensing/motor_measurement.h`，只定义小型数据契约，
不包含BSP、FOC或读写函数；实例由`MotorAxis`持有。示意如下，字段名保留旧名称，
减少本轮不必要的命名及Watch歧义：

```c
typedef struct
{
    float bus_voltage_v;
    float bus_voltage_filtered_v;
    float phase_a_current_a;
    float phase_b_current_a;
    float phase_c_current_a;
    float temperature_c; /* MCU */
} MotorMeasurement;
```

- `MotorVoltage_Update`、`MotorCurrent_Update`、`MotorCurrentOffsets_Init`与
  `McuTemperature_Update`接收`MotorMeasurement *measurement`，按原位置写对应字段；
  `MotorConfiguration`、`MotorControlState`、ADC快帧、故障副作用和校正状态照旧。
- `Foc_UpdateCurrentLimits`、`Foc_Step`及内部准备/后处理、观测器、热保护、故障恢复、
  命令/维护准入等仅需读取采样值的接口接收`const MotorMeasurement *measurement`。
  其他必要的`FocState *`仍用于控制状态写入；沿调用链明确传递，不引入模块全局引用。
- 状态机同时持有两个显式对象：测量值只读，停机对`id_a`/`iq_a`及PI的清理仍作用于
  `FocState`。标定/无感调用链同时透传；诊断遥测分读两个对象并保持帧布局和字段语义。
- 快环传指针，不逐拍复制整对象，不添加锁、动态分配、等待或隐式发布。温度有效性
  暂仍由现有`McuTemperature_IsFresh`提供；未规划新标志或新鲜度算法。
- `s_motor_measurement`作为Axis持有的Watch对象，更新当前Watch文档；
  `s_foc_state`保留控制状态。结构布局与Watch表达式会改变，必须使用匹配AXF；不假设
  其是稳定二进制接口。避免保留一套影子字段，因为复制和一致性会增加风险。

核心接口形态示意（参数名与排列是待确认设计，调用契约以执行时头文件为准）：

```c
void MotorCurrentOffsets_Init(CurrentOffsetCalibration *calibration,
                              MotorConfiguration *configuration,
                              MotorMeasurement *measurement);
void MotorVoltage_Update(MotorMeasurement *measurement, MotorControlState *motor,
                         bool output_expected, const BspAdcFastSample *sample);
void MotorCurrent_Update(const MotorConfiguration *configuration,
                         MotorMeasurement *measurement, MotorControlState *motor,
                         const BspAdcFastSample *sample);
void McuTemperature_Update(MotorMeasurement *measurement, MotorControlState *motor);
void Foc_UpdateCurrentLimits(FocState *foc, const MotorMeasurement *measurement);
void Foc_Step(const MotorConfiguration *configuration, FocState *foc,
              const MotorMeasurement *measurement, MotorControlState *motor,
              FocControlLaw law, float phase, float phase_vel,
              float d_reference, float q_reference);
void FluxObserver_Update(const MotorConfiguration *configuration, FocState *foc,
                         const MotorMeasurement *measurement, FluxObserverState *observer);
```

这里`FluxObserver_Update`暂时仍接收可写的`FocState *`，因为当前它会写α/β电流；
把它改为只读或重排Clarke计算属于后续行为变化，不能作为本轮类型迁移的隐含结果。

关键兼容细节：原电流零偏非法时报告故障但不主动覆盖已有三相值；无效快帧时设NAN。
母线滤波、温度无效和首次有效更新必须保持原逻辑；包括滤波在失效后的行为，不能
借迁移重置或修复。保持所有早退、故障优先级、PWM失败后处理和STOP清零时点。

## 建议实施阶段与验收

1. 经确认后先建立与当前实际工作区匹配的基线，记录两个正式工程HEX、map、资源、
   离线测试及结构/Watch；不使用过期产物作为基线。保留既有未提交工作。
2. 新增测量类型及Axis实例，迁移采样、保护、状态机、控制和维护的传参；同步所有
   实际调用者、测试夹具、目标工程及Watch/接口文档。分小步编译，保证无隐式双写。
3. 按相同输入逐拍对照三相、电压、温度、FOC状态/输出、故障、状态和PWM事件；
   覆盖初始化、无效/缺帧、零偏非法、过流/过压、温度超时、STOP、标定、观测器、
   PWM失败和过滤历史。协议与RTT帧布局及实测/估算选择不变。
4. 运行PR完整检查，重建两套正式APP并核对AT32快环SRAM/Flash放置、代码和RAM/栈
   资源、HEX差异；记录差异而不假定字节一致。比较实际代码生成与快环调用成本，
   目标板最坏时序和闭环行为仅在用户明确授权台架场景后验证。
5. 若任一步不满足行为、构建或时序要求，回退本阶段的类型、调用及文档修改，保留
   其他工作区改动；未完成前不宣称发布或硬件验收通过。

## 实施及验证记录

- 基线源码归档`outputs/foc_measurement_20261029/baseline_source.zip`；两个正式APP
  基线构建均0错误0警告，HEX、map和AXF在`baseline_build/`。基线PR中的测试、架构、
  接口和文档检查通过，格式检查因既有状态机格式差异失败；本轮接触该文件时已格式化。
- 六个字段已移入Axis持有的`MotorMeasurement`，采样写入、控制/保护只读；
  α/β和dq计算、PI及估算母线电流保留在FOC，状态机及标定链显式传递测量对象。
- 零偏初始化、电流、电压和MCU温度四函数经字段所有者投影后与本次冻结基线一致，
  见`outputs/foc_measurement_20261029/sensing_source_comparison.json`。
- 冻结旧FOC状态布局独立对照两种计时配置各61440组；热保护、采样失效、
  启动零偏、无感切换、标定、状态机规则及遥测测试通过，证据在同一输出目录。
- 固定HEAD状态机逐事件对照出现一处关断临界区深度差异，因工作区中并行状态机
  中断优化而不能判定逐事件完全一致；当前状态机专项断言与262144组规则通过。
- 本轮工作区有并行状态机调整，构建、资源和HEX差异为合并结果，不可单独归因于
  测量对象迁移；未接触硬件，快环实际最坏时序与闭环行为未测。
- 最终PR检查通过：`outputs/runs/20261029T111449336010Z-2dbb4003/summary.json`；
  另有本轮PR通过：`outputs/runs/foc-measurement-final-20261029/summary.json`。
- 最新两板正式APP均0错误0警告。构建哈希、HEX比较、map及AT32关键符号位置见
  `outputs/foc_measurement_20261029/final_build/comparison.json`；所有要求的快环符号
  位于SRAM，LUT发布位于Flash。两套HEX与基线不同。
- 合并工作区相对本轮基线：STM32 Code 83940→84108、RO 3236→3236、RW 328→328、
  ZI 18504→18512字节；AT32 Code 86312→86548、RO 3568→3572、RW 376→376、
  ZI 34456→34464字节。包含并行状态机修改，不单独归因。Keil静态调用图最大栈
  分别为696和664字节并带Unknown项；缺少相同方法的基线调用图，不推断栈增量。
- `Foc_UpdateCurrentLimits`、`Foc_Prepare`与`Foc_Complete`在参数及字段所有者投影后
  与本轮冻结源码一致，见`outputs/foc_measurement_20261029/foc_source_comparison.json`。
  软件检查不证明20kHz最坏执行时间，也不替代用户授权的实机验证。
