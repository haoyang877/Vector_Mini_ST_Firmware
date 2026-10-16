# Keil Watch 只读观测指南

更新：2026-10-09。适用核心 APP v1.3.0 主工程：`YG_ESC_APP`。
本页描述人工观测步骤和当前源码中的变量，不授权烧录、复位或电机运动。
旧 SWD 台架请求邮箱的退役记录保留在文末，不能作为当前操作入口。

## 调试前检查

1. 先确保电机功率输出关闭。连接调试器可能暂停或复位 MCU；首次配置宜在电机功率断电、
   板卡逻辑供电仍有效的条件下进行，具体供电方式按实际硬件确认。
2. 仅观察板上现有固件时，在 `Options for Target → Utilities` 关闭
   `Update Target before Debugging`，避免进入调试时自动更新 Flash。
3. 核对 `Debug` 页的初始化脚本、`Run to main()`、恢复断点及驱动连接/复位设置。
   进入会话不能视为无扰动连接；不要在电机出力过程中停在 `main()` 或其他断点。
4. 加载与板上实际镜像一致的 AXF。不要用最新结构体布局或旧 RAM 地址解释不同固件。
   HEX 没有类型符号，不能替代 AXF 做结构体观测。
5. 需要单独替换符号时，在已安全连接的 Debug Command 窗口使用以下命令，
   将示例路径替换为匹配的 AXF 路径；引号内路径使用双反斜杠：

   ```text
   LOAD "C:\\matching-build\\YG_ESC_APP.axf" NOCODE CLEAR INCREMENTAL
   ```

   `NOCODE` 不写入程序，`CLEAR INCREMENTAL` 替换符号且避免该 LOAD 命令复位目标。
   这些选项不保证连接调试器本身不会暂停或复位 MCU。

Keil 官方说明：[J-Link Flash 下载设置](https://www.keil.com/support/man/docs/jlink/jlink_configflash.asp)、
[LOAD 命令与 NOCODE](https://www.keil.com/support/man/docs/uv4/uv4_cm_load.asp)。

## 添加变量与刷新

- 在 Debug 会话打开 `View → Watch Windows → Watch 1/2`，双击 `<Enter expression>` 添加表达式。
- 开启 `View → Periodic Window Update`，支持的调试驱动可在 CPU 运行时刷新值。
- 当前源码按用户要求将 MotorAxis 运行对象和 `s_context` 改为非静态全局符号，
  保留既有 `s_` 名称，可直接添加短名并展开成员。历史冻结 AXF 仍可能是 `static` 实例；
  短名无法解析时，在 `View → Symbols` 中选择对应源码模块实例并添加完整限定名。
  始终以匹配板上镜像的 AXF 为准，不硬编码 RAM 地址。
- Watch 中只输入无副作用的读表达式，不加入赋值、递增/递减或目标函数调用。
  不编辑 Value 列；刷新得到的多个字段不是同一时刻的原子快照。

操作方式见 [Keil Watch 官方说明](https://www.keil.com/support/man/docs/uv4/uv4_db_dbg_watchwin.asp)。

## Watch 1：状态、安全与维护

`s_motor_state`、`s_foc_state`、`s_rotor_encoder` 和 `s_config_revision` 归
`firmware/app/runtime/motor_axis.c`；`s_context` 归
`firmware/app/motor_state_machine/motor_state_machine.c`。下表使用短名表示成员路径，
实际 Watch 名称以加载的 AXF 和 Symbols 生成的限定名为准。

| 表达式 | 单位或含义 | 观察重点 |
| --- | --- | --- |
| `s_context.rules.state` | 主状态枚举 | 待命为 IDLE；IDLE 不等于所有模式均可启动，详见下方八状态映射 |
| `s_motor_state.control_mode` | 内部控制枚举 | 无待启动的 IDLE 为 NONE；START 受理后可先显示所选模式，不能据此认定已经启相 |
| `s_motor_state.fault_code` | 故障枚举 | NONE=0；其他值查 `motor_types.h`，不得手动清零 |
| `s_motor_state.configuration_confirmed` | bool | 配置是否已确认；不能靠 Watch 写 true 绕过检查 |
| `s_motor_state.configuration_source` | 配置来源枚举 | 默认值、存储值、损坏/不支持记录等来源 |
| `s_context.self_test_stable_ticks` | INIT 连续健康计数 | 连续 4 个监督拍通过；STOP 不重启窗口，不是所有模式就绪缓存 |
| `s_context.fast_sample_epoch` | 采样计数 | CPU 运行时持续推进；Watch 不能证明每拍均按期执行 |
| `s_foc_state.bus_voltage_v` | V | 与外部电压读数核对，确认启动窗口与保护边界 |
| `s_foc_state.bus_current_a` | A | 软件估算值，不替代电源读数或外部电流测量 |
| `s_foc_state.phase_a_current_a` | A | A 相电流，检查零偏、方向及异常 |
| `s_foc_state.phase_b_current_a` | A | B 相电流 |
| `s_foc_state.phase_c_current_a` | A | C 相电流；软件低频观测不是瞬时峰值证据 |
| `s_foc_state.temperature_c` | °C | 固件温度读数，不等于电机绕组温度 |
| `s_context.last_maintenance` | 维护进度结构 | 展开查看对应 ticket 与终态；旧结果不代表当前任务 |
| `s_context.background_busy` | bool | 后台工作是否仍占用；单独为 false 不证明全部资源已释放 |
| `s_context.control_progress` | 控制进度结构 | 区分命令受理与实际应用结果，结合来源身份查看 |
| `s_config_revision` | 活动配置修订 | 不等于存储 schema，也不直接证明已经保存到 Flash |
| `(htim1.Instance->CCER & 0x0555)` | STM32G4 三相使能位 | 关闭时为 0；仅平台寄存器只读核对，不替代实际驱动关断测量 |

`htim1` 定义在 CubeMX 的 `Core/Src/tim.c`；寄存器表达式仅适用于本 STM32G4 平台。
CCER 可能保留其他通道的位，不要求整个 CCER 寄存器为零。

## Watch 2：控制目标与编码器反馈

| 表达式 | 单位或含义 | 观察重点 |
| --- | --- | --- |
| `s_motor_state.speed_target_rad_s` | rad/s | 受理后的速度目标，不表示电机已达到目标 |
| `s_motor_state.speed_ramped_rad_s` | rad/s | 斜坡规划后的速度参考 |
| `s_rotor_encoder.mechanical_velocity_rad_s` | rad/s | 机械速度反馈；乘 `9.5492966` 换算 rpm |
| `s_motor_state.id_target_a` | A | d 轴电流目标 |
| `s_motor_state.iq_target_a` | A | q 轴电流目标；维护任务可能使用单独参考，需结合任务判断 |
| `s_foc_state.id_a` / `s_foc_state.iq_a` | A | d/q 电流反馈；两条表达式分别添加 |
| `s_motor_state.position_target_rad` | rad | 位置目标 |
| `s_motor_state.position_reference_rad` | rad | 规划后的位置参考 |
| `s_rotor_encoder.mechanical_position_rad` | rad | 机械位置反馈 |
| `s_rotor_encoder.has_valid_sample` | bool | 编码器是否已有有效样本 |
| `s_rotor_encoder.sample_epoch` | 有效帧计数 | 持续推进与坏帧状态一起检查 |
| `s_rotor_encoder.calib_flag` | bitmask | bit0 线性化、bit1 电零位、bit2 可选机械零位 |
| `s_rotor_encoder.read_status` | 编码器读帧枚举 | 当前读帧状态，OK=0 |
| `s_rotor_encoder.bad_frame_streak` | 连续坏帧数 | 检查持续增长，不在带电运行中拔插传感器做试验 |

## 参数布局与编号

### 上电电流零点与 PI 初始化

本次修复后，上电保持三相关闭，累计1秒的新鲜ADC2 JEOS帧（20kHz下20000帧）。
偏置采用四次求和乘0.25后的12位counts均值；成功前三相偏置及电流显示NAN，
这表示尚无本次上电的有效校正结果，不应手动改成2048或绕过自检。
当前上电 INIT 期限为1500ms；FAULT CLEAR 满足完整恢复条件后直接 IDLE，
不会重新开始500ms自检。INIT STOP 不重启本次期限或健康窗口。
完成校正只更新RAM及活动revision，不自动保存，也不自动启相。
硬件采样、真实残余零电流及启动时间仍需实机验证。

| 表达式 | 观察重点 |
| --- | --- |
| `s_current_offset_calibration.sample_count` | 新鲜帧计数，从0到20000，完成后不再累加 |
| `s_current_offset_calibration.complete` | 成功后为true；不能单独证明整个自检完成 |
| `s_current_offset_calibration.failed` | 输出、模式、原始范围或偏置失败；不自动重试 |
| `s_motor_state.device.phase_a_offset_counts` | A相零点，成功后为1948..2148内实测均值 |
| `s_motor_state.device.phase_b_offset_counts` | B相零点，不要求与A相相同 |
| `s_motor_state.device.phase_c_offset_counts` | C相零点，不要求为固定2048 |
| `s_foc_state.id_pi` / `s_foc_state.iq_pi` | 上电已配置活动Kp/Ki及Current_Ts；展开分别添加 |
| `s_speed_pi` | 上电已配置活动速度Kp/Ki、Speed_Ts和-1..1归一化输出限幅 |

电流PI首次有效母线采样之前电压限幅为零；限幅刷新修复版在每次母线采样后，
即使空闲也刷新d/q PI的output_min/output_max，单位为V，不是相电流额度A。
范围复用FOC_MaximumVoltage（约±0.95×母线电压/√3），32.84V时约±18.01V；
无效/非正母线对应零限幅，不改变PI增益、周期或积分状态。
旧a2a6e97c候选仍有空闲限幅为零的遗漏，不能用该候选的观测替代新版本验收；
新限幅修复候选须另行下载并加载匹配AXF，见[修复计划](../plans/active/2026-09-29-current-pi-limit-refresh-v1.md)。
活动参数仍在`model.id_kp/id_ki/iq_kp/iq_ki`和`application.speed_kp/speed_ki`，
不以PI积分/输出为零判断增益缺失。校正失败时保持无效资格与锁存故障，
先排除原因再重新上电，不通过Watch改值恢复。本修复增加运行对象，必须重建匹配AXF。

参数归属重构前是平铺字段，重构后分成 `model/application/device` 子对象。
这种 RAM 布局变化不由 Flash schema 编号识别，始终以匹配的 AXF 为准。

| 参数 | 重构前表达式 | 重构后表达式 |
| --- | --- | --- |
| 极对数 | `s_motor_state.motor_pole_pairs` | `s_motor_state.model.motor_pole_pairs` |
| 校准电流 A | `s_motor_state.calibration_current_a` | `s_motor_state.model.calibration_current_a` |
| 电流上限 A | `s_motor_state.current_limit_a` | `s_motor_state.application.current_limit_a` |
| 速度上限 rad/s | `s_motor_state.speed_limit_rad_s` | `s_motor_state.application.speed_limit_rad_s` |
| A 相零偏 ADC counts | `s_motor_state.phase_a_offset_counts` | `s_motor_state.device.phase_a_offset_counts` |

新布局的 `s_motor_state.device.node_id` 是设备节点参数，
`s_motor_state.application.speed_ceiling_rad_s` 是应用上限；不要通过 Watch 改动它们。
采样电阻 `CURRENT_SENSE_SHUNT_MILLIOHM` 是编译宏，不是 Watch 可修改的 RAM 变量。
6 mΩ 硬件需要核对实际构建输入为 `CURRENT_SENSE_SHUNT_MILLIOHM=6U`。

内部主状态值：INIT=1、IDLE=2、STARTING=3、RUNNING=4、MAINTENANCE=5、
STOPPING=6、FAULT=7、FATAL=8；0 不再是 BOOT 状态。
CAN 反馈使用 INIT=3、IDLE=5、STARTING=2、RUNNING=6、STOPPING=7、FAULT=8、
FATAL=9，MAINTENANCE 按任务投影为 4／5；线上 0 保留退役。
INIT／IDLE STOP 原地保持，STOPPING 完成和 FAULT CLEAR 成功直接 IDLE，
FATAL 没有软件出口。完整条件见[状态输入与跳转表](../architecture/motor_state_flow_simplification_v1.md)。
内部控制模式值：CURRENT=0、SPEED=1、POSITION=2、VQ=3、NONE=4。
这些值不是 CAN 编号；例如 CAN 速度模式 2 对应内部 SPEED=1。

## 观察顺序与安全边界

先核对空闲状态、故障、配置、采样推进和输出使能位；经单独授权，使用正式协议接口
启动维护或控制请求，观察目标、反馈与状态变化。取消或停止后，结合实际输出关闭、
对应任务终态和资源释放一起判定，不能只看一个变量。

- Watch 不用于写模式、目标、故障、生命周期上下文或 PWM；不调用内部函数绕过正式受理入口。
- 当前 117 普通启动候选已接 CURRENT／SPEED／POSITION／VQ，116 只更新 RUNNING 同模式目标；
  能力及实机边界见[普通启动 v0.2](../protocols/yg_protocol_ordinary_start_wire_v0_2.md)，Watch 不代替正式入口。
- 电机出力时不用断点、单步或 Keil CPU Stop 停车。CPU 停止不保证定时器输出停止，
  固件保护与命令处理也可能停止运行。
- 运行中的 Periodic Window Update 适合趋势观察，不能证明电流尖峰、周期抖动或 STOP 最坏延迟。
- 正式 STOP 后无法确认功率输出关闭属于硬失败，应立即人工断电，不自动恢复或重试。

## 旧 SWD 台架请求邮箱退役记录

状态：2026-09-28 随容量适配批次退役。原 `foc_bench_speed.h` 定义的 `FocBenchSpeed`
调试邮箱（版本 3、92 B）、`RunState_ServiceBenchRequest` / `RunState_BenchFastGuard` /
`RunState_PublishBenchState` / `RunState_BenchCancel` 及速度台架会话已从固件删除，
用于回收 Flash。本节保留为退役记录，不再是操作入口。

### 退役范围

- 该邮箱是 SWD/J-Link 台架调试入口，不是线上 CAN ABI，也不参与产品控制路径。
- 删除内容：`firmware/app/foc_bench_speed.h`；运行所有者内的台架请求受理、序号
  回放、状态发布、SWD 输出区，以及快环期限/相电流/超速截止。
- 删除钩子：`FocRunState_CheckFastFaults`、`FocRunState_Tick`、
  `FocRunState_FastFaultStop`、`FocRunState_RequestRemoteStop`、`FocRunState_Init`
  中原有的台架调用与共享状态。
- 当前公开入口位于 `motor_state_machine.h`，统一使用 `MotorStateMachine_*`；
  `FocRunState_*` 旧门面已删除。生命周期纯核心、带身份维护请求和保存认领仍保留。
- 受影响：原 START/UPDATE/STOP 速度台架、command 4..12 维护直达、独立试验期限
  与采样式相电流截止不再可用。产品相电流/过流保护仍由既有保护路径负责，
  不依赖本邮箱。

### 替代入口

- 内部使能、目标、停机、清错：`MotorStateMachine_RequestEnable`、`SubmitControl`、
  `RequestRemoteStop`、`RequestClear`。CAN 117 走有身份启动候选入口，116 仅更新运行中的同模式目标。
- 维护、标定、保存：带来源身份的维护受理、ticket 查询/取消及前台保存认领。
- 升级交接：`MotorStateMachine_RequestBootStop` / `MotorStateMachine_BootHandoffReady`。
- 受理不是物理关断或资源释放证明，须按 [主状态机契约](../architecture/motor_state_machine_v1.md) 确认。

容量批次记录见 [APP / Loader 容量适配 v1](../plans/completed/2026-09-28-capacity-fit-v1.md)。

## 2026-10-08 状态机合并更新

主状态为 INIT、IDLE、STARTING、RUNNING、MAINTENANCE、STOPPING、FAULT、FATAL。
INIT／IDLE STOP 保持原状态；停止完成与 CLEAR 成功直接 IDLE；FATAL 没有软件恢复出口。
不再观察已删除的 tested 字段；INIT 的期限和连续健康窗口不因 STOP 重启。
Watch 内部状态数值不能当作 CAN 枚举，当前契约见[状态恢复 v1.2](../protocols/yg_protocol_motor_state_v1_2.md)。
