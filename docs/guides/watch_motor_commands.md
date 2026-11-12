# Keil Watch 观测与 AT32 台架标定

适用于两板 APP 的 `g_app` 实例。外设寄存器示例针对 STM32G431。
本页描述人工观测步骤和当前源码中的变量，不授权烧录、复位或电机运动。
旧SWD台架邮箱的删除范围见[历史附录](../archive/guides/watch_motor_commands_retirement.md)。

运行对象和参数供只读观测。AT32 台架构建另提供下述一次性标定命令。

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
   LOAD "C:\\matching-build\\ESC-APP-STM32G431.axf" NOCODE CLEAR INCREMENTAL
   ```

   `NOCODE` 不写入程序，`CLEAR INCREMENTAL` 替换符号且避免该 LOAD 命令复位目标。
   这些选项不保证连接调试器本身不会暂停或复位 MCU。

Keil 官方说明：[J-Link Flash 下载设置](https://www.keil.com/support/man/docs/jlink/jlink_configflash.asp)、
[LOAD 命令与 NOCODE](https://www.keil.com/support/man/docs/uv4/uv4_cm_load.asp)。

## 添加变量与刷新

- 在 Debug 会话打开 `View → Watch Windows → Watch 1/2`，双击 `<Enter expression>` 添加表达式。
- 开启 `View → Periodic Window Update`，支持的调试驱动可在 CPU 运行时刷新值。
- 两板 main.c 中的 `g_app` 是 Watch 根符号，展开 axis、protocol 或 endpoint 查找所属对象。
  始终以匹配板上镜像的 AXF 为准，不硬编码 RAM 地址。
- 全局符号便于观察，公共头不提供这些对象的可写 `extern` 接口；产品代码仍通过请求、快照和显式对象参数访问。
- Watch 中只输入无副作用的读表达式，不加入赋值、递增/递减或目标函数调用。
  仅下述台架命令通过 Value 列写入。刷新得到的多个字段不是同一时刻的原子快照。

操作方式见 [Keil Watch 官方说明](https://www.keil.com/support/man/docs/uv4/uv4_db_dbg_watchwin.asp)。

## AT32 台架标定

Loader 配套的 `ESC-APP-AT32F456` 和独立的 `ESC-APP-AT32F456-Standalone` 均默认启用 `AT32_KEIL_CALIBRATION=1`。加载与板上镜像匹配的 AXF，在 Watch 中添加 `s_keil_calibration_command`（可从 main.c 的 Symbols 中选择），按 F5 连续运行。

1. 确认轴在 IDLE、无故障、配置已确认。PITCH/YAW 联合标定采样需要连续旋转 100 电机轴圈，40:1 装配约为 2.5 输出轴圈，须在脱开机械限位的工装上执行。
2. 写一次 `1` 请求编码器 LUT 标定。命令回零表示已读取。观察 `g_app.axis.state_machine.state` 进入 CALIBRATION，并通过 `g_app.axis.calibration.session.lut.phase` 查看进度。拒绝的请求不会重试。
3. 直驱轴在 LUT 保存后回到 IDLE，再写一次 `3` 请求电气零位标定。
4. 中止时写一次 `2`，保持 CPU 运行直至关相。正常完成后，核对回到 IDLE、故障为 0、编码器 LUT 和电零位有效位，以及 PITCH/YAW 的 Hall 标定结果。[标定流程](calibration_process.md)

## Watch 1：状态、安全与维护

下表给出板级实例的成员路径。
实际 Watch 名称以加载的 AXF 和 Symbols 生成的限定名为准。

采样值直接保存在 `g_app.axis.foc` 的成员中，不再有 measurement 层或独立测量对象。
配置单独保存在 `g_app.axis.configuration`；`g_app.axis.motor` 只含运行态，
`g_app.axis.rotor_encoder` 只含采样、PLL和位置/速度动态状态。LUT、零位、方向和标定标志
单份保存在 `g_app.axis.configuration.device.encoder`。

| 表达式 | 单位或含义 | 观察重点 |
| --- | --- | --- |
| `g_app.axis.state_machine.state` | 主状态枚举 | 待命为 IDLE；IDLE 不等于所有模式均可启动，见下方状态与线路映射 |
| `g_app.axis.motor.control_mode` | 内部控制枚举 | 无待启动的 IDLE 为 NONE；START 受理后可先显示所选模式，不能据此认定已经启相 |
| `g_app.axis.motor.fault_code` | 故障枚举 | NONE=0；其他值查 `motor_types.h`，不得手动清零 |
| `g_app.axis.configuration.configuration_confirmed` | bool | 配置是否已确认；不能靠 Watch 写 true 绕过检查 |
| `g_app.axis.configuration.configuration_source` | 配置来源枚举 | 默认值、存储值、损坏/不支持记录等来源 |
| `g_app.axis.state_machine.fast_sample_ticks` | 采样计数 | CPU 运行时持续推进；Watch 不能证明每拍均按期执行 |
| `g_app.axis.foc.bus_voltage_v` | V | 与外部电压读数核对，确认启动窗口与保护边界 |
| `g_app.axis.foc.bus_current_estimated_a` | A | 软件估算值，不替代电源读数或外部电流测量 |
| `g_app.axis.foc.bus_current_a` | A | 硬件实测母线电流；未提供或未校零时为 NaN |
| `g_app.axis.foc.phase_current_a.a` | A | A 相电流，检查零偏、方向及异常 |
| `g_app.axis.foc.phase_current_a.b` | A | B 相电流 |
| `g_app.axis.foc.phase_current_a.c` | A | C 相电流；软件低频观测不是瞬时峰值证据 |
| `g_app.axis.foc.temperature_c` | °C | 固件温度读数，不等于电机绕组温度 |
| `(htim1.Instance->CCER & 0x0555)` | STM32G4 三相使能位 | 关闭时为 0；仅平台寄存器只读核对，不替代实际驱动关断测量 |

`htim1` 定义在 CubeMX 的 `Core/Src/tim.c`；寄存器表达式仅适用于本 STM32G4 平台。
CCER 可能保留其他通道的位，不要求整个 CCER 寄存器为零。

## Watch 2：控制目标与编码器反馈

| 表达式 | 单位或含义 | 观察重点 |
| --- | --- | --- |
| `g_app.axis.motor.speed_target_rad_s` | rad/s | 受理后的速度目标，不表示电机已达到目标 |
| `g_app.axis.motor.speed_ramped_rad_s` | rad/s | 斜坡规划后的速度参考 |
| `g_app.axis.rotor_encoder.mechanical_velocity_rad_s` | rad/s | 机械速度反馈；乘 `9.5492966` 换算 rpm |
| `g_app.axis.motor.id_target_a` | A | d 轴电流目标 |
| `g_app.axis.motor.iq_target_a` | A | q 轴电流目标；维护任务可能使用单独参考，需结合任务判断 |
| `g_app.axis.foc.current_dq_a.d` / `g_app.axis.foc.current_dq_a.q` | A | d/q 电流反馈；两条表达式分别添加 |
| `g_app.axis.motor.position_target_rad` | rad | 位置目标 |
| `g_app.axis.motor.position_reference_rad` | rad | 规划后的位置参考 |
| `g_app.axis.rotor_encoder.mechanical_position_rad` | rad | 机械位置反馈 |
| `g_app.axis.rotor_encoder.has_valid_sample` | bool | 是否已取得首个角度样本；不表示做过硬件诊断 |
| `g_app.axis.rotor_encoder.sample_epoch` | 采样计数 | 每次角度更新推进 |
| `g_app.axis.configuration.device.encoder.calib_flag` | bitmask | bit0 线性化、bit1 电零位、bit2 可选机械零位；属于配置，非动态采样状态 |

## 状态与线路映射

上电后 Watch 中的主状态为 IDLE。SELF_TEST 保留枚举。当前关相零偏窗口为 2048 个有效快拍，按 20 kHz 约为 102.4 ms。控制器的轴对象见[APP 文件与公共入口](../../firmware/app/README.md)。

Watch 显示源码枚举；CAN 108/124 使用独立的线路编号。当前 IDLE、RUNNING、CALIBRATION、FAULT 对应线路值 5、6、4、8。START 受理后可以先看见模式和待启动意图，实际启相后主状态才变为 RUNNING。最近一次解析的命令位于 `g_app.endpoint.command`。[电机状态机](../architecture/motor_state_machine_v1.md) · [反馈格式](../protocols/README.md#feedback)

## 观察顺序与安全边界

先核对空闲状态、故障、配置、采样推进和输出使能位；经单独授权，使用正式协议接口
启动维护或控制请求，观察目标、反馈与状态变化。取消或停止后，结合实际输出关闭、
对应任务终态和资源释放一起判定，不能只看一个变量。

- Watch 不用于写模式、目标、故障、生命周期上下文或 PWM；不调用内部函数绕过正式受理入口。
- 当前 117 可请求电流、速度、位置、Vq、开环电压或开环电流模式，116 只更新 RUNNING 同模式目标；
  能力及实机边界见[普通启动 v0.2](../protocols/README.md#control)，Watch 不代替正式入口。
- 电机出力时不用断点、单步或 Keil CPU Stop 停车。CPU 停止不保证定时器输出停止，
  固件保护与命令处理也可能停止运行。
- 运行中的 Periodic Window Update 适合趋势观察，不能证明电流尖峰、周期抖动或 STOP 最坏延迟。
- 正式 STOP 后无法确认功率输出关闭属于硬失败，应立即人工断电，不自动恢复或重试。
