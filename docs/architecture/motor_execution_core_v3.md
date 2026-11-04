# 核心控制执行模块

更新：2026-10-21。当前模式包括电流、速度、位置、q轴电压及开环电压/电流。

## 模块边界

主状态机批准模式和目标，MotorAxis调度，motor层的motor_control执行一拍控制。
控制任务不自行启相、不切换主状态、不写Flash；错误由所有者处理。

MotorAxis_FastTick按编码器启动、母线与相电流采样、观测器、编码器更新、
故障检查、维护或控制、结果发布的顺序执行。校零未完成时只累计原码，不读取被丢弃的
三相物理电流。维护、普通控制和空闲预载互斥，所有分支统一发布结果并复核关断。
本轮仅完成源码精简，未编译、未测试或测量执行时间。

磁链观测器接口只借用电机模型和αβ电流/电压物理值，不读取整份配置、测量对象或FOC状态。
MotorAxis在观测器调用前完成本拍电流的Clarke变换，并以原式
`上一拍mod_alpha/beta × 本拍滤波Vdc / 1.5`提供αβ电压；更新频率与位置模式跳过条件不变。
观测器不再回写FOC或重复换算母线电压。

热模型调用已从初始化、快慢环、启动/维护准入、故障清除和配置提交中移除。
速度/位置外环与FOC直接使用配置电流上限，保留d/q总电流限幅和电压模式实测过流检查。
热模型源码、配置字段及其纯值校验暂时保留；本轮不改参数存储布局。

| 源码 | 核心工作 |
| --- | --- |
| [motor_lifecycle.c](../../firmware/app/runtime/motor_lifecycle.c) | APP命令格式与数值范围校验，不拥有控制器或控制计算 |
| [motor_control.c](../../firmware/motor/control/motor_control.c) | 协调2kHz速度/位置外环，分发20kHz任务；不读取APP状态机或服务命令 |
| [position_control.c](../../firmware/motor/control/position_control.c) | 直接位置PD → 限幅速度参考 → 速度PI → 电流参考 |
| [foc.c](../../firmware/motor/control/foc.c) | 电流变换、d/q PI、电压限幅与调制，显式绑定控制器配置 |
| [foc_output.c](../../firmware/motor/control/foc_output.c) | 调用FOC计算、提交PWM并完成后处理；提交失败仍报告故障 |
| [pi_controller.c](../../firmware/motor/control/pi_controller.c) | PI状态更新、限幅和输出跟踪 |
| [encoder_feedback.c](../../firmware/motor/feedback/encoder_feedback.c) | 编码器角度校正及快慢估计 |

## 输入与输出

控制入口显式接收电机、FOC、编码器和观测器对象。
Axis持有的`FocState`直接包含母线电压、三相电流及MCU温度；
`bus_current_estimated_a`及功率为估算值，`bus_current_a`为硬件实测值。
快慢拍测量字段不构成同一时刻的采样快照。
目标和反馈使用A、rad/s、rad、V；线路整数尺度由通信边界换算。
`motor_lifecycle.c`内的`validate_target`只验证格式及数值范围，不授予运行许可。
`MotorOuterLoop_SlowTick`写电流参考，快环读取并执行，禁止通信直接写控制器积分。

| 模式 | 慢拍 | 快拍 |
| --- | --- | --- |
| CURRENT | 不生成速度/位置外环参考 | d/q电流闭环 |
| SPEED | 速度PI生成q轴电流参考 | d/q电流闭环 |
| POSITION | 位置PD和速度PI生成q轴电流参考 | d/q电流闭环 |
| VQ | 不执行位置/速度闭环 | 按编码器电角度施加q轴电压 |
| OPEN_VOLTAGE / OPEN_CURRENT | 不执行位置/速度闭环 | 按指定电角频率执行开环电压或电流控制 |

## 位置串级只保留什么

位置误差乘Kp，减去Kd乘滤波速度，经过原速度包络限幅后送速度PI。
PI输出限制在相电流额度内，保留原积分抗饱和；电流限值下调立即生效。
状态只有配置、速度PI、滤波速度和分频计数，单实例串行调用。
目标更新不清支撑积分，Reset清控制状态。
到位结果仅是观测事实，不自动STOP；RUNNING下仍可维持位置控制。

已删除轨迹规划、加减速/S曲线、摩擦前馈、复杂保持阶段和独立遥测协调。
参数与测量非法时返回失败，由运行所有者决定关相及故障。
增益、限值与PI单位保持已有约定；简化后的运动行为需要当前候选实机确认。

## 阅读与操作入口

先看MotorAxis的SupervisorTick/FastTick，再看执行模块，最后读具体算法。
启动用117，运行中同模式目标用116，切模式先STOP再START。
请求格式见[启动v0.2](../protocols/README.md#control)。
软件接线不等于完成当前候选的实机验收。
