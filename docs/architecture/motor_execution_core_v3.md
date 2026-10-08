# 核心控制执行模块

更新：2026-10-09。适用v1.3.0；当前四种模式为电流、速度、位置、q轴电压。

## 模块边界

主状态机批准模式和目标，MotorAxis调度，motor_execution执行一拍控制。
控制任务不自行启相、不切换主状态、不写Flash；错误由所有者处理。

| 源码 | 核心工作 |
| --- | --- |
| [motor_execution.c](../../firmware/app/runtime/motor_execution.c) | 校验目标，协调2kHz速度/位置外环，分发20kHz任务 |
| [position_cascade.c](../../firmware/motor/position/position_cascade.c) | 直接位置PD → 限幅速度参考 → 速度PI → 电流参考 |
| [foc_algorithm.c](../../firmware/motor/foc/foc_algorithm.c) | 电流变换、d/q PI、电压限幅与调制 |
| [foc_pid.c](../../firmware/motor/foc/foc_pid.c) | PI状态更新、限幅和输出跟踪 |
| [angle_feedback.c](../../firmware/motor/feedback/angle_feedback.c) | 编码器角度校正及快慢估计 |

## 输入与输出

控制入口显式接收电机、FOC、编码器和观测器对象。
目标和反馈使用A、rad/s、rad、V；线路整数尺度由通信边界换算。
`MotorExecution_ValidateTarget`只验证格式及数值范围，不授予运行许可。
`MotorOuterLoop_SlowTick`写电流参考，快环读取并执行，禁止通信直接写控制器积分。

| 模式 | 慢拍 | 快拍 |
| --- | --- | --- |
| CURRENT | 不生成速度/位置外环参考 | d/q电流闭环 |
| SPEED | 速度PI生成q轴电流参考 | d/q电流闭环 |
| POSITION | 位置PD和速度PI生成q轴电流参考 | d/q电流闭环 |
| VQ | 不执行位置/速度闭环 | 按编码器电角度施加q轴电压 |

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
请求格式见[启动v0.2](../protocols/yg_protocol_ordinary_start_wire_v0_2.md)。
软件接线不等于完成当前候选的实机验收。
