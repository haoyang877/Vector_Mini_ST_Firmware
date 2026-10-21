# 当前位置串级控制

核对日期：2026-10-21。当前POSITION是直接位置→速度→电流串级。
源码为[`position_cascade.c`](../../firmware/motor/position/position_cascade.c)及
[公开头](../../firmware/motor/position/position_cascade.h)，由Motor外环调度。

## 实际计算

位置误差乘位置Kp，减去Kd乘滤波速度，经过速度包络限幅，送入速度PI得到q轴电流参考。
配置/测量单位为rad、rad/s、s和A；速度PI沿用既有增益乘标称配置电流上限的口径。
动态可用电流取配置与热额度较小值，立即约束输出及积分跟踪，不通过改增益实现降额。

单实例状态只有配置、速度PI、滤波速度、分频计数及配置有效位。
新目标不清除支撑积分；Reset清状态。到位判据同时检查位置窗口和低速事实，
到位标志不自动STOP。运行中的非有限值或无效配置返回失败，由运行所有者处理。
位置模式没有轨迹规划、jerk/S曲线、摩擦前馈、HOLD阶段机或RTT遥测。
SPEED模式独立的可选速度整形/惯量前馈不等于恢复位置轨迹。

## 配置与入口

配置字段见 `PositionCascadeConfig`；活动参数读写见[参数目录](../protocols/yg_protocol_parameter_catalog_v0_1.md)。
AXS数值配置可能使位置增益/最大速度参数只读，见[轴配置](motor_axis_profiles.md)。
固件保留线路mode3的候选启动实现；GET_CAPS目前只公布SPEED，不能把候选编码当作已公布的主机能力。
具体请求见[启动v0.2](../protocols/yg_protocol_ordinary_start_wire_v0_2.md)。
运行中Type116只更新同模式目标；切模式先STOP并确认IDLE，再显式START。

## 验证入口

离线回归使用 `tests/unit/native/run_position_servo_tests.py`，总入口见[测试说明](../../tests/README.md)。
历史Mode3/HIL报告中的轨迹、摩擦与保持测试属于旧实现，不能作为当前位置控制验收。
机械边界和控制稳定性需要与当前配置、源码及装置匹配的独立证据。
