# 电机域功能导航

| 目录 | 职责 |
| --- | --- |
| foc | 电流/速度控制、无感观测器与补偿应用 |
| feedback | 编码器采样、角度校正、多圈与速度估计，供各模式共用 |
| position | 位置级联控制 |
| trajectory | 平滑轨迹计算 |
| math | 确定性的数学、角度与滤波计算，无硬件或应用状态依赖 |
| calibration | 编码器装配标定、齿槽标定和补偿表生成 |
| identification | 相电阻及摩擦辨识 |
| protection | 故障与母线保护策略 |

根目录头文件提供电机域值契约及现有 profile，不持有 APP 实例。
motor_config.h 区分电机模型、应用配置和设备标定值；motor_types.h 保留易失运行状态。
motor_profile_defaults.h 只提供实际电机模型与识别默认值，不包含板卡或节点配置。
feedback 不决定生命周期，calibration/identification 不直接保存 Flash 或取得通信所有权。
APP 的运行与维护所有者调度任务，平台端口实现实际 I/O。
源码和对应头文件存放在同一职责目录。
