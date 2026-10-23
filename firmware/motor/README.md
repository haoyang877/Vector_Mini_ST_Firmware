# 电机域核心功能导航

当前核心范围；本域不拥有APP生命周期，不直接保存Flash。

| 目录 | 核心工作 |
| --- | --- |
| foc | 控制模式分发、速度/位置外环协调、d/q电流控制、PI、零偏及测量换算，以及LUT依赖的无感启动/观测器 |
| feedback | 编码器采样、LUT/零位校正、多圈与速度估计 |
| position | 直接位置PD→速度PI→电流参考；无轨迹和前馈 |
| math | 确定性角度、滤波和数学计算 |
| calibration | LUT及电零位任务，显式输入的纯标定核心 |
| protection | 电流、母线、温度等故障判定和恢复条件，单份绕组热历史及降额计算 |

motor_config.h定义model/application/device配置，motor_types.h定义易失运行值。
motor_profile_defaults.h提供电机模型默认值，板卡与采样配置属于平台。
型号参数与HT阻尼环配置见[电机配置](profiles/README.md)，统一1kHz电流环默认带宽。
算法通过显式对象获得输入、返回结果；APP选择何时执行和关相，平台执行实际I/O。
源码与头文件同目录。齿槽标定/补偿、摩擦/电阻辨识及复杂位置轨迹已删除。

详见[控制执行](../../docs/architecture/motor_execution_core_v3.md)、
[维护和标定](../../docs/architecture/motor_maintenance_core_v3.md)、
[参数所有权](../../docs/architecture/parameter_ownership_v1.md)。
