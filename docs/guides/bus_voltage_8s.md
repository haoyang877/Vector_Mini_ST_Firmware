# 8S母线电压保护

核对日期：2026-10-21。配置来源为
[`bus_voltage_profile.h`](../../firmware/motor/protection/bus_voltage_profile.h)，
测量及运行判断由[`foc_sensing.c`](../../firmware/motor/foc/foc_sensing.c)负责。

| 条件 | 当前门限 | 确认 |
| --- | --- | --- |
| 运行欠压 | 滤波值≤24.0V | 连续100ms |
| 运行过压 | 滤波值≥34.0V | 连续2ms |
| 快速过压 | 原始值≥34.5V | 连续3个快拍，名义20kHz下150µs |
| 启动/恢复资格 | 滤波值25.6～33.8V且原始值<34.5V | 状态机启动或显式CLEAR时复核 |

这些是固件测量值和软件门限；滤波、硬件RC、ADC与调度还会增加响应延迟。
对应8串常规满充33.6V的编译配置，不是逐节电芯/BMS保护，也不是整板耐压认证。
原硬件超过35V可能损坏的限制仍有效，当前没有制动电阻控制或主动吸收回生能量。

故障检测先走快速关相，再由主状态机收敛FAULT。电压恢复不自动重新使能；
CLEAR还须完整健康、关相与资源释放，成功只回IDLE。当前未保留摩擦/相电阻辨识任务。
恢复矩阵见[故障与保护](../architecture/fault_protection.md)。

历史静态和低速试验、备份摘要、未测的危险电压/回生范围见[归档原记录](../archive/guides/bus_voltage_8s.md)。
离线边界回归为 `tests/unit/native/test_bus_voltage_protection.py`；历史通过不外推到当前候选。
