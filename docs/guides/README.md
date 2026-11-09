# 当前开发与操作指南

| 主题 | 指南 |
| --- | --- |
| 验证环境、doctor、verify | [项目工具](project_harness.md) |
| 文档新增、同步、合并和归档 | [文档维护](documentation.md) |
| 命名、版本、产物和正式发布 | [发布标准](firmware_release_standard.md) |
| 仅下载匹配LoaderApp | [Keil调试](keil_loader_app_debug.md) |
| 双板Loader、原样参数迁移和保护区检查 | [双板Loader](dual_board_loader.md) |
| 更换MCU的输入与验收边界 | [Loader移植](loader_mcu_porting_v1.md) |
| 单项LUT/零位与真实保存 | [标定流程](calibration_process.md) |
| 当前运行层标定入口与完成事件 | [标定请求接口](motor_calibration_request.md) |
| 已删除的台架功能及历史记录 | [MotorBench 退役说明](unified_motor_bench.md) |
| S 曲线位置规划与串级 | [位置控制](position_servo_control.md) |
| AXS1/AXS2身份及应用包络 | [轴配置](motor_axis_profiles.md) |
| AT32F456 五轴节点分配、参数对照及改参入口 | [五轴电机 ID 与参数维护](at32f456_five_axes.md) |
| 独立CAN地址 | [节点ID](motor_can_node_ids.md) |
| SI单位及线路缩放 | [模型单位](motor_model_units.md) |
| ADC过采样与计数口径 | [4×采样](current_oversampling.md) |
| 控制模型、延迟辨识、PI推导、扫频与阶跃结果 | [电流环整定全过程](current_loop_tuning_v1.md) |
| 母线门限与硬件限制 | [8S保护](bus_voltage_8s.md) |
| MCU温度来源与失效处理 | [温度检测](mcu_temperature.md) |
| 调试只读符号、单位与枚举 | [Watch观测](watch_motor_commands.md) |

旧Mode3轨迹/摩擦、齿槽、RTT、HIL和USB操作说明均在[历史指南](../archive/guides/README.md)。
它们保留旧工况与测量，不作为当前操作入口。指南本身不构成硬件操作授权。
