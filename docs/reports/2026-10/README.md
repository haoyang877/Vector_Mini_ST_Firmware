# 2026-10报告

- [电气零位标定PWM超时定位](zero_calibration_pwm_deadline_20261029.md)：全局受理锁18.894µs推迟首拍ADC，窗口只余208计数；保留保护，仅做定位。

- [AT32标定和速度闭环实测](at32_calibration_speed_hil_20261029.md)：LUT完成，零位存在偶发PWM超时，1rad/s速度目标触发主机截止；整体未通过，已关相。

- [AT32 ADC中断关相实测](adc_irq_timing_at32_20261029.md)：60.5万次、平均27.341µs、最大31.039µs；已恢复无插桩固件，未启相。

- [BSP整理后AT32烧录与关相验收](bsp_cleanup_at32_hil_20261027.md)：当前整理镜像读回、60秒运行、CAN及参数保留验证通过；未转动。

- [AT32 BSP命名与实现收敛](at32_bsp_cleanup_20261026.md)：平铺11文件、移除实现期测试代码，九工程及完整回归通过；未烧录。


结果只适用于各记录的原始范围和基线，未测/失败/限制仍按原文保留。

- [BSP私有接口收敛后的AT32实测](bsp_private_at32_hil_20261026.md)：时序问题修复、九工程和最终关相/CAN验证通过；未转动。

- [BSP私有头合并与直接查询收敛](bsp_private_headers_20261026.md)：当前候选，九工程/PR/独立release全部通过；未操作硬件。

- [BSP逻辑与文件架构收敛离线验收](bsp_architecture_offline_20261026.md)：前一阶段候选，九工程/完整回归通过；统一实测未运行。
- [BSP API v1实施与此前台架结果](bsp_api_v1_implementation_20261026.md)：含零偏收尾修复和未完成下载的历史状态。

- [两板能力配置统一后的 AT32 关相实测](platform_capabilities_at32_hil_20261026.md)

- [APP 启动身份来源离线交付记录](boot_epoch_provider_20261008.md)
- [齿槽核心任务解耦 v2 交付记录](cogging_core_decoupling_20261008.md)
- [齿槽标定终止路径简化与离线验收](cogging_core_simplification_20261008.md)
- [齿槽最小任务重写 v3 验收](cogging_minimal_task_20261008.md)
- [交付功能集成与新固件包记录](delivery_feature_integration_20261017.md)
- [编码器标定调用与边界简化 v3](encoder_calibration_call_boundaries_20261008.md)
- [编码器标定核心抽离 J 验收 v2](encoder_calibration_core_20261008.md)
- [编码器标定最小实现 v4](encoder_calibration_minimal_20261008.md)
- [编码器标定任务等价简化验收 2026-10-08](encoder_calibration_simplify_20261008.md)
- [16 KiB Loader 联合升级实测](loader_layout6_upgrade_20261020.md)
- [Loader 多 MCU 预备改造与离线验证](loader_mcu_boundaries_20261021.md)
- [Loader 多 MCU 改造复审与修复](loader_mcu_review_20261021.md)
- [CAN FD 连续二十次升级实测](loader_repeat20_20261020.md)
- [Loader 与参数存储优化记录](loader_storage_optimization_20261020.md)
- [模块核心解耦集成记录](module_core_integration_20261008.md)
- [平台主／子流程交付 v7](motor_platform_lifecycle_v7_20261008.md)
- [主状态流转简化第一批验收 2026-10-08](motor_state_flow_batch1_20261008.md)
- [八状态与静默 STOP 验收 2026-10-08](motor_state_flow_batch2_20261008.md)
- [主状态出口收敛交付 v4](motor_state_flow_batch3_20261008.md)

- [BSP / 板级配置 / 原样SDK收敛与AT32实测](platform_bsp_sdk_at32_hil_20261026.md)

- [共用电机BSP与AT32关相验收](common_motor_bsp_at32_hil_20261026.md)

- [BSP最小直接实现与AT32关相验收](minimal_bsp_at32_hil_20261026.md)
