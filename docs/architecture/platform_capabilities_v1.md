# 双板能力与功能使能契约 v1

更新：2026-10-26。板上资源在platform/config，应用/台架选择在app/config。

BSP_SUPPORTED_*表示已有硬件和驱动，BSP_ENABLED_*表示当前构建选择，BSP_REQUIRED_*属于应用要求。
bsp_config.h检查布尔值、板型匹配及不能启用未支持功能；app_feature_requirements.h检查产品必需项。
已使能不表示样本新鲜或硬件健康，运行结果必须查询对应功能接口。

| 功能 | AT32F456 | STM32G4 |
| --- | --- | --- |
| 相电流、母线电压、编码器、PWM、CAN | 支持 | 支持 |
| 独立母线电流 | 支持，同步快采样 | 不支持 |
| MCU温度、VDDA | 支持；温度有效性依赖校准 | 支持 |
| 电机NTC、sin/cos原码诊断 | 支持，按目标启用 | 不支持 |
| LED | 支持，共享双灯 | 支持，独立指示/灯带 |
| UART、硬件版本采样 | 未绑定，NOT_SUPPORTED | 未绑定，NOT_SUPPORTED |

相电压没有采样电路，已删除接口及能力项；旧宏启用时编译拒绝。
完整APP要求相电流、母线电压、编码器、PWM和CAN。Foundation需要CAN，四个电机台架明确禁用CAN。
STM32完整APP不可取消原MCU温度保护。AT32仍保持原关相/限时目标的温度策略。
永久关相和一次性限时许可由app/config配置，不能用能力开关绕过。
BSP只消费BSP_PWM_HARD_LOCK；应用校验它与许可模式一致。软件限流在app/config/current_sense_limits.h，板头只保存真实电气输入和量程。

关闭功能后统一接口仍存在并返回NOT_SUPPORTED，不创建另一套命名或假成功实现。
关闭CAN不触碰控制器/IRQ；关闭LED不启动输出；UART当前只提供禁用契约。
AT32辅助DMA仅包含启用的NTC/sin/cos，母线电流独立于辅助序列，由ADC2注入采样。
关闭辅助项不应停止仍需要的快母线电流；温度/VREF内部采样按各自需求启用。

职责及各模块行为见[统一接口](bsp_interfaces_v1.md)、[采样](sampling_chain_v1.md)、
[编码器](encoder_chain_v1.md)、[PWM](pwm_power_chain_v1.md)、[CAN](can_chain_v1.md)。
