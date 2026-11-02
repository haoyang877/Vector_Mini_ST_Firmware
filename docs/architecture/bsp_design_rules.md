# BSP 职责约定

更新：2026-10-30。这里汇总当前有效约定，阶段过程保留在对应计划和报告。
文件对应关系见[BSP 架构](bsp_architecture_v1.md)。

## 公开能力与实现

BSP 统一管理实际使用的时钟、看门狗、ADC、GPIO、PWM、SPI、DMA、CAN、定时器和 Flash。
应用使用功能接口，硬件差异集中在板卡资源、引脚、通道、比例、器件类型和对应 MCU 实现。

- `bsp/include` 只有八个公共头：bsp、bsp_system、bsp_adc、bsp_pwm、bsp_encoder、bsp_can、bsp_flash、boot_hw。
- `bsp_internal.h` 只保存跨文件初始化、采样启停和台架专用声明，没有函数实现、可变状态定义或板卡值。
- 各 MCU 的 adc/pwm/encoder_spi/can/flash/system 实现直接定义统一接口，不增加转发层或驱动注册表。
- ADC 零偏和物理换算、PWM 占空比换算、CAN DLC 换算随硬件实现保留在对应 MCU 文件。
- 单文件辅助函数为 static；只有真实多消费者共享完整契约时才提取共用逻辑。
- LED、UART、独立器件包装、功率窗口和旧诊断入口已删除，不保留空接口或工程引用。

## 当前接口语义

| 功能 | 契约 |
| --- | --- |
| ADC | float 原码/物理量读取；不可用返回 NaN；不提供状态、时间戳、同拍保证或保护 |
| 零偏 | void 直接设置，调用者负责有效范围、关相及原子发布 |
| PWM | void 直接设置占空比和启停；不验证许可、不限幅、不锁存故障 |
| PWM 查询 | 直接读取六路通道位，不把主输出位作为通道关相证据 |
| 编码器 | void Begin 与 uint16_t ReadAngle；统一 65536 counts/圈，无方向/LUT/零位修正 |
| 编码器等待 | 不提供 CRC、硬件状态或超时，调用者不能假定这些诊断存在 |
| CAN | void Start；bool 收发与忙/完成查询；不提供软件票据、恢复或取消 |
| Flash | 固定记录区，前台关相擦写、回读、失败后禁止不确定重试 |

PWM 三个比较寄存器分别写入，不承诺跨更新事件原子提交。
两板编码器和 CAN 的 GPIO、SPI/DMA、寄存器操作及 IRQ 都归对应 MCU。
协议队列、升级握手、状态准入、故障策略和标定资格由其业务所有者维护。

## 配置与启动

`firmware/config/config.h` 提供板型编号、工程输入要求、电机默认值和公共快慢时基。
`board_config.h` 提供能力、电路、时钟和存储范围；功能直接由实际板卡能力决定。
每个 MCU 的 `mcu_resources.h` 统一管理引脚、外设、采样结果、DMA 和 IRQ 映射，
仅供该 MCU 使用。STM32 改动资源必须同步审查 `.ioc` 与生成区，手改限 USER CODE。

main 使用 `Bsp_Init → App_Init → Bsp_Start → App_Start`，BSP 初始化不启相。
ADC 回调由 main 连接 APP，监督与 CAN 业务回调由组合层实现；MCU 只通知已约定的事件。
快采样可抢占低频监督，原有中断优先级和短临界区语义不因整理改变。

## 验证

离线检查覆盖统一声明、调用者、工程输入、数值范围和错误语义；实际波形、控制时序和电机响应另行验收。
不得把目录一致、头文件存在或单板构建成功当作双板替代证据。
用户已取消逐项接口评审要求，后续可在授权范围内直接完善实现、文档和验证。
历史重构步骤见[活动计划](../plans/active/README.md)，当前入口见[平台实现表](platform_interface_implementations_v1.md)。
