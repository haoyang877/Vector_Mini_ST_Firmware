# 当前架构与源码职责

## 固件设计主文档

| 设计文档 | 阅读重点 |
| --- | --- |
| [架构设计](software_architecture.md) | 两个镜像的职责、模块依赖、执行上下文和数据所有者 |
| [状态机设计](motor_state_machine_v1.md) | 电机启停与标定、故障恢复、Loader 下载阶段与跨复位标记 |
| [通信协议设计](../protocols/README.md) | CAN FD 帧、角色与命令、请求与应答、重发和完成判据 |
| [参数管理设计](parameter_ownership_v1.md) | 参数来源、启动校验与回退、标定提交、Flash 布局和版本兼容 |

## 工程与专题

| 要回答的问题 | 现行入口 |
| --- | --- |
| 当前板卡链接哪个镜像，从哪里启动？ | [固件工程](../../README.md#选择板卡与固件工程)、[软件架构](software_architecture.md) |
| 修改一个行为，从哪个源文件和测试进入？ | [代码结构](code_structure.md)、[APP 事件阅读路径](app_file_responsibilities_v1.md) |
| 谁接收命令，回复代表什么？ | [协议与命令归属](../protocols/README.md) |
| 何时真正启相、关相、结束标定或清错？ | [电机五态与功率交接](motor_state_machine_v1.md)、[标定发布](motor_maintenance_core_v3.md)、[故障恢复](fault_protection.md) |
| 速度、位置与电流控制分别在哪里执行？ | [控制执行](motor_execution_core_v3.md) |
| 配置从哪里来，谁持有活动值，哪些会写 Flash？ | [参数来源](parameter_sources_v1.md)、[参数所有权](parameter_ownership_v1.md)、[存储布局](../../firmware/parameters/README.md#storage) |
| Loader 何时驻留、擦写、激活或跳转？ | [Loader 启动与升级](loader/loader_product_v3.md) |

## BSP 与板级专题

| 查什么 | 专题 |
| --- | --- |
| BSP功能接口、初始化与板级配置 | [BSP接口设计v1](bsp_interfaces_v1.md) |
| 逐函数契约、类型及可独立包含的头文件 | [BSP接口头文件设计稿](bsp_api_v1/README.md) |
| BSP完整文件树、所有权及运行流程 | [BSP架构总图](bsp_architecture_v1.md) |
| BSP长期职责约定 | [用户确认的BSP职责](bsp_design_rules.md) |
| 硬件支持、目标使能、准入要求与禁用资源 | [双板能力配置](platform_capabilities_v1.md) |
| 当前平台接口落点与后续精简目标 | [公共接口与实现对应表](platform_interface_implementations_v1.md) |
| 双板采样初始化、IRQ、原码及存储协调 | [完整采样链路](sampling_chain_v1.md) |
| 双板编码器接口、器件协议与SPI传输 | [编码器三层链路](encoder_chain_v1.md) |
| PWM比较值、功率启停及目标许可 | [PWM与功率链路](pwm_power_chain_v1.md) |
| AT32F456 上下桥极性、死区、ADC 触发窗口与 FOC 写入截止 | [AT32F456 电机驱动说明与单周期时序图](at32f456_pwm_adc_driver.md) |
| CAN初始化、收发、发送完成与错误恢复 | [双板CAN链路](can_chain_v1.md) |
| 新旧板适配对象、改动范围与冻结边界 | [统一框架适配范围草案](dual_board_adaptation_scope_v1.md) |

已被当前实现替代的设计见[架构归档](../archive/architecture/README.md)。板级实测结论按日期和镜像见[报告索引](../reports/README.md)。
