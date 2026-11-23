# 设计文档入口

长期维护当前设计、接口契约和设计输入。调试过程、阶段计划、历史报告及发布批次不在文档树保留。

| 设计主题 | 入口 |
| --- | --- |
| 系统架构 | [产品、依赖、调度和数据所有权](architecture/software_architecture.md) |
| 状态机与保护 | [启停、标定、故障和并发交接](architecture/motor_state_machine_v1.md) |
| 通信协议 | [帧、命令、结果与事务](protocols/README.md) |
| 参数与存储 | [来源、所有权、持久化和兼容](architecture/parameter_ownership_v1.md) |
| 电机控制算法 | [FOC、PI、速度、位置OTG与反馈](architecture/motor_execution_core_v3.md) |
| BSP与板型适配 | [接口、硬件边界和文件归属](architecture/bsp_design_rules.md) |
| 采样与功率时序 | [采样链路](architecture/sampling_chain_v1.md)、[PWM/ADC时序](architecture/at32f456_pwm_adc_driver.md) |
| Loader | [启动、升级、镜像与存储](architecture/loader/loader_product_v3.md) |
| Qt生产工具 | [任务、身份与报告设计](architecture/qt_production_design_v1.md) |
| Rust工作台 | [会话、控制、维护及验收](architecture/yg_mc_host_design_v1.md) |

## 支撑入口

- [architecture/](architecture/)：设计索引及传感器专题。
- [protocols/](protocols/)：当前通信及仍有消费者的存储兼容格式。
- [hardware/](hardware/)：原理图、板卡资源、电机数据及曲线。
- [guides/](guides/)：设计补充、工程规则和打包依赖的产品说明。

编程规范见[STYLE](../STYLE.md)，稳定边界见[ARCHITECTURE](../ARCHITECTURE.md)，
维护方式见[文档规则](guides/documentation.md)。
