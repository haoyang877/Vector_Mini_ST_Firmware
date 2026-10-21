# architecture历史归档

本索引记录原路径、归档原因及当前主题。未完成的旧方案归档不等于已经实施；当前有效缺口见plans的现行索引。

| 历史材料 | 原位置 | 归档原因 | 当前入口 |
| --- | --- | --- | --- |
| [APP 模块、对象与变量命名 v1](app_naming_v1.md) | `docs/architecture/app_naming_v1.md` | 阶段方案或重复总览；现行职责已归并到软件架构与代码结构。 | [现行](../../architecture/software_architecture.md) |
| [齿槽标定采样、成表与运行边界 v1](cogging_calibration_v1.md) | `docs/architecture/cogging_calibration_v1.md` | 生产齿槽任务与补偿已删除；保留历史设计。 | [现行](../../architecture/motor_maintenance_core_v3.md) |
| [齿槽核心、标定任务与运行边界 v2](cogging_core_boundaries_v2.md) | `docs/architecture/cogging_core_boundaries_v2.md` | 生产齿槽任务与补偿已删除；保留历史设计。 | [现行](../../architecture/motor_maintenance_core_v3.md) |
| [齿槽最小任务与交接事实 v3](cogging_minimal_task_v3.md) | `docs/architecture/cogging_minimal_task_v3.md` | 生产齿槽任务与补偿已删除；保留历史设计。 | [现行](../../architecture/motor_maintenance_core_v3.md) |
| [电机通信与通用升级开发建议](communication_development_plan.md) | `docs/architecture/communication_development_plan.md` | 阶段方案或重复总览；现行职责已归并到软件架构与代码结构。 | [现行](../../architecture/software_architecture.md) |
| [电机通信协议重构技术实现规范 v1.0](communication_protocol_refactor_spec_v1.md) | `docs/architecture/communication_protocol_refactor_spec_v1.md` | 阶段方案或重复总览；现行职责已归并到软件架构与代码结构。 | [现行](../../architecture/software_architecture.md) |
| [公司通信协议框架梳理](company_communication_framework.md) | `docs/architecture/company_communication_framework.md` | 阶段方案或重复总览；现行职责已归并到软件架构与代码结构。 | [现行](../../architecture/software_architecture.md) |
| [核心电机工程 v3](core_only_v3.md) | `docs/architecture/core_only_v3.md` | 阶段方案或重复总览；现行职责已归并到软件架构与代码结构。 | [现行](../../architecture/software_architecture.md) |
| [编码器标定核心/外围数据流 v2](encoder_calibration_core_v2.md) | `docs/architecture/encoder_calibration_core_v2.md` | 任务与计算职责合并到核心维护说明和当前标定指南。 | [现行](../../architecture/motor_maintenance_core_v3.md) |
| [Mode13/Mode15 编码器标定任务职责 v1](encoder_calibration_task_v1.md) | `docs/architecture/encoder_calibration_task_v1.md` | 任务与计算职责合并到核心维护说明和当前标定指南。 | [现行](../../architecture/motor_maintenance_core_v3.md) |
| [故障保护模块、接口与参数设计](fault_protection_design_v1.md) | `docs/architecture/fault_protection_design_v1.md` | 旧保护提案包含未实施设计；现行说明按实际故障、母线与热保护代码重建。 | [现行](../../architecture/fault_protection.md) |
| [关节身份识别与自动参数配置](joint_identity_configuration_design.md) | `docs/architecture/joint_identity_configuration_design.md` | 旧身份自动映射方案已被地址与应用包络分离策略替代。 | [现行](../../guides/motor_axis_profiles.md) |
| [D-loader 软件架构及 Flash / RAM 分配](loader/d_loader_architecture_memory_20260917.md) | `docs/architecture/loader/d_loader_architecture_memory_20260917.md` | 旧分区、SHA/启动计数或尚未实现的多链路方案；当前为layout6单槽CRC32。 | [现行](../../architecture/loader/loader_product_v3.md) |
| [D-loader 设计与优化方案](loader/d_loader_design_20260917.md) | `docs/architecture/loader/d_loader_design_20260917.md` | 旧分区、SHA/启动计数或尚未实现的多链路方案；当前为layout6单槽CRC32。 | [现行](../../architecture/loader/loader_product_v3.md) |
| [D-loader 通用 MCU 升级框架](loader/d_loader_portable_framework_20260917.md) | `docs/architecture/loader/d_loader_portable_framework_20260917.md` | 旧分区、SHA/启动计数或尚未实现的多链路方案；当前为layout6单槽CRC32。 | [现行](../../architecture/loader/loader_product_v3.md) |
| [D-loader：USB 移除与空间优化分析](loader/d_loader_space_optimization_20260917.md) | `docs/architecture/loader/d_loader_space_optimization_20260917.md` | 旧分区、SHA/启动计数或尚未实现的多链路方案；当前为layout6单槽CRC32。 | [现行](../../architecture/loader/loader_product_v3.md) |
| [Loader 独立产品设计与实现 v2（简化候选）](loader/loader_product_v1.md) | `docs/architecture/loader/loader_product_v1.md` | 旧分区、SHA/启动计数或尚未实现的多链路方案；当前为layout6单槽CRC32。 | [现行](../../architecture/loader/loader_product_v3.md) |
| [电机控制命令合并评审](motor_command_consolidation_review.md) | `docs/architecture/motor_command_consolidation_review.md` | 阶段方案或重复总览；现行职责已归并到软件架构与代码结构。 | [现行](../../architecture/software_architecture.md) |
| [电机通信扩展与固件升级协议设计参考](motor_communication_upgrade_reference.md) | `docs/architecture/motor_communication_upgrade_reference.md` | 阶段方案或重复总览；现行职责已归并到软件架构与代码结构。 | [现行](../../architecture/software_architecture.md) |
| [整机启动、控制、维护与状态机调用流程](motor_platform_lifecycle_v1.md) | `docs/architecture/motor_platform_lifecycle_v1.md` | 阶段方案或重复总览；现行职责已归并到软件架构与代码结构。 | [现行](../../architecture/software_architecture.md) |
