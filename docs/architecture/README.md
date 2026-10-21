# 当前架构与源码职责

以下说明按当前代码维护；历史提案已移入[归档](../archive/architecture/README.md)。

| 查什么 | 唯一主题入口 |
| --- | --- |
| 系统职责与产品支持范围 | [软件架构](software_architecture.md) |
| 目录、依赖与工程 | [代码结构](code_structure.md) |
| Keil逐文件阅读 | [APP文件职责](app_file_responsibilities_v1.md) |
| 模块输入/结果和全局所有者 | [模块核心职责](module_core_responsibilities.md) |
| 状态职责与主流程 | [八状态总图](motor_state_machine_v1.md) |
| 全部状态输入/出口 | [状态跳转表](motor_state_flow_simplification_v1.md) |
| 自检与逐模式启动资格 | [准备资格](motor_readiness_v1.md) |
| 控制算法及外环边界 | [控制执行](motor_execution_core_v3.md) |
| 单项维护、计算核心、工作区与保存 | [核心维护](motor_maintenance_core_v3.md) |
| 四类配置与运行对象 | [参数所有权](parameter_ownership_v1.md) |
| 参数默认/派生/标定来源 | [参数来源](parameter_sources_v1.md) |
| 故障、快速关相、热保护与恢复 | [故障保护](fault_protection.md) |
| 通信到业务的依赖 | [通信分层](communication_layering.md) |
| 独立Loader启动/升级/分区 | [Loader设计](loader/loader_product_v3.md) |

文件名中保留版本后缀是为减少无意义路径变更；是否现行以本索引、正文和实际代码为准。
