# 文档入口

## APP 与 Loader 固件设计

- [架构设计](architecture/software_architecture.md)说明两种镜像如何协作，以及前台、中断和业务模块如何分工。
- [状态机设计](architecture/motor_state_machine_v1.md)给出电机运行与标定的转移条件，以及 Loader 下载、激活和确认的状态。
- [通信协议设计](protocols/README.md)定义 CAN FD 帧、各镜像的命令、应答和重发语义。
- [参数管理设计](architecture/parameter_ownership_v1.md)说明参数的来源、校验、活动对象、保存和重启恢复。

具体板型和 Keil 工程见[镜像选择表](../README.md#选择板卡与固件工程)。

## 主题索引

| 主题 | 入口与用途 |
| --- | --- |
| [architecture/](architecture/) | 软件职责、文件归属、状态机、控制/维护、参数与保护 |
| [protocols/](protocols/) | 唯一现行通信协议说明 |
| [guides/](guides/) | 当前开发、操作、标定与发布指南 |
| [YG_MC_HOST 功能需求清单](guides/yg_mc_host_requirements_v1.md) | 当前 Rust 上位机功能、状态和验收范围 |
| [YG_MC_HOST 软件设计说明](architecture/yg_mc_host_design_v1.md) | Rust 上位机模块边界、页面流程、通信和安全门禁 |
| [AT32F456 五轴电机配置](guides/at32f456_five_axes.md) | 电机 ID、轴角色、型号、参数对照及后续维护 |
| [hardware/](hardware/) | 板卡PDF、器件资料与性能曲线 |
| [analysis/](analysis/) | 专题分析及其独立基线 |
| [reports/](reports/) | 原始日期与基线对应的验证结论，包括失败和未测项 |
| [releases/](releases/) | 交付/候选记录，不能当作当前最新发布清单 |
| [plans/](plans/) | 真正未结计划、完成记录、决策与当前限制 |
| [历史归档](archive/README.md) | 退役实现、旧提案、合并前说明；每项都有归档原因与当前入口 |

推荐先读[软件架构](architecture/software_architecture.md)，再查
[代码结构](architecture/code_structure.md)或[APP逐文件对照](architecture/app_file_responsibilities_v1.md)。
验证命令见[项目工具](guides/project_harness.md)，发布规则见[发布标准](guides/firmware_release_standard.md)，
任务与检查说明见[测试入口](../tests/README.md)。

新增、更新、归档、合并和检查流程见[文档维护规则](guides/documentation.md)。
