# 文档入口

通信只保留一份[现行协议说明](protocols/README.md)，已按 2026-11-03 的
`the-system-refactor` 代码核对。报文、命令、反馈、标定和升级均在该页；
回归向量位于 `tests/fixtures/protocol/`，不作为设备支持清单。

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
