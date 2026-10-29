# 文档入口

通信协议与设计请先读 [yg_protocol 主文档](protocols/README.md)。已按 2026-10-26 的工作树整理帧格式、全部 CMD ID、控制/参数/维护/升级流程及软硬件分工；专题字段与测试向量从该页进入。

现行文档按2026-10-21的 `codex/system-refactor` 核对：源码提交3a099dc8（含已提交四入口），另保留原有未提交参数说明等修改。
本轮只整理文档；源码HEAD、工作树与实际设备镜像分别确认，版本号不表示发布或实机PASS。

| 主题 | 入口与用途 |
| --- | --- |
| [architecture/](architecture/) | 软件职责、文件归属、八状态、控制/维护、参数与保护 |
| [protocols/](protocols/) | 当前消息注册、请求/响应、SPEED能力、schema14及兼容向量 |
| [guides/](guides/) | 当前开发、操作、标定与发布指南 |
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

本轮共享目标声明APP1.5.7、Loader2.1.0、layout6、schema14、参数ABI0x000E08F0，
唯一配置来源为shared/targets。当前公布SPEED和两种开环控制能力，能力声明不能替代实机验收。
历史报告只对其源码与设备条件有效；后续候选的缺口见[当前待验收项](plans/active/2026-10-21-current-candidate-acceptance-v1.md)。

新增、更新、归档、合并和检查流程见[文档维护规则](guides/documentation.md)。
