# 功能任务驱动主状态机重构验收（2026-09-28）

实现提交 `aaff067cb956cc766e67edd091d2e0e09d903220`，分支 `codex/motor-state-machine`；
基准 `fda84c2c9d99407f9b9ffe8adc43d8118a651ade`。本批完成功能端和唯一状态所有者迁移，
接受范围为离线源码与编译验证。**不是 release PASS，也不是烧录或实机验收结论。**
原工作区和容量优化工作树的未提交实验不在本批修改范围。

## 实际结果

旧 foc_run_state、foc_mode_dispatch、foc_run、motor_work 与 ModeNow/ModeLast 枚举、
模式差分迁移、下一模式/功率动作返回协议已删除，没有兼容副本。
主状态机持有控制、维护 kind/epoch、后台借用、功率许可与保存衔接；保留 AppLifecycle
纯规则作为唯一迁移核心。实际功能只报告运行、准备、完成、前台整理、停止或失败事实。
详见[当前主状态机契约](../../architecture/motor_state_machine_v1.md)。

齿槽先准备零矢量，所有者复核当前身份、健康和输出状态后启相。错误或启相失败终止功能，
重复准备通知不重复启相。结果消费原子撤销执行资格；过期结果、STOP/取消后的完成不能
重新运行或保存。故障保持快速关断；输出未实际关闭时，电压保护继续运行。

相电阻与摩擦功能原先依赖禁用周期的分发取消归还内部状态，本批改为终态自行归还。
相电阻新请求明确清除旧功能终态；摩擦候选完成后仍可应用。保留原算法计算与抽点频率。
取消按当前维护类型执行，前台借用必须真实归还；原标定完成与派生保存提交分别可查询。

## 离线证据

| 检查 | 当前结果 |
| --- | --- |
| 24 项原生入口 | 全部 PASS；包含控制/协议黄金向量、参数、数值、实时调度边界、Loader 核心 |
| 真实状态机所有者 | PASS；当前/过期身份、STOP、取消借用、拒绝故障语义、故障优先、启相和关相失败 |
| 实际相阻/摩擦适配及核心 | PASS；理想离线模型下重复完成、候选保留、失败与自主释放 |
| 齿槽算法 | PASS；原完整采集模型及两种电流分流配置，前台整理与取消边界 |
| Python 集成 | 19/19 PASS；包括禁止旧状态契约、禁止功能侧正常启相 |
| Python 单元 | 38/39 PASS；唯一失败为基线历史证据缺失链接 |
| 改动范围格式与 lint | 43 个自有源码文件，0 问题 |
| 架构/工程输入/卫生 | PASS；架构 5 项已知债务、0 新增 |
| 正式 PR / release | FAIL；以下全局问题与基线诊断逐字相同，没有扩大基线 |

全局 format 16 项、lint 603 项、未登记接口契约 34 项、历史 reports 中证据链接 10 项，
均与重构前相同；接口检查另外显示 54 项登记债务。未改写基线豁免或重写历史报告。
release 因前置门禁失败未进入其内置构建，release-manifest 也为 FAIL。
单独对实现提交进行 Keil 全量重建，普通 APP 为 0 Error / 0 Warning；这不构成正式发布通过。

最终正式证据为 `outputs/runs/state-machine-final-release/summary.json`，其中 Git 为上述
实现提交且 clean，hardware_contacted=false。PR 日志为 `outputs/runs/state-machine-final-pr/`；
其唯一随后修复项为协议文档残留的旧源码链接，最终 release 已回到基线的 10 项历史链接问题。
改动范围证据为 `outputs/state-machine/scoped-style.json`；构建及产物摘要为
`outputs/state-machine/committed/` 和 `outputs/state-machine/evidence.json`。
摘要包含实现提交、仓库编译输入哈希、当前 AXF/HEX/MAP 与日志 SHA-256。

## 资源与部署边界

使用同样 Keil/ARMCC 设置，MAP 的 Total ROM Size 含压缩后 RW 初始化，不用
Program Size 的未压缩 RW 数直接相加。

| 镜像 | 基准 ROM | 当前 ROM | 差值 | RAM |
| --- | ---: | ---: | ---: | ---: |
| 普通 APP | 89808 B | 89216 B（87.13 KiB） | −592 B | 31600 B，未增加 |
| Loader APP 诊断 | 97512 B | 96912 B（94.64 KiB） | −600 B | 31840 B，未增加 |

Loader APP 实际链接占用跨度为 96952 B，分区 79872 B（78 KiB），超出 17080 B。
正式 Loader APP 构建仍因空间不足失败；诊断仅扩大虚拟链接区域，名称
`NONDEPLOYABLE-app`，不生成可部署 HEX、不修改正式分区，严禁烧录。
容量闭合继续由独立 Flash 优化任务处理。

FOC 数学、CAN Payload、模式线上编号、schema11 与参数持久化结构保持原契约，
原有黄金向量通过；私有结构与 SWD 字段已变更，必须使用同版 ELF。
20 kHz / 2 kHz 频率和算法分频未改；目标 MCU 周期、动态栈、物理栅极停机、真实运动
及标定电流尚未实测。未访问探针、烧录、设备参数写入、复位或运动操作。
最终文档调整运行 quick 验证，证据为 `outputs/runs/state-machine-docs-final/summary.json`，
仍只受相同基线门禁阻塞；另行文档扫描没有新增链接问题。本批文档完成提交不改任何
编译输入，因此实现提交上的产物证据仍适用。
