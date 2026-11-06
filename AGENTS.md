# Vector Mini ST Firmware Agent Guide

本文件只定义工作入口与执行流程，不重复编程和架构细则。

## 规则入口

- [STYLE.md](STYLE.md)：唯一编程规范，包括决策优先级、拆分判定表、命名、格式、注释及重构验收。
- [ARCHITECTURE.md](ARCHITECTURE.md)：唯一稳定架构边界，包括职责、依赖、状态所有权与执行环境。
- [docs/README.md](docs/README.md)：现行主题入口；具体契约从这里定位。
- [文档规则](docs/guides/documentation.md)：计划、结果和归档方式。
- [固件发布规范](docs/guides/firmware_release_standard.md)：发布要求。

优先级及例外按 STYLE.md 执行；具体模块契约只在所属文档维护并由架构入口引用。
修改细则时更新其唯一来源，本文件不复制一份，也不把历史报告当作现行规范。

## 仓库地图

- firmware：STM32G4 / AT32F456 共用电机 APP。
- projects：板级入口、Keil 工程、启动/链接与 CubeMX 输入。
- loader、host_app、shared：独立产品和共享契约。
- tests、tools：离线测试、构建及显式台架工具。
- docs：架构、协议、指南、计划和报告；outputs：忽略的生成证据。
- templates：新模块起点；生产标准样例为
  [PI 控制器](firmware/motor/control/pi_controller.h)，适用边界见 STYLE.md。

## 执行流程

1. 确认分支、工作树和未提交变更，保留用户修改与忽略的实验数据。
2. 阅读所属模块契约，搜索现有实现并检查受影响调用方。
3. 显著工作先在 docs/plans/active 记录目标、不变行为、基线、验收、回退；
   按 STYLE.md 判定是否需要函数、文件、结构体或共用抽象。
4. 在最小范围实施，同步接口引用、工程输入、必要测试和持久文档。
   不为规范整理批量改名、格式化无关文件或修改第三方代码。
5. 执行下表要求，记录当前结果、既有失败和未测范围；完成计划后归档并更新索引。
6. 交付说明改变了什么、为何改变、验证结果和实际限制，不把局部通过写成整仓通过。

源码、工程目标和文档使用 AT32F456，路径/CLI 使用 at32f456。
跟踪文件不写绝对工作站路径、探针序号或凭据；产物归 outputs。
协议或参数 ABI 变更须同步版本化文档、黄金向量、兼容说明和对应测试。
其他编程、生成代码和硬件边界直接执行上述规则来源。

## 环境与验证

安装 uv 后运行 uv sync --locked --extra dev。

- Quick：uv run python tools/run.py verify --profile quick
- PR：uv run python tools/run.py verify --profile pr
- Release：uv run python tools/run.py verify --profile release
- 诊断：uv run python tools/run.py doctor --profile <profile>

| 变更 | 最低验证 |
| --- | --- |
| 仅文档或普通工具 | quick；harness 规则变化使用 PR |
| 固件逻辑、协议、参数、工程输入、模板或测试 | PR，加与变更相匹配的验证 |
| CubeMX、链接、发布或硬件台架 | release；实机部分按明确授权的台架场景 |

verify 已包含 format --check 和 lint，无需无原因重复运行全部检查。
doctor、quick、PR 均不接触硬件；release 构建也不下载固件。
硬件命令必须有明确台架与场景；默认验证和 CI 不得触发硬件动作。

## 评审与交付

- 不新增架构违规，不扩大债务豁免，也不靠削弱测试掩盖失败。
- 重构证据按 STYLE.md；保留 ABI、并发和约定行为，功能裁剪单独说明。
- 发布结论必须有当前 summary.json 和匹配的产物哈希；旧 outputs 不能作为当前证明。
