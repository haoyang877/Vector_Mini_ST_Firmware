# 齿槽最小任务重写 v3 验收

日期：2026-10-08。独立分支 `codex/cogging-core-simplify`，base
`79cf380dfb58abe2e23ca85801c7e0c7a5cf5874`。本轮仅改任务实现、两份现用夹具与本模块文档；
v1/v2、继承现场、运行适配/三份纯计算 `.c`、公开头、APP 和 Keil 原文件保持。
最终源码提交/head、原始 SHA 与证据索引见 `outputs/cogging-core-v3/review-bundle.json`。

## 最小职责与实际删除

[一页流程](../../architecture/cogging_minimal_task_v3.md)对应启动、定位采样、统一结束、候选交接。
标量聚合到 s_task；算法观察符号 s_cogging_calib 与全部运行观察符号保留。
仓库工具/Keil 配置没有原任务散落标量引用，只有已同步修改的两份夹具。

- 原 finalize_pending/save_pending 在真实入口下互斥，收为 pending；claimed/cancelled
  收为 claim。删除失效的“已归还但 cancelled 仍 true”组合，所有公开查询/动作保持。
- active 保留，直到目标及两类控制器完成清理才归零，新增夹具验证清理中仍未释放。
- 删除 service_tick2k_hz 的 bool 终态转译与主入口 complete 临时变量；每拍入口直接消费
  采样核心 FAILED/COMPLETE，结束只调用 finish_task。
- 相同运动参数和电压/温度判据各集中一次，调用顺序/阈值及启动与运行电流门差异保持。
- 定点保持与启动初始化逐语句比较仅有字段重命名；分频/平均、迟滞测速、双向采样、
  成表与 CRC 数学不改。未删除保护、补偿、取消、工作区/真实保存边界。

原公开调用者仍是 MotorMaintenance、MotorStateMachine、MotorAxis 前台与参数服务。
COMPLETE 仍只表示采样完成；APP 控制功率、原 epoch/关相认领及参数真实存储提交。
本轮没有新源文件/共享工程改动；原 v2 runtime 登记仍由 ROOT 统一接线。

## 同输入行为验证

增强同一现用夹具，分别包含 base Git 原始任务与新任务，不写入或回退生产树。
参考仅重映射夹具注入的私有字段，真实旧函数完整编译；输入、核心和假 I/O 一致。

- 完整 15,030,360 拍控制/状态摘要仍 `37c45ddf`，候选记录 CRC `72892f5a`。
- 140 个有限性/阈值输入案例摘要均 `0eb82685`，两份完整齿槽日志逐字节相同，
  SHA-256 `4a9e486ec5c942ec34051ce262bacd223680d1394990c28f378f84bb28e9dcf5`。
- 两版均通过清理期间未释放、发布前/后取消、旧表或已发布 RAM 保持、成表失败、
  保存失败/成功结果通知。模拟通知不等于实物 Flash 写入成功。
- 既有运行独立链接、owner epoch/关相/归还等夹具继续通过。
- format --check、lint 与完整 PR 全部通过：
  `outputs/runs/20261008T102303929690Z-0adb5aba/summary.json`，Git clean，hardware_contacted=false。
  源码提交 `0adb5aba8cc972e79f43ec215d745ccea151453d`；当前任务原始 SHA-256 为
  `9d6ef0b8fd72a7582091de930b101fff91a29cb300ee3d633779bc140dbcb335`。
  最终文档收尾复用相同固件/夹具/工程输入，单独核对最终文档链接，不再重建或重跑 PR。

参考/当前日志及数学/模块不变核对在 `outputs/cogging-core-v3/`，机器对比为 comparison.json。

## 容量、初始化与限制

复用同冻结输入的 v2 normal 产物，不重建基线。v3 使用同 SHA 的 outputs 工程副本，
原逐文件设置/布局不改；不操作 Keil 原配置，不构建 LoaderApp 或其他工程。

首轮全零聚合上下文转入 ZI，导致原 RW 压缩取消：代码减少但整包 ROM 增加 140 B
到 93396 B，原始诊断产物保存在 zero-init-diagnostic，不作收益或交付镜像。
最终 pending/claim NONE 使用私有非零编码、显式初值，恢复原 RW 压缩；无 section 属性、
无平台宏或公开 ABI 改动。实际输入改变后只重编该任务并重新链接，未重复编译未变源。

| B | v2 基线 | v3 最终 | 差值 |
| --- | ---: | ---: | ---: |
| 任务 Code+RO | 2198+10=2208 | 2148+10=2158 | -50 |
| 任务 RW / ZI | 48 / 2112 | 48 / 2112 | 0 / 0 |
| 整包 Code / RO | 90120 / 3124 | 90068 / 3128 | -52 / +4 |
| 整包 RW / ZI | 304 / 26600 | 304 / 26600 | 0 / 0 |
| map ROM（初始化压缩后） | 93256 | 93212 | -44 |
| 整包 RW+ZI | 26904 | 26904 | 0 |

最终实际日志 0 错误、0 警告；仅此独立输入的链接结果，不代表 ROOT 组合包容量。
Task 自身帧 48→40 B，新增 finish_task 自身帧 24 B；不推断全调用链动态峰值或 WCET。
源文件 323→335 行，类型/单位/事实表达更明确；不将行数或重排本身当容量收益。

没有实机、真实关相回读、Flash 断电、动态峰值栈/抖动/WCET、release 或 LoaderApp 结论。
所有产物/工程/源哈希和最终可审提交在 review-bundle.json，当前计划见
[重写 v3](../../plans/completed/2026-10-08-cogging-minimal-task-v3.md)。
回退只撤销本轮 v3 到 base，保留 v1/v2 与继承冻结；等待 ROOT/用户审查，不自动集成。
