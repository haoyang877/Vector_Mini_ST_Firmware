# 主状态机条件与职责精简验收

日期：2026-09-29。功能基线 ccd8d8a8；同步目录主线 83f5aae0；实现提交 f23e6eb5、
48aa68c4。范围是保留电机功能、九个主状态和原外部行为，改善检查来源及阅读结构。

## 最终结构

监督入口按原顺序消费 STOP、故障、任务结果、归还与恢复；`advance_current_state`
用九状态 switch 展开状态进展。BOOT/SELF_TEST 保留原期限、关相和稳定窗口；
STARTING/RUNNING/STOPPING 共用纯规则的持续守卫与停止确认。READY 等请求，
MAINTENANCE 等功能结果，FAULT 等显式恢复，FATAL 不被普通 CLEAR 恢复。

自检分为感测、配置、停机确认，八个线上诊断位保持。共同的有限数与温度新鲜度谓词
只保留一个实现；输出、资源和配置验证复用本次事实。完整配置验证在一次条件读取中
只调用一次，READY 资格查询由原三次降为一次；没有跨动作或长期有效缓存。

删除 `fault_sources_clear` 和 owner 的 `self_test_complete`。无故障由活动故障值
推导，自检完成只由 `rules.tested` 保存。计数达到四拍只是提交时窗口证据。
纯规则先处理活动故障，随后普通分支删除重复故障判断和一行转发的健康包装函数。

目标格式与限值由现有 `motor_execution` 的只读函数拥有；生命周期、输出、标定和
反馈准入仍由状态机拥有。保持 INVALID、FAULT、DENIED、OUT_OF_RANGE 的原结果
优先级。派生保存与控制使能分开处理，真正启相继续在短临界区重新读取条件。

没有新文件层、状态副本、配置缓存或 goto。数字输出回读、取消归还、任务原始代际、
SAVE 实际效果、远程重试、迟到结果和 Loader 停机票据保持原契约。

## 行为证据

同一更新后的 owner 夹具编译真实基线和最终源码，公开结果日志逐字节一致。

| 对照 | 数量与结果 |
| --- | --- |
| 目标格式、限值和拒绝优先级 | 1280 组，9f89d0ff40a00225，两版一致 |
| 启动资格与模式受理 | 2048 组，89ae9240561b4167，两版一致 |
| 规则优先级、身份、取消和效果 | 262144 组，14acc14ec440764b，两版一致 |
| 真实新 LoaderApp 镜像内存组合 | 冷启动、ENTER、看门狗救援、损坏、断链、喂狗进展、Flash 有界等待通过 |

旧规则黄金摘要 f98662bc6ed8b902 的夹具把“活动故障为零”和“无故障为假”作为
独立输入。删除派生字段前，先用原规则按实际 owner 不变量归一化这个矛盾组合，
取得 14acc14ec440764b，再对照最终规则；不是单独更新最终期望值。

参数和通信黄金向量、全部原生控制/标定测试通过。motor、services、communication、
Loader、shared、host 源码相对同步主线未改变；本轮只改变 APP 所有者/控制职责和测试。

## 资源与容量

静态 ROM 按 Code + RO + 未压缩 RW 计，内联字节已经包含在 Code 中。

| 目标 | 基线 ROM | 最终 ROM | RAM | owner + rules ROM |
| --- | ---: | ---: | ---: | ---: |
| 普通 APP | 90604 B | 90764 B | 31592 B，未变 | 7700 → 7374 B |
| LoaderApp | 92216 B | 92376 B | 31632 B，未变 | 7928 → 7602 B |

完整 APP ROM 增加 160 B。状态机模块减少 326 B，其中一部分是校验代码转入控制
模块；状态机与控制模块合计增加 162 B，链接对齐产生 2 B 差别。不能把模块下降
当成整个 APP 缩减。本轮优先明确职责，仍满足 Loader 槽容量。

实际打包镜像 92080 B，94208 B 槽余 2128 B。owner 上下文仍为 184 B，删除一字节
布尔记录后对齐占用不变。owner 源码 1519 → 1501 行，去掉注释/空行的代码
1399 → 1371 行；私有函数因明确检查组和动作由 56 → 60，对外接口仍为 28。
全部相关模块总行数没有减少，新增检查分组和控制边界有明确可读性成本。

两种 APP 构建均 0 错误、0 警告。静态已知最大栈仍 548 B，调用图含未知路径，
不能据此宣称实机栈峰值或中断延迟保持不变。

## 验证记录与边界

基线 release：outputs/runs/state-conditions-baseline-clean-20260929/summary.json。
最终 release：outputs/runs/state-conditions-final-release-v2-20260929/summary.json。
产物、基线源码、日志和 SHA256：outputs/state-conditions/summary.json。

首次基线验证被新合入发现功能的两处格式问题挡住，已只修正格式并提交。
初次 Loader 构建启动方式产生重复实例，via 临时文件失败；改为顺序等待构建后通过。
首次 PR 发现新声明在类型 include 之前，以及旧测试字段引用，已修正并通过完整 release。
失败日志保留为过程证据，不作为通过依据。

本轮没有连接设备、烧录、写参数或命令电机。没有 CubeMX 再生成、物理关断、实机
ISR 时延、抖动或栈峰值验收。新镜像只有离线资格，没有量产/台架发布批准。

阅读入口见[主状态机当前设计](../../architecture/motor_state_machine_v1.md)。

## 参数所有权批次适配

2026-09-29 将参数批次 f6fadad49aa30acfdef5c6426a2c5436207e2b67 合入独立分支，
适配提交 ab6aab7。状态机、控制目标校验和真实 owner 夹具的配置访问改为
model/application/device；还原这些成员路径后，三份文件的代码 token 与 81dc01e
完全一致，不修改分支、常量或调用次序。纯规则源码与前次通过版本完全相同。

真实 owner 使用当前生产参数/维护/故障模块重新编译；1280/2048 摘要和完整日志与
冻结基线一致，format/lint 通过。结果见 outputs/state-conditions/parameter-adaptation/
summary.json、projection.json 和 owner/test.log。没有重跑 262144 组纯规则矩阵、
完整 release、多目标构建或实机检查；依赖不变部分复用上文证据。

上表资源属于参数分组前的状态机交付，不能当作适配后新镜像的实测资源。
适配相对 f6fadad 只增加本轮状态机、目标校验、直接测试及文档共 14 个文件差异，
不包含其自身参数或 host_app 改动。主线由“00- 软件架构管理”独占集成，
最终集成仍须通过仓库要求的 PR 门禁后才能完成交付。

## ROOT 参数与状态机集成

ROOT 将适配交付 680d0ce3659c8291ff706fa41cbe274f973432e0 合入
codex/system-refactor，集成源码为 00590e3d002ce319725db8d26583175e9fb3dc93。
参数分组与状态机职责修改共同编译；没有再次运行冻结基线对照或多目标构建。

本次 PR 的 tests、architecture、interfaces、lint、project-layout、docs、hygiene
均通过，证据位于 outputs/runs/state-parameter-integration-pr-20260929。
全仓库 PR 仍为 FAIL：format 扫描到了并行上位机会话尚未完成的未跟踪源码。
保留原始失败结果，固件集成不据此声明全仓库 PR 通过；上位机收口由其所有者负责。
随后对全部非 host_app 的仓库自有源码补查格式，通过；报告更新后的文档检查通过。
这两项证据位于 outputs/state-parameter-integration-20260929/followup_checks.json，
不替换原 PR 的失败记录。

normal 主工程本次编译 0 错误、0 警告。ROM 为 90828 B，静态 RAM 为 31600 B；
相对参数批次 ROM 增加 468 B，RAM 未变。HEX SHA256 为
ac5e56295cb9fe3567a8500a17c6106aa8cfdc88b93918e354e4904189762531。
以上为普通 APP 的当前结果，不能沿用前表声明当前 LoaderApp 的容量。

冻结实机候选没有替换。ROOT 未连接硬件、烧录、复位或运动；新的参数 RAM 布局
不适用于冻结候选的调试脚本。后续持久化角色/设备身份分离不属于本次实现。
