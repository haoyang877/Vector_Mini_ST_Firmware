# Completed Plans

- [参数来源与编译期派生 v1](2026-10-20-parameter-sources-and-constants-v1.md)：输入、派生、标定和运行分类；固定宏逐位等价与PR通过，未烧录。

- [齿槽最小任务 v3](2026-10-08-cogging-minimal-task-v3.md)：启动、安全门、采集/保持和统一结束，独立候选已审并接入，待组合验证。
- [MotorAxis 最小实现 v3](2026-10-08-motor-axis-minimal-implementation-v3.md)：合并零点失败处理，所有模式及调试能力保持，独立PR继承文档失败不改写为PASS。

- [齿槽任务简化 v1](2026-10-08-cogging-core-simplification-v1.md)及[运行所有权解耦 v2](2026-10-08-cogging-core-decoupling-v2.md)：独立交付已选择性集成，任务和运行补偿分开；非整机验收。
- [MotorAxis 简化 v1](2026-10-08-motor-axis-simplification-v1.md)及[参数事务所有权 v2](2026-10-08-motor-axis-parameter-owner-v2.md)：调度/事务职责分开；原有状态和符号未复制。

- [取消 common 与通用堆分配器 v1](2026-09-29-common-heap-retirement-v1.md)：数学归 motor/math，单份固定维护工作区替代 heap；PR 与 normal 编译通过。

- [参数所有权解耦 v1](2026-09-29-parameter-ownership-v1.md)：四类默认值归位，model/application/device 与运行目标分开，节点由参数所有者传值；schema12 不变，PR 与 normal 构建通过。

- [目录职责收口 v1](2026-09-29-directory-ownership-v1.md)：28 个源码/头文件归位，APP 三部分、反馈/标定分开，八组 Keil 保持；PR 和 normal 编译通过，HEX 与原候选一致。

- [设备发现与当前契约收口 v1](2026-09-29-discovery-contract-closeout-v1.md)：GET_INFO 身份与升级目标清单同源，当前契约和 schema12 兼容边界统一；本批未测试或编译。
- [APP/Loader 重复实现收敛 v1](2026-09-29-duplicate-removal-v1.md)：删除旧控制入口，CRC/字节序/请求条件集中，FOC 文件内复用；逐拍对照、ARM ABI、三工程资源与最终门禁。
- [APP 工程更名与导航修正 v3](2026-09-29-keil-app-name-v3.md)：主工程、目标和输出更名为 `YG_ESC_APP`，移除恢复的头文件导航，同步工具入口；未构建、未测试。

- [Keil APP 精简分组 v2](2026-09-29-keil-app-groups-v2.md)：主/配套 APP 统一八组，移除头文件导航项，保留实际源码与编译、调试设置；未编译、未测试。

- [Keil APP 职责分组 v1（历史）](2026-09-29-keil-app-groups-v1.md)：最初细分组与头文件导航，已由 v2 替代；未编译、未测试。

- [APP 模块与变量命名统一 v1](2026-09-29-app-naming-v1.md)：类型、对象、成员单位及职责模块统一；ARM ABI、逐拍差分、三工程容量与最终门禁。

- [框架边界与验证门禁收尾 v1](2026-09-29-framework-closure-v1.md)：五项反向依赖清零，完整门禁、三工程构建、实际包模型与资源对照；实机项目另行验收。

- [齿槽控制状态所有权收口 v1](2026-09-29-cogging-state-owner-v1.md)：四个可写公开对象删除，请求/租约/遥测改值接口，APP与LoaderApp容量通过；完整门禁既有失败未掩盖。

- [快环调度与控制执行边界收口 v1](2026-09-29-execution-boundary-v1.md)：控制执行去状态机/维护依赖，38,880组特征与基线一致，APP容量与RAM未增加；完整门禁的既有失败仍记录。

Completed execution plans are retained here as searchable design history. Their recorded
outputs are historical evidence; rerun the current harness before making a release claim.

- [齿槽转矩补偿解耦与运行路径优化 v1.0](2026-09-19-cogging-compensation-decoupling.md)：
  纯核心与适配层拆分、去重、契约补齐与 PR 验证证据。
- [最简启动/升级方案（去 Boot/Record）v1](2026-09-28-simplified-boot-handoff.md)：
  删除 Boot/Record、boot_epoch、试启动确认与 SHA；APP 扩到 92K，Loader 跳前整包 CRC32 校验；
  三工程离线构建 0 错误 0 警告。

## 已被当前架构替代的阶段记录

以下归档表示阶段计划已被替代，保留原始内容；未实施和未测项没有因此变为 PASS。

- [2026-09-28-system-refactor-e-h-d](2026-09-28-system-refactor-e-h-d.md)
- [2026-09-28-loader-product-refactor](2026-09-28-loader-product-refactor.md)
- [2026-09-28-loader-simplification-review-v1](2026-09-28-loader-simplification-review-v1.md)
- [2026-09-28-loader-system-merge-v1](2026-09-28-loader-system-merge-v1.md)
- [2026-09-28-foc-run-state-analysis-v1](2026-09-28-foc-run-state-analysis-v1.md)
- [2026-09-28-lifecycle-guards-e1](2026-09-28-lifecycle-guards-e1.md)
- [2026-09-28-bench-speed-owner-e2](2026-09-28-bench-speed-owner-e2.md)
- [2026-09-28-internal-decoupling-e3](2026-09-28-internal-decoupling-e3.md)
- [2026-09-28-app-boundaries-e4](2026-09-28-app-boundaries-e4.md)
- [2026-09-28-app-closeout-e5](2026-09-28-app-closeout-e5.md)
- [2026-09-28-capacity-fit-v1](2026-09-28-capacity-fit-v1.md)
- [2026-09-21-yg-parameter-read-adapter](2026-09-21-yg-parameter-read-adapter.md)
