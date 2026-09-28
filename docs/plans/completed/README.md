# Completed Plans

- [齿槽控制状态所有权收口 v1](2026-09-29-cogging-state-owner-v1.md)：四个可写公开对象删除，请求/租约/遥测改值接口，APP与LoaderApp容量通过；完整门禁既有失败未掩盖。

- [快环调度与控制执行边界收口 v1](2026-09-29-execution-boundary-v1.md)：控制执行去状态机/维护依赖，38,880组特征与基线一致，APP容量与RAM未增加；完整门禁的既有失败仍记录。

Completed execution plans are retained here as searchable design history. Their recorded
outputs are historical evidence; rerun the current harness before making a release claim.

- [齿槽转矩补偿解耦与运行路径优化 v1.0](2026-09-19-cogging-compensation-decoupling.md)：
  纯核心与适配层拆分、去重、契约补齐与 PR 验证证据。
- [最简启动/升级方案（去 Boot/Record）v1](2026-09-28-simplified-boot-handoff.md)：
  删除 Boot/Record、boot_epoch、试启动确认与 SHA；APP 扩到 92K，Loader 跳前整包 CRC32 校验；
  三工程离线构建 0 错误 0 警告。
