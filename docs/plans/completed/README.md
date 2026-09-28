# Completed Plans

Completed execution plans are retained here as searchable design history. Their recorded
outputs are historical evidence; rerun the current harness before making a release claim.

- [齿槽转矩补偿解耦与运行路径优化 v1.0](2026-09-19-cogging-compensation-decoupling.md)：
  纯核心与适配层拆分、去重、契约补齐与 PR 验证证据。
- [最简启动/升级方案（去 Boot/Record）v1](2026-09-28-simplified-boot-handoff.md)：
  删除 Boot/Record、boot_epoch、试启动确认与 SHA；APP 扩到 92K，Loader 跳前整包 CRC32 校验；
  三工程离线构建 0 错误 0 警告。
