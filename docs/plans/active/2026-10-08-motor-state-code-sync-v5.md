# 按已审状态图同步固件 v5

日期：2026-10-08。基点：`e73e6e5` 加保留的集成功能工作区。
用户要求按当前 v1.2 总图更新状态机代码；主要出口已在 `bd5b210` 实施。

目标：核对 INIT／IDLE STOP、三种活动状态停止、停止完成直接 IDLE、CLEAR 直接 IDLE、
FATAL 无软件出口及真实输出确认；清除公开接口中的旧 BOOT／READY 状态说明。

核对发现：FAULT 恢复时若工作代际耗尽，核心锁入 FATAL，却仍返回 ACCEPTED，
公开 CLEAR 因此可能报告清错成功。修复为明确 REJECT_FATAL，保留故障与关相动作。
正常 CLEAR、STOP 屏障、控制数学、任务／SAVE 取消与归还、线上状态值和参数 ABI 保持。

基线与验证证据置于 `outputs/motor-state-flow/code-sync-20261008/`。
接受：新增代际耗尽 CLEAR 必须拒绝且保持 FATAL 的回归检查，原 72000 维护失败摘要保持；
完整状态摘要保持 `9dd9307dcc7bb53f`，新增定向检查覆盖原随机输入未命中的代际耗尽恢复。
运行 PR profile；旧四处历史输出链接问题单独记录，不补造证据。
回退：revert 本批独立提交；不修改其他树、实验数据或硬件。

针对性检查已验证先失败后通过；生产所有者、72000 维护失败矩阵和实际任务完成／取消检查均通过。
核心中 INIT 仅由实例初始化建立，已无 BOOT／READY、tested 或自检重启动作。

PR 检查：格式、lint、布局、架构、接口、卫生、集成和全部 29 个原生测试组通过。
总结果为 FAIL，仅原四处历史输出链接导致文档检查及对应 Python 单元检查失败。
证据：`outputs/runs/motor-state-code-sync-v5-20261008/summary.json`。
本批未构建或烧录 MCU；上一 ROOT release 证据只覆盖本批前的生产快照，不能作为新修改的 release 验证。
