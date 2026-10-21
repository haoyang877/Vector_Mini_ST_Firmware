# 主状态出口收敛交付 v4

日期：2026-10-08。源码提交：`bd5b210`；本批基点：`dcaa9c5` 加冻结的集成功能工作区。
依据[批准计划](../../archive/plans/2026-10-08-motor-state-flow-simplification-v4.md)实施。

## 结果

- INIT／IDLE STOP 保持主状态、工作代际、自检期限和连续窗口；撤销待执行请求及 STOP 序号屏障保持。
- 只有 STARTING／RUNNING／MAINTENANCE 进入 STOPPING，真实停止动作完成后仅回 IDLE。
- FAULT 在源恢复、完整基础健康及收尾确认后，明确 CLEAR 直接 IDLE；不恢复旧控制目标。
- FATAL 没有软件出口；整程序复位从程序入口建立 INIT。
- 删除 tested、自检重启动作、enter_initialization 和返回 INIT 分支；生产 C 净减少 35 行。
- [当前总图](../../architecture/motor_state_machine_v1.md)重新排版并逐图检查，文字与箭头不重叠。

线上值、载荷及参数 ABI 保持；行为兼容说明见[协议 v1.2](../../protocols/yg_protocol_motor_state_v1_2.md)。
任务、SAVE、实际输出确认、取消身份、借用归还、故障优先级、控制数学和保护阈值保持。
故障恢复仍使用现有明确 CLEAR 入口；没有新增自动清错或自动启动。

## 验证与限制

专项 PR：全部 29 个原生测试组及集成检查通过，格式、lint、布局、架构、接口和卫生检查通过。
规则 262144 输入摘要 `9dd9307dcc7bb53f`；72000 维护失败对照剩余字段全部相等，
摘要保持 `afc991123a3208fd`；所有者覆盖待启动撤销、资源占用、自检期限、直接恢复和 FATAL 锁存。
证据：`outputs/runs/motor-state-exits-v4-20261008/summary.json`、
`outputs/motor-state-flow/exit-review/failure-differential.log`、`targeted/`。

专项 PR 总结果为 FAIL：两份已有计划中的四个历史 outputs 链接缺失，影响文档及对应 Python 单元检查。
新总图 PNG 已补齐，最终文档检查仅剩这四处基线问题；未补造历史证据。
最终文档证据：`outputs/motor-state-flow/exit-review/docs-final.json`。

ROOT 独立候选 `7f44bd083902081b00d1761648c158c34350d098` 的当前 release summary 为 PASS，
工作树干净，`hardware_contacted=false`。本批六个生产文件与 ROOT 审核接入的原始快照
SHA-256 逐一相同；其后仅更新图文，无新增生产差异，不重复构建。
对照：`outputs/motor-state-flow/exit-review/release-review-production-comparison.json`。
该集成候选与专项树的基线／检查结果分开记录，不将专项 FAIL 写成 PASS。

本专项不接触硬件、不烧录、不写参数；保留导入功能为未提交工作区差异。
最终以本批独立提交及 ROOT 已审快照交付；当前生产差异为零，新增图文按提交接入。
旧批次差异补丁与应用检查保留为历史；共享源已被其他工作更新，未对新共享源声称旧基线补丁可直接应用。
本次交付身份在 `outputs/motor-state-flow/delivery-v4.json` 中。共享集成树未由本专项修改。
