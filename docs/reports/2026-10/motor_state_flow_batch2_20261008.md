# 八状态与静默 STOP 验收 2026-10-08

本批已在隔离源码实施用户选择的八状态及静默 STOP，未合并或进行硬件操作。
计划见[v3](../../plans/active/2026-10-08-motor-state-flow-simplification-v3.md)，
实际图与出口见[主说明 v1.1](../../architecture/motor_state_machine_v1.md)，
协议变化见[状态反馈 v1.1](../../protocols/yg_protocol_motor_state_v1_1.md)。

## 真实改动与冻结输入

代码 base：`c1ca674` 加第一批所保留的未提交集成功能；
代码 head：`3d7899bb45e55ae14bf54f6d5c04ff7097019af5`。
完整原始 base 与 771 文件冻结补丁／清单见[第一批报告](motor_state_flow_batch1_20261008.md)。
本批待改原文件单独归档到 `outputs/motor-state-flow/eight-baseline/`。

- BOOT／SELF_TEST 合并为 INIT，READY 改 IDLE；删除 BOOT_DONE 事件、规则出口和所有者提交。
  直接初始化规则为 INIT、epoch=2，保留原正常 BOOT 完成后的起始身份。
  内部 INIT=1、IDLE=2，后续 3..8 值保持，0 不再是合法内部状态。
  计时从首次监督拍开始；旧 BOOT 完成时的起点重置随该阶段删除。资源晚归还不能再延长
  单次初始化期限，499 ms 归还后 500 ms 仍超时的特征测试明确覆盖该合并差异。
- 静默 INIT／IDLE STOP 不进入 STOPPING；仍关闭输出、撤销目标与任务意图并推进代际。
  INIT 不重启原期限或连续健康窗口；资源未静默、待确认取消或故障仍按真实收尾处理。
  INIT 静默 STOP 代际只推进一次，原未 tested 的停止路径会推进两次；这是批准的行为差异。
- FATAL、STARTING、取消／清错资格、动态启相复核、STOP 序号屏障、快慢环时基保持。
  没有新增初始化阶段／等待布尔、第二状态机、兼容头、框架或接口。
- INIT CAN 状态值为 3，0 退役；其余状态投影、控制模式、45B payload、CRC、统一帧版本
  `0x01` 和参数 schema12 保持。新增 v1.1 兼容说明和黄金 payload。
- 原任务与 SAVE 仍分别收尾并保留独立票据／结果；维护内串行 SAVE 未实施。
  电机算法、保护阈值、参数、工程、链接和平台输出实现未改。

暂存源码从 Git HEAD 重放本批变换，未整文件提交导入的其他任务修改。
工作区继续包含上电零点、PI、Watch、协议和测试功能。已导入但原来未跟踪的
`tests/unit/startup_current_offsets_test.c` 仅将两个 SELF_TEST 引用改为 INIT，仍不提交他人的
整份新测试。Watch 既有未提交指南中的枚举也同步；这些集成差异通过专项工作区补丁交付。
单独检出代码 head 不包含这些功能，不能声称重建了本次完整验证输入。

## 旧新比较与针对性验证

以同一输入调用冻结旧规则与新规则，比较状态、任务类型／活动性、结果／效果、epoch、
首／累计故障、tested、动作和拒绝。只允许两类明确投影：静默 INIT／IDLE STOP 政策，
以及内部 BOOT 完成位退役后的事件拒绝。不用旧测试全部相同掩盖批准变化。

334146 次调用：326417 次逐字段完全相同；7729 次经批准政策投影后完全相同。
输入包含非法组合，用于拒绝及边界覆盖；不是实际设备可能经历全部组合的主张。
该比较不包括旧 BOOT 状态 0，它已被删除；初始化合并由完整生产所有者测试覆盖。

八状态 262144 组新摘要：`0bf3e5aa3c3ea0e0`；
72000 组失败摘要保持 `afc991123a3208fd`。旧九状态摘要保留于第一批报告，不冒充本批摘要。

新增生产所有者特征测试通过：连续 4 拍均 STOP 仍完成健康窗口；500 ms 普通期限和
1500 ms 初次期限不会被反复 STOP 延长；IDLE STOP 推进一次代际且保留旧序号屏障；
资源未释放继续 STOPPING，释放后仍需 INIT，自检完成不启相。
既有自检超时、回绕计时、初始化无法关相 FATAL、CLEAR、启相回读、故障优先、
控制目标、任务／SAVE 身份、后台认领／取消归还和真实维护 worker 测试全部通过。

真实状态映射核对八状态、两类标定及非法内部值；真实编码器核对八组完整 45B 黄金
payload，原 CAN FD pack／unpack、长度、填充与 CRC 向量继续通过。
八状态调度矩阵在诊断开／关两种构建各执行 17280 组，通过。

## PR profile 与证据

`outputs/runs/motor-state-eight-candidate-20261008/summary.json`：FAIL。
doctor、format、lint、layout、architecture、interfaces、hygiene PASS；
29 个原生组和 integration PASS；Python unit 58 PASS、1 FAIL，仍是文档链接检查。
docs 的四处历史 `outputs` 链接缺失与冻结基线日志逐行一致，无新增失败。
没有复制旧实验输出或修改其他任务计划来伪造完整通过。

| 当前证据／源码 | SHA-256 |
| --- | --- |
| PR summary | `535639a24d06f12cffba768ca39f30ef1114031b1ad4ac2a98bfba5350ef1782` |
| `eight-baseline/files.json` | `6caffe6c6341f9b131ad979f89e60c9b6ee0fa2a8932e4760b666aa404da7ac2` |
| `eight-equivalence.log` | `ecce7e2ad9cef82d576574bd2350f1df7e900ab0bf508e2a2d1db735eac9914f` |
| 实际工作区规则 | `fa9f0042e12d1503b4145b5e0760d31f6f5dc1b5fbd5c92eb1985fc91d015126` |
| 实际工作区所有者（含保留的集成功能） | `011c3c213e2b508722e4f721a869b15df68d6db15bc8e144d2959f10b1ac6808` |
| 八状态规则夹具 | `7a1a1cf7737f0ab9b810f3176bc18c61ae7fa2900c2fe882e5eefbdfbf73b06fd` |

报告中的证据为本次新运行；未构建 MCU，因此没有本批 map／HEX、栈／ROM／RAM或
实际时延证明，也未验证实机输出。独立 revert `3d7899b` 可撤销本批源码和测试；
回退时仍保留原未提交集成功能，不能 reset／clean 其他工作区。
集成前需核对冻结功能差异及同步未跟踪测试的枚举变化；托管树和输出保留供审阅。
