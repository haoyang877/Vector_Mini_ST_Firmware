# 主状态流转简化第一批验收 2026-10-08

范围：完整设计对照、维护失败重复出口删除；八状态尚未实施。
计划见[专项 v2](../../plans/active/2026-10-08-motor-state-flow-simplification-v2.md)，
设计见[全出口与职责](../../architecture/motor_state_flow_simplification_v1.md)。

## 基线和真实改动

base：`6be266be288c54b0e904a9ace7a7a12cb0f5dbe0` 加计划所列 771 文件冻结工作区。
验证代码 head：`664dcfab0e96460da2bee4d81f0947380aeb66ae`；分支 `codex/motor-state-flow`。
其他任务的上电零点、PI、Watch、协议、测试、工程和文档差异保留未提交。
复制不含忽略实验数据；基线文件在提交首批代码时与共享集成树逐字节一致。

| 提交 | 内容 |
| --- | --- |
| `f3bb56c9f23b5c20b425fe41d94bf873859838fa` | 全部旧新出口、不迁移／拒绝、伪代码、函数职责、字段所有权、候选总图和待决项 |
| `604037774a90f0053c231b99bf35739d688f496d` | 在旧规则上先冻结 72000 组失败矩阵，沿用现有原生测试入口 |
| `664dcfab0e96460da2bee4d81f0947380aeb66ae` | 维护失败统一由已有优先级入口处理；普通状态分支仅保留成功和错误身份拒绝，C 净减 7 行 |

代码只改 `motor_state_rules.c` 的普通 MAINTENANCE 分支：匹配当前代际及任务类别的
失败已经在 `detected_faults` 中锁存，删除后方不可达的第二次失败写入、故障迁移和
重复成功条件。未命中优先级的失败仍按旧身份／类别拒绝，不误记成功。
不新增函数、字段、依赖、状态机、接口或快环／监督交接。

用户随后选择八状态并保留独立 FATAL，静默 INIT STOP 不重启自检窗口或期限。
这属于后续行为变化设计决定，本批代码仍是九状态。图与设计记录该决定，原七状态
候选图保留为历史。INIT 的线上状态码随后确定为 3，0 退役、文档升级 v1.1。
这些决定另批实施；维护内 SAVE 尚未定案，本批不能混用其后续验证结论。

## 行为验证

在同一原生程序链接冻结旧规则与候选新规则，逐次比较所有上下文成员及返回字段，
不比较 C 结构填充。334146 次调用完全相同，含 NULL 参数误用、262144 组原冻结输入
和 72000 组专项失败输入；不是只比较成功路径或摘要。

原摘要：`14acc14ec440764b`；专项失败摘要：`afc991123a3208fd`。
失败矩阵覆盖有效／非法类别、活动／非活动身份、原代／旧代、代际耗尽、
UNKNOWN／UNCOMMITTED／COMMITTED／非法效果、许可／输入守卫、
失败与 STOP／CANCEL／完成／新启动／FATAL 的组合。

完整生产所有者、真实齿槽候选、相电阻与摩擦 worker 的既有原生检查 PASS。
覆盖初始化与恢复期限、控制准备、启相回读、STOP 撤销、故障优先、独立任务／SAVE
票据、原启动进展、序号屏障、迟到结果、真实资源借用及归还。硬件由原离线桩替代。
范围内的协议映射、schema12、控制数学、保护阈值、目标赋值、调度和资源布局未修改。

## 完整 PR 验证及基线失败

| 检查 | 冻结基线 | 代码 head |
| --- | --- | --- |
| doctor／format／lint／project-layout | PASS | PASS |
| 29 个原生测试组／integration | PASS | PASS |
| Python unit | 58 PASS，1 FAIL | 58 PASS，同一项 FAIL |
| architecture／interfaces／hygiene | PASS | PASS |
| docs | 4 处历史输出链接失败 | 相同 4 处，日志逐行一致 |
| PR 汇总 | FAIL | FAIL，无新增失败 |

失败文件：导入的 `2026-09-29-current-pi-limit-refresh-v1.md` 与
`2026-09-29-startup-current-offset-pi-fix-v1.md` 引用了 4 个不在 Git 中的历史
`outputs/.../summary.json`。隔离工作树没有复制这些实验输出，unit 中的文档检查
同样报告它们。未修改其他任务计划或伪造文件绕过检查，不能宣称完整 PR PASS。

当前证据位置（均相对专项工作树；这些输出不是持久文档链接）：

| 文件 | SHA-256 |
| --- | --- |
| `outputs/runs/motor-state-flow-baseline-20261008/summary.json` | `00103022798565d50b39501f75aca1702ddb2a4c178028dc8afedf7b84c583c2` |
| `outputs/runs/motor-state-flow-candidate-20261008/summary.json` | `85a3ff3ea257352d2e97e9bc5e100b0f12377fa8f6eb7cb98a5586d925f561ee` |
| `outputs/motor-state-flow/equivalence.log` | `0ad6d428cda2b0c3d8eb7c4e4c86441501adb9da3d96ee3b1c68f745c41f8d6f` |
| 冻结规则源码 | `dcba81ef9ac33b1f6ad1222836f4b2a9ae9e27f700d9651e43924129e5d4de5c` |
| 候选规则源码 | `391bc9a4b7833543ec1a5dd6b80e59b0df6d5da26bd188f9ca12ec067ec952f8` |
| 失败矩阵夹具 | `0352d7fa7529f14f90aa8e4d0409fca389e347fc4b59f3fbffc3ed7baaba1665` |

直接对照程序及 reference 编译对象存于 `outputs/motor-state-flow/`。
永久测试仍是现有 `tests/unit/native/test_run_state.py` 入口，会执行两组冻结矩阵。
不向 MCU 链接对照路径。

## 回退、交付和限制

首批代码可独立 revert `664dcfab`，测试及设计也分别保存；不 reset／clean 其他工作区。
交付分支不包含导入的其他任务未提交代码，不能单独检出该 head 便声称重建了本次
完整功能输入；需在冻结集成基线上应用专项提交或专项补丁，并保留该基线差异。

本批未做 MCU 构建、map／HEX 比较、真实时延／栈／ROM／RAM 测量或硬件验收。
不宣称当前发布通过或控制全状态空间的形式证明。新增测试证明所列输入一致；
结构证明依据是失败匹配条件与公共优先级先行。后续状态数和原地 STOP 会有明确
行为差异，不能沿用本批的等价结论。尚未合并，托管工作树和证据保留供协调审阅。
