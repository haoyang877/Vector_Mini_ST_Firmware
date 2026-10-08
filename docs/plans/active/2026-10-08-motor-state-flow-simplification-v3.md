# 八状态与静默 STOP 实施计划 v3

日期：2026-10-08。状态：源码与离线验证完成，待集成审阅；完整 PR 仍有基线文档链接失败。
承接[第一批与基线 v2](2026-10-08-motor-state-flow-simplification-v2.md)，
以及[全出口设计](../../architecture/motor_state_flow_simplification_v1.md)。

## 目标与批准的行为差异

用户选择八状态：INIT、IDLE、STARTING、RUNNING、MAINTENANCE、STOPPING、FAULT、FATAL。
BOOT 与 SELF_TEST 合并为 INIT，READY 改名 IDLE；保留独立 STARTING 和 FATAL。
实际关相且资源静默的 INIT STOP 原地确认，不重启自检期限或连续健康窗口。
IDLE 的静默 STOP 也原地确认；仍撤销请求、清目标、保留故障优先和 STOP 序号屏障。
资源未静默则保留 STOPPING 真实归还；CANCEL、CLEAR 和故障不扩大恢复资格。

用户同时决定 INIT 的 CAN 反馈用 3，0 保留退役；协议文档升级 v1.1。
其他状态、模式和 45B 字节布局、参数 schema12、请求／结果 ABI 保持。
不加隐藏 BOOT 阶段、兼容旧头或第二状态机，不为删除状态新增布尔等待变量。

维护与自动 SAVE 仍按旧路径分别经过停止和待命；两者身份／终态和自动保存种类不变。
控制数学、所有启相动态复核、保护阈值、20 kHz／2 kHz 调度、固定资源与 Loader 交接保持。
本轮不访问硬件、不改 CubeMX／链接／工程输入、不烧录或写设备参数。

## 基线、接受和回退

基线为 `c1ca674` 加 v2 冻结的未提交集成功能；已完成的等价删除仍有效。
本批是批准的行为变化，不能要求旧九状态全文摘要不变，也不能重用第一批的等价结论。
首先保存所有待改文件的原字节、Git HEAD 字节及清单到 `outputs/motor-state-flow/eight-baseline/`。
源共享树保持只读；其未提交功能不进入专项提交。

实现只在原拥有者内删 BOOT 完成事件／分支，统一 INIT 自检，并按实际静默确认原地 STOP。
沿用旧 SELF_TEST 内部值 1 和后续值，0 内部值不再为合法状态；CAN 明确由适配器映射。
首次 INIT 规则代际直接为 2，保持原正常 BOOT 完成后的起始任务身份，不新增初始化阶段。
INIT 静默 STOP 仍推进代际使旧事件失效，但不发送 SELF_TEST 重启动作；原来经停止再自检
会推进两次代际，差异随政策批准单独记录。IDLE 正常 STOP 仍推进一次代际。
合并后 INIT 使用首次监督拍开始的单一期限；删除原 BOOT_DONE 在资源晚归还时重置起点的行为。
正常初始化资源已静默时不多加等待；资源晚至期限边界归还也不能延长初始化总期限。

接受：完整原生所有者回归；新增重复 STOP 的 500／1500 ms 期限、4 拍稳定窗口、
未归还资源、不能关相、故障／FATAL 优先、STOP 屏障和八状态线上映射；
当前规则失败矩阵不变；新规则完整摘要按已批准差异冻结；PR profile 与基线失败对照。
协议 v1.1 有兼容说明、黄金状态 payload 向量及真实映射／编码检查。
不宣称 MCU 时延、栈、镜像或实机验证；这些后续独立按当前源码验收。

回退：本批独立提交 revert，恢复九状态与旧 STOP 路径；不 reset／clean 其他工作区。
提交从 HEAD 仅应用本批变化，工作区继续叠加导入功能；相关文件禁止整文件提交他人改动。

## 执行与证据

代码提交：`3d7899bb45e55ae14bf54f6d5c04ff7097019af5`。
旧新 334146 次规则调用逐字段比较：326417 次完全相同，7729 次仅有批准政策投影；
无额外差异。新八状态摘要 `0bf3e5aa3c3ea0e0`，72000 组失败摘要保持 `afc991123a3208fd`。
生产所有者新增稳定窗口、500／1500 ms 重复 STOP、序号屏障及资源未归还测试 PASS。
八状态真实 CAN 映射与 45B 黄金编码 PASS；全部 29 原生组和 integration PASS。

PR 汇总：`outputs/runs/motor-state-eight-candidate-20261008/summary.json`，结果 FAIL；
doctor／format／lint／layout／architecture／interfaces／hygiene PASS；
四处历史输出链接失败与 v2 基线完全相同，Python unit 的同一项文档检查失败，余 58 项 PASS。
验收、哈希、工作区功能保留和补丁交付见[第二批报告](../../reports/2026-10/motor_state_flow_batch2_20261008.md)。
最终补充测试提交 `b6c8dbb`；最终 PR 为 `outputs/runs/motor-state-eight-final-20261008/summary.json`，
结果及基线失败不变。增量补丁已对未修改的共享基线只读检查通过，尚未合并。
