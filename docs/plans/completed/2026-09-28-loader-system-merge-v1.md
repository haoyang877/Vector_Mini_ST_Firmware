> 历史阶段记录，2026-09-29 归档。下文是当时的设计、状态和证据，不代表当前待办或当前版本通过。现状见[框架收尾](2026-09-29-framework-closure-v1.md)与[历史清理](../active/2026-09-29-legacy-cleanup-v1.md)。

# Loader 修复合入 system-refactor v1

日期：2026-09-28。目标：按用户要求将 codex/capacity-fit 合入 codex/system-refactor，
保留目标分支的新电机状态机、功能任务和当前保护行为；不接触硬件。

## 基线与范围

- 来源：ceaaac3e 后新增 Loader 修复提交 5c01e42。
- 目标：f3f1cb83；原未提交流程图先保存为 0b2c12f，原始图可从该提交恢复。
- 来源未提交的 angle_feedback.c 空白改动和 Keil 用户配置保留在来源工作树，不纳入本次提交。
- 合并包含容量分区与寄存器 Flash 的依赖提交，不能只移植修复而遗漏新 CRC32 布局。

## 冲突处理与不变项

保留目标 motor_state_machine.c/.h 及 MotorTaskResult 功能任务模型，不恢复旧
foc_run_state、ModeNow 或模式分派代码。简化启动取消持久 TRIAL/确认及 BootHwInfo；
健康读取迁移为 MotorStateMachine_ReadBootHealth(progress)，监督完成计数仍由原所有者推进。
APP 使用修复后的 Type56 交接与100ms健康喂狗，通信协议和升级核心保持新v2契约。
原配置守卫、维护准入、故障锁存、取消释放、功率关断和控制限值使用目标实现。
来源针对旧状态机的局部合并不能覆盖这些不同的控制/任务路径。

目标原流程图包含已删除的试运行确认阶段，当前设计采用修复版的三个短图；
原图完整保存在上述文档提交与本地合并备份，不将旧 TRIAL 能力登记为当前实现。
生成目标工程时复用目标新的APP源清单，避免缺失重命名模块或留下已删除源码。

## 验收与回退

先验证真实Loader启动、APP喂狗、Flash等待、完整运行所有者与真实维护任务，再运行PR及release档，
离线重建Loader/LoaderApp/普通APP并核对当前map及实际打包镜像。
接受：无冲突标记和旧启动ABI引用；两边提交成为目标祖先；目标状态机与Loader测试通过；
构建零错误/警告、Flash/RAM符合各自分区。完整门禁现有失败单独记录，不包装为发布通过。
不烧录、不复位、不命令电机，不用来源旧产物证明合并后的代码。

回退通过保留合并前目标提交或审阅后的merge revert，不删除工作树、历史或用户实验数据。
生成证据只放outputs。

## 合并后验证

所有检查使用来源工作树已有Python依赖环境、目标工作树的实际源码；不安装新依赖。
真实Loader runtime、喂狗调度、Flash等待与目标完整状态机/实际维护任务的聚焦检查通过。
release档重新执行完整离线检查：25项native全部通过，17项integration通过，Python unit42/43，
唯一失败仍是历史报告的10条outputs断链。architecture、hygiene、工程布局和doctor通过。
format17条、lint601条、interfaces34条既有缺失与来源验证一致，本次修改文件无新增门禁问题。
执行时合并尚未提交，clean-release-source也失败；完整release不是PASS，未由profile发起构建。

独立Keil重建均为0错误/0警告，实际LoaderApp打包及真实runtime镜像模型通过：

| 目标 | ROM | 保守未压缩加载跨度 | 静态RAM含栈堆 |
| --- | --- | --- | --- |
| Loader | 13,520B | 13,520B / 20,480B | 4,952B |
| LoaderApp | 89,964B | 90,456B / 94,208B | 31,616B / 32,736B |
| 普通APP | 88,316B | 88,740B / 114,688B | 31,568B |

当前LoaderApp链接BIN90,004B，8B补齐后90,008B，APP槽余4,200B，普通RAM余1,120B。
这是合并源码的静态占用，未验收最坏栈、实机CAN/ECC/掉电，开发版本0不得视为正式发布。

证据入口：`outputs/runs/loader-system-merge-20260928/summary.json`；
当前构建日志、实际包和`runtime-real-image/test.log`在
`outputs/merges/capacity-into-system-20260928/`。
同目录`summary.json`绑定合并提交、来源/目标祖先、源码及当前产物哈希，并确认原无关改动保留。
