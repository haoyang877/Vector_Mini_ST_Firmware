# 主状态机可读性优化验收（2026-09-29）

行为基线 e7c0f75d，代码提交 1e3e9c8。实现与离线验收完成。
本记录描述结构调整和资源代价，不代表已烧录或通过实机运动验收。

## 结构与阅读入口

- [转换规则](../../../firmware/app/motor_state_machine/motor_state_rules.c)：
  MotorStateRules_Step 依次处理参数、公共优先级、普通状态事件和统一输出约束。
  handle_state_event 按九个主状态组织，入口、守卫和出口集中在对应分支。
- [监督所有者](../../../firmware/app/motor_state_machine/motor_state_machine.c)：
  MotorStateMachine_Tick 展示有序阶段，工作由同文件私有函数完成；不新增公开 API。
- 单一上下文、协议、参数 schema12、维护身份和共享镜像契约保持。
  事实采集改名 state_read_facts，短变量使用 context/event/facts 等明确名称。
- 不使用 goto；模块边界测试约束生产 C/H 文件，防止跳转逻辑重新进入。

| 主干函数体（含花括号） | 基线行数 | 当前行数 |
| --- | ---: | ---: |
| MotorStateRules_Step | 336 | 24 |
| MotorStateMachine_Tick | 86 | 19 |

行数表示主干的阅读长度，完整工作仍由状态分支和私有阶段完成；不是删除安全条件的数量。
STOP 消费后仍检查同拍故障，取消归还仍要求实际资源释放，动作后重新读取事实。
完成与新请求同拍不能串成自动启动，最终启相保留原短临界区复核。

## 当前验证

基线与最终 release、收尾 PR 的 format、lint、工程布局、原生/Python 测试、
架构、接口、文档、仓库卫生检查通过；基线/最终 Keil 编译均零错误和零警告。

- 原有 262144 组规则摘要保持 f98662bc6ed8b902。
- 使用基线原生产规则和当前规则直接差分：294912 组独立输入，覆盖全部 15 位事件
  组合及九个状态；1048576 次连续转换，连续序列单独确认覆盖全部九个状态。
- 比较 state、epoch、具体维护 kind、活动标志、结果、效果、首故障、锁存、自检事实、
  动作位和拒绝原因；全部一致。生成夹具、原规则源码和日志保留在 outputs。
- 基线与最终真实所有者/叶任务测试使用同夹具，输出逐字节一致，覆盖取消、借用、
  派生保存、过期结果、同拍故障、自检期限及功率启停失败。
- 当前 LoaderApp 打包镜像通过真实 Loader 组合代码的内存模型测试：冷启动、ENTER、
  看门狗救援、损坏镜像、断线、APP 本地健康和有界 Flash 等待。

复用固定版本开发虚拟环境调用 tools/run.py；doctor 核对 Python 3.13、Zig 0.16.0、
格式/lint 依赖和 Keil 5.36。相关汇总：

```text
outputs/runs/state-readability-baseline-20260929/summary.json
outputs/runs/state-readability-accepted-20260929/summary.json
outputs/runs/state-readability-pr-final-20260929/summary.json
outputs/state-readability/summary.json
outputs/state-readability/rules_difference.log
outputs/state-readability/real-image-runtime/test.log
```

本轮验收没有失败或未解释的功能差分。普通 APP AXF/HEX/MAP 与当前 release 哈希核对，
LoaderApp 与基线产物哈希纳入独立汇总；源码调整使布局、调用指令和产物哈希变化。

## 容量与栈

Flash 按 Code + RO + RW 计算，Code 中的内嵌数据已包含，不能再加一次。
模块统计包含 motor_state_machine.o 和 motor_state_rules.o，不包含所调用的算法。

| 统计对象 | 基线 Flash/B | 当前 Flash/B | 增量/B | 当前静态 RAM/B |
| --- | ---: | ---: | ---: | ---: |
| 普通 APP 状态机模块 | 7790 | 8054 | +264 | 192 |
| LoaderApp 状态机模块 | 8018 | 8282 | +264 | 192 |
| 普通 APP 整体静态 ROM | 90652 | 90924 | +272 | 31600 |
| LoaderApp 整体静态 ROM | 92272 | 92536 | +264 | 31640 |

结构化分支与阶段函数增加少量指令；普通 APP 另有 8 B 布局/常量对齐差异。
所有静态 RAM 不变。LoaderApp 打包镜像 92240 B，94208 B 槽内余 1968 B。
打包元数据仍为 integrity_only、release_approved=false；内存组合验证单独记录，
没有将开发镜像改称正式发布镜像。

当前链接器静态报告最大已知调用栈为 548 B，加未知函数/循环/间接调用路径。
规则入口本地栈 32 B，监督入口本地栈 16 B；这些数字不能相加作为完整 ISR 栈峰值。
当前调用图 HTM 已与 AXF 一起保存。调度频率、原动作顺序和临界区复核保留，
实际中断时延、抖动、栈高水位与物理输出关断均未测量；本轮未接触硬件。

## 回退与收尾

按[执行计划](../../plans/completed/2026-09-29-motor-state-readability-v1.md)分阶段提交。
代码变更可 revert 回 e7c0f75d；没有设备回滚步骤。
合入 codex/system-refactor 后保存本轮基线、最终产物和汇总，再归档临时 worktree。
主线 Keil 用户界面偏好和原工作区电机实验修改保留。
