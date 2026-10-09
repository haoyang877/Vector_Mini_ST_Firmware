# 编码器标定最小实现 v4

日期：2026-10-08。基点 `e46a8cca09a364c8b3c8506bed8f55cf12f9becf`；干净实现提交 PR 全通过，待 ROOT 审查/集成。
目标与回退见[最小实现计划](../../plans/active/2026-10-08-encoder-calibration-minimal-v4.md)。
四个功能保持：LUT 标定及验证、电零位平均、安全退出/收尾、复用现有后台保存。

实现仍是一个任务流程和一个纯计算核心，没有新增接口、状态机、框架、配置或依赖。
保留所有时间/阈值/边界、20k/2kHz、每拍8 bin、四遍、候选验证后发布、零位失效/rebase、
取消/错误/卸流和主状态机关相/后台 SAVE；task API、现用 Watch 与6144 B工作区布局不变。

| 本轮简化 | 保留的约束 |
| --- | --- |
| 删除第二份 sample_window，采样复用 verify_state.window | 两阶段互斥；验证开始仍整体清零并设置当前previous，不继承采样行程 |
| begin_sample 统一稳定准入、正行程和 relative≥0 | 负relative/NaN不产生误差样本；合法正行程仍独立计入；previous最后更新 |
| 环形差使用 `(int16_t)(uint16_t)(target-source)` | 同16位周期，半圈仍-32768；对冻结旧函数按差值全集和边界比较 |

任务持有唯一窗口、构表游标和残差状态；核心仅通过显式参数写计算结果，无可变全局/I/O。
原点/零位的独立锚点、累加量和观测符号保留；不为了复用而混用不同采样基准。
保存和真实关断复用 APP，不在本模块另造实现。主调用关系仍见
[v3调用与边界表](encoder_calibration_call_boundaries_20261008.md)，本轮只收敛核心采样准备与状态。

直接证据：现有纯核心/18类失败/1个恢复窗口/取消/工作区检查及基点完整任务比较。
旧环形差函数从冻结源码提取为离线oracle，对262144个角对核对（涵盖差值-65535..65535）。
增加NaN relative边界检查；完整任务归约一致同时验证采样行程不能提前结束新验证窗口。
宿主float-cast检查限制保留，不宣称ARM/实机数值等价。

同ARMCC5.06u7 build960、Cortex-M4.fp.sp/C99/O3/Ospace及相同defines/includes只编译真实对象：

| 当前对象指标，B | 基点 | 当前 | 差值 |
| --- | ---: | ---: | ---: |
| Core Code+RO | 854 | 824 | -30 |
| Task Code+RO | 2866 | 2826 | -40 |
| 两对象 Code+RO | 3720 | 3650 | -70 |
| 两对象 RW+ZI | 128 | 120 | -8 |

命令/源码/对象/日志归档于 `outputs/encoder-calibration-v4/`，没有共享工程修改或整包链接。
不能从对象收益直接扣正式LoaderApp超限，也不将旧批收益相加作为组合容量。
冻结发布候选未改，实际组合容量/动态栈/ISR/实物关断和Flash保存未测，无硬件/烧录/复位/运动。
待 ROOT 审查本基点后的提交；不自动集成或扩大范围。

已验证实现 head `9c3575ed54a0419f478c4326d7265704f10a0daf`；最终报告提交 HEAD 在交付消息提供。
PR 证据 `outputs/runs/encoder-calibration-minimal-pr-v4/summary.json`：59 unit、22 integration、全部原生/架构/接口/文档/卫生 PASS。
两配置完整轨迹分别为34320/36367个LUT拍与30000个零位拍，zero=1，
hash `58d6cc0c6e6e1ca2` / `29dd13f990dad1d0` 与基点一致。
四生产文件含契约行数1216→1198（-18），对象与源码统计分开。

集成现场四文件匹配更早 b6f08c5b，而本独立基点 e46a8cca 包含已接纳待集成的 v3；
因此保留 v3 源码，没有从现场回拷旧实现或纳入其他 dirty 修改。此前继承提交仍只作基线材料。

| 本轮实际源码 | 原字节 SHA256 |
| --- | --- |
| `firmware\motor\calibration\encoder_calibration_core.c` | `92f4fbb2f1eb1220fe695c87067d4e67d2ae550b942c37bf2aac2e641e623482` |
| `firmware\motor\calibration\encoder_calibration_core.h` | `d4a6fa5b8e0876af3989f3d3859ad36dcde3a7ebe7cbea19a69714c8352996c1` |
| `firmware\motor\calibration\encoder_calibration_task.c` | `7a8af442e1c71bfee64159026cd12b1bbd5430823a217afaa0e2a2206ccbe2d4` |
| `firmware\motor\calibration\encoder_calibration_task.h` | `069dc6c6bc29062d8e7e48e029646e54e87880d81b8a68489165469f3b320b93` |

`evidence.json`（同证据目录）绑定基点、源码、对象、编译参数和各日志 SHA。
目标对象反汇编归档，环形差短式的ARM输出可独立审查；未在ARM硬件执行测试。
报告补充只跑文档/harness，不重复编译或原生矩阵。下一候选由ROOT统一组合链接，
本轮不调整算法/物理参数，不删除质量检查，不修改认证或发布候选。
