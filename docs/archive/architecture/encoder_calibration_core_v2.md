> 历史归档（2026-10-21）：任务与计算职责合并到核心维护说明和当前标定指南。 本文的“当前”指原始基线；现行入口见[当前说明](../../architecture/motor_maintenance_core_v3.md)。归档不新增执行授权。

# 编码器标定核心/外围数据流 v2

更新：2026-10-09。v1.3.0当前生产入口是 `MotorMaintenance_Step` →
`EncoderCalibrationTask_StepLut/StepElectricalZero`；核心为 `encoder_calibration_core.c/.h`。

本页保留 v2 初次提取的职责基线；当前 v4 进一步合并原点/零位累加接口，
采样与验证互斥复用 `verify_state.window`，验证开始完整清零。
当前调用/边界见[v3报告](../../reports/2026-10/encoder_calibration_call_boundaries_20261008.md)，
最小实现和不变约束见[v4报告](../../reports/2026-10/encoder_calibration_minimal_20261008.md)。
维护准入、关相、工作区及保存职责见[核心维护模块](../../architecture/motor_maintenance_core_v3.md)。

| 输入 → 核心计算 → 结果 | 状态与唯一写入所有者 | 外围任务保留的职责 |
| --- | --- | --- |
| directed Q15 → 原点解卷绕累加/均值 → origin Q15 | 原点 anchor/sum/count 由 20 kHz 任务持有，通过核心显式指针更新 | 静态对齐时窗、无感启动、过零查找及原点应用 |
| observer/relative/required 电气 rad、directed Q15、stable 事实 → 正行程/单 bin 误差累积 | `sample_window` 与借用的 6144 B 工作区由 20 kHz 任务持有 | 闭环/epoch 跟踪和速度稳定准入、机械圈数与超时 |
| error_sum/count、每拍 8 bin、覆盖16 → 四遍覆盖/均值解卷绕/平移/候选 → RUNNING/COMPLETE/INVALID | `build_state` 由任务持有，核心逐批推进，不拥有运动阶段 | 跟踪检查、失败映射、完成后启动验证；无第二套电机阶段机 |
| 候选、同一帧事实 → 残差平方/峰值/计数；RMS256/peak1024 → 合格布尔值 | `verify_state` 由 20 kHz 任务持有，核心归约 | 1 机械圈/超时判定；通过才发布，不合格仍报原故障 |
| linearized Q15、启动 anchor → 原 int16 环形解卷绕/均值 → 电零位 Q15 | `s_zero_sample_anchor/count/unwrapped_sum` 保留原符号，由任务持有 | 0.5 s 斜坡+1 s 保持、READ_OK、限流、结果 setter 和控制复位 |

核心 include 仅自身头、标准 `stdbool/stdint/limits`；没有 FOC、MotorControlState、
编码器运行对象、平台 API、全局可变状态、调度、Flash 或功率动作。任务传入明确的
`EncoderCalibrationSample`，核心只消费其 stable 事实，不决定准入。
开始/验证阶段对计算状态置零归任务；核心写入均发生在同一个任务调用上下文，不保留借用指针。

任务验证通过 → 复制 LUT → 当前角插值 → 清电/机械零位及有效位 → 置线性化有效 →
请求慢估计 rebase/重启测速 → 清采样引用 → 减速/卸流 → 返回完成。
该顺序及低速截止、故障先上报再清理、已锁存故障不重复上报、取消幂等均不变。
APP 确认释放才归还工作区；主状态机负责真实关相，后台负责独立 SAVE。
20 kHz/2 kHz 调度、参数/线路 ABI 和 Workspace 布局不变，边界静态核对 Q15 与表尺寸。

保留现用 Keil Watch 项目、公开阶段/getter、原点标量、`s_zero_*` 和任务时间符号。
无生产/登记诊断调用的构表、残差及行程内部标量改为 `build_state`、`verify_state`、
`sample_window` 的具名成员，不保留重复缓存；不是删除其计算状态。

原任务中的环形差、隐式工作区候选插值、采样/四遍构表/残差数学、原点/零位均值移除；
生产只走新核心。两处完全同契约的正行程加法由核心文件私有 helper 复用。
保留反馈层私有插值、全部保护/边界和历史阶段编号，没有退出任何有效功能。
精确删除证据、源码 SHA 和接线清单见[报告](../../reports/2026-10/encoder_calibration_core_20261008.md)。
