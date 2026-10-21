# 编码器标定调用与边界简化 v3

日期：2026-10-08。实现、直接比较与干净实现提交的 PR 全通过，待 ROOT 审查/集成。
基点 `b6f08c5b115199ad1e73a77db62c65afe3d761f8`；集成现场四个本模块文件的 LF 内容一致。
沿用 v1/v2 继承快照，不修改集成现场或共享 Keil 项目。
计划见[调用与边界简化](../../archive/plans/2026-10-08-encoder-calibration-call-boundaries-v3.md)。

## 实际调用关系

下文 Core_ 代表 EncoderCalibrationCore_；同一行的箭头是跨拍阶段顺序，非核心函数互调。

```text
MotorMaintenance_Step
├─ StepLut
│  ├─ 基本输入检查 → prepare_samples → 启动/电流/速度控制器复位
│  ├─ SensorlessStartup_Run（停止卸流阶段改调 FOC_Current）
│  ├─ 找原点至验证：共用 is_tracking 检查
│  ├─ 对齐：选择首锚点 → Core_AddAngleSample → Core_MeanQ15
│  ├─ 找原点：Core_Q15Difference → 过零插值
│  ├─ 采样 → 构表 → 验证：Core_SampleLut / BuildLutStep / VerifyLut / ValidateLut
│  ├─ 合格：commit_candidate_lut → Core_ApplyLut → 零位失效/rebase/重置测速
│  └─ 停止：can_brake / get_stop_speed → finish → reset → COMPLETED
└─ StepElectricalZero
   ├─ 独立前置检查 → FOC_Current（固定相位，升流/保持）
   ├─ Core_AddAngleSample → Core_MeanQ15 → Encoder_SetElectricalZeroQ15
   └─ 清 id/iq、复位电流 PI → COMPLETED/FAILED

Core_SampleLut → add_positive_travel、Core_Q15Difference
Core_VerifyLut → add_positive_travel、Core_ApplyLut、Core_Q15Difference
Core_BuildLutStep → 一套8-bin循环，四遍仍分别处理覆盖/均值/平移/范围
失败 → fail（MotorFault_Report）→ reset；已有故障只 reset，不重复上报
取消 → Cancel（清引用/阶段/零位计数）；实际关相/归还/保存由 APP 所有者衔接
```

## 边界检查职责

| 位置 | 检查与处理 | 整理方式 |
| --- | --- | --- |
| APP 准入 | 配置有效、母线、安全许可、编码器/LUT 前置 | 保留，保护使能前入口；不能替代运行中检查 |
| LUT 任务每拍 | 借用地址不变、在线、极对数、R/L/磁链、电流限值及标定最小电流 | 保留优先级；同 INVALID_PARAMETERS 的两分支合并 |
| 找原点/采样/构表/验证 | 无感闭环且 position_epoch 不变，否则 SENSORLESS | 四处重复拒绝合为一处，仍在控制器运行后复核 |
| 稳速阶段 | epoch 改变失败；未闭环/速度不稳定只重置稳定计数，并保留超时 | 独立保留，不能套用测量阶段立即失败规则 |
| 核心采样 | stable 才计正行程；relative≥0 才采样；count 未饱和才累加 | 不合并不同条件；无效帧仍更新 previous |
| 四遍构表 | 每拍8 bin、每 bin≥16样本、连续解卷绕、平移后 int16 范围 | 同一循环推进；顺序、失败游标和全部检查不变 |
| 核心验证/任务结束 | 非空、峰值≤1024、RMS≤256；任务另判断1机械圈与超时 | 保留整数比较和原阈值，通过后才发布 |
| Mode15 | 在线/LUT有效/对齐电流；保持窗口且 READ_OK 才采样；空均值失败 | 保留独立规则，不强行合并两个模式的输入检查 |
| 停止阶段 | 闭环/epoch、观测器速度/相位、独立编码器速度，减速/卸流计时 | 不纳入测量守卫；已发布 LUT 后失去制动条件仍零指令完成 |

核心有效指针、独立状态、清零起始、正 required_theta 与8/16参数由任务契约保证，
不在每个纯计算函数重复添加通用验证。四遍变成一个循环，不改变每拍预算或丢掉校验遍。

## 删除与复用

- 原点和零位各自累加接口合为 AddAngleSample：锚点选择归任务，纯整数累加共用。
  半圈/跨零的两种旧差值形式等价，经262144组环形差及累加比较核对；未改浮点公式。
- 四遍重复循环合并；两处相同错误码的参数分支、两段停止阶段分派和重复 stop_speed 求值合并。
- 四个测量阶段的相同跟踪拒绝统一；不删 SPEED_STABLE 的恢复窗口或停止段独立截止。
- 一行置空引用的 release_samples 包装删除，四个真实调用点直接清借用引用；
  APP 的实际工作区归还仍保留。该包装此前已被 ARM 编译器内联，不计额外空间收益。
- 新增框架/配置/状态/回调为零。核心公开计算接口9→8；task public API、Watch和状态/表布局不变。

## 当前直接证据

独立核心与18类失败/准入、1个稳速恢复窗口及完成/取消/工作区检查 PASS；两配置均与本基点完整 task/core
逐拍归约一致：profile0 LUT34320拍/零位30000拍，hash `58d6cc0c6e6e1ca2`；
profile1 LUT36367拍/零位30000拍，hash `29dd13f990dad1d0`；zero_q15均为1。
profile1 用于确认正最小电流拒绝条件，采样密度按既有圈数维持覆盖；不构建多工程矩阵。
仍保留宿主 float-cast-overflow 检查限制，不宣称 ARM/实机数值等价。

同 ARMCC5.06u7 build960、Cortex-M4.fp.sp、C99、O3/Ospace等参数仅编译两个真实对象，
命令、源哈希和对象哈希写入 outputs；没有链接或触碰共享项目。

| 对象指标，B | 基点 | 当前 | 差值 |
| --- | ---: | ---: | ---: |
| Core Code+RO | 988 | 854 | -134 |
| Task Code+RO | 2890 | 2866 | -24 |
| 两对象合计 Code+RO | 3878 | 3720 | -158 |
| 两对象合计 RW+ZI | 128 | 128 | 0 |

对象净减158 B（约4.1%）不等于整包 Flash 净收益；静态数据不变不等于动态栈不变。
本轮没有整包链接、ISR/动态栈或硬件/物理收尾验证。共享输入/接线仍由 ROOT 集成，
旧32 B不相加。证据目录为 `outputs/encoder-calibration-v3/`，
对象数据为 `object-sizes.json`，编译参数为 `arm-compile-config.json`。

已验证实现 head：`49417eecf73755f66a668cdb2d4ccc13e2a75bc8`；最终报告提交 HEAD 在交付消息提供。
PR 证据：`outputs/runs/encoder-calibration-boundaries-pr-v3/summary.json`（59 unit、22 integration、全部原生/架构/接口/文档/卫生 PASS）。
四个生产文件含契约行数1296→1216（-80）；该源码统计与对象158 B分开记录。
本轮没有新的接线变更，沿用 v2 的新增 core.c 接线补丁。

| 实际实现源码 | 原字节 SHA256 |
| --- | --- |
| `firmware/motor/calibration/encoder_calibration_core.c` | `8ce93fe8125f45b621c359cc3c344b136fd53af710b53176cc0ae0c48444ad18` |
| `firmware/motor/calibration/encoder_calibration_core.h` | `1bda30269e50a3a00580d1d2d1d8d01dc80552bd2ed78cc1d2cfabc2e057092d` |
| `firmware/motor/calibration/encoder_calibration_task.c` | `886fb25c2490fb6f6ea2f7de1c2920b7937b4427b1be16815294956f72a41b05` |
| `firmware/motor/calibration/encoder_calibration_task.h` | `069dc6c6bc29062d8e7e48e029646e54e87880d81b8a68489165469f3b320b93` |

机器基点/源码/对象/证据记录为 `outputs/encoder-calibration-v3/evidence.json`；
直接比较日志为 `equivalence-final-0/test.log`、`equivalence-final-1/test.log`（同目录）。
报告补充只运行文档与 harness 检查，不重复原生矩阵或对象编译；原现场全部保留。
回退仅撤销本轮基点之后的提交。实际链接容量、ISR/动态栈、实物输出及保存仍待明确范围验收。
