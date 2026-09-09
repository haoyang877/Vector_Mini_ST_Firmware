# Pitch mode 3 台架验证记录（2026-09-09）

> 本文保留早期试验过程。用户随后将实际限位修正为 **−0.3～+0.9 rad**，并授权短时相电流提高到 6～8 A、最多 5 s；原 −9°～+30°和 4 A 主机停止条件已经被后续试验替代。当前配置、保护与结果见 [最新续测记录](pitch_mode3_burst_20260909.md)。以下“当前”均指早期记录当时。

## 当前结论

RTT 恢复正常采集后，已完成小角度参数对照和机械范围修正。修正后 ±3°及 −4°～+12.5°运动全部满足末段 ±0.3°和持续 HOLD 要求；正向扩大到 +25°时，两轮分别采用 45°/s和15°/s巡航上限，都在约 +18°触发 4 A 主机停止阈值。当前关闭使能，等待核查正向机械阻力；未将候选增益写入 Flash。

早期用户确认存在结构限位，实际允许范围应为 **−9°～+30°**。已经改写轴 Flash 记录并复位回读验证，后续测试目标收窄到 **−4°～+25°**。原 −12.5°目标超出真实机械范围，该段作废，不作为继续加大增益或电流的依据。下面保留原始过程，修正后结果单独补记。

当前属于调试中间结果，不是 pitch 全范围验收。结构抖动仍需现场反馈或结构侧测量确认，电机端编码器误差不能替代此项。

## 电机与数据隔离

- 用户确认更换的是整套 pitch 电机和驱动硬件，型号与 roll 相同；使用 pitch 自身校准数据。
- 用户已手工校正零位。回读校准标志 7、电角度零位 43804、机械零位 36688，参数 schema 10。
- Flash 当前保存轴名称 `pitch`、范围 −9°～+30°、最高巡航速度 45°/s，即 8 s/rev。修正后轴记录 CRC32 为 2613564763；旧 ±30°记录的 CRC 为 4192660675，已被替换。
- 修正后台架命令范围为 −4°～+25°，主机位置停止边界为 −7°～+28°。物理结构限位不能作为正常制动点。
- 使用独立会话 `outputs/pitch_validation_20260909/session`；不得使用 roll 会话的符号或参数备份。
- `tests/mode3/pitch.json` 是 Flash 中原始控制参数基线；`tests/mode3/pitch_candidate_20260909.json` 是未持久化的 RAM 调试候选。
- pitch 的 NTC 装配情况未知，不能沿用 roll 未装配的结论；本轮不是温升验收。

## 参数对照

所有角度均为电机端机械角度。位置精度验收：每段最后 2 s 的最大绝对误差 ≤0.3°，HOLD 占比 100%；同时检查记录完整、停止已确认、无丢帧标志、无电流饱和、实际 Iq 小于 4 A。

| 项目 | Flash 基线 | 最后 RAM 候选 |
|---|---:|---:|
| cascade_pos_Kp | 0.05 | 8 |
| cascade_pos_Kd | 0.5 | 2 |
| speed_Kp | 0.05 | 0.5 |
| speed_Ki | 0.5 | 2 |
| current_limit | 6 A | 6 A（未修改） |
| 巡航速度上限 | 45°/s | 小/中范围验证 45°/s，最后诊断轮 RAM 为 15°/s |
| 加速度 | 45°/s² | 45°/s² |
| Flash 减速度 | 45°/s² | 45°/s² |
| mode 3 有效减速度上限 | 30°/s² | 30°/s² |
| jerk 时间参数 | 0.2 s | 0.2 s |

速度 PI 在内核中乘以 6 A 电流上限换算到电流域。摩擦模型有效标志为 false，目前仍使用既有默认库仑补偿 1.55/1.50 A，尚未辨识 pitch 的正反向、位置相关负载；不能把该默认值当作新电机的辨识结果。

保持进入/退出阈值仍为 0.18°/0.26°，没有为通过测试放宽验收容差。五次速度 S 曲线、前馈卸载速率和保护阈值均未改变。

## 实机过程

原始数据均位于独立会话中，每轮保留 `trial.json`、`capture.tsv`、`summary.json`，完整可分析的试验另有 `mode3_acceptance.json`。

| 试验目录 | 主要变化/动作 | 结果 |
|---|---|---|
| pitch01_hold | 发现 RTT 尚未就绪 | ARM 前终止，确认关闭输出；增加有超时的 RTT 就绪等待 |
| pitch02_hold | 原参数静止保持 | 误差约 0.0275°，但采集率仅约 1.33 kHz，数据链路需排查 |
| pitch03_one_degree_baseline | 初次 ±1° | RTT 读取中断，普通 STOP 未确认；紧急关闭 PWM 并 halt，后续复位核查；此轮不计通过 |
| pitch04_one_degree_baseline | 用户断开其他 RTT 后，原参数 ±1° | 采集恢复约 2 kHz；负向超调约 1.13°，多段未持续 HOLD |
| pitch05_velocity_p05 | 仅 speed_Kp 提高至 0.5 | 负向超调约 0.35°，仍有 HOLD 不通过 |
| pitch06_position_p2 | position Kp=2 | 保持没有改善；187 次 IRQ 耗时达到/超过 50 μs，最大 50.41 μs |
| pitch07_cached_p2 | 配置常量计算优化，同参数复测 | 最大 IRQ 49.35 μs，无 ≥50 μs 样本；HOLD 仍不通过 |
| pitch08_position_p8 | position Kp=8 | 4 段中 3 段末段持续 HOLD |
| pitch09_damping2 | Kd=2 | 负向超调约 0.154°，保持仍有延迟 |
| pitch10_integral1 | speed Ki=1 | 负向超调约 0.093°；一次回零末段 HOLD 约 93% |
| pitch11_integral15 | speed Ki=1.5，±1° | 全部通过；末段误差最大约 0.220°，各段 HOLD 100%，无越过目标，峰值 Iq 2.939 A |
| pitch12_small5 | 同参数 ±5° | 位置误差通过，首段末段 HOLD 约 88%，因此整体不通过 |
| pitch13_small5_i2 | speed Ki=2，±5° | 全部通过一次；末段误差最大约 0.254°，HOLD 100%，无越过目标，峰值 Iq 3.023 A |
| pitch14_medium125 | 同参数 ±12.5°，含计划中的换向 | +12.5°及回零已完成；随后负向运动触发 4 A 停止，余下动作未执行 |
| pitch15_corrected_small3 | 修正范围后，0,+3,0,−3,0，每点6 s | 全部通过；最大末段误差 0.259°，HOLD 100%，峰值 Iq 2.603 A |
| pitch16_corrected_medium | +12.5,0,−4,+12.5,0，每点7 s | 全部通过；最大末段误差 0.250°，HOLD 100%，峰值 Iq 3.726 A |
| pitch17_corrected_range | 计划 +25,−4,0,+3,−3,0，每点10 s | 第一段在约 +18.26°停机，主机采样 Iq=4.0499 A，未执行后续动作 |
| pitch18_range_speed15 | 仅将 RAM 巡航上限降为15°/s，先回零再+25° | 回零完成；向+25°运动时约+18.09°再停机，主机采样 Iq=4.0116 A；后续动作未执行 |

`pitch14` 的主机停机采样 Iq 为 −4.0026 A，RTT 全段峰值为 4.093 A。轮询停止存在采样间隔，因此 4 A 是停止触发阈值，不是硬件瞬时限流。电机固件未报告故障，不能把此次主机主动停止称为板载过流故障。

停止前正向末段仍需约 1.8 A 积分支撑，负向运动中叠加了既有摩擦前馈和反馈电流。用户随后确认负向结构限位为 −9°，因此此次目标超出了结构范围。保留该异常记录，不把它解释成正常行程内需要更大控制力；没有提高 4 A 主机停止阈值。

修正后两轮小/中范围试验没有观察到越过目标，均无丢帧/电流饱和标志。正向 +18°附近两次停机不能仅解释成巡航速度过快；尚需核查断使能后是否可以顺畅手动转到 +30°。台架不能测定具体接触位置或力矩来源。没有提高电流停止阈值。

主机时间来自接收帧锚点，有批处理延迟；表中时间和超调是本轮记录的观测，不是硬件精密时间戳或可重复性保证。软件设置的45°/s是巡航上限，短行程并不一定达到这个速度。候选 Ki=2 尚未完成 ±1°、微动、长时间保持和全范围重复回归。

## 固件计算优化与验证

在 `Foc/position_cascade.c` 的私有状态中缓存保持确认周期数、静摩擦等待周期数和速度滤波系数。配置完整校验通过后更新这些量，避免每个 2 kHz 周期重复计算；目标变化和非法配置仍按原逻辑校验。

这项修改没有加入 HAL 依赖、跨模块全局变量或 ISR 内分配/阻塞。新增动态滤波频率/周期变更及 50 ms 保持确认时间回归；原有 72 组模拟工况、轨迹、保持、捕获、编码器、轴配置和 HIL 命令测试通过。普通/HIL 两个 Keil target 均 0 error、0 warning。

构建和测试证据：

- `outputs/pitch_validation_20260909/host_final/`
- `outputs/pitch_validation_20260909/build_cached_normal.log`
- `outputs/pitch_validation_20260909/build_cached_hil.log`
- `session/image_pitch_cached_constants/verification.json`

下载回读验证了 102672 个镜像字节，参数及校准区逐字节保持不变。优化后试验中未再观察到 ≥50 μs 的 IRQ 样本，但部分峰值仍接近周期上限，不能据此宣称实时余量充分。

## 后续执行

机械范围已由用户纠正，并持久化、复位验证。候选增益和最后15°/s诊断巡航仅在 RAM 中，复位会恢复 Flash 基线。恢复前先核查 +18°～+30°机械阻力，再决定是否辨识位置相关负载；任何停止或不通过均不得计入全范围通过。

范围持久化证据：`outputs/pitch_validation_20260909/pitch_corrected_envelope/verified.json`。仅轴记录字节发生变化，固件与其他参数/校准字节保持不变。

离线重新分析（不会驱动电机）：

```powershell
python tools/run_mode3_validation.py analyze --profile outputs/pitch_validation_20260909/pitch_candidate_i2.json --trial outputs/pitch_validation_20260909/session/pitch13_small5_i2 --case small
python tools/run_mode3_validation.py analyze --profile outputs/pitch_validation_20260909/pitch_candidate_i2.json --trial outputs/pitch_validation_20260909/session/pitch14_medium125 --case medium
```

第一条应返回通过，第二条应返回失败。这两条只使用归档的旧范围 profile 复核历史数据，该旧 profile 不再允许用于实机运动。后续实机测试必须使用已修正的 `tests/mode3/pitch_candidate_20260909.json`，并显式使用 `--session-dir outputs/pitch_validation_20260909/session`，不能沿用 roll 的默认会话。

修正范围后可离线复核 `pitch15_corrected_small3`、`pitch16_corrected_medium`，使用当前候选 profile，不指定旧的对称 `--case medium`。低速诊断的独立 profile 为 `outputs/pitch_validation_20260909/pitch_candidate_speed15.json`；新增可选 `motion.test_cruise_deg_s` 只指定本轮 RAM 巡航值，`motion.cruise_deg_s` 仍校验 Flash 轴速度上限。分析器会严格比对对应 RAM 回读值，不能以45°/s记录冒充15°/s对照。
