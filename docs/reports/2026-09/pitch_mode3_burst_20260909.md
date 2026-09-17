# Pitch mode 3：弧度限位及短时相电流续测

> 后续用户将短时保护改为 30 s，最新重复测试见 [30 秒保护及排查记录](pitch_mode3_repeat_20260909.md)。本文中的 4.5 s 指当时的试验条件。

日期：2026-09-09。承接 [早期记录](pitch_mode3_validation_20260909.md)。本次使用独立 pitch 电机、驱动和校准，不使用 roll 参数备份。

## 本轮结论

最新轴范围已保存并复位核验。带约 5°余量的大跨度双向运动、延长到每段 15 s 的保持均通过自动验收；最后 ±3°运动的位置及保持检查通过。6 A 相电流触发阈值已足够完成这些运动，未使用 8 A。

**尚未整体验收通过**：微小角度的一轮曾在 HOLD 后偏离 0.659°，原参数重试未复现；最后小角度试验仍有 11 次控制中断 ≥50 μs，最坏 8540 cycles（50.24 μs）。下一步应分别定位保持电流与静摩擦的关系、分段测量最坏 IRQ 路径，不能通过只挑成功试验或放宽阈值宣称解决。

结束时回读确认 mode 0、error 0、三相 PWM 输出关闭，位置约 −0.203°。候选增益仍在 RAM；Flash 只保存已确认的轴信息和原有基线增益、pitch 自身校准。本轮未执行 Git 提交。

## 当前边界和电流口径

- 用户最新确认机械范围：向下 −0.3 rad，向上 +0.9 rad，即约 −17.1887°～+51.5662°。AXS1 已写入 Flash，复位回读通过；最高巡航保持 45°/s，即 8 s/rev。
- 台架目标保留 5°余量，允许 −12.1887°～+46.5662°；实际扩大范围试验使用 −12°～+46°。主机停止边界约 −15.1887°～+49.5662°；固件仍检查实际位置和目标的 AXS1 范围。
- 旧工具的 4 A 是 **FOC 实测 Iq 主机停止阈值**，不是母线电流，也不是固件 `current_limit=6 A`。幅值不变 Clarke/Park 变换下，Id≈0 且三相平衡时，Iq 对应相电流正弦幅值，不能直接当相电流 RMS。代码中的 Ibus 由调制度及 Id/Iq 计算，并非本次独立母线电流测量。
- 用户允许 6～8 A 相电流最多 5 s，本轮选择 **6 A 瞬时相电流上限**，未使用 8 A。固件已有 Iq 指令上限仍为 6 A。

## 板端调试保护

`ServoHil_ObservePhaseCurrents()` 每个 20 kHz 电流采样周期检查 Ia/Ib/Ic，取三相绝对值最大值。任一相达到 6 A 或采样非有限数，锁存台架停止。任一相超过 4 A 的采样时间在一次 ARM 内累计，达到 **4.5 s** 停止，短暂回落不清零。时间核算和诊断发布在交错的 10 kHz 命令轮询完成；异常经既有模式停止入口处理，不新增独立故障管理器。

下一次 ARM 才清零本次预算。这个计数不是热模型，重新 ARM 不代表电机已冷却，也没有证明 4 A 可以无限持续。保护仅存在于 **HIL 调试镜像**，普通镜像没有这项短时保护。候选控制增益仍只写 RAM，未作为量产设置保存。

主机须显式传入 `--phase-burst`，并验证板上保护契约的 magic、6 A、4 A、4.5 s；缺失或不匹配拒绝提高 Iq 停止阈值。每轮记录相电流峰值、累计超限时间、保护状态及最终关断验证。ARM、运行、STOP 三阶段中断耗时均必须小于 8500 CPU cycles（170 MHz 下为 50 μs）。

## 本轮修正

1. 首轮使能触发异常采样：Ia≈0.027 A、Ib≈10.61 A、Ic≈27.00 A，板端保护立即停止。检查发现停用状态原先预装 100% 占空比，改为三相相同的 50% 中性预装，仍保持输出关闭。随后的 ARM-only 检查相电流峰值约 0.672 A，未再出现该首样异常。根因判断基于代码时序和修改后的对照结果，尚未用示波器独立确认 ADC/PWM 波形。
2. 配置派生常量只在参数变化并验证通过后计算；保留动态配置、每次调用的目标及反馈检查。
3. 相电流每次采样都检查，时间核算和诊断镜像发布移到与位置环交错的命令轮询周期。
4. mode 3 的首次控制器初始化与 PWM 使能分到相邻两个 50 μs 周期；中间若停止或出现错误，不执行延后的使能。其他模式使能方式保持原逻辑。
5. RTT 编码同时避开位置环计算、HIL 命令轮询及编码器速度估计更新周期，保留 2 kHz 正常采集节拍。偶遇冲突时只延后可选遥测，不延后电流/PWM 控制。编码器提供只读的“本次更新了速度”接口；主机回归检查它的十次分频节拍、复位及坏帧行为。

上述修改没有在算法或调试服务中新增 MCU/Vendor 依赖；PWM 预装通过项目既有占空比接口完成。仓库现存 Foc 层与 HAL 耦合属于已有架构债务，本轮未展开移植重构。

## 候选控制参数

| 参数 | RAM 候选 |
| --- | ---: |
| cascade_pos_Kp / cascade_pos_Kd | 8 / 2 |
| speed_Kp / speed_Ki | 0.5 / 2 |
| Iq 指令上限 | 6 A |
| 巡航上限 | 45°/s |
| 加速度 / 有效减速度上限 | 45 / 30 °/s² |
| jerk 时间参数 | 0.2 s |
| HOLD 进入 / 退出位置窗 | 0.18° / 0.26° |
| 位置验收 | 每段末 2 s：最大绝对误差 ≤0.3°，HOLD 100% |

后续针对延迟滑动增加 `maximum_post_hold_error_deg=0.3`：指令后先观察连续 50 ms 的 HOLD，再检查本段剩余全部采样，包括随后退出 HOLD 和恢复的过程。指令后的前 50 ms 不用于确认 HOLD，避免把命令传播期间的旧状态当作新目标已保持。该项能够捕捉“曾经到位，随后滑动，最后又恢复”的情况；它仍不是结构振动测量。

速度 PI 按既有代码乘以 6 A 电流上限换算至电流域。pitch 摩擦模型尚未辨识，仍使用既有默认库仑补偿，不将默认值视为辨识结果。结构抖动需现场反馈，编码器位置验收不能替代结构振动验收。

## 中间试验结果

所有数据在 `outputs/pitch_validation_20260909/session/`。未通过项保留，不删除后仅报告成功试验。

| 试验 | 路径 / ° | 结果 |
| --- | --- | --- |
| pitch19 / pitch20 | ARM | 首次使能相电流异常，板端停止；保留三相快照 |
| pitch21_neutral_arm | 原地使能约 2.5 s | 中性预装后相电流峰值 0.672 A，超 4 A 时间 0 |
| pitch22_burst25 | 0,25,0；每段 6 s | 最后 HOLD 占比不足且 IRQ 超时 |
| pitch23_guard_fast25 | 0,25,0；每段 8 s | 定位通过；IRQ 8632 cycles，不能算整体通过；旧分析器当时未纳入 IRQ |
| pitch24_guard_split25 | 0,25,0；每段 8 s | 全部自动检查通过，启动 8485、运行 8393 cycles |
| pitch25_burst40 | 40,0,−10,0；每段 8 s | 定位通过；启动 8595 cycles 超时 |
| pitch26_staged40 | 40,0,−10,0；每段 8 s | 自动通过，末段最大误差 0.253°；启动 7674、运行 8370 cycles；相电流峰值 4.553 A，超 4 A 累计 0.80005 s |
| pitch27_margin_range | 46,−12,0；每段 10 s | 定位通过；运行 8508 cycles、57 次 ≥50 μs，整体失败；相电流峰值 5.399 A，超 4 A 累计 2.08025 s |
| pitch28_slot_range | 46,−12,0；每段 10 s | 自动通过；末段误差最大 0.242°，相电流峰值 5.413 A、超 4 A 累计 2.0627 s，运行 8336 cycles |
| pitch29_reverse_range | −12,46,0；每段 10 s | 自动通过；末段误差最大 0.228°，相电流峰值 5.346 A、超 4 A 累计 2.07715 s，运行 8383 cycles |
| pitch30_micro | 0,0.09,−0.09,0；每段 8 s | 定位通过；运行 8598 cycles、36 次 ≥50 μs，整体失败；相电流峰值 3.317 A，未超过 4 A |
| pitch31_encoder_micro | 0,0.09,−0.09,0；每段 8 s | IRQ 8236 cycles、无超时；第 3、4 段 HOLD 不连续，确认 HOLD 后最大偏差约 0.475°、0.659°，整体失败 |
| pitch32_micro_repeat | 完全相同参数和序列重试 | 自动通过；确认 HOLD 后最大偏差约 0.244°，相电流峰值 2.028 A、超 4 A 时间 0，IRQ 8367 cycles |
| pitch33_long_hold | 46,−12,0；每段 15 s（含运动时间） | 自动通过；确认 HOLD 后最大偏差约 0.253°，相电流峰值 5.359 A、超 4 A 累计 2.1931 s，IRQ 8258 cycles |
| pitch34_small | 3,0,−3,0；每段 8 s | 定位、持续 HOLD、保持后偏差检查均通过，最大偏差约 0.242°；相电流峰值 2.646 A，未超 4 A；但 IRQ 8540 cycles、11 次 ≥50 μs，整体失败 |

用户两次反馈未看清微小运动，不能由此认定异常有外力或没有结构抖动。pitch31 与 pitch32 使能前实际位置、此前运动历史不同，不是完全相同的机械初始状态。pitch31 波形提示保持积分支撑电流与静摩擦可能有关，但目前仅为待验证假设；未据此加入未经验证的保持泄漏算法，也未放宽验收窗口。位置环、速度 PI 增益在这两轮之间未变。

最终镜像 `image_pitch_encoder_slot`，AXF SHA-256：`0a6be2494a96af1470ea4093c309ffdce38f8b63896805e3016b93b8a59dc2f8`。对应普通/HIL 构建均 0 错误 0 警告；普通镜像仅构建，未下载。主机 C 回归包含 25 组伺服场景及 72 组工况组合、轨迹/实际编码器速度估计/HIL/AXS1；Python 最后为 13 项验收工具检查、5 项边界检查、2 项轴记录检查及 5 项恢复检查通过。恢复检查日志里的模拟 probe disconnect 是离线故障注入，不是本轮台架断连。

## Flash 与数据追溯

最新轴记录 CRC32：2802317620。参数区 SHA-256：`160ba507965334ab3bc9b0327cc55e1705653b41767a71ffef6e86db9c076c6e`。pitch 校准标志 7、电角零位 43804、机械零位 36688、schema 10 保留。

写入证据：`outputs/pitch_validation_20260909/radian_envelope/patch/verified.json`。每次镜像下载分别保存 HEX/AXF/MAP、原 Flash 和验证报告，验证完整参数区不变。每轮 `trial.json` 保存实际参数、镜像哈希、关断结果和电流保护诊断；后期轮次额外保存配置快照。`mode3_acceptance.json` 保存自动验收，不能替代人工结构振动判断。

最终停用回读：`outputs/pitch_validation_20260909/final_disabled_state.json`。图形对照：`session/pitch28_slot_range.png`、`session/pitch31_encoder_micro.png`、`session/pitch32_micro_repeat.png`。

## 后续重复执行

仅针对当前已校零的 pitch 台架，使用匹配的 HIL 镜像及其独立 session；新电机应先重新建立镜像、参数备份及轴配置。下面命令会使能并运动，结束后自动停止；下载和校准不包含在命令内。

```powershell
$taskPython = 'C:/Users/yang yang/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe'
$taskSession = 'outputs/pitch_validation_20260909/session'
$taskProfile = 'tests/mode3/pitch_candidate_20260909.json'
$taskName = 'pitch_range_' + [Guid]::NewGuid().ToString('N')
& $taskPython tools/servo_hil_run.py --name $taskName --session-dir $taskSession --axis-profile $taskProfile --phase-burst --kp 8 --kd 2 --speed-kp .5 --speed-ki 2 --targets=46,-12,0 --seconds 10
if ($LASTEXITCODE -ne 0) { throw 'Trial failed: inspect shutdown status before any further motion' }
& $taskPython tools/run_mode3_validation.py analyze --profile $taskProfile --trial (Join-Path $taskSession $taskName)
if ($LASTEXITCODE -ne 0) { throw 'Recorded-data acceptance failed' }
```

可将该命令的目标和每段时间替换为下列用例；每次使用新名称，先完成一轮分析再决定下一轮，不将重新 ARM 当作冷却判据。

| 用例 | `--targets` | `--seconds` |
| --- | --- | ---: |
| 微小角度 | `0,0.09,-0.09,0` | 8 |
| 小角度 | `3,0,-3,0` | 8 |
| 大跨度 | `46,-12,0` | 10 |
| 相反顺序的大跨度 | `-12,46,0` | 10 |
| 较长保持 | `46,-12,0` | 15 |

这里使用自定义非对称路径，不传通用计划的 `--case`，避免把几何中心为 17.19°的通用用例与以零位为基准的本次序列混用。默认 `pitch.json` 是 Flash 增益基线，不能替换候选 profile 后继续沿用当前增益结果。
