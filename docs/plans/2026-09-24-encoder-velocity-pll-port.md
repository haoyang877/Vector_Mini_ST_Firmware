# ovp36 分支移植：20 kHz PLL 测速估计替换 2 kHz 滑窗

日期：2026-09-24。状态：**已实施（构建 0/0、离线测试全绿）；台架验证待办（J-Link 未恢复）**。

来源：worktree `D:\Work\Code\Vector_Mini_ST_Firmware`（分支 `codex/wheel-motor-2mohm-no-damper`）
的 PLL 实现（`firmware/motor/position/angle_feedback.c` + 决策
`docs/plans/decisions/0002-encoder-velocity-estimator-ownership.md`）。

## 1. 目标

把该分支已验证的 20 kHz 二阶角度跟踪观测器（PLL）测速引入本实验分支（`codex/ovp-36v`），
替换 2 kHz 滑窗割线，降低速度反馈延时以提高转速环可用带宽。

## 2. 与源分支的差异（为何不是直接拷文件）

| 项 | 源分支 | 本分支 |
| --- | --- | --- |
| 分层 | `motor/position/angle_feedback.c` + `platform/api/encoder_sensor.h` + port driver | `platform/stm32g4/bsp/encoder.c` 单体 |
| 采样入口 | `Encoder_BeginSample/CompleteSample`（已分离） | 同（已存在） |
| 快拍速率 | 20 kHz | 20 kHz（同） |

→ 采取**算法移植 + 就地改造**：PLL 算法与参数逐行等价移植，接口保持本分支既有 API。

## 3. 改动

- `firmware/platform/stm32g4/bsp/encoder.h`
  - 新增 PLL 常量：`ENCODER_PLL_OMEGA_N_HZ 300.0f`、`ENCODER_PLL_ZETA 0.707f`、
    `ENCODER_PLL_OMEGA_MAX_RAD_S 2000.0f`、`ENCODER_PLL_VEL_ZERO_THRESHOLD_RAD_S 0.05f`。
  - 移除滑窗状态（`velocity_delta_history[16]`、`velocity_fast_delta_history[4]`、各计数器、
    `velocity_shadow_q15`）与窗口宏；新增 `pll_theta_rad`、`pll_omega_rad_s`。
  - `velocity_divider` 保留，但语义改为"2 kHz 快拍分频"（供 `Encoder_DidUpdateVelocity()`）。
- `firmware/platform/stm32g4/bsp/encoder.c`
  - `Encoder_UpdateVelocity2kHz()` → `Encoder_UpdateVelocityPll()`（20 kHz，仅有效帧执行）：
    `err=wrap(θ_meas−θ_est)`；`θ_est += Ts·ω_est + kp·err`；`ω_est += ki·err`；
    `kp=2ζωnTs`、`ki=ωn²Ts`（Ts = `Current_Ts` = 1/20 kHz）；输出限幅 ±2000 rad/s；
    `vel_mech` 加 0.05 rad/s 软死区，`vel_mech_continuous`/`vel_mech_fast` 不受死区影响。
  - 首帧/重启：`velocity_ready=false` → 下一拍以当前角自初始化（ω=0），不产生速度冲击。
  - 调用点：`Encoder_CompleteSample()` 内替换为 `Encoder_UpdateVelocityPll()`。
  - `Encoder_ResetVelocity()` 改为复位 PLL 状态与全部速度输出。
- `tests/unit/test_speed_loop_bandwidth.py`（新增，宿主脚本，不占固件）：旧割线 vs PLL 的
  群延迟/相位/噪声/延迟受限带宽离线验收。

## 4. 参数与行为变化

| 项 | 改造前 | 改造后 |
| --- | --- | --- |
| 测速速率 | 2 kHz（16 抽头）/ 另有 4 抽头短窗 | **20 kHz PLL** |
| 群延迟 | 4.0 ms（短窗 0.75 ms） | **0.68 ms**（ωn=300） |
| 零点死区（`vel_mech`） | 0.096 rad/s（8 Q15）；短窗 0.012 rad/s | **0.05 rad/s** |
| 速度噪声 | 基准 | **6.58×**（ωn=300）/ 2.24×（ωn=150，离线白噪声模型） |
| `vel_mech_fast` | 4 抽头短窗 | 与 `vel_mech_continuous` 同值（PLL） |
| RAM / Code | Code 80108 / ZI 25100（normal） | **Code 79956 / ZI 25012**（−152 B / −88 B） |

## 5. 受影响消费方（已核对）

| 调用点 | 接口 | 影响 |
| --- | --- | --- |
| `foc_run.c:88` | `Encoder_GetMecVelFast` | 速度环反馈（`SPEED_LOOP_USE_FAST_VELOCITY`），主受益者 |
| `foc_run.c:90` | `Encoder_GetMecVel` | 非 fast 分支，动态改善 |
| `foc_run.c:689/701/703` | `Encoder_GetMecVelContinuous` | 位置串级（mode 3）→ 动态变快，**整定可能需复核** |
| `foc_friction_identification.c:167` | `Encoder_GetMecVel` | 摩擦辨识（mode 19）输入 → **辨识结果可能变化，需复核** |
| `foc_run.c:34/94/688/737/1046`、`foc_task.c:458/469` | `Encoder_GetEleVel` | 电角速度，语义不变、动态更快 |
| `interface_can.c:634`、`foc_task.c:129/277` | `vel_mech` | 死区口径 0.096→0.05 rad/s（低速反馈分辨率变细） |

## 6. 证据

- 构建：Keil normal 与 HIL 双目标 `0 Error(s), 0 Warning(s)`（2026-09-24）。
- 离线测试：`tests/run.py` unit + integration **PASS**；新增带宽脚本 6 项 PASS，关键输出：

```
group delay : secant 4.00 ms | PLL 300Hz 0.68 ms | PLL 150Hz 1.43 ms
crossover @PM45 : 29 Hz -> 156 Hz (300Hz) -> 81 Hz (150Hz)
crossover @PM60 : 19 Hz -> 104 Hz (300Hz) -> 54 Hz (150Hz)
velocity noise  : 6.58x @300Hz, 2.24x @150Hz
```

## 7. 待办（台架，需 J-Link）

1. 烧写 + 停机态检查（`mode=0`、无功输出、RTT 正常、静止 `vel_mech` 无漂移/极限环）。
2. 恒速噪声 A/B 与速度阶跃（对比旧 2 kHz 割线），确认延时下降且噪声在可接受范围。
3. 转速环 PI 重新整定（PLL 改变了反馈动态）+ 最终带宽验收（FRF 实测）。
4. 复核受影响消费方：mode 3 位置串行整定、mode 19 摩擦辨识。
5. 低载工作点（0.05–0.2 A）下的死区口径变化确认（CAN/RTT 反馈）。

## 8. 回滚

单一改动面（`encoder.c/.h` + 一个新增测试脚本）→ `git checkout -- firmware/platform/stm32g4/bsp/encoder.c encoder.h`
即恢复 2 kHz 滑窗语义。

## 9. 后续可选改进（移植时未做，保持与源分支行为一致）

源分支计划 §12 自己列为必做但未实现，建议后续补：

1. **创新/坏点门限**：`ki·Δθ` 会把单点角度跳变放大成速度尖峰（Δθ=0.1 rad → 17.8 rad/s），
   当前 ±2000 rad/s 限幅拦不住 → 应加 `|err| > GATE` 时只滑行不校正（GATE≈0.02 rad）。
2. **坏帧滑行**：取帧失败时未推进 `θ_est += Ts·ω_est` → 恢复时误差累计（20 rad/s 每丢帧≈0.18 rad/s 速度踢动）。
3. **限幅收紧**：2000 rad/s 相对有效域 20 rad/s 过松，建议按应用上限 2–3× 收紧。
4. **按速调度 ωn（已实施）**：12 项实验证明限制速度环带宽的是**反馈路径相位滞后**而非噪声
   （ωn=150 Hz 时 Kp=0.04 即失稳；300 Hz 边界 ~0.07；600 Hz 边界 ~0.11），故低速用 600 Hz、
   高速用 300 Hz（`ENCODER_PLL_OMEGA_N_LOW/HIGH_SPEED_HZ`），过渡 5→10 rad/s。

## 10. 调参与带宽实测（2026-09-24，台架）

**参数更新**：电机实测值写入 profile（R=2.144 Ω、Ld=Lq=0.9 mH、电流环带宽 750 Hz → 默认即生成
已验证增益 Kp=4.2412/Ki=10103.4，Flash 与 RAM 双向确认）；速度环默认 Kp=0.06/Ki=0.6。

**速度环增益扫描**（3 rad/s，按速调度 ωn）：Kp=0.10 稳定、0.12 失稳 → 边界 ~0.11。
全速域（1/5/10/15/20 rad/s）跟踪误差 ≤1%，调度版把 15/20 rad/s 纹波从 0.707/0.836 降到 0.561/0.476。

**扫频实测（SWD 注入 speedRef，2 kHz RTT，基准 3 rad/s、幅度 0.3 rad/s）**：

| f [Hz] | \|T\| [dB] | ∠T [°] | \|L\| | PM [°] |
| --- | --- | --- | --- | --- |
| 8 | +0.08 | −5.6 | 10.3 | 82 |
| 18 | −0.27 | −12.2 | 4.60 | 92 |
| 35 | −0.01 | −22.3 | 2.59 | 79 |
| 50 | −0.07 | −37.0 | 1.57 | 72 |
| 70 | −0.83 | −50.7 | 1.11 | 70 |
| 100 | −1.13 | −59.0 | 0.94 | 67 |

→ **穿越频率 ≈ 88 Hz，相位裕度 ≈ 70°，|S| 峰 ≈ 0.9**（= 1/(2sin(PM/2)) = 0.89 自洽）。
等效环路延时由 PM 反推 ≈ **0.63 ms**（PLL 0.38 @600 Hz + 2 kHz ZOH 0.25 + 传感器 0.09）；
140 Hz 点因电流饱和无效。

**结论与后续杠杆**：速度环已接近 2 kHz 外环速率下的可用上限，**第一杠杆是把外环从 2 kHz 提到
4 kHz**（ZOH 0.25 → 0.125 ms，预期穿越可到 ~110–130 Hz）；其次才考虑再抬 ωn 或上模型辅助观测器。

**方法学修正（重要）**：早期用"阶跃上升时间"估带宽得到 ~15 Hz 是**错的**——阶跃被参考斜坡
（50 rps² = 314 rad/s²）限速，测到的是斜坡而非环路。带宽必须用扫频，不能用限速阶跃。

## 11. A/B 对照：旧滑窗估计器 vs PLL（2026-09-24，同限流 6 A、同增益口径）

**A = HEAD 的 16 抽头/8 ms 滑窗**（构建时把 `SPEED_LOOP_USE_FAST_VELOCITY` 置 0）；
**B = PLL 按速调度（600/300 Hz）**。两者同电流环（4.2412/10103）、同限流 6 A、同标定、
同一 2 kHz 真值捕获工具。

| 项 | A（旧滑窗） | B（PLL） |
| --- | --- | --- |
| 增益上限 @3 rad/s | **Kp=0.04**（0.05 起失稳） | **Kp=0.11** |
| 极限环频率 | **~51–60 Hz** | ~250 Hz |
| 由 f=0.25/Td 反推延时 | **~4.2–4.9 ms** ✓ | **~1.0 ms** ✓ |
| 稳定区纹波 @0.04 | std 0.211–0.223 | —（B@0.06 为 0.246） |
| @Kp=0.06（B 的工作点） | **1/5/10/15/20 rad/s 全部失稳** ✗ | 全程稳定 ✓ |
| 有效增益上限（×6 A） | 0.24 A/(rad/s) | **0.66 A/(rad/s)（2.75×）** |
| 扫频可用性 | 在其上限 0.04 一注入正弦即掉出模式（无余量） | 88 Hz 处 PM 70° ✓ |

**结论**：
1. **口径归一化后 PLL 的可用环路增益是旧法的 2.75 倍**；旧法"Kp=0.10/边界 0.15"是在
   **2 A 限流**下调的，换算到 6 A 仅相当于 Kp≈0.04（本次实测确认）。
2. **极限环频率精确印证延时理论**：极限环出现在"延时 −90° + 积分器 −90° = −180°"处，
   即 `f = 0.25/Td`——旧 55 Hz ↔ 4.3 ms，新 250 Hz ↔ 1.0 ms，与理论完全一致。
3. 旧法在 PLL 的工作增益（0.06）上**无法运行**，"换回旧法"等价于把增益降到 0.04 以下。
4. 唯一对旧法有利的区域是**低速噪声**（8 ms 长窗平均）：早期测得 1–2 rad/s 处旧 4.9% vs
   新 18.5%，但该点尚未在同增益口径下复测 → **待办**。
5. 正确解法是**按转速调度估计器带宽**（低速低 ωn/长窗、高速高 ωn），而非整体退回旧法。

### 11.1 低速纹波：同口径复测（同 Kp=0.04/Ki=0.4、同 6 A、2 kHz 真值）

| 转速 | A（旧滑窗）std | B（PLL）std | 胜 |
| --- | --- | --- | --- |
| 0.5 rad/s | **32.4%** | 45.8% | A |
| 1.0 rad/s | 29.9% | **23.6%** | B |
| 2.0 rad/s | 16.2% | **11.9%** | B |
| 3.0 rad/s | — | 8.8% | — |

**结论与两条重要修正**：

1. **早期"旧 4.9% vs 新 18.5%"不是同口径数据**（限流/增益/乃至统计口径不同），已作废 ✗。
   同口径下两者在 0.5–3 rad/s 都是 **9–46%** → **低速纹波由扰动（齿槽）主导，不是估计器噪声主导** ✓。
2. A 在 0.5 rad/s 的"优势"主要是**测量伪影** ✗：① 8 ms 长窗本身是 ~20 Hz 低通，把被测速度纹波**平滑掉**（掩盖而非消除）；② A 的 0.096 rad/s 硬死区在 0.5 rad/s 处占 19%，把小速度直接钳零 ✗。
   → 实际机械纹波 A 并不优于 B；B 在 ≥1 rad/s 全胜 ✓。
3. 因此"旧法在低速更好"**不成立** ✓。PLL 在同口径下等于或优于旧法，且可用环路增益是 2.75 倍 ✓。
