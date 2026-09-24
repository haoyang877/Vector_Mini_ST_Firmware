# 决策记录：速度环改用短窗测速反馈以降低反馈延时

- 日期：2026-09-23；分支：`codex/ovp-36v`（实验分支，不发布）。
- 目标：提高转速闭环刚度。实测表明主瓶颈是**测速反馈延时 4.5 ms**（16 抽头 / 8 ms 滑窗），
  其次是**零点门限 ±0.096 rad/s** 造成的低速静差带。
- 依据：`outputs/damping_ring_20260922/15_speed_delay.svg` 与本次台架实测（mode 2，2 A 测试限流）：
  - 互相关法测得编码器测速估计器相对 2 ms 微分的延时 **+4.5 ms**（corr 0.497）；
    观测器速度延时 −0.5 ms、但噪声 0.492 rad/s（滑窗为 0.221 rad/s）。
  - 0→0.6 rad/s 慢斜坡显示死区 **0.05–0.1 rad/s**（与 8 Q15 = 0.096 rad/s 门限一致）。
  - 小阶跃超调 69%/100%（欠阻尼），1 Hz 抖动增益 1.016、相位滞后 0.8°。

## 改动内容（本次）

1. `encoder.c/.h`：新增**短窗测速** `vel_mech_fast` = 最近 `ENCODER_VELOCITY_FAST_WINDOW = 4`
   个 0.5 ms 位置增量之和（2 ms 平均，群延时约 0.75 ms），零点门限降为
   `ENCODER_VELOCITY_FAST_ZERO_THRESHOLD_Q15 = 1`（≈0.012 rad/s）。原 `vel_mech`（16 抽头 /
   8 ms / 8 Q15 门限）保持不变，继续供位置模式与既有遥测口径使用。
2. `hw_conf.h`：新增编译开关 `SPEED_LOOP_USE_FAST_VELOCITY`（0/1），用于 A/B。
3. `foc_run.c`：`Task_Speed_Mode` 的速度反馈在该开关下改用 `Encoder_GetMecVelFast()`。
4. `foc_task.c`：标定 RTT 帧的 `encoder_speed` 通道在同开关下记录同一个"环路实际反馈"，
   使 A/B 的波形与环路反馈严格一致。

## 不变的行为

- 位置模式、无感启动、标定流程、电流环、参数 ABI、RTT 帧布局（20 路、40 B）均不变。
- `vel_mech` / `vel_mech_continuous` 的定义与数值不变；`vel_elec` 仍由 `vel_mech` 导出。
- 默认值 `SPEED_LOOP_USE_FAST_VELOCITY = 0` 时行为与改动前完全一致。

## 基线 / 验收

- 基线（改动前，2 A 限流）：反馈延时 4.5 ms；死区 0.05–0.1 rad/s；±5 rad/s 稳态误差 ≤0.009、
  std 0.27、无停滞带；+1 rad/s std 0.43、10 段短停滞；小阶跃超调 69%/100%。
- 验收（开关置 1 后）：① 互相关延时 ≤1.5 ms；② 慢斜坡死区 ≤0.03 rad/s；
  ③ ±5 rad/s 稳态误差与 std 不劣于基线；④ 小阶跃超调明显下降；⑤ 电流噪声 std 仍 ≪ 限流。

## 回滚

- 开关置 0 重新构建即可恢复原行为；改动不涉及 Flash 参数与协议，无数据迁移。

## 证据

- 构建：Keil 双目标 0 error / 0 warning；`tests/run.py` unit + integration 全通过。
- 台架：`speed_stiff` 三类探针（互相关延时、慢斜坡死区、小阶跃）+ 波形 SVG 归档到
  `outputs/damping_ring_20260922/`。
