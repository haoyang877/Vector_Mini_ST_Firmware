# Mode 13 观察器标定实机调试报告（2026-09-05）

## 结论

Mode 13 已修复并在最终构建上实机通过。根因是磁链电压模型直接复用物理相电阻
`1.905 Ω`，在当前逆变器、电机和约 1.5Nm 阻尼负载的目标工作点上造成电阻压降
估计过大，观察器速度显著偏低，状态机一直停在 `speed_lock`。

最终代码将电流环/电机模型使用的真实相电阻与观察器等效电阻解耦：

```text
observer_Rs = 1.905 Ω × 0.4199475 ≈ 0.800 Ω
```

持久化 `mrs` 保持 `1.905 Ω`。Mode 13 在这一条件下连续三次完整通过，并完成最终
Mode 15 电角零位标定。

## 基线与 main 对照

- 对照分支：`main@5e88066b72a2e536ecd69da57abec2e8d414ea36`；
- main 的 Mode 13、无感启动状态机和磁链观察器方程与重构版逐项等价；阻尼器专用
  对齐电流、Iq 斜坡、目标电角速度、锁定判据、超时、采样圈数和停止过程已迁移；
- main 同样直接使用运行参数 `motor_phase_resistance` 作为观察器 Rs。由此可推断，
  main 曾通过时的 Flash/运行参数状态对结果有影响；main 默认值本身仍为 `1.905 Ω`；
- 本次尝试直接下载 main 构建做 A/B 时，MCU 进入 NMI 且 USB 未启动，所以该次不是
  有效的 main 实机基线；没有用它替代用户此前“main 已通过”的事实。

## 证据链

1. 在修改前保存模式 5 的 1024 点 LUT、运行参数和元数据；CSV SHA-256：
   `7D3BF2DA8C74B028F4E521ACB50E61F17693392BAB30A10B9D50927952508C2E`。
2. `mrs=1.905 Ω` 的基线 Mode 13 在 `16.65 s` 报故障 13。编码器实际电角速度峰值
   `426.93 rad/s`，观察器峰值 `164.0 rad/s`，锁速低通峰值 `93.4 rad/s`；状态只到
   align → open-loop → speed-lock。
3. 稳态 Iq 约 `4.5 A`、Vq 约 `11.4 V`。使用 `1.905 Ω` 时，仅电阻压降估计就约
   `8.57 V`，留给反电势的电压约 `2.8 V`，与观察器只估到约 `160 rad/s` 一致。
4. 运行时临时把 Rs 改为 `0.800 Ω` 后，Mode 13 连续两次通过，排除了供电、启动
   转矩、编码器在线状态和锁定门限本身。
5. 增加观察器电阻系数，把持久化 Rs 恢复为 `1.905 Ω` 后，解耦版本连续三次通过。

## 最终实机结果

最终镜像 SHA-256：
`F22A6BBD75833298C586AFD1C2035BF9FF0260E402F2E176649EC622AE684CA2`。

最终一轮 Mode 13：

| 项目 | 结果 |
| --- | ---: |
| 总耗时 | 14.4347 s |
| 最终模式/错误 | mode 0 / error 0 |
| 母线最低电压 | 27.54 V |
| 母线输入电流峰值 | 2.74 A |
| 机械速度峰值 | 22.33 rad/s |
| 编码器电角速度峰值 | 480.27 rad/s |
| 观察器电角速度峰值 | 487.8 rad/s |
| 锁速低通峰值 | 421.0 rad/s |

状态时间线：

```text
0.00 align
2.61 open_loop / wait_closed_loop
6.61 speed_lock
7.01 handoff
7.21 closed_loop / speed_stable
8.21 find_origin
8.29 sample_cw
11.43 build_lut
11.48 verify_cw
12.11 stop_decel
14.11 stop_current
14.43 mode 0, error 0
```

最终 LUT：1024 点，峰峰值 `0.94482°`，去均值 RMS `0.21953°`，去均值峰值
`0.60334°`；CSV SHA-256：
`1D1898665F02DF70B570120D6A0B45207F716070352B6BD94D4FEFF33B4D4250`。

Mode 15 随后在 `1.6679 s` 完成，母线最低 `27.56 V`、Id 峰值 `4.51 A`、Iq 最大
`0.02 A`、最终 `mode=0/error=0`。硬件复位后再次确认编码器 Online、
`mrs=1.905 Ω`、母线 `27.79 V`。

## 代码与构建验证

- Keil ARMCC 5.06u7：0 error、0 warning；Code 96328 B、RO-data 5848 B、
  RW-data 504 B、ZI-data 28664 B；
- J-Link SWD 4 MHz：Program + Verify 成功；最终下载时 VTref 3.264 V；
- `tools/verify_architecture.ps1 -StrictCommunication`：122 个工程条目，0 failure，
  0 warning；
- `git diff --check`：无 whitespace error；
- 最终实机 Mode 13：连续通过且最后一轮使用最终构建；
- 最终 Mode 15、硬件复位和参数回读通过。

## 产物索引

- 修改前失败波形：
  `validation/mode13_debug_2026-09-05/attempt_02_instrumented/mode13_rtt.png`
- 最终通过波形：
  `validation/mode13_debug_2026-09-05/attempt_07_final_binary/mode13_rtt.png`
- 最终 LUT 波形：
  `validation/mode13_debug_2026-09-05/mode13_final_lut/mode13_encoder_lut.png`
- 最终 USB/RTT 数据与摘要：
  `validation/mode13_debug_2026-09-05/attempt_07_final_binary/`
- 最终 LUT CSV、原始串口文本、元数据与摘要：
  `validation/mode13_debug_2026-09-05/mode13_final_lut/`

当前下载的是温度保护跳过的有人值守验证镜像，使用了用户明确要求的
`HARDWARE_VALIDATION_SKIP_TEMPERATURE_PROTECTION`。它不能作为量产固件；恢复温度
采样后必须重新构建并至少完成上电、保护、Mode 13、Mode 15 和低速闭环回归。
