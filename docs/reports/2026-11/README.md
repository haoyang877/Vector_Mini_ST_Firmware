# 2026-11报告

- [APP 与 Loader 固件架构审查](app_loader_architecture_review_20261110.md)梳理双板的协议、状态机与文件职责，记录激活票据的两项故障注入结果。

- [PITCH 1024 点 Hall LUT 与滤波验收](2026-11-06-hall-lut-1024-filter-v1.md)：独立数据峰值0.356°，延迟/时序及参数复位恢复通过，发布门禁限制见报告。

- [PITCH 新板 Hall LUT 标定与精度检查](2026-11-06-hall-lut-new-board-v1.md)：标定、保存及软件复位恢复通过，0.5° 峰值门槛未通过，未提交。
- [AT32 Loader压测前置复测](dual_board_loader_stress_20261103.md)：完整备份前9轮通过、第10轮失败；暂停CPU且冻结WDT时仍读错，未安装或升级，等待Keil对照。

- [AT32 Loader首次实机预检](dual_board_loader_at32_bench_20261102.md)：J-Link连接成功但SWD回读不稳定；保护门槛阻止安装，零Flash写入，已恢复运行状态。

- [双板Loader离线开发](dual_board_loader_development_20261102.md)：双板128 KiB布局、配套工程、维护工具和验证记录；未上板。

- [正式速度参数与 0～200 rad/s 阶梯实测](speed_profile_ladder_20261105.md)记录各档平均反馈、稳态波动、峰值电流和末次设备回读。

- [10 rad/s 运行中 q 轴与低延迟测速验证](speed_pll_120hz_20261105.md)记录 90/300/600 Hz 速度反馈、分级调参、速度开环裕量及 CAN 入口复核。

- [新标定表的速度闭环带宽](new_lut_speed_bandwidth_20261105.md)记录原 PLL、原 PI 下的正反向分阶梯频响、单频复测和低速测量边界。

- [修正参考角后重新标定与测速反馈对照](speed_recalibration_estimator_ab_20261105.md)记录 10、40 rad/s 的波动及同板 PLL、滑窗、PLL 重复测试。

- [速度域阶梯与闭环带宽上限](speed_domain_bandwidth_20261105.md)记录正反向阶梯、频响矩阵、增益边界与测量限制。

- [电周期纹波与补偿表A/B诊断](speed_order_diagnosis_20261105.md)记录用户波形、分阶次拟合及临时去除LUT电周期项的实测。

- [转速环实测整定与带宽](speed_loop_tuning_20261104.md)记录增益对比、共振峰和带宽测量，修正原离线参数推荐。

- [低电流短脉冲与角度速度复核](current_short_pulses_20261102.md)：12次短脉冲无保护截止；0.3A阶跃通过，0.1/0.2A未全达标，保留同步角度/速度波形。
- [计入内部延迟的电流PI重整定](current_loop_delay_20261102.md)：D轴扫频1193.9Hz达标；噪声、零电流及0.3A稳态通过，0.75A限速停止，后续阶跃未测。
- [D轴正弦扫频](d_axis_sweep_20261102.md)：0.5A±0.1A下−3dB约2.96kHz；1.5A偏置在采集前限速，高电流频响未得到。
- [Vd/Vq修复与1.2kHz实测](current_loop_1200hz_20261102.md)：电压反馈、稳态和6次Id阶跃通过；Iq第2次速度截止，未完成全项验收。
- [电流阶跃等效带宽](current_bandwidth_20261102.md)：Id约372～640Hz，随阶跃幅值变化；Iq两次约356～448Hz，第三次限速停止；未提高增益。
- [电流环稳定性与采样噪声](current_loop_stability_20261102.md)：实读PI对应1kHz整定，关相噪声7.67～9.31mA RMS；零电流、Id 0.3/0.75A的已测工况通过。
- [速度闭环持续振荡的历史分析](speed_loop_oscillation_20261104.md)保留旧波形与离线推导，原增益推荐和穿越频率估计已由后续实测修正，使用前参阅页首说明。

结论只适用于所列硬件、固件和工况；未测项不因已测子项通过而变为通过。
