# 2026-11报告

- [CAN FD故障解析/清除与节点6编码器诊断](canfd_fault_clear_encoder_20261113.md)：v1.10.1 Release通过，保留样本显示标定方向相反；未清错或主动转动。

- [CAN FD v1.10.0共用电机与页面顺序](canfd_shared_motor_20261113.md)：共享身份、六页重排、Release与窗口验证。

- [YAW标定观测器闭环抖动](yaw_calibration_oscillation_20261113.md)：确认标定Kp过大，速度标准差下降88.8%；正式版保存/关相与参数保留通过。

- [CAN FD v1.9.2标定曲线与原码](canfd_calibration_curves_20261113.md)：独立样机/测试号、63,394点原码、零位/相位/极值与配置保存；保留桌面点选及其他型号未测范围。

- [30 rad/s目标边界修复](speed_target_boundary_20261113.md)：修复浮点换算误拒绝，节点6±30受理、±30.001拒绝，当前角区十项正常。

- [CAN FD v1.8.2中止提示和速度上限](canfd_abort_speed_limit_20261113.md)：确认Hall cos4044 cnt越界先中止，修正提示；保存本板30 rad/s包络，限值写入与±5 rad/s通过。

- [CAN FD v1.8.1角度标定与速度控制](canfd_calibration_speed_20261113.md)：独立页面和平铺标定按钮；500 ms标定保存、正反速度及GUI工作进程通过，视觉复核和其余型号未测。

- [APP v1.8.2 / Loader v2.3.4 首答修复](can_first_answer_v1_8_2_20261113.md)：仲裁80%消除本台架发送位错误；10000查询、30静态及3 A双向零超时，Hall独立异常保留。

- [PITCH 重新标定与霍尔连续性](hall_recalibration_20261113.md)：两次保存成功；其他角区十项正常，返回原角区仍有原码尖峰，电机命令/编码器方向通过。

- [APP v1.8.1 趋势、命令方向与 CAN FD 上位机实测](hardware_check_v1_8_1_20261113.md)：完整双向及打包 GUI 均确认电机命令/编码器方向正常；剩余霍尔动态与 CAN 首答可靠性未通过。

- [APP v1.8.0 合并后硬件检测与交付](hardware_check_v1_8_0_20261113.md)：离线 release 档通过，八项静态通过，调整转角/电流后旋转动态仍失败，保持 debug。

- [YG_MC_HOST v3.0.1续租计数回退防护](yg_mc_host_lease_counter_20261111.md)：真实5/0拒绝、使能前预留与缺失记录门禁；离线通过，回退来源与实机复测待确认。

- [YG_MC_HOST v3.0.0候选验收](yg_mc_host_acceptance_20261111.md)：整体验收未通过；81项测试及PR通过，现有GUI续租拒绝后断联与曲线60Hz未达标仍阻断。

- [YG_MC_HOST v3.0.0仪器工作台](yg_mc_host_instrument_20261111.md)：全参数表、81项测试和PITCH四模式GUI回归；曲线稳定60Hz、动态精度及正式发布限制见报告。

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
