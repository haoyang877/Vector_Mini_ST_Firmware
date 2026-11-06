# 左轮 HT8115 标定锁定失败排查

基线为 be91e0fe，左轮节点 0x03、HT8115-4、21 极对、直驱；用户已确认轮子架空、
可以连续转动、现场可断电，驱动采样为 4 mΩ / 20 倍。不沿用 PITCH 的标定数据。

两次实测均在速度锁定阶段等待 3 s 后报 SENSORLESS，尚未进入 LUT 扫圈。
2 kHz RTT 无丢帧，观测器平均速度正确但瞬时值跨越锁定范围，Id/Iq 同时出现尖峰。
关相零偏复核与启动 2048 拍独立均值一致。型号及采样比例已核对，R/L/磁链仍为规格预设。

先核对采样窗口和电流环响应，用最小单变量试验区分测量与控制问题；临时 RTT、故障快照、
镜像、原始数据和比较图只放 outputs。不得用删除锁定条件或修改有效标志伪造成功。
保持节点、运动包络、看门狗、过流/过压保护、无 Hall 运行滤波及参数 ABI。

验收为完整编码器 LUT + 电零位标定成功、Flash CRC/回读及重启恢复正确，检查电流波形和
故障状态。生产修改执行 PR 与两板编译；release 门禁和实际硬件证据分别记录。
无关工作树修改继续保留。试验失败时 STOP 并核对关相；可恢复原生产镜像及本板备份记录。

## 结果（2026-11-06）

根因定位到当前工况下 ADC 触发过早。模式 A、上桥高有效的中心对齐 PWM 中，
计数峰值对应下桥导通窗口中心；旧触发在 18.75us，三相 4 倍转换窗口约 5us。
仅将比较值 3375 改为 4050（22.5us），使窗口围绕 25us。PI、模型、锁定阈值与超时均未改。
这是单变量台架证据；未用示波器直接测量功率边沿与模拟放大器建立时间。

- 三相静态偏置约 2051.85 / 2052.26 / 2053.36 counts，独立静止均值与之偏差小于 0.06 counts。
  HT8115-4 / Kv15、4mΩ / 20 倍由用户确认；R/L/磁链仍是预设，未做参数辨识。
- 原触发在速度锁定 3s 后报 SENSORLESS，未进入速度闭环与 LUT 扫圈；角度方向相同。
  新触发在要求的连续 0.2s 内通过锁定，随后完成 100 圈 LUT。
- 2kHz RTT 对比：锁定阶段 Iq 误差 RMS 0.08691→0.01129A，
  三相和 RMS 0.09925→0.02493A；统计窗口分别为旧 3s 与新 0.2s。
  对比图使用双方起始 0.2s；旧 13255 帧、新 92639 帧，帧校验与序号均无异常。
- 用户反馈诊断固件整轮无明显抖动或异常声音。恢复生产 RTT / 故障处理后，
  再次完成 100 圈 LUT 和独立电零位作业；最终 flags=3、reverse=0、electrical_zero_q15=65332。
  Flash CRC 正确，重启 RAM 中 LUT/零位/标志逐字节匹配 Flash，当前 IDLE、fault=0、PWM 关闭。
- 无计算超时故障，但未测量 20kHz 快环最坏执行周期；不以 fault=0 代替 WCET 证明。
  未设置工装机械零点，未复测 PITCH、其他电机、高占空比或带载工况。

生产改动只有 AT32 adc.c 的一个触发系数和解释注释；不带临时 RTT、故障快照或测试脚本。
独立源码快照基于 be91e0fe 加此修改，AT32 与 STM32 APP 编译均 0 error / 0 warning。
AT32 Code/RO/RW/ZI 为 69116/8408/192/48232，与改动前相同。
生产 HEX SHA-256：809facfd216dceb39cb7d4de0b429489b949b34a024716ceb8c7fa582d039f95。

PR 全部通过：outputs/runs/20261106T114955069030Z-be91e0fe/summary.json。
Release 检查中的环境、格式、测试、架构、接口和文档通过；工作树未提交导致 clean-release-source
失败，因而没有 release 构建/清单通过，不能称为正式发布。
证据：outputs/runs/20261106T115141979555Z-be91e0fe/summary.json。

台架数据保存在 outputs/left-wheel-8115-20261106：offset-model-audit.json、
calibration-03-current / calibration-04-trigger4050、adc-window-comparison.png、
calibration-06-production-lut-result.json、calibration-07-production-eleczero-result.json、
production-reboot-result.json、adc-window-production-program-result.json。
临时数据不纳入提交；上述构建与台架验证在修复提交前完成。
