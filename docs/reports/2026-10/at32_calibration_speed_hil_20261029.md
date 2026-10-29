# AT32标定和速度闭环实测（2026-10-29）

## 结论
本轮未通过整体验收。LUT标定完成；电气零位4次尝试中3次成功、1次触发PWM提交超时；速度闭环在第一个+1rad/s目标下出现9.796rad/s反馈，主机按5rad/s上限立即STOP。未继续后续负向及±2/±3rad/s阶跃，不把未执行项算通过。

## 版本与范围
固件为已通过双板及PR的adc-temperature-final-20261029快照，目标ESC-APP-AT32F456-Commissioning。镜像HEX SHA-256为4cd95a02c1c5e746ebcf7ae0d77b46e0bb56953704b0c88861397f3dd5a4a0fc。未合入同期其他任务的CAN修改，未修改生产源码。
180MHz、20kHz快环、2kHz监督；沿用800Hz编码器PLL和实际读取的速度Kp=0.005、Ki=0.05。没有临时改增益。Commissioning绕组热模型按用户此前要求关闭，固定3A限流、9000周期IRQ限制、PWM提交保护、硬件看门狗和一次性窗口保留。
母线约30.8V。用户已授权固定空载、电源限流1A、标定目标30rad/s及60rad/s截止；闭环目标±1/±2/±3rad/s，并保留主机5rad/s及0.75A参考/反馈提前截止。

## 标定结果

| 项目 | 实测 |
| --- | --- |
| LUT | PASS，6.765s；测速峰值34.815rad/s，Iq峰值0.529A |
| 电气零位第1次 | PASS，1.563s，14189 |
| 电气零位第2次初次 | FAIL，启相约42ms后PWM提交超时 |
| 电气零位第2次重试 | PASS，1.631s，14191 |
| 电气零位第3次 | PASS，1.621s，14191 |

成功标定均检查了Flash CRC、RAM/Flash一致性、复位后重新加载和无关参数不变。LUT后标定标志为1，零位后为3；最终LUT有效，电气零位14191。
失败现场内部故障码15（CONTROL_OVERRUN）、s_output_fault=1、ADC fault=0且采样仍运行、启相拒绝计数=0；据生产代码，指向McuPwm提交期限检查失败。不是以关相IRQ耗时推断故障来源。PWM已经关闭，未改保护阈值；失败原始记录保留在zero2-first-failure目录。失败标定清除了RAM标志但未覆盖前一次有效Flash记录，复位后重新加载。

## 速度闭环结果
零速保持0.2s通过，尾段RMS误差0.071rad/s。随后+1rad/s目标未完成：反馈从接近0上升至2.578、9.796rad/s，后者超过主机5rad/s截止，立即STOP并确认输出关闭。最后一帧Iq参考0.104A、反馈0.098A，CAN固件故障位为0；本项由主机速度限制终止，不是过流或固件故障触发。
停止前位置也由1.566变化至1.576、1.663rad；不能仅凭静止时PLL偶发噪声，把此次运行反馈异常解释为显示噪声。具体机械速度与测速误差占比需要同步角度/速度数据继续定位，本轮不假定根因是PI参数。
未继续后续速度点，也未放宽5rad/s或0.75A阈值来获得通过结果。速度测试不写持久参数。

## 最终状态与证据
最终CAN状态为IDLE、固件故障0，PWM及六路输出关闭，CPU运行，ADC/监督约20kHz/2kHz，看门狗未调试冻结。最终标定记录CRC有效、RAM/Flash一致，且与零位第3次成功后的记录完全相同。
当前保留本轮Commissioning镜像，输出关闭；没有恢复旧标定，也没有改PI或增加生产测试代码。
全部证据位于outputs/bsp-v1-implementation-20261026/hil-cal-speed-20261029/：summary.json、can-lut-result.json、can-zero1/2/3-result.json、zero2-first-failure/、can-control-speed-result.json、speed-feedback-sequence.json、final-stop-check.json。原始CAN收发保存在对应capture.json。
下一步应分别定位偶发PWM提交期限失败与低速起动/测速反馈跃升，不能用关相计时通过代替带电验收。

记录与当前工作区quick验证通过：outputs/runs/at32-cal-speed-hil-20261029/summary.json；这不改变上述硬件测试FAIL结论。
