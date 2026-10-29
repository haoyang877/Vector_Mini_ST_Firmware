# AT32标定和速度闭环实测 v1

用户要求执行标定和转速闭环测试。使用已通过双板/PR的adc-temperature-final-20261029 Commissioning候选，不修改正式控制代码或PI参数。
沿用已确认台架：固定空载、电源限流1A、相电流3A、标定30rad/s及60rad/s截止；速度目标±1/±2/±3rad/s，主机5rad/s及0.75A参考/反馈提前截止。保持9000周期IRQ保护、PWM提交守卫、硬件看门狗及一次性功率窗口。沿用用户之前仅在Commissioning关闭绕组热模型的设置。

先只读核对板型、USB CAN、30.8V母线、关相状态；备份全部Flash，下载候选并独立读回，检查关相通信。执行一次LUT和三次电气零位标定，核对作业结果、持久化CRC、无关参数不变及复位恢复。通过后运行双向速度阶跃，记录CAN反馈和RMS误差，0.3rad/s为已有测试标准。任何故障/新鲜度/限值越界先STOP，失败则独立关相。失败不放宽阈值、不伪报通过。

证据保存在outputs/bsp-v1-implementation-20261026/hil-cal-speed-20261029。结束核对功率关闭，保留失败原始数据。不以关相IRQ时间代替带电闭环验证。

## 实测结果
未通过整体验收。LUT成功；零位4次尝试中3成功1次PWM提交超时，完整失败记录已保留。+1rad/s速度目标出现9.796rad/s反馈，主机5rad/s截止触发STOP，后续阶跃未执行。没有修改PI或放宽任何保护。最终IDLE、故障0、输出关闭，保存的标定CRC与RAM/Flash一致。
完整结果见docs/reports/2026-10/at32_calibration_speed_hil_20261029.md；证据为本轮summary.json和final-stop-check.json。
