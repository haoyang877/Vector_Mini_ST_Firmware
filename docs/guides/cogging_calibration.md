# 齿槽标定与运行补偿

当前说明：2026-09-29。标定作为独立维护任务执行，不使用旧 mode 6、旧状态机或直接写电机对象的入口。

## 代码与状态归属

- `motor/identification/cogging_map`：2064 B 记录、安装签名、CRC、1024点插值与成表。
- `motor/identification/cogging_calibration`：正反向静态定位、稳定窗口与采样，内部状态私有。
- `motor/foc/cogging_compensation`：补偿准入、200 ms 渐变、查表与总电流限幅。
- `motor/identification/foc_cogging_calibration`：私有标定表、补偿控制、台架保护与遥测。
- APP 主状态机：维护身份、输出许可、停止、取消释放及独立保存衔接。

`TorqueGuard`、`TorqueTelemetryState`、`TorqueTelemetry`、`FocCoggingCompensationRequest`
四个旧公开可写对象已删除，不提供符号兼容，也不支持按旧地址或字段写入。
模块内部状态只通过受控请求与值快照访问，不能取得内部可写指针。

## 标定与保存

应用通过 `MotorStateMachine_RequestMaintenance(MOTOR_MAINTENANCE_COGGING, ...)` 或现有
160～162维护协议受理、查询和取消。受理不表示已经运动或持久化；任务必须通过轴身份、
编码器、安装标定、母线与控制限值准入。有行程约束的轴不允许执行整圈标定。

算法对一圈1024个位置做正反向静态采样，合成后去除均值；角度间隔为0.3515625°。
每点为有符号Q15电流：`Iq_A = iq_q15 * full_scale_a / 32768`，不是1mA/count。
理论满量程由采样电阻与放大倍数决定，2mΩ/10倍为82.5 A，6mΩ/10倍为27.5 A；
实际运行限流和保护继续使用已受理配置，不能把编码满量程当作可输出电流。

PREPARED由主状态机复核后启相；FINALIZE先关闭输出，再由前台认领、成表并发布。
保存为独立SAVE会话，只有Flash写入和回读成功才报告COMMITTED；标定原ticket不冒认保存成功。
STOP/FAULT撤销执行资格，取消等实际工作区归还；陈旧结果不能恢复任务或请求SAVE。

## 运行补偿请求

仅编码器反馈的电流模式使用补偿；速度、位置、无感与标定任务不叠加。
`Iq_total = clamp(Iq_command + blend * Iq_table, -current_limit_a, current_limit_a)`。
使用1024点相邻线性插值，查表电流限制为±1 A，开关使用200 ms渐入/渐出；不改写用户iqRef。

前台调用 `FocCogging_RequestCompensation(enabled)` 只复制开关请求，最新值覆盖未消费值。
现有 `MotorStateMachine_ServiceCompensation` 在稳定配置借用下消费请求，完整校验表CRC、
编码器签名与电流量程，并在提交前复核模式和故障。请求受理不等于补偿已启用，重启默认关闭。
快环只核对安装身份与表CRC字段，不重复计算整表CRC。

当前CAN不新增补偿、保护或遥测命令；历史0x26/0x27和SWD写变量说明已经退役。
这些接口供应用前台与离线测试调用，PC接入按后续显式协议开发，不绕过主状态机。

## 台架保护与遥测快照

`FocCogging_SetTorqueGuard(lease_ticks, speed_limit_rad_s)` 原子提交租约与测速上限。
租约按20 kHz倒计时，零租约关闭保护；开启时上限必须有限且大于零，非法值保持原保护。
`FocCogging_ReadTorqueGuard` 复制剩余租约与trip；trip1表示到期，trip2表示超速或测速无效。
跳闸清零电流参考并报告STOP_REQUESTED，由主状态机关闭输出；关闭保护不能撤回挂起STOP。
该保护默认关闭、RAM-only，不是量产主机断连保护，也不能使能电机。

`FocCogging_RequestTorqueSample()` 请求下一电流快环冻结一帧；已有请求或未消费帧时返回false。
`FocCogging_ReadTorqueSample(&sample)` 复制36B快照并消费；尚未发布或空指针返回false，帧保持。
快照包含同拍tick、机械角度rad、速度rad/s、用户Iq、补偿Iq、总Iq、反馈Iq、母线电压和blend。
读取后副本属于调用方，下一帧不会改写它。RTT继续使用当前4个int16通道。

## 持久化与验证边界

参数schema12、4304B原记录和16KiB参数区保持。表CRC、采样量程或安装签名变化时拒绝旧表；
用户机械零位变化不改变物理查表索引。本批没有升级设备参数或修改CAN/升级v2契约。

离线检查运行真实标定与补偿核心，覆盖1024点双向采样、跨圈、符号/量程、CRC、超时、
取消、先关相后保存，以及请求、租约和快照的边界。这些结果不证明实物定位参数、纹波或掉电行为。
本批记录见[状态所有权计划](../plans/completed/2026-09-29-cogging-state-owner-v1.md)。

历史硬件结果见[标定报告](../reports/2026-09/cogging_calibration_bench_20260917.md)、
[精度报告](../reports/2026-09/cogging_precision_20260917.md)和
[补偿报告](../reports/2026-09/cogging_compensation_20260918.md)，仅对应各报告镜像。
不要复用历史RTT地址、旧字段或旧操作脚本作为当前固件入口。
