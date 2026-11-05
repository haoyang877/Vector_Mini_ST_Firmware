# 单轴位置 S 曲线与位置串级

POSITION 每个外环周期采样机械位置与速度参考；AT32F456/YQ-5708 为 250 µs，其他配置为 500 µs。轨迹由最多七段恒 jerk 多项式组成，
到达目标时速度和加速度为零。短行程可省略恒加速度段或巡航段。

## 调用和所有权

规划器是纯 C 模块。各轴分配自己的 `PositionOtg`，其中包括当前参考、段表及计时。
每个实例由一个执行上下文串行更新。外环在复位或切模式后用实测位置、速度和零加速度
初始化；通信入口只写最终目标，由外环拍读取。外环在实际交接拍继承旧轨迹的位置、速度、加速度。
前台预测 4 ms 后的状态并建表，外环在约定拍切换。期间旧轨迹持续推进，连续改目标时会合并中间目标；
若上一请求正在计算，最新目标最多再等一个交接窗口。撤销代际的结果丢弃，错过交接拍触发超时故障。
下列同步纯 C 接口也可独立用于离线规划。
接口见[头文件](../../firmware/motor/control/position_otg.h)。

```c
PositionOtg axis;
PositionOtgReference initial = {position_rad, speed_rad_s, 0.0f};
PositionOtgLimits limits = {maximum_speed_rad_s, maximum_acceleration_rad_s2, maximum_jerk_rad_s3};
bool ok = PositionOtg_Reset(&axis, initial);
/* 每个外环周期；target_rad 可随时更新，相同目标及限值复用段表。 */
ok = ok && PositionOtg_SetTarget(&axis, target_rad, limits);
ok = ok && PositionOtg_Sample(&axis, period_s);
```

成功采样后读取 `axis.reference` 的位置、速度、加速度及 `axis.finished`。
设目标调用本身不推进时间；非法目标或当前状态无法满足新限值时保留原计划并返回失败。
固件对失败使用原有外环故障路径。运动中反向或目标落在最短制动距离内时，参考会先沿原方向
制动，再折返。外环同时检查最终目标、实测位置和逐拍参考是否在轴允许范围内。

## 限值和控制输出

- 规划速度上限取活动位置最大速度和已配置轴限速中的较小值。位置反馈修正后的速度指令也受该上限约束。
- 对称加速度上限取活动位置加速度、减速度中的较小值，单位 rad/s²。
- 固件按 `Jmax = Amax / POSITION_OTG_ACCEL_RISE_TIME_S` 得到 jerk 上限，单位 rad/s³。
  首版取 0.1 s。独立 C 接口允许调用方直接给出三项上限。
- 规划速度加上位置误差的比例修正，减去原速度阻尼项，限速后送速度 PI；电流继续受原饱和与抗积分饱和逻辑约束。
  轨迹结束、实际位置进入窗口、滤波速度足够低时才报告到位。轨迹约束作用于指令，实际电机响应由反馈和负载决定。

Type 117 启动，Type 116 更新同模式目标。108/124 中的 `reference_position` 是当前规划参考，
`actual_position` 是实测位置。见[协议](../protocols/README.md#control)。

## 离线验证

运行 `tests/unit/native/run_position_servo_tests.py`。其中 `position_otg_test.c`
覆盖非零加速度下反向改目标、逐拍位移和速度积分误差、速度/加速度/jerk 上限、终点、重复更新、
失效输入及独立实例交错调用。实机跟随误差和重规划最坏周期数需单独测量。


## 手动位置测试与 RTT

运行 `tools/bench/position_console.py --bench <本机配置.json>`。配置需包含节点地址
`node`、12 字节十六进制 `uid`、实际输出零点对应的电机位置 `zero_motor_mrad`、
减速比 `ratio`、允许输出角度 `max_output_deg` 和 CAN FD SDK 路径 `can_dll`。
控制台先读取 UID 和状态。用 `status` 核对当前角度，IDLE 且无故障时用
`start` 保持当前位置，`move 80` 表示输出侧的绝对 +80°，可在运动中再次
`move` 改目标。用 `stop` 确认停机，再用 `quit` 退出。正式 APP 保持最后受理的目标。

RTT 上行通道 2 在 2 kHz 监督拍输出 12 个有符号 16 位数。data0/1/2 为目标、
规划、实测位置，原点取本次进入位置模式的电机位置，单位 2 mrad。data3
为规划减实测位置，单位 0.1 mrad。data4/5/6 为规划速度、速度 PI 指令与反馈，
单位 mrad/s。data7 为规划加速度，单位 mrad/s²。data8/9 为 Iq 指令与实测值，
单位 mA。data10 为未启用 (0)、跟随 (1)、已到位 (2)。data11 为采样序号，
按无符号 16 位差值检查丢帧。位置/速度按电机侧计，换算输出角度时除以减速比。
