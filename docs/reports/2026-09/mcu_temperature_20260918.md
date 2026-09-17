# MCU内部温度替代未接NTC：静止实测（2026-09-18）

用户确认本电机未安装NTC。基于齿槽功能提交ff0a2b3，将当前温度输入改为MCU芯片内部传感器。保留node1、reverse0、电零位15611、2 mΩ、无阻尼器配置和齿槽表；本次没有下发运动命令。

## 实现

ADC1的软件触发两rank采样TS和VREFINT，1 kHz监督调用；工厂30/130°C校准、实际VDDA修正和显示低通。采样640.5周期，初始化等待1 ms。原始芯片温度90°C触发故障10；无效或停止更新100 ms触发故障17，不覆盖已存在故障。上位机温度字段应标识为MCU温度，不表示绕组/MOS温度。详细设计、阈值依据及来源见[指南](../../guides/mcu_temperature.md)。

## 静止回读

- J-Link SN602722271；MCU UID 5700460010504e4831303320；USB-CAN USBCANFD202609107432/channel0。
- 工厂TS_CAL1=1040、TS_CAL2=1384、VREFINT_CAL=1654。
- ADC1 JSQR=0x00092001：rank1 IN16、rank2 IN18、软件触发、2次转换。SMPR2=0x071c0000：两个内部通道均640.5周期。
- 20组SWD/CAN交叉回读：芯片温度39.609～39.674°C，均值39.640°C；VDDA均值3316.96 mV。
- 代表性原始值ADC_TS=971，ADC_VREFINT=1496，重算温度39.754°C；未滤波和滤波读数不应强制相等。
- 20组GET温度与SWD差值均小于1°C；来源查询0x16E=1，有效查询0x16F=1。
- 收到21个48字节状态帧，offset32温度39.57～39.70°C，与单独查询一致；测试后状态流关闭。
- 所有有效采样missed_ms=0，样本计数持续增长，未发生故障。静止实物没有加热到保护阈值；失效、陈旧及过温分支使用生产代码原生回归验证，不能将此记录写为实物热保护验收。

原始数据、CAN帧、工厂常数和最终状态：`outputs/mcu_temperature_20260918/temperature_readback.json`；脚本`read_temperature.py`。SWD多字段读取可能跨1 ms更新，单组原始值与派生值允许一个采样的细小差异。

## 构建、保存与最终状态

普通/HIL均0错误0警告，RAM32040/32296字节。40项单元、36项集成、15组原生测试通过，另跑生产CAN来源/有效性/波特率回复回归通过。

APP更新前备份Flash并校验原镜像；更新后0x0801c000开始16 KiB参数区逐字节不变。4304字节schema11及标定数据保持。最终mode0、fault0、三相PWM关闭、CPU运行、Iq/Id0，current_limit6 A/speed_limit20 rad/s，J-Link与USB-CAN已关闭并交回协调任务。

HEX SHA256：`90f8b3b00ea60a59cdba829acb4833843024b5254d3cac8ffb23043897629052`。
AXF SHA256：`0b9b731b70d676f610e59d0e5c9a082812520563e61aa7f7c6547a3995ae4e9a`。
全Flash SHA256：`d127c19a1a72ff580a9910010637c68ed2adbbe2f05c4240ced67bfd2146226e`。

镜像、符号、map、匹配scope位于`outputs/mcu_temperature_20260918/firmware/`，RTT=0x20006a0c。该镜像替代齿槽报告中的旧APP，但不更改齿槽功能。后续任务应使用本提交的源码及配套符号；以上outputs为本机实测工件，不纳入Git。
