# MCU 温度检测

当前轮电机没有安装 NTC，温度字段改为 **STM32G431 芯片内部温度**。它不表示电机绕组、MOS 管或环境温度，不能替代这些部件的独立热保护。节点、编码器方向、齿槽标定表、参数 schema 11 和4304字节参数结构保持不变。

## 采样与换算

ADC1 注入序列改为软件触发的两通道：rank1=内部TS（IN16），rank2=VREFINT（IN18）。开启扫描，采样时间640.5周期，在42.5 MHz下约15.07 µs，满足TS最少5 µs；初始化等待1 ms覆盖内部传感器启动时间。配置位于CubeMX USER CODE段。生成代码中的PA3常规通道不启动、不作为温度来源。

启动阶段进行一次采样，之后 2 kHz 监督任务在 JEOS 后读取完整两通道结果、清标志并发起下一次转换。没有新增ADC1中断，也不延长ADC2电流采样或20 kHz FOC中断。一次序列约30.7 µs，在两次监督调用之间完成。

使用每颗芯片ROM中的TS_CAL1（0x1FFF75A8，30°C）、TS_CAL2（0x1FFF75CA，130°C）及VREFINT_CAL（0x1FFF75AA，3.0 V标定）。本仓库旧LL库的110°C常量有误，不使用其温度转换宏。

```
VDDA_mV = 3000 * VREFINT_CAL / ADC_VREFINT
TS_at_3V = ADC_TS * VREFINT_CAL / ADC_VREFINT
temperature_C = 30 + (TS_at_3V - TS_CAL1) * 100 / (TS_CAL2 - TS_CAL1)
```

显示采用系数 0.01 的一阶低通（2 kHz 节拍下与原 1 kHz/0.02 等效，约 50 ms 时间常数），首次有效值直接初始化；过温判定使用未滤波温度，不等待低通响应。`McuTemperature`调试结构给出原始ADC、VDDA、未滤波温度、有效标志、失效 tick 数（0.5 ms/tick）及累计有效样本数。

## 保护与兼容性

芯片温度90°C触发原有`High_Temprature`（故障10）。此阈值比数据手册最低温度等级的105°C结温上限低15°C，给校准公差、测量误差和热延迟留出余量；它是本版本保守软件阈值，尚未进行实物热箱验证，不表示整机热安全验收。

原始ADC为0/满量程、工厂常数无效、VDDA不在1.62～3.6 V或换算温度超出-40～150°C时拒绝读数并输出NaN。连续无有效样本100 ms（包括JEOS停止更新）触发新增`TemperatureSensor_Error`（故障17）。已有故障保持，不由温度覆盖；恢复有效读数不会自动清除已锁存故障。读数短时不更新可保持最近有效值，最多 99.5 ms。

CAN GET `0x43`与48字节状态帧offset32仍使用原编码，只改变来源；无效时分别返回NaN与int16哨兵0x8000。新增GET `0x6E`返回1（MCU芯片），`0x6F`返回有效状态。请求和回复均4字节大端float32，左轮ID分别0x16E/0x16F。上位机应显示“MCU温度”，来源查询不支持时应显示来源未知。

## 验证

原生测试执行生产换算函数及监督任务，覆盖30/130°C两点、供电修正、负温、无效ADC/ROM、滤波与原始温度保护、100 ms（200 tick）超时、启动无ADC以及故障保持。普通/HIL构建和全套回归通过。静止实机验证与固件校验值见[报告](../reports/2026-09/mcu_temperature_20260918.md)。

依据：[STM32G431数据手册DS12589](https://www.st.com/resource/en/datasheet/stm32g431cb.pdf)第3.18、5.3.1、5.3.23节；[ST对130°C校准点的确认](https://community.st.com/stm32-mcus-embedded-software-32/stm32g431-ts-cal2-temperature-21128)。
