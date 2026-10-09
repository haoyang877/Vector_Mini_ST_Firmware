# 位置速度滤波持久化 PVF1 v1

2026-09-29。Mode3的基准速度一阶低通仍为10 Hz；`MotorControl.position_velocity_filter_half_cutoff`为0时10 Hz、为1时5 Hz，MOVE/SETTLE/HOLD共用，额外HOLD低通仍默认关闭。滤波更新频率2 kHz，5 Hz对应系数约0.01546504、时间常数31.8 ms。该配置不会改变PLL参数。

参数上传/Save_Param保存此标志，参数下载/复位加载它。预定位电流、运行限流、PI增益、轴身份与运动范围仍由原字段保存。半频标志只作用于Mode3位置闭环，不能据此推断速度模式或Mode18的滤波设置。

## 布局及黄金向量

保留schema10，原字段与magic偏移不变。在`InterfaceParam_TypeDef`尾部偏移2256追加16字节，总大小2272；记录地址为参数基址加2256。AXS1仍在2208，DMP1仍在2240。所有字段little-endian uint32，CRC32使用反射多项式0xEDB88320、初值/末异或0xFFFFFFFF，覆盖前12字节。

| 字节偏移 | 字段 | 值 |
| --- | --- | --- |
| 0 | magic | 0x31465650，即PVF1 |
| 4 | version | 1 |
| 8 | half_cutoff | 0为10 Hz，1为5 Hz |
| 12 | crc32 | 前12字节CRC |

黄金向量：[position_filter_v1.json](../../tests/vectors/position_filter_v1.json)。10 Hz为`505646310100000000000000ec28e1b1`，5 Hz为`505646310100000001000000894f5d09`。

## 兼容及故障

全零或全FF旧尾部按10 Hz加载，不为了补写尾部自动重写Flash。非空记录magic/版本/CRC错误或标志超过1时关闭参数准入，返回无迁移重写请求；不启动位置输出，不用静默10 Hz替代坏记录。恢复参数须重新提供合法记录。

旧固件忽略PVF1，回退10 Hz。旧固件Save_Param会擦除/清空新尾部，再升级后默认10 Hz，需重新保存5 Hz。CAN参数协议不新增ID，原位宽和ID保持；本轮通过停机后的J-Link配置和Flash回读确认。物理标定、零位及LUT不随PVF1改变。

首次下载需完整保留本机参数区；不得用另一台电机的标定替代。固件/参数下载后先验证整片回读、复位实际加载和STOP，再运行已授权的台架场景。滤波延迟增大可能改变运动跟随，定位噪声降低不能替代轨迹验收。
