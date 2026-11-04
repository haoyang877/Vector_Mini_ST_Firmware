# 双板编码器链路与职责

更新：2026-10-30。公开契约见[bsp_encoder.h](../../firmware/platform/bsp/include/bsp_encoder.h)，
当前边界见[Encoder硬件归MCU v3](../plans/active/2026-10-30-encoder-hardware-boundary-v3.md)。

```c
void BspEncoder_Begin(void);
uint32_t BspEncoder_ReadAngle(BspEncoderResolution resolution);
```

工程选择本板MCU的encoder_spi.c，直接定义以上两个函数和初始化函数。
初始化、SPI/DMA读写、片选和启停全部归MCU。BSP仅保留公开声明，没有通用逻辑或转发文件。
应用统一先Begin，再ReadAngle；方向、LUT、零位、速度和多圈计算保留在motor层。
读取时选择`BSP_ENCODER_16BIT`或`BSP_ENCODER_21BIT`，分别使用65536或2097152计数表示一圈。
现有控制算法显式选择16位；不能把21位结果直接截断成16位使用。

| 项目 | AT32F456 + MT6835 | STM32G4 + TLE5012B |
| --- | --- | --- |
| 传输 | SPI2模式3、16位、32分频，DMA2三个半字 | SPI2模式1、16位、32分频；读回前释放MOSI，完成恢复 |
| 命令 | 0xA003 | 0x8021 |
| 角度 | 16位返回ANGLE[20:5]；21位拼合ANGLE[20:0] | 原15位左移1位或6位，不增加真实精度 |
| 对外单位 | 由参数选择16位或21位单圈计数 | 由参数选择16位或21位单圈计数 |

只等待硬件完成并直接返回角度，不检查CRC、状态或超时；没有事务所有权、配置缓存或读错枚举。
AT32仅保留DMA必需的私有收发缓冲。上层首样本标志只表示已取得角度，不提供硬件诊断。

2026-11-03：AT32的收发缓冲使用`Mt6835TxFrame`/`Mt6835RxFrame`描述，每帧6字节。
DMA直接访问结构体地址，以三个16位字传输，不经过额外拷贝。

| 字序号 | 发送字段 | 接收字段 |
| --- | --- | --- |
| 0 | `command`：0xA，`address`：0x003 | `ignored`：命令阶段，不使用 |
| 1 | `dummy[0]`：0 | `angle_high`：ANGLE[20:5] |
| 2 | `dummy[1]`：0 | `angle_low`：[15:11]，`status`：[10:8]，`crc`：[7:0] |

命令、地址、角度低位、状态和CRC均可直接按字段访问；位域按AT32小端16位存储布局定义。
结构大小和非位域字段偏移由编译期检查约束，位域映射另用本机测试与ARMCC常量字节核对。
16位模式返回`angle_high`；21位模式返回`(angle_high << 5) | angle_low`，两种模式均不校验状态或CRC。
芯片原始角度为21位，先前18位表述更正；寄存器定义见
[MT6835手册第26、32页](https://www.magntek.com.cn/upload/pdf/202407/MT6835_Rev.1.3.pdf)。
本次验证范围见[直接字段记录](../plans/completed/2026-11-03-mt6835-direct-fields-v2.md)。

2026-11-03新增位宽选择，调用示例（每次读取均需与Begin配对）：

```c
BspEncoder_Begin();
uint32_t angle21 = BspEncoder_ReadAngle(BSP_ENCODER_21BIT);
float degrees = (float)angle21 * (360.0f / 2097152.0f);
```

该值为传感器单圈角度，未做方向、零位或LUT修正。范围及验证见
[位宽选择记录](../plans/completed/2026-11-03-encoder-output-bits-v1.md)。
