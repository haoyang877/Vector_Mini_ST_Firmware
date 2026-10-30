# 双板编码器链路与职责

更新：2026-10-30。公开契约见[bsp_encoder.h](../../firmware/platform/bsp/include/bsp_encoder.h)，
当前边界见[Encoder硬件归MCU v3](../plans/active/2026-10-30-encoder-hardware-boundary-v3.md)。

```c
void BspEncoder_Begin(void);
uint16_t BspEncoder_ReadAngle(void);
```

工程选择本板MCU的encoder_spi.c，直接定义以上两个函数和初始化函数。
初始化、SPI/DMA读写、片选和启停全部归MCU。BSP仅保留公开声明，没有通用逻辑或转发文件。
应用统一先Begin，再ReadAngle；方向、LUT、零位、速度和多圈计算保留在motor层。

| 项目 | AT32F456 + MT6835 | STM32G4 + TLE5012B |
| --- | --- | --- |
| 传输 | SPI2模式3、16位、32分频，DMA2三个半字 | SPI2模式1、16位、32分频；读回前释放MOSI，完成恢复 |
| 命令 | 0xA003 | 0x8021 |
| 角度 | 原18位角度的高16位 | 原15位角度乘2 |
| 对外单位 | 0..65535对应一圈 | 0..65535对应一圈 |

只等待硬件完成并直接返回角度，不检查CRC、状态或超时；没有事务所有权、配置缓存或读错枚举。
AT32仅保留DMA必需的私有收发缓冲。上层首样本标志只表示已取得角度，不提供硬件诊断。

本轮只检查源码与工程引用，未运行测试、编译或实机；旧测试需后续按统一接口迁移。
