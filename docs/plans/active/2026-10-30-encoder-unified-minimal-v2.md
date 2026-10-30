# 两板编码器统一最简接口 v2

> 此为阶段记录。当前硬件归属以[Encoder硬件归MCU v3](2026-10-30-encoder-hardware-boundary-v3.md)为准。

## 最新要求

用户要求AT32与STM32G4使用相同公开接口、数值和单位，板型由工程编译配置选择。
不通过公共头选择新旧接口，不在上层分散板型分支；MCU只初始化并暴露寄存器，
BSP直接操作，删除保护、软件事务状态和多余封装。用户明确先不测试或编译。
本记录取代2026-10-30-at32-encoder-minimal-v1.md中“保留STM32旧接口”的阶段方案。

## 唯一公开接口

```c
void BspEncoder_Begin(void);
uint16_t BspEncoder_ReadAngle(void);
```

公开bsp_encoder.h不含板型判断、厂商类型、状态码或不同板的头文件选择。
两板均先Begin，完成计算后ReadAngle；返回0..65535计数域表示一圈，不施加方向/零位/LUT。
MT6835使用原18位角度的高16位；TLE5012B将原15位角度乘2，保留各自分辨率。

## 实现

- bsp_encoder.c内部按工程BSP_BOARD选择真实寄存器操作。
- AT32保持SPI2模式3、原分频和三半字DMA2收发，命令A003。
- STM32使用SPI2模式1、32分频、16位传输，命令8021；Begin完成命令阶段并释放MOSI、
  启动读回，ReadAngle等待读回结束、恢复MOSI并将15位数据换算到共同计数域。
- 两个MCU目录各自的encoder_spi.c仅实现McuEncoder_Init，头文件mcu_encoder.h仅暴露硬件定义。
- STM32不再重复调用MX_SPI2_Init或通过运行期HAL句柄收发；只在初始化使用GPIO配置函数。
- 删除bsp_encoder_stm32g4.c/.h和devices下旧encoder_spi/encoder_protocol/两个器件解码包装。
- 所有工程引用同一bsp_encoder.c和本板初始化文件；旧协议源移出工程。
- encoder_feedback.c/.h删除板型条件和旧三阶段包装，统一Encoder_Update消费角度并保留数学处理。
- motor_axis.c仅替换编码器相关片段，两板统一Begin和Encoder_Update；保留其它模块并行改动。

## 语义变化

两板均不再检查CRC、传感器状态、超时、忙/所有权、初始化重入，也不缓存诊断或配置字。
硬件传输完成后直接消费角度，读取完成不代表进行过CRC或连线诊断。
应用只保留角度/速度/多圈计算及首样本业务状态；删除读取错误枚举、状态锁存、帧字、CRC字节和错误计数。
删除连续坏帧查询及相关故障触发；Encoder_IsOnline仅表示已经取得首个角度样本。
自检、标定和遥测有效位使用首样本业务状态，不再查询已删除的硬件诊断。
RTT调试帧仍为88字节，原编码器状态槽改为固定0的保留槽；不改变产品通信和参数布局。
方向、LUT、电零位、速度PLL和机械多圈算法不改。

## 当前状态与回退

实现供审阅；测试、编译、实机均未运行，不沿用原STM32验证结论。
旧编码器测试依赖被删除的接口和源文件，后续须按共同数值接口迁移。
修改前副本在outputs/encoder-unified-20261030/before/；上层诊断清理前副本在
outputs/encoder-boundary-20261030/before/。
共享文件存在其它模块并行编辑，回退必须仅恢复编码器相关差异，不能整文件覆盖其它改动。

## 源码核对记录

- firmware内无旧编码器读错枚举、连续坏帧查询或CRC诊断字段的引用。
- 七个方向、LUT、电角度、PLL、多圈相关函数与本轮清理前副本逐函数比较，函数体无变化。
- 九个Keil工程均只引用一次公共bsp_encoder.c，不再引用旧器件协议源。
- 仅做源码引用检查、工程XML检查与格式整理；未运行测试、编译或实机操作。
