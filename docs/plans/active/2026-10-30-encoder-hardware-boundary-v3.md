# Encoder硬件归MCU v3

## 目标与边界

用户更新分层要求：BSP只保留通用逻辑，硬件初始化、读写、启停和中断处理放在对应MCU。
AT32与STM32G4保持相同公开接口，由工程选择MCU源文件。删除多余转发，不新增保护、
软件状态、结果码或采样缓存。用户明确本轮不测试、不编译。
本记录取代v2中“MCU仅初始化、BSP直接访问寄存器”的分工。

## 基线与实现

基线是当前工作区v2实现；其它模块正在并行修改，只修改Encoder相关差异。
- 保留`void BspEncoder_Begin(void)`和`uint16_t BspEncoder_ReadAngle(void)`。
- 各MCU的encoder_spi.c直接实现这两个公开函数，同时保留McuEncoder_Init。
- 原bsp_encoder.c没有可保留的通用算法，删除文件及工程引用，不增加转发层。
- DMA收发缓冲归AT32 encoder_spi.c私有，不再通过mcu_encoder.h向外暴露。
- 角度归一化属于器件解码：AT32取高16位，STM32将15位值乘2；统一65536计数域。
- 公共头只描述统一数值和调用顺序；方向、LUT、速度、多圈和业务状态仍在motor层。

## 保持不变

保持命令、SPI模式/分频、DMA长度、片选时序、硬件等待顺序、上层Begin/ReadAngle调用位置。
不恢复已删除的CRC、超时、读错状态、诊断/配置缓存或旧三阶段接口。
不修改通信/参数ABI，不增加动态分配或运行时驱动注册。

## 审阅与回退

检查两板MCU均直接定义相同接口，工程各引用本板encoder_spi.c一次且无bsp_encoder.c引用。
只核对源码和工程XML，不运行测试、编译或实机。旧测试仍需后续迁移。
修改前副本位于outputs/encoder-hardware-boundary-20261030/before/。
回退仅恢复Encoder差异；共享工程和文档不能整文件覆盖其它模块并行修改。

## 源码核对结果

- 两板Begin/ReadAngle函数体与搬迁前一致，仅AT32的DMA缓冲由外部符号改为文件私有。
- 九个Keil工程均保留本板encoder_spi.c一次，且仅移除bsp_encoder.c引用。
- 两个AT32链接脚本移除已删除对象；现有encoder*.o选择继续覆盖运行代码。
- 公开接口及上层调用无变化，未运行测试、编译或实机。
