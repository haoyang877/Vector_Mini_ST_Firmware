# 双板CAN链路与职责 v1

更新：2026-10-30。公开契约见[bsp_can.h](../../firmware/platform/bsp/include/bsp_can.h)。

## 文件职责

两板共用公开头，DLC与字节数转换直接放在各MCU收发函数中，不再单独保留源文件或换算函数。
mcu/*/can.c直接实现BspCan_Start/TrySend/Receive及硬件忙/完成查询，没有中间转发入口。
AT32在其MCU实现中操作PTB/RX FIFO，STM32在其MCU实现中操作FDCAN消息RAM。
两板初始化和接收中断也在各自can.c；初始化声明统一在bsp_internal.h，不暴露消息RAM全局指针。
STM32保留生成的初始化和IRQ入口，HAL接收回调仅通知应用，收发不经过HAL包装。
独立can_frame_codec.c/.h及旧STM32接口副本继续保持删除。
协议CRC、地址、队列、重组及业务属于communication。

## 生命周期和收发

Bsp_Init配置硬件，应用绑定端点后调用返回void的BspCan_Start。
TrySend和Receive以bool表示硬件接受/忙和已取到帧/FIFO空，两板语义相同。
length是字节数，ID是11位标准ID或29位扩展ID；发送者保证格式和DLC对应长度合法。
BSP不加锁、不校验、不建立软件状态或票据、不处理超时/取消，也不自动恢复。

两板均一次提交一帧。AT32使用PTB；STM32将TX设为FIFO模式，使用硬件写指针定位发送槽。
初始化保留原位时序、引脚、ISO FD和扩展数据帧接收范围。
RX通知由各MCU实现进入应用；AT32轮询台架不开接收中断。

## 发送完成

IsTransmitBusy直接读硬件挂起位；IsTransmitComplete只在最近一帧已成功发送且不忙时为true。
STM32使用FIFO上一写入槽的TXBTO完成位，不保存软件句柄或完成队列。
FIFO索引语义依据[ST FDCAN说明](https://www.st.com/content/st_com/en/technical-documents/an5348-introduction-to-fdcan-peripherals-for-stm32-mcus-stmicroelectronics_asset/chunk-002-topic-an5348-implementation-can-fd.html)。

升级回复提交后，现有协议握手暂缓其它发送，实际完成后才推进交接。
两板共用该流程，沿用业务层从排队到完成的100ms期限，不在BSP重建计时器。
发送失败不会产生细分原因或自动恢复；上层CAN调用不再包含板型分支。

## 当前验证状态

按用户要求仅完成源码、文档与工程引用调整，未编译、未跑测试、未进行硬件验证。
历史票据/保护测试仍需按新接口迁移，不代表当前实现通过。
当前实现与回退副本见[CAN长度换算就地实现](../plans/active/2026-10-30-can-local-dlc-v1.md)。
