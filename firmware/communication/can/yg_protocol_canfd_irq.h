#ifndef YG_PROTOCOL_CANFD_IRQ_H
#define YG_PROTOCOL_CANFD_IRQ_H

/**
 * @brief 唯一 CAN FD RX FIFO0 中断入口。
 * @note 只复制扩展 FD+BRS 帧并入协议队列，不执行 CRC、分片或电机业务。
 */
void YgProtocolCanfd_RxIrqHandler(void);

#endif
