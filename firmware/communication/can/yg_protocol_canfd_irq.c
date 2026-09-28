#include "yg_protocol_canfd_irq.h"

#include "comm_hw.h"
#include "yg_protocol_link.h"

void YgProtocolCanfd_RxIrqHandler(void)
{
    CommHwCanFrame frame;

    /* 每次最多读取 3 帧，持续输入不能把一次 ISR 拉成无界循环。 */
    for (uint8_t count = 0U; count < 3U; ++count)
    {
        if (!comm_hw_can_receive(&frame))
        {
            /* 空 FIFO 或被端口层丢弃的非 CAN FD 帧都计入本次上限；继续清理剩余槽位。 */
            continue;
        }
        /* 格式验证由 link 唯一负责；ISR 只提供帧值，不重复维护准入条件。 */
        (void)YgProtocolLink_OnRxFrame(&frame);
    }
}
