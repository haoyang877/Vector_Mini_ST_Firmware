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
        if (frame.extended && frame.fd && frame.bitrate_switch && !frame.remote)
        {
            (void)YgProtocolLink_OnRxFrame(&frame);
        }
    }
}
