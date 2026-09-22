#include "yg_protocol_canfd_irq.h"

#include "comm_hw.h"
#include "yg_protocol_link.h"

void YgProtocolCanfd_RxIrqHandler(void)
{
    CommHwCanFrame frame;

    /* FIFO 可能在一次中断中积累多帧；每帧只做一次有界复制，业务留给后台。 */
    while (comm_hw_can_receive(&frame))
    {
        if (frame.extended && frame.fd && frame.bitrate_switch && !frame.remote)
        {
            (void)YgProtocolLink_OnRxFrame(&frame);
        }
    }
}
