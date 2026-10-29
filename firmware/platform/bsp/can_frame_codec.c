#include "can_frame_codec.h"
#include <stddef.h>
/* 两板共同的物理DLC表；协议负载、填充、CRC和队列仍归协议层。 */
static const uint8_t s_lengths[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
bool CanFrame_EncodeTx(const BspCanFrame *frame, uint8_t *dlc)
{
    if (frame == NULL || dlc == NULL ||
        frame->identifier > (frame->extended ? 0x1FFFFFFFU : 0x7FFU) ||
        (frame->remote && frame->fd) ||
        (!frame->fd && (frame->bitrate_switch || frame->length > 8U)))
    {
        return false;
    }
    for (uint8_t index = 0U; index < 16U; ++index)
    {
        if (s_lengths[index] == frame->length)
        {
            *dlc = index;
            return true;
        }
    }
    return false;
}
bool CanFrame_DecodeLength(uint32_t dlc, uint8_t *length)
{
    if (dlc >= 16U || length == NULL)
    {
        return false;
    }
    *length = s_lengths[dlc];
    return true;
}
