#include "comm_hw.h"
#include "critical_hw.h"

#include "fdcan.h"

/* CAN FD 收发端口：只做硬件帧取出和完整帧提交，不解释公司协议。 */

bool comm_hw_can_receive(CommHwCanFrame *frame)
{
    static const uint8_t lengths[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
    FDCAN_RxHeaderTypeDef header;
    CommHwCanFrame received = {0};
    if (frame == NULL ||
        HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &header, received.data) != HAL_OK)
    {
        return false;
    }
    if (header.DataLength > 15U)
    {
        return false;
    }
    received.identifier = header.Identifier;
    received.extended = header.IdType != FDCAN_STANDARD_ID;
    received.remote = header.RxFrameType != FDCAN_DATA_FRAME;
    received.length = lengths[header.DataLength];
    received.fd = header.FDFormat == FDCAN_FD_CAN;
    received.bitrate_switch = header.BitRateSwitch == FDCAN_BRS_ON;
    if (!received.extended || !received.fd || !received.bitrate_switch || received.remote)
    {
        return false;
    }
    *frame = received;
    return true;
}

bool comm_hw_can_try_send_frame(const CommHwCanFrame *frame)
{
    static const uint8_t lengths[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
    FDCAN_TxHeaderTypeDef header = {0};
    if (frame == NULL || !frame->extended || !frame->fd || frame->remote ||
        !frame->bitrate_switch || frame->identifier > 0x1FFFFFFFU)
    {
        return false;
    }
    for (uint32_t dlc = 0U; dlc < 16U; ++dlc)
    {
        if (lengths[dlc] == frame->length)
        {
            /* 本仓库 G4 HAL 使用未移位的 DLC 编码 0..15。 */
            header.DataLength = dlc;
            header.Identifier = frame->identifier;
            header.IdType = FDCAN_EXTENDED_ID;
            header.TxFrameType = FDCAN_DATA_FRAME;
            header.FDFormat = FDCAN_FD_CAN;
            header.BitRateSwitch = frame->bitrate_switch ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
            header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
            /* 后台提交不可被监督中断抢占。 */
            uint32_t state = critical_hw_enter();
            bool accepted = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, frame->data) == HAL_OK;
            critical_hw_exit(state);
            return accepted;
        }
    }
    return false;
}
