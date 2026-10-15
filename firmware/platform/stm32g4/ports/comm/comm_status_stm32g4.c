#include "comm_hw.h"
#include "fdcan.h"

bool comm_hw_can_receive(CommHwCanFrame *frame)
{
    static const uint8_t lengths[16] = {0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64};
    FDCAN_RxHeaderTypeDef header;
    if (frame == NULL || HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &header, frame->data) != HAL_OK)
        return false;
    if (header.DataLength > 15U) return false;
    frame->identifier = header.Identifier;
    frame->extended = header.IdType != FDCAN_STANDARD_ID;
    frame->remote = header.RxFrameType != FDCAN_DATA_FRAME;
    frame->length = lengths[header.DataLength];
    return true;
}
bool comm_hw_can_try_send_status(uint16_t identifier, const uint8_t *data, size_t length)
{
    FDCAN_TxHeaderTypeDef header = {0};
    if (identifier > 0x7FFU || data == NULL || length != 48U) return false;
    /* This G4 HAL allocates three TX elements. In queue mode TFFL is not a
     * free-slot count (an empty queue reports zero on this target). Inspect
     * pending requests instead so telemetry only enters an empty queue.
     * Queue arbitration still lets lower-ID control replies go first. */
    if (hfdcan1.Init.TxFifoQueueMode != FDCAN_TX_QUEUE_OPERATION ||
        HAL_FDCAN_IsTxBufferMessagePending(&hfdcan1,
            FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2) != 0U) return false;
    header.Identifier = identifier;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.FDFormat = FDCAN_FD_CAN;
    header.BitRateSwitch = FDCAN_BRS_ON;
    header.DataLength = FDCAN_DLC_BYTES_48;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, data) == HAL_OK;
}

/* 维护回复只进入空队列，后续使用本次缓冲TXBTO确认真实发送。 */
bool comm_hw_can_try_send_reply(uint16_t identifier, const uint8_t *data, size_t length)
{
    FDCAN_TxHeaderTypeDef header = {0};
    if (identifier > 0x7FFU || data == NULL || (length != 2 && length != 4 && length != 8 && length != 16) ||
        HAL_FDCAN_IsTxBufferMessagePending(&hfdcan1,
            FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2) != 0U)
    {
        return false;
    }
    header.Identifier = identifier;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.FDFormat = FDCAN_FD_CAN;
    header.BitRateSwitch = FDCAN_BRS_ON;
    header.DataLength = length == 2 ? FDCAN_DLC_BYTES_2 :
        (length == 4 ? FDCAN_DLC_BYTES_4 : (length == 8 ? FDCAN_DLC_BYTES_8 : FDCAN_DLC_BYTES_16));
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, data) == HAL_OK;
}

bool comm_hw_can_reply_sent(void)
{
    uint32_t mask = hfdcan1.LatestTxFifoQRequest;
    return mask != 0U && (hfdcan1.Instance->TXBTO & mask) == mask &&
        HAL_FDCAN_IsTxBufferMessagePending(&hfdcan1, mask) == 0U;
}

bool comm_hw_can_set_node_filter(uint8_t node)
{
    FDCAN_FilterTypeDef filter = {0};
    if (node > 7U) {
        return false;
    }
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_RANGE;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = (uint32_t)node << 8;
    filter.FilterID2 = filter.FilterID1 | 0xFFU;
    /* 此 HAL 允许 READY/BUSY；标准过滤器最终只写一个 32 位 Message RAM 字。
     * 失败分支发生在写入之前，原过滤器不变，无需 Stop/Init 或全局关中断。 */
    return HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) == HAL_OK;
}
