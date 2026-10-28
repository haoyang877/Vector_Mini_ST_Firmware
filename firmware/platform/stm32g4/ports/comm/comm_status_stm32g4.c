#include "comm_hw.h"
#include "fdcan.h"

/* 端口拥有已成功安装的过滤器节点，重新初始化不依赖业务全局变量。 */
static uint8_t can_configured_node;

void comm_hw_can_start(uint8_t node)
{
    hfdcan1.Init.FrameFormat = VECTOR_CAN_CLASSIC_ONLY ? FDCAN_FRAME_CLASSIC : FDCAN_FRAME_FD_BRS;
    if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }
    if (!comm_hw_can_set_node_filter(node))
    {
        Error_Handler();
    }
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
        FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, (FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_MESSAGE_LOST), 0) != HAL_OK)
    {
        Error_Handler();
    }
}

void comm_hw_can_set_bitrate(uint32_t bitrate_kbps)
{
    if ((bitrate_kbps != 100U && bitrate_kbps != 125U && bitrate_kbps != 200U &&
         bitrate_kbps != 250U && bitrate_kbps != 500U && bitrate_kbps != 1000U &&
         bitrate_kbps != 2000U && bitrate_kbps != 2500U && bitrate_kbps != 5000U) ||
        (VECTOR_CAN_CLASSIC_ONLY && bitrate_kbps > 1000U)) { return; }
    if (HAL_FDCAN_Stop(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }
    hfdcan1.Init.DataPrescaler = 10000U / bitrate_kbps;
    hfdcan1.Init.NominalPrescaler = bitrate_kbps <= 1000U ?
        hfdcan1.Init.DataPrescaler : 10U;
    /* HAL 重初始化后重新应用部署格式、节点过滤器及通知。 */
    comm_hw_can_start(can_configured_node);
}

/**
 * @brief 非阻塞取出完整帧，并保留请求的 FD/BRS 格式用于回复。
 * @param[out] frame 调用者拥有的完整帧缓冲区；NULL 时不读取硬件。
 * @return 取帧成功且 DLC 合法返回 true；无帧、HAL 错误或非法 DLC 返回 false。
 * @note 仅消费 FIFO0 中一帧，不修改位速率、过滤器或电机状态。
 */
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
    frame->fd = header.FDFormat == FDCAN_FD_CAN;
    frame->brs = header.BitRateSwitch == FDCAN_BRS_ON;
    frame->length = lengths[header.DataLength];
    return true;
}
bool comm_hw_can_try_send_status(uint16_t identifier, const uint8_t *data, size_t length)
{
#if VECTOR_CAN_CLASSIC_ONLY
    (void)identifier; (void)data; (void)length;
    return false;
#else

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
#endif
}

/**
 * @brief 在空发送队列中提交已冻结格式的回复，拒绝经典帧无法表达的载荷。
 * @param[in] identifier 标准 CAN ID，范围 0～0x7FF。
 * @param[in] data 至少 length 字节的只读载荷；HAL 入队时复制，不保留指针。
 * @param[in] length 允许 2、4、8 字节，FD 另允许 16 字节。
 * @param[in] fd true 使用 CAN FD，false 使用经典 CAN。
 * @param[in] brs FD 速率切换标志；经典帧必须为 false。
 * @return HAL 入队成功返回 true；参数非法、队列忙或 HAL 失败返回 false。
 * @note 不等待、不重试，不切换控制模式；维护调用者须另查 TXBTO 确认完成。
 */
bool comm_hw_can_try_send_reply_format(uint16_t identifier, const uint8_t *data,
    size_t length, bool fd, bool brs)
{
    FDCAN_TxHeaderTypeDef header = {0};
    if (identifier > 0x7FFU || data == NULL || (length != 2 && length != 4 && length != 8 && length != 16) ||
        (!fd && (brs || length > 8U)) ||
        (VECTOR_CAN_CLASSIC_ONLY && (fd || brs || length > 8U)) ||
        HAL_FDCAN_IsTxBufferMessagePending(&hfdcan1,
            FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2) != 0U)
    {
        return false;
    }
    header.Identifier = identifier;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.FDFormat = fd ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN;
    header.BitRateSwitch = brs ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
    header.DataLength = length == 2 ? FDCAN_DLC_BYTES_2 :
        (length == 4 ? FDCAN_DLC_BYTES_4 : (length == 8 ? FDCAN_DLC_BYTES_8 : FDCAN_DLC_BYTES_16));
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, data) == HAL_OK;
}

/**
 * @brief 使用历史 FD/BRS 格式提交维护回复，保持 BOOT 调用行为不变。
 * @param[in] identifier 标准 CAN ID，范围 0～0x7FF。
 * @param[in] data 至少 length 字节的载荷；函数返回后不再使用该指针。
 * @param[in] length 允许 2、4、8 或 16 字节。
 * @return 入队成功返回 true；参数非法、队列忙或 HAL 失败返回 false。
 * @note 不等待、不重试，实际完成仍由 comm_hw_can_reply_sent 确认。
 */
bool comm_hw_can_try_send_reply(uint16_t identifier, const uint8_t *data, size_t length)
{
    return comm_hw_can_try_send_reply_format(identifier, data, length, true, true);
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
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK) { return false; }
    can_configured_node = node;
    return true;
}
