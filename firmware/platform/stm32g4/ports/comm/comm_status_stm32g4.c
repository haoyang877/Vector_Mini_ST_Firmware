#include "comm_hw.h"
#include "critical_hw.h"

#include "fdcan.h"
#include "main.h"

/* CAN FD 收发端口：只做硬件帧取出和完整帧提交，不解释公司协议。 */

#define COMM_TRACKED_TX_TIMEOUT_MS 100U

/* 前台独占；提交和寄存器取样使用临界区，ISR不读写该所有者。
 * 完成记录消费前不复用任何TX槽；失败后保留mask直到硬件撤销结束。 */
static struct
{
    uint64_t ticket;
    uint32_t buffer_mask;
    uint32_t started_ms;
    bool failed;
} tracked_tx;

static bool controller_unavailable(void)
{
    return (hfdcan1.Instance->PSR & FDCAN_PSR_BO) != 0U ||
           (hfdcan1.Instance->CCCR & FDCAN_CCCR_INIT) != 0U;
}

bool comm_hw_can_is_healthy(void)
{
    uint32_t state = critical_hw_enter();
    bool healthy = hfdcan1.Instance != NULL && hfdcan1.State == HAL_FDCAN_STATE_BUSY;
    if (healthy)
    {
        healthy = (hfdcan1.Instance->CCCR & FDCAN_CCCR_INIT) == 0U &&
                  (hfdcan1.Instance->PSR & (FDCAN_PSR_BO | FDCAN_PSR_EP | FDCAN_PSR_EW)) == 0U;
    }
    critical_hw_exit(state);
    return healthy;
}

static bool tracked_slot_busy(void)
{
    if (tracked_tx.ticket != 0U || (hfdcan1.Instance->TXBRP & tracked_tx.buffer_mask) != 0U)
    {
        return true;
    }
    tracked_tx.buffer_mask = 0U;
    return false;
}

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

static bool try_send_frame(const CommHwCanFrame *frame, uint64_t ticket)
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
            if (tracked_slot_busy() || controller_unavailable())
            {
                critical_hw_exit(state);
                return false;
            }
            bool accepted = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, frame->data) == HAL_OK;
            if (accepted && ticket != 0U)
            {
                /* TXBAR的新请求清除该槽旧TXBTO/TXBCF；禁止复用直到本票据被消费。 */
                tracked_tx.buffer_mask = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&hfdcan1);
                tracked_tx.ticket = ticket;
                tracked_tx.started_ms = HAL_GetTick();
                tracked_tx.failed = false;
            }
            critical_hw_exit(state);
            return accepted;
        }
    }
    return false;
}

bool comm_hw_can_try_send_frame(const CommHwCanFrame *frame)
{
    return try_send_frame(frame, 0U);
}

bool comm_hw_can_try_send_tracked_frame(const CommHwCanFrame *frame, uint64_t ticket)
{
    return ticket != 0U && try_send_frame(frame, ticket);
}

bool comm_hw_can_take_tx_completion(CommHwCanTxCompletion *completion)
{
    if (completion == NULL)
    {
        return false;
    }
    uint32_t state = critical_hw_enter();
    if (tracked_tx.ticket == 0U)
    {
        critical_hw_exit(state);
        return false;
    }
    bool failed = tracked_tx.failed || controller_unavailable();
    bool sent = (hfdcan1.Instance->TXBTO & tracked_tx.buffer_mask) != 0U;
    bool cancelled = (hfdcan1.Instance->TXBCF & tracked_tx.buffer_mask) != 0U;
    bool expired = (uint32_t)(HAL_GetTick() - tracked_tx.started_ms) >= COMM_TRACKED_TX_TIMEOUT_MS;
    if (!failed && !sent && !cancelled && !expired)
    {
        critical_hw_exit(state);
        return false;
    }
    completion->ticket = tracked_tx.ticket;
    completion->success = !failed && !expired && sent;
    if (!completion->success)
    {
        (void)HAL_FDCAN_AbortTxRequest(&hfdcan1, tracked_tx.buffer_mask);
    }
    tracked_tx.ticket = 0U;
    critical_hw_exit(state);
    return true;
}

/* 恢复前先锁定失败，避免恢复后的旧ACK被误认为有效升级交接。 */
bool comm_hw_can_service_bus_off(void)
{
    uint32_t state = critical_hw_enter();
    if (!controller_unavailable())
    {
        critical_hw_exit(state);
        return false;
    }
    if (tracked_tx.ticket != 0U)
    {
        tracked_tx.failed = true;
    }
    if (tracked_tx.buffer_mask != 0U)
    {
        (void)HAL_FDCAN_AbortTxRequest(&hfdcan1, tracked_tx.buffer_mask);
    }
    critical_hw_exit(state);
    /* 硬件bus-off不改变HAL的BUSY状态，必须先Stop完成生命周期转换。 */
    if (hfdcan1.State == HAL_FDCAN_STATE_BUSY && HAL_FDCAN_Stop(&hfdcan1) != HAL_OK)
    {
        return false;
    }
    if (hfdcan1.State != HAL_FDCAN_STATE_READY || HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        return false;
    }
    return true;
}
