#include "bsp_internal.h"
#include "bsp_can.h"

#if BSP_ENABLED_CAN
#include "bsp_system.h"
#include "can_frame_codec.h"
#include "fdcan.h"
#include "main.h"
#include <stddef.h>
#include <string.h>

/* 本模块独占控制器状态与发送票据；IRQ只锁存/分发，不解释协议。 */
static void irq_enable_receive(void);
static bool s_rx_overflow;

/* CAN为1，ADC为0；只保护HAL控制器访问，快采样始终可抢占。 */
static uint32_t can_enter_critical(void)
{
    uint32_t state = __get_BASEPRI();
    __set_BASEPRI_MAX(1U << (8U - __NVIC_PRIO_BITS));
    __DSB();
    __ISB();
    return state;
}

static void can_exit_critical(uint32_t state)
{
    __DMB();
    __set_BASEPRI(state);
    __ISB();
}

bool McuCan_Init(void)
{
    /* 生成器HAL失败进入原Error_Handler；正常返回即表示初始化完成。 */
    MX_FDCAN1_Init();
    return true;
}
void McuCan_Start(void)
{

    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                     FDCAN_REJECT,
                                     FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_FILTER_REMOTE,
                                     FDCAN_FILTER_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }
}

bool McuCan_IsAvailable(void)
{
    return (hfdcan1.Instance->PSR & FDCAN_PSR_BO) == 0U &&
           (hfdcan1.Instance->CCCR & FDCAN_CCCR_INIT) == 0U;
}
bool McuCan_NeedsRecovery(void)
{
    return !McuCan_IsAvailable();
}
bool McuCan_Recover(void)
{
    uint32_t key = can_enter_critical();
    /* 硬件bus-off不改变HAL的BUSY状态，先Stop完成生命周期转换。 */
    bool stopped = hfdcan1.State != HAL_FDCAN_STATE_BUSY || HAL_FDCAN_Stop(&hfdcan1) == HAL_OK;
    bool recovered =
        stopped && hfdcan1.State == HAL_FDCAN_STATE_READY && HAL_FDCAN_Start(&hfdcan1) == HAL_OK;
    can_exit_critical(key);
    return recovered;
}

static void irq_enable_receive(void)
{
    if (HAL_FDCAN_ActivateNotification(
            &hfdcan1, (FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_MESSAGE_LOST), 0) !=
        HAL_OK)
    {
        Error_Handler();
    }
}
void McuCan_OnReceiveInterrupt(BspCanReceiveHandler receive_handler, uint32_t events)
{
    if ((events & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != 0U)
    {
        s_rx_overflow = true;
    }
    /* HAL已确认FIFO中断标志；只转发当前控制器的有界接收回调。 */
    if (receive_handler != 0)
    {
        receive_handler();
    }
}
void McuCan_EnableReceiveInterrupt(void)
{
    irq_enable_receive();
}

BspResult McuCan_Receive(BspCanFrame *frame)
{
    if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) == 0U)
    {
        return BSP_NOT_READY;
    }
    FDCAN_RxHeaderTypeDef header;
    BspCanFrame received = {0};
    if (frame == NULL)
    {
        return BSP_IO_ERROR;
    }
    uint32_t key = can_enter_critical();
    bool accepted =
        HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &header, received.data) == HAL_OK;
    can_exit_critical(key);
    if (!accepted)
    {
        return BSP_IO_ERROR;
    }
    if (!CanFrame_DecodeLength(header.DataLength, &received.length))
    {
        return BSP_FRAME_ERROR;
    }
    received.identifier = header.Identifier;
    received.extended = header.IdType != FDCAN_STANDARD_ID;
    received.remote = header.RxFrameType != FDCAN_DATA_FRAME;
    received.fd = header.FDFormat == FDCAN_FD_CAN;
    received.bitrate_switch = header.BitRateSwitch == FDCAN_BRS_ON;
    uint8_t dlc;
    if (!CanFrame_EncodeTx(&received, &dlc))
    {
        return BSP_FRAME_ERROR;
    }
    *frame = received;
    return BSP_OK;
}
bool McuCan_SlotIdle(uint32_t token)
{
    return (hfdcan1.Instance->TXBRP & token) == 0U;
}
bool McuCan_TxCancelled(uint32_t token)
{
    return (hfdcan1.Instance->TXBCF & token) != 0U;
}
bool McuCan_TxCompleted(uint32_t token)
{
    return (hfdcan1.Instance->TXBTO & token) != 0U;
}
void McuCan_ReadTerminal(uint32_t token, bool *matches, bool *sent)
{
    (void)token;
    *matches = false;
    *sent = false;
}
void McuCan_Abort(uint32_t token)
{
    (void)HAL_FDCAN_AbortTxRequest(&hfdcan1, token);
}
bool McuCan_Submit(const BspCanFrame *frame, uint8_t dlc, uint32_t *token)
{
    FDCAN_TxHeaderTypeDef header = {0};
    /* 本仓库 G4 HAL 使用未移位的 DLC 编码 0..15。 */
    header.DataLength = dlc;
    header.Identifier = frame->identifier;
    header.IdType = frame->extended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    header.TxFrameType = frame->remote ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
    header.FDFormat = frame->fd ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN;
    header.BitRateSwitch = frame->bitrate_switch ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    uint32_t key = can_enter_critical();
    bool accepted = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, frame->data) == HAL_OK;
    if (accepted)
    {
        *token = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&hfdcan1);
    }
    can_exit_critical(key);
    return accepted;
}

bool McuCan_TakeRxOverflow(void)
{
    bool pending = (hfdcan1.Instance->IR & FDCAN_IR_RF0L) != 0U;
    bool lost = s_rx_overflow || pending;
    s_rx_overflow = false;
    if (pending)
    {
        __HAL_FDCAN_CLEAR_FLAG(&hfdcan1, FDCAN_FLAG_RX_FIFO0_MESSAGE_LOST);
    }
    return lost;
}

BspCanState McuCan_GetState(void)
{
    uint32_t status = hfdcan1.Instance->PSR;
    if ((status & FDCAN_PSR_BO) != 0U)
    {
        return BSP_CAN_BUS_OFF;
    }
    if (!McuCan_IsAvailable() || (status & FDCAN_PSR_EP) != 0U)
    {
        return BSP_CAN_ERROR;
    }
    return (status & FDCAN_PSR_EW) != 0U ? BSP_CAN_WARNING : BSP_CAN_READY;
}
#endif
