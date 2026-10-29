#include "bsp_config.h"
#include "bsp_can.h"
#include "bsp_internal.h"
#if BSP_ENABLED_CAN
#include "can_frame_codec.h"
#include "bsp_system.h"
#include <stddef.h>
/* 唯一软件票据所有者；token是后端返回的物理槽身份，不能代替64位票据。 */
#define CAN_TX_TIMEOUT_MS 100U
#define CAN_RECOVERY_INTERVAL_MS 1000U
static bool s_initialized, s_started, s_recovery_seen;
static uint32_t s_recovery_ms;
volatile uint32_t g_bsp_can_rx_overflows;
static struct
{
    uint64_t ticket;
    uint32_t token, started_ms;
#if BSP_CAN_RESERVE_UNTRACKED_SLOT
    bool active, terminal;
    BspCanTxResult result;
#else
    bool failed;
#endif
} s_tx;
#if BSP_CAN_RESERVE_UNTRACKED_SLOT
static void set_tx_terminal(BspCanTxResult result)
{
    s_tx.terminal = true;
    s_tx.result = result;
    if (result != BSP_CAN_TX_OK && !McuCan_SlotIdle(s_tx.token))
    {
        McuCan_Abort(s_tx.token);
    }
}

/* PTB在终态消费且硬件空闲后才允许重用，包括不带票据的发送。 */
static void refresh_tx(void)
{
    if (!s_tx.active)
    {
        return;
    }
    if (!s_tx.terminal && s_tx.ticket != 0U &&
        (!McuCan_IsAvailable() ||
         (uint32_t)(BspSystem_TimeMs() - s_tx.started_ms) >= CAN_TX_TIMEOUT_MS))
    {
        set_tx_terminal(!McuCan_IsAvailable() ? BSP_CAN_TX_ERROR : BSP_CAN_TX_TIMEOUT);
    }
    if (!s_tx.terminal)
    {
        bool timeout = (uint32_t)(BspSystem_TimeMs() - s_tx.started_ms) >= CAN_TX_TIMEOUT_MS;
        bool unavailable = !McuCan_IsAvailable();
        bool cancelled = McuCan_TxCancelled(s_tx.token);
        bool completed = McuCan_TxCompleted(s_tx.token);
        bool matches, sent;
        McuCan_ReadTerminal(s_tx.token, &matches, &sent);
        bool matching_terminal = McuCan_SlotIdle(s_tx.token) && matches;
        if (unavailable || timeout || cancelled || (completed && McuCan_SlotIdle(s_tx.token)) ||
            matching_terminal)
        {
            bool success = !unavailable && !timeout && !cancelled && completed &&
                           McuCan_SlotIdle(s_tx.token) && sent;
            set_tx_terminal(success       ? BSP_CAN_TX_OK
                            : unavailable ? BSP_CAN_TX_ERROR
                            : timeout     ? BSP_CAN_TX_TIMEOUT
                            : cancelled   ? BSP_CAN_TX_CANCELLED
                                          : BSP_CAN_TX_ERROR);
        }
    }
    if (s_tx.terminal && s_tx.ticket == 0U && McuCan_SlotIdle(s_tx.token))
    {
        s_tx.active = false;
    }
}
#endif
static BspResult try_send(const BspCanFrame *frame, uint64_t ticket)
{
    uint8_t dlc;
    if (!CanFrame_EncodeTx(frame, &dlc))
    {
        return BSP_INVALID_ARGUMENT;
    }
    if (!s_started)
    {
        return BSP_NOT_READY;
    }
    /* 发送状态仅由前台所有者访问；硬件互斥由MCU限定到CAN中断。 */
#if BSP_CAN_RESERVE_UNTRACKED_SLOT
    refresh_tx();
    bool busy = !McuCan_IsAvailable() || s_tx.active || !McuCan_SlotIdle(s_tx.token);
#else
    bool busy = s_tx.ticket != 0U || !McuCan_SlotIdle(s_tx.token);
    if (!busy)
    {
        s_tx.token = 0U;
        busy = !McuCan_IsAvailable();
    }
#endif
    if (busy)
    {
        return BSP_BUSY;
    }
    uint32_t token = 0U;
    if (!McuCan_Submit(frame, dlc, &token))
    {
        return BSP_IO_ERROR;
    }
    if (ticket != 0U || BSP_CAN_RESERVE_UNTRACKED_SLOT)
    {
        s_tx.token = token;
        s_tx.ticket = ticket;
        s_tx.started_ms = BspSystem_TimeMs();
#if BSP_CAN_RESERVE_UNTRACKED_SLOT
        s_tx.active = true;
        s_tx.terminal = false;
#else
        s_tx.failed = false;
#endif
    }
    return BSP_OK;
}
BspResult BspCan_TrySend(const BspCanFrame *frame)
{
    return try_send(frame, 0U);
}
BspResult BspCan_TrySendTracked(const BspCanFrame *frame, uint64_t ticket)
{
    return ticket != 0U ? try_send(frame, ticket) : BSP_INVALID_ARGUMENT;
}
BspResult BspCan_TakeCompletion(BspCanCompletion *completion)
{
    if (completion == NULL)
    {
        return BSP_INVALID_ARGUMENT;
    }
#if BSP_CAN_RESERVE_UNTRACKED_SLOT
    refresh_tx();
    if (!s_tx.active || !s_tx.terminal || s_tx.ticket == 0U)
    {

        return BSP_NOT_READY;
    }
    completion->ticket = s_tx.ticket;
    completion->result = s_tx.result;
    s_tx.ticket = 0U;
    refresh_tx();
#else
    if (s_tx.ticket == 0U)
    {

        return BSP_NOT_READY;
    }
    bool failed = s_tx.failed || !McuCan_IsAvailable();
    bool sent = McuCan_TxCompleted(s_tx.token), cancelled = McuCan_TxCancelled(s_tx.token);
    bool expired = (uint32_t)(BspSystem_TimeMs() - s_tx.started_ms) >= CAN_TX_TIMEOUT_MS;
    if (!failed && !sent && !cancelled && !expired)
    {

        return BSP_NOT_READY;
    }
    completion->ticket = s_tx.ticket;
    completion->result = failed      ? BSP_CAN_TX_ERROR
                         : expired   ? BSP_CAN_TX_TIMEOUT
                         : sent      ? BSP_CAN_TX_OK
                         : cancelled ? BSP_CAN_TX_CANCELLED
                                     : BSP_CAN_TX_ERROR;
    if (completion->result != BSP_CAN_TX_OK)
    {
        McuCan_Abort(s_tx.token);
    }
    s_tx.ticket = 0U;
#endif

    return BSP_OK;
}
BspResult BspCan_Recover(void)
{
    if (!s_started)
    {
        return BSP_NOT_READY;
    }
    uint32_t now = BspSystem_TimeMs();
    if (s_recovery_seen && (uint32_t)(now - s_recovery_ms) < CAN_RECOVERY_INTERVAL_MS)
    {
        return BSP_NOT_READY;
    }
    if (!McuCan_NeedsRecovery())
    {

        return BSP_NOT_READY;
    }
    s_recovery_seen = true;
    s_recovery_ms = now;
#if BSP_CAN_RESERVE_UNTRACKED_SLOT
    if (s_tx.active)
    {
        set_tx_terminal(BSP_CAN_TX_ERROR);
    }
#else
    if (s_tx.ticket != 0U)
    {
        s_tx.failed = true;
    }
    if (s_tx.token != 0U)
    {
        McuCan_Abort(s_tx.token);
    }
#endif
#if BSP_CAN_RECOVERY_CRITICAL
    bool recovered = McuCan_Recover();

    return recovered ? BSP_OK : BSP_IO_ERROR;
#else

    return McuCan_Recover() ? BSP_OK : BSP_IO_ERROR;
#endif
}

BspCanState BspCan_GetState(void)
{
    if (!s_started)
    {
        return BSP_CAN_DISABLED;
    }
    BspCanState state = McuCan_GetState();

    return state;
}

bool Can_Init(void)
{
    if (s_initialized)
    {
        return false;
    }
    s_initialized = McuCan_Init();
    return s_initialized;
}

BspResult BspCan_Start(void)
{
    if (!s_initialized || s_started)
    {
        return BSP_NOT_READY;
    }
    McuCan_Start();
    if (!McuCan_IsAvailable())
    {
        return BSP_IO_ERROR;
    }
    s_started = true;
    McuCan_EnableReceiveInterrupt();
    return BSP_OK;
}

BspResult BspCan_Receive(BspCanFrame *frame)
{
    if (frame == NULL)
    {
        return BSP_INVALID_ARGUMENT;
    }
    if (!s_started)
    {
        return BSP_NOT_READY;
    }
    if (McuCan_TakeRxOverflow())
    {
        ++g_bsp_can_rx_overflows;
        return BSP_OVERFLOW;
    }
    return McuCan_Receive(frame);
}

void Can_OnReceiveInterrupt(BspCanReceiveHandler handler, uint32_t events)
{
    McuCan_OnReceiveInterrupt(handler, events);
}
#else
bool Can_Init(void)
{
    return true;
}
BspResult BspCan_Start(void)
{
    return BSP_NOT_SUPPORTED;
}
BspResult BspCan_TrySend(const BspCanFrame *frame)
{
    (void)frame;
    return BSP_NOT_SUPPORTED;
}
BspResult BspCan_TrySendTracked(const BspCanFrame *frame, uint64_t ticket)
{
    (void)frame;
    (void)ticket;
    return BSP_NOT_SUPPORTED;
}
BspResult BspCan_Receive(BspCanFrame *frame)
{
    (void)frame;
    return BSP_NOT_SUPPORTED;
}
BspResult BspCan_TakeCompletion(BspCanCompletion *completion)
{
    (void)completion;
    return BSP_NOT_SUPPORTED;
}
BspCanState BspCan_GetState(void)
{
    return BSP_CAN_DISABLED;
}
BspResult BspCan_Recover(void)
{
    return BSP_NOT_SUPPORTED;
}
void Can_OnReceiveInterrupt(BspCanReceiveHandler handler, uint32_t events)
{
    (void)handler;
    (void)events;
}
#endif
