/* G431固定消息RAM：28标准/8扩展过滤器、各3个RX和TX；无APP队列和协议会话。 */
#include "can_port.h"
#include "platform.h"
#include "board.h"
#include "stm32g4xx.h"
#include <string.h>
#define RX0_OFFSET (28U * 4U + 8U * 8U)
#define TX_OFFSET (RX0_OFFSET + 6U * 72U + 3U * 8U)
#define RAM_BYTES (TX_OFFSET + 3U * 72U)
#define TX_TIMEOUT_MS 500U
static const uint8_t lengths[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
typedef struct
{
    uint64_t ticket;
    uint32_t started_ms;
    bool occupied, armed, cancelled;
} TxSlot;
static TxSlot slots[3];
static LoaderCanFrame ring[8];
static volatile uint8_t head, tail;
static volatile bool running, rx_deferred;
static bool initialized_once;
static uint8_t address;

static void fail_link(void)
{
    running = false;
    FDCAN1->IE = 0;
    FDCAN1->CCCR |= FDCAN_CCCR_INIT;
    FDCAN1->TXBCR = 7U;
}
static bool await_cccr(uint32_t mask, uint32_t value)
{
    uint32_t start_ms = LoaderPlatform_NowMs(), remaining = 4000000U;
    while ((FDCAN1->CCCR & mask) != value)
    {
        if ((uint32_t)(LoaderPlatform_NowMs() - start_ms) >= 10U || --remaining == 0)
        {
            return false;
        }
    }
    return true;
}
bool LoaderCan_Ready(void)
{
    return running && !(FDCAN1->PSR & FDCAN_PSR_BO) && !(FDCAN1->CCCR & FDCAN_CCCR_INIT);
}
bool LoaderCan_Init(uint8_t node)
{
    volatile uint32_t *ram = (volatile uint32_t *)SRAMCAN_BASE;
    uint32_t i;
    if (initialized_once || node == 255)
    {
        return false;
    }
    initialized_once = true;
    if (!LoaderBoard_CanInit())
    {
        return false;
    }
    NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
    FDCAN1->IE = 0;
    FDCAN1->ILE = 0;
    FDCAN1->CCCR &= ~FDCAN_CCCR_CSR;
    if (!await_cccr(FDCAN_CCCR_CSA, 0))
    {
        return false;
    }
    FDCAN1->CCCR |= FDCAN_CCCR_INIT;
    if (!await_cccr(FDCAN_CCCR_INIT, FDCAN_CCCR_INIT))
    {
        return false;
    }
    FDCAN1->CCCR |= FDCAN_CCCR_CCE;
    FDCAN_CONFIG->CKDIV = 0;
    FDCAN1->CCCR = (FDCAN1->CCCR & ~(FDCAN_CCCR_DAR | FDCAN_CCCR_TXP | FDCAN_CCCR_TEST |
                                     FDCAN_CCCR_MON | FDCAN_CCCR_ASM)) |
                   FDCAN_CCCR_FDOE | FDCAN_CCCR_BRSE | FDCAN_CCCR_PXHD;
    FDCAN1->TEST &= ~FDCAN_TEST_LBCK;
    FDCAN1->NBTP = ((YG_CAN_SJW - 1U) << FDCAN_NBTP_NSJW_Pos) |
                   ((YG_CAN_SEG1 - 1U) << FDCAN_NBTP_NTSEG1_Pos) |
                   ((YG_CAN_SEG2 - 1U) << FDCAN_NBTP_NTSEG2_Pos) |
                   ((YG_CAN_PRESCALER - 1U) << FDCAN_NBTP_NBRP_Pos);
    FDCAN1->DBTP = ((YG_CAN_SJW - 1U) << FDCAN_DBTP_DSJW_Pos) |
                   ((YG_CAN_SEG1 - 1U) << FDCAN_DBTP_DTSEG1_Pos) |
                   ((YG_CAN_SEG2 - 1U) << FDCAN_DBTP_DTSEG2_Pos) |
                   ((YG_CAN_DATA_PRESCALER - 1U) << FDCAN_DBTP_DBRP_Pos);
    FDCAN1->TXBC = FDCAN_TXBC_TFQM;
    for (i = 0; i < RAM_BYTES / 4; ++i)
    {
        ram[i] = 0;
    }
    /* 扩展经典掩码过滤：PF=EF/R=DP=0，匹配目的节点，源和优先级留给通信层。 */
    ram[28] = (1U << 29) | (0xEFU << 16) | ((uint32_t)node << 8);
    ram[29] = (2U << 30) | 0x03FFFF00U;
    ram[30] = (1U << 29) | (0xEFU << 16) | (255U << 8);
    ram[31] = (2U << 30) | 0x03FFFF00U;
    FDCAN1->RXGFC = (2U << FDCAN_RXGFC_LSE_Pos) | (2U << FDCAN_RXGFC_ANFS_Pos) |
                    (2U << FDCAN_RXGFC_ANFE_Pos) | FDCAN_RXGFC_RRFS | FDCAN_RXGFC_RRFE;
    address = node;
    head = tail = 0;
    rx_deferred = false;
    memset(slots, 0, sizeof(slots));
    FDCAN1->IR = UINT32_MAX;
    FDCAN1->ILS = 0;
    FDCAN1->CCCR &= ~FDCAN_CCCR_INIT;
    if (!await_cccr(FDCAN_CCCR_INIT, 0))
    {
        return false;
    }
    running = true;
    FDCAN1->IE = FDCAN_IE_RF0NE | FDCAN_IE_RF0LE | FDCAN_IE_BOE;
    FDCAN1->ILE = FDCAN_ILE_EINT0;
    NVIC_ClearPendingIRQ(FDCAN1_IT0_IRQn);
    NVIC_SetPriority(FDCAN1_IT0_IRQn, 1);
    NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
    return true;
}
void FDCAN1_IT0_IRQHandler(void)
{
    uint32_t pending = FDCAN1->IR & FDCAN1->IE, index, h0, h1, i, budget = 3;
    uint8_t next, bytes;
    volatile uint32_t *rx;
    LoaderCanFrame *frame;
    FDCAN1->IR = pending;
    if ((pending & FDCAN_IR_RF0L) || (FDCAN1->PSR & FDCAN_PSR_BO))
    {
        fail_link();
        return;
    }
    while (running && budget-- && (FDCAN1->RXF0S & FDCAN_RXF0S_F0FL))
    {
        index = (FDCAN1->RXF0S & FDCAN_RXF0S_F0GI) >> FDCAN_RXF0S_F0GI_Pos;
        if (index >= 3)
        {
            fail_link();
            break;
        }
        rx = (volatile uint32_t *)(SRAMCAN_BASE + RX0_OFFSET + index * 72U);
        h0 = rx[0];
        h1 = rx[1];
        bytes = lengths[(h1 >> 16) & 15U];
        next = (uint8_t)((head + 1U) & 7U);
        if ((h0 & (1U << 30)) && !(h0 & (1U << 29)) && (h1 & (3U << 20)) == (3U << 20) &&
            bytes >= 20 && ((uint8_t)(h0 >> 8) == address || (uint8_t)(h0 >> 8) == 255))
        {
            if (next == tail)
            {
                fail_link();
                break;
            }
            frame = &ring[head];
            frame->identifier = h0 & 0x1FFFFFFFU;
            frame->bytes = bytes;
            for (i = 0; i < bytes; ++i)
            {
                frame->data[i] = (uint8_t)(rx[2 + i / 4] >> ((i % 4) * 8));
            }
            __DMB();
            head = next;
        }
        FDCAN1->RXF0A = index;
    }
    if (running && (FDCAN1->RXF0S & FDCAN_RXF0S_F0FL))
    {
        FDCAN1->IE &= ~FDCAN_IE_RF0NE;
        rx_deferred = true;
    }
}
bool LoaderCan_Receive(LoaderCanFrame *out)
{
    bool popped = false;
    uint32_t key;
    if (out == NULL || !LoaderCan_Ready())
    {
        return false;
    }
    if (tail != head)
    {
        __DMB();
        *out = ring[tail];
        __DMB();
        tail = (uint8_t)((tail + 1U) & 7U);
        popped = true;
    }
    key = LoaderPlatform_Lock();
    if (running && rx_deferred)
    {
        rx_deferred = false;
        FDCAN1->IE |= FDCAN_IE_RF0NE;
        if (FDCAN1->RXF0S & FDCAN_RXF0S_F0FL)
        {
            NVIC_SetPendingIRQ(FDCAN1_IT0_IRQn);
        }
    }
    LoaderPlatform_Unlock(key);
    return popped;
}
bool LoaderCan_Send(const LoaderCanFrame *frame, uint64_t ticket)
{
    uint32_t key, index, i, word;
    uint8_t dlc = 11;
    volatile uint32_t *tx;
    TxSlot *slot;
    bool sent = false;
    if (frame == NULL || frame->bytes < 20 || frame->bytes > 64 || frame->identifier > 0x1FFFFFFFU)
    {
        return false;
    }
    while (dlc < 15 && lengths[dlc] < frame->bytes)
    {
        ++dlc;
    }
    if (lengths[dlc] != frame->bytes)
    {
        return false;
    }
    key = LoaderPlatform_Lock();
    index = (FDCAN1->TXFQS & FDCAN_TXFQS_TFQPI) >> FDCAN_TXFQS_TFQPI_Pos;
    if (LoaderCan_Ready() && index < 3 && !(FDCAN1->TXFQS & FDCAN_TXFQS_TFQF) &&
        !(FDCAN1->TXBRP & (1U << index)) && !slots[index].occupied)
    {
        slot = &slots[index];
        slot->ticket = ticket;
        slot->started_ms = LoaderPlatform_NowMs();
        slot->occupied = true;
        slot->armed = false;
        slot->cancelled = false;
        tx = (volatile uint32_t *)(SRAMCAN_BASE + TX_OFFSET + index * 72U);
        tx[0] = frame->identifier | (1U << 30);
        tx[1] = ((uint32_t)dlc << 16) | (3U << 20);
        for (i = 0; i < frame->bytes; i += 4)
        {
            word = (uint32_t)frame->data[i] | ((uint32_t)frame->data[i + 1] << 8) |
                   ((uint32_t)frame->data[i + 2] << 16) | ((uint32_t)frame->data[i + 3] << 24);
            tx[2 + i / 4] = word;
        }
        __DMB();
        FDCAN1->TXBAR = 1U << index;
        __DSB();
        /* 必须观察本次TXBAR清掉旧完成位；极快完成未见清零时宁可超时，不误认旧帧。 */
        slot->armed = ((FDCAN1->TXBTO | FDCAN1->TXBCF) & (1U << index)) == 0;
        sent = true;
    }
    LoaderPlatform_Unlock(key);
    return sent;
}
bool LoaderCan_TxEvent(LoaderCanTxEvent *out)
{
    uint32_t key, pending, done, cancelled, bit, now_ms;
    unsigned i;
    bool found = false;
    if (out == NULL)
    {
        return false;
    }
    key = LoaderPlatform_Lock();
    if (!LoaderCan_Ready())
    {
        fail_link();
    }
    now_ms = LoaderPlatform_NowMs();
    pending = FDCAN1->TXBRP;
    done = FDCAN1->TXBTO;
    cancelled = FDCAN1->TXBCF;
    for (i = 0; i < 3; ++i)
    {
        if (!slots[i].occupied)
        {
            continue;
        }
        bit = 1U << i;
        if (!((done | cancelled) & bit))
        {
            slots[i].armed = true;
        }
        if ((uint32_t)(now_ms - slots[i].started_ms) >= TX_TIMEOUT_MS)
        {
            slots[i].cancelled = true;
            fail_link();
        }
        if (!running || (!(pending & bit) && ((done | cancelled) & bit) && slots[i].armed))
        {
            out->ticket = slots[i].ticket;
            out->success = running && !slots[i].cancelled && (done & bit) && !(cancelled & bit);
            slots[i].occupied = false;
            found = true;
            break;
        }
    }
    LoaderPlatform_Unlock(key);
    return found;
}
void LoaderCan_Shutdown(void)
{
    fail_link();
    FDCAN1->ILE = 0;
    NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
    (void)await_cccr(FDCAN_CCCR_INIT, FDCAN_CCCR_INIT);
    NVIC_ClearPendingIRQ(FDCAN1_IT0_IRQn);
}
