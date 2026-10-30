/**
 * @file can.c
 * @brief STM32G4 CAN初始化、寄存器收发、启动及接收中断。
 */
#include "../../bsp_internal.h"
#include "bsp_can.h"
#include "board_config.h"
#include <string.h>
#include "fdcan.h"

#define MCU_CAN_ELEMENT_WORDS 18U
#define MCU_CAN_TX_BUFFERS 3U

#if BSP_ENABLED_CAN
static const uint8_t s_dlc_lengths[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

void McuCan_Init(void)
{
    MX_FDCAN1_Init();
    FDCAN1->RXGFC = (FDCAN1->RXGFC &
                     ~(FDCAN_RXGFC_ANFS | FDCAN_RXGFC_ANFE | FDCAN_RXGFC_RRFS | FDCAN_RXGFC_RRFE)) |
                    (FDCAN_REJECT << FDCAN_RXGFC_ANFS_Pos) | FDCAN_RXGFC_RRFS | FDCAN_RXGFC_RRFE;
    /* 两板统一只允许一帧在途，FIFO硬件指针用于识别最近提交的槽。 */
    FDCAN1->TXBC &= ~FDCAN_TXBC_TFQM;
    FDCAN1->IE = 0U;
    FDCAN1->ILE = 0U;
    FDCAN1->ILS = 0U;
}

void BspCan_Start(void)
{
    FDCAN1->CCCR &= ~FDCAN_CCCR_INIT;
    FDCAN1->IE = FDCAN_IE_RF0NE;
    FDCAN1->ILE = FDCAN_ILE_EINT0;
}

bool BspCan_TrySend(const BspCanFrame *frame)
{
    if (BspCan_IsTransmitBusy())
    {
        return false;
    }
    uint8_t dlc = 0U;
    while (s_dlc_lengths[dlc] < frame->length)
    {
        ++dlc;
    }
    uint32_t index = (FDCAN1->TXFQS & FDCAN_TXFQS_TFQPI) >> FDCAN_TXFQS_TFQPI_Pos;
    volatile uint32_t *message =
        (volatile uint32_t *)(uintptr_t)hfdcan1.msgRam.TxFIFOQSA + index * MCU_CAN_ELEMENT_WORDS;
    message[0] = (frame->extended ? (1UL << 30U) | frame->identifier : frame->identifier << 18U) |
                 (frame->remote ? 1UL << 29U : 0U);
    message[1] = ((uint32_t)dlc << 16U) | (frame->fd ? 1UL << 21U : 0U) |
                 (frame->bitrate_switch ? 1UL << 20U : 0U);
    volatile uint32_t *data = message + 2U;
    for (uint8_t offset = 0U; offset < frame->length; offset += 4U)
    {
        uint32_t word = 0U;
        uint8_t count = frame->length - offset;
        if (count > 4U)
        {
            count = 4U;
        }
        memcpy(&word, frame->data + offset, count);
        data[offset / 4U] = word;
    }
    FDCAN1->TXBAR = 1UL << index;
    return true;
}

bool BspCan_Receive(BspCanFrame *frame)
{
    uint32_t fifo = FDCAN1->RXF0S;
    if ((fifo & FDCAN_RXF0S_F0FL) == 0U)
    {
        return false;
    }
    uint32_t index = (fifo & FDCAN_RXF0S_F0GI) >> FDCAN_RXF0S_F0GI_Pos;
    volatile uint32_t *message =
        (volatile uint32_t *)(uintptr_t)hfdcan1.msgRam.RxFIFO0SA + index * MCU_CAN_ELEMENT_WORDS;
    uint32_t first = message[0], second = message[1];
    frame->extended = (first & (1UL << 30U)) != 0U;
    frame->identifier = frame->extended ? first & 0x1FFFFFFFU : (first >> 18U) & 0x7FFU;
    frame->remote = (first & (1UL << 29U)) != 0U;
    frame->fd = (second & (1UL << 21U)) != 0U;
    frame->bitrate_switch = (second & (1UL << 20U)) != 0U;
    uint8_t dlc = (uint8_t)((second >> 16U) & 0xFU);
    volatile uint32_t *data = message + 2U;
    frame->length = frame->fd ? s_dlc_lengths[dlc] : (dlc > 8U ? 8U : dlc);
    for (uint8_t offset = 0U; !frame->remote && offset < frame->length; offset += 4U)
    {
        uint32_t word = data[offset / 4U];
        uint8_t count = frame->length - offset;
        if (count > 4U)
        {
            count = 4U;
        }
        memcpy(frame->data + offset, &word, count);
    }
    FDCAN1->RXF0A = index;
    return true;
}

bool BspCan_IsTransmitBusy(void)
{
    return FDCAN1->TXBRP != 0U;
}

bool BspCan_IsTransmitComplete(void)
{
    /* FIFO的写指针已前进一格；前一槽就是本次单帧发送，避免软件保存句柄。 */
    uint32_t next = (FDCAN1->TXFQS & FDCAN_TXFQS_TFQPI) >> FDCAN_TXFQS_TFQPI_Pos;
    uint32_t last = next == 0U ? MCU_CAN_TX_BUFFERS - 1U : next - 1U;
    return FDCAN1->TXBRP == 0U && (FDCAN1->TXBTO & (1UL << last)) != 0U;
}

/* 生成的向量仍由HAL确认中断标志，报文读写在本MCU文件直接完成。 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *handle, uint32_t events)
{
    (void)handle;
    (void)events;
    BspApp_OnCanReceive();
}
#endif
