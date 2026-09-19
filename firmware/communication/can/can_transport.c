#include "can_transport.h"
#include "can_motor_status.h"

/* 传输机制实现：只做启停编排、波特率切换、帧级过滤与发送重试。
 * 硬件细节全部在 platform/stm32g4/ports/comm；本层不持有电机或服务状态。 */

/** 当前生效的波特率设置，单位 kbps；唯一写者是 CAN_SET_CAN_BR 与初始化。 */
static uint32_t baudrate = 1000;
/** 上一次已下发硬件的波特率，用于把切换动作限制在设置真正变化时。 */
static uint32_t baudrate_last = 1000;

void CanTransport_Init(uint8_t node)
{
    comm_hw_can_start(node);
    baudrate = 1000;
}

void CAN_BaudRateSwitching(void)
{
    static uint8_t bus_off_divider = 0U;

    if (baudrate_last != baudrate)
    {
        comm_hw_can_set_baudrate(baudrate);
    }

    baudrate_last = baudrate;

    /* bus-off 自恢复：本函数由 2 kHz 监督每 200 拍调用一次（100 ms），故此处按 10 拍
     * 限频（约 1 Hz），避免总线真断时反复恢复抖动。 */
    if (++bus_off_divider >= 10U)
    {
        bus_off_divider = 0U;
        (void)comm_hw_can_service_bus_off();
    }
}

uint32_t CanTransport_Baudrate(void)
{
    return baudrate;
}

void CanTransport_SetBaudrate(uint32_t kbps)
{
    baudrate = kbps;
}

bool CanTransport_ReceiveFrame(CommHwCanFrame *frame)
{
    /* 其他节点的 48 字节状态帧可能落进节点 7 的遗留范围滤波；
     * 因此先收进完整 CAN FD 缓冲，再按长度与 ID 区间拒收，避免误判为命令。 */
    if (!comm_hw_can_receive(frame) || frame->extended || frame->remote ||
        (frame->length != 2U && frame->length != 4U) || frame->identifier > 0x7FFU ||
        (frame->identifier >= CAN_MOTOR_STATUS_ID_BASE &&
         frame->identifier < CAN_MOTOR_STATUS_ID_BASE + 8U))
    {
        return false;
    }
    return true;
}

void CanTransport_SendReply(uint32_t identifier, const uint8_t *data, uint8_t length)
{
    uint8_t send_num = 0;

    while (!comm_hw_can_try_send_reply((uint16_t)identifier, data, length))
    {
        /* blocked*/
        if (++send_num == 5)
        {
            break;
        }
    }
}

bool CanTransport_TrySendStatus(uint16_t identifier, const uint8_t *data, size_t length)
{
    return comm_hw_can_try_send_status(identifier, data, length);
}

/* CAN_RING_BEGIN
 * ---- 定长环实现：RX 单生产者/单消费者，TX 单上下文 ---- */

typedef struct
{
    volatile uint16_t head;
    volatile uint16_t tail;
    CanQueuedCommand_TypeDef items[CAN_RX_RING_CAPACITY];
} CanRxRing_TypeDef;

typedef struct
{
    volatile uint16_t head;
    volatile uint16_t tail;
    CanTxReply_TypeDef items[CAN_TX_RING_CAPACITY];
} CanTxRing_TypeDef;

static CanRxRing_TypeDef rx_ring;
static CanTxRing_TypeDef tx_ring;
static volatile uint32_t rx_drop_count;
static volatile uint32_t tx_drop_count;

bool CanTransport_PushRxCommand(uint8_t param_id, float data)
{
    if ((uint16_t)(rx_ring.head - rx_ring.tail) >= CAN_RX_RING_CAPACITY)
    {
        ++rx_drop_count;
        return false;
    }
    /* 先写负载、后推进 head：消费者只读取 head 之下的槽位，中断抢占读取端安全。 */
    rx_ring.items[rx_ring.head % CAN_RX_RING_CAPACITY].param_id = param_id;
    rx_ring.items[rx_ring.head % CAN_RX_RING_CAPACITY].data = data;
    ++rx_ring.head;
    return true;
}

bool CanTransport_PopRxCommand(CanQueuedCommand_TypeDef *command)
{
    if (command == NULL || rx_ring.tail == rx_ring.head)
    {
        return false;
    }
    *command = rx_ring.items[rx_ring.tail % CAN_RX_RING_CAPACITY];
    ++rx_ring.tail;
    return true;
}

bool CanTransport_PushTxReply(const CanTxReply_TypeDef *reply)
{
    if (reply == NULL)
    {
        return false;
    }
    if ((uint16_t)(tx_ring.head - tx_ring.tail) >= CAN_TX_RING_CAPACITY)
    {
        ++tx_drop_count;
        return false;
    }
    tx_ring.items[tx_ring.head % CAN_TX_RING_CAPACITY] = *reply;
    ++tx_ring.head;
    return true;
}

bool CanTransport_PopTxReply(CanTxReply_TypeDef *reply)
{
    if (reply == NULL || tx_ring.tail == tx_ring.head)
    {
        return false;
    }
    *reply = tx_ring.items[tx_ring.tail % CAN_TX_RING_CAPACITY];
    ++tx_ring.tail;
    return true;
}

bool CanTransport_TxPending(void)
{
    return tx_ring.tail != tx_ring.head;
}

uint32_t CanTransport_GetRxDropCount(void)
{
    return rx_drop_count;
}

uint32_t CanTransport_GetTxDropCount(void)
{
    return tx_drop_count;
}
/* CAN_RING_END */
