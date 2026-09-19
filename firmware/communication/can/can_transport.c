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
    if (baudrate_last != baudrate)
    {
        comm_hw_can_set_baudrate(baudrate);
    }

    baudrate_last = baudrate;
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
