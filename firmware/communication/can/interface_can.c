#include "interface_can.h"

#include "control_config.h"
#include "utils.h"
#include "foc_algorithm.h"
#include "foc_errhandle.h"
#include "param_comm_bridge.h"
#include "can_motor_status.h"
#include "can_command_binding.h"
#include "can_transport.h"
#include "can_status_source.h"
#include "time_hw.h"

/* CAN 门面：对外保持全部公共入口签名不变，内部只保留
 * 运行态（CANMsg）、应答暂存、心跳状态机与发送调度；
 * 传输机制在 can_transport，读/写路径分别在各 binding 文件。 */

CANMsg_TypeDef CANMsg;

extern MotorControl_TypeDef MotorControl;

/* 参数模块桥接口实现：节点身份与心跳超时的存储所有权在本文件（CANMsg），
 * 此处只做读写转发，不触发保存或总线重配置。 */
uint8_t CAN_NodeId_Get(void)
{
    return CANMsg.node_id;
}

void CAN_NodeId_Set(uint8_t node_id)
{
    CANMsg.node_id = node_id;
}

uint32_t CAN_HeartbeatMs_Get(void)
{
    return CANMsg.can_hb_set;
}

void CAN_HeartbeatMs_Set(uint32_t heartbeat_ms)
{
    CANMsg.can_hb_set = heartbeat_ms;
}

/**
    * @brief  FDCAN1 Filter Init  
              Stdandard ID, Range Mode 
 **/
void FDCAN1_Param_Init(void)
{
    CanMotorStatus_Init();

    CanTransport_Init(CANMsg.node_id);
}

/**
    * @brief  Handle CAN heartbeat disconnect protection
 **/
void CAN_DisConnect_Handle(void)
{
    /* can_hb_set 单位为 ms；监督 tick 2 kHz，按 tick 换算以保持毫秒级超时契约。 */
    uint32_t timeout_ticks = CANMsg.can_hb_set * SUPERVISOR_TICKS_PER_MS;

    /* 每个监督 tick 重算：上一轮 RUN 不得在 STOP 之后继续武装看门狗；
     * 已锁存的故障在本函数中保留不清除。 */
    CANMsg.can_hb_en = CANMsg.can_hb_set != 0U && CanStatus_HeartbeatArmed();
    if (!CANMsg.can_hb_en)
    {
        CANMsg.can_hb_count = 0U;
        return;
    }
    if (CANMsg.can_rx_en)
    {
        /* 饱和累加，避免持续断连把计数器绕回；计数粒度 0.5 ms。 */
        if (CANMsg.can_hb_count < timeout_ticks)
            ++CANMsg.can_hb_count;
        if (CANMsg.can_hb_count >= timeout_ticks && MotorControl.ErrorNow == No_Error)
            Set_ErrorNow(CAN_DisConnect);
    }
}

/**
    * @brief  只读判断控制心跳是否仍然存活
    * @retval 非 0 表示心跳未超时
 **/
bool CAN_IsHeartbeatAlive(void)
{
    return CANMsg.can_hb_set > 0U &&
           CANMsg.can_hb_count < CANMsg.can_hb_set * SUPERVISOR_TICKS_PER_MS;
}

/**
    * @brief  Update CAN transmit message data
    * @param  param_id: CAN parameter id
    * @param  data: CAN transmit data
 **/
void CAN_SendMessage_Update(CAN_PARAM_ID param_id, float data)
{
    CanTxReply_TypeDef reply;
    CanValueEncoding encoding = CanParamWire_ReplyEncoding(param_id);

    reply.param_id = (uint8_t)param_id;
    reply.length = CanParamWire_Length(encoding);
    if (encoding == CAN_VALUE_MILLI_I16)
    {
        uint16_t value = (uint16_t)CanParamWire_Milli16(data);
        reply.data[0] = (uint8_t)(value >> 8);
        reply.data[1] = (uint8_t)value;
        reply.data[2] = 0U;
        reply.data[3] = 0U;
    }
    else
    {
        uint32_t value;
        if (encoding == CAN_VALUE_MILLI_I32)
            value = (uint32_t)CanParamWire_Milli32(data);
        else if (encoding == CAN_VALUE_CENTI_I32)
            value = (uint32_t)CanParamWire_Centi32(data);
        else
            value = FloatToIntBit(data);
        reply.data[0] = (uint8_t)(value >> 24);
        reply.data[1] = (uint8_t)(value >> 16);
        reply.data[2] = (uint8_t)(value >> 8);
        reply.data[3] = (uint8_t)value;
    }
    (void)CanTransport_PushTxReply(&reply);
}

/**
    * @brief  2 kHz communication service: drain and dispatch the RX queue, then
              drain the TX queue; when idle, submit the periodic status stream.
 **/
void CAN_Service(void)
{
    CanTxReply_TypeDef reply;
    bool status_allowed;

    CanCommand_ServiceRx();
    /* 应答优先：本拍有任何待发应答（含本拍派发产生）时不提交状态流。 */
    status_allowed = !CanTransport_TxPending();
    while (CanTransport_PopTxReply(&reply))
    {
        uint32_t identifier = CanParamWire_Identifier(CANMsg.node_id, (CAN_PARAM_ID)reply.param_id);
        CanTransport_SendReply(identifier, reply.data, reply.length);
    }
    if (status_allowed)
    {
        uint8_t payload[CAN_MOTOR_STATUS_SIZE];
        uint16_t identifier;
        if (MotorStatus_IsRequested())
        {
            MotorStatus snapshot;
            CanStatus_BuildSnapshot(&snapshot);
            MotorStatus_Publish(&snapshot);
        }
        if (CanMotorStatus_Prepare(
                time_hw_now_ms(), CANMsg.node_id, &identifier, payload, sizeof(payload)))
        {
            (void)CanTransport_TrySendStatus(identifier, payload, sizeof(payload));
        }
    }
}

uint32_t CAN_GetRxDropCount(void)
{
    return CanTransport_GetRxDropCount();
}

uint32_t CAN_GetTxDropCount(void)
{
    return CanTransport_GetTxDropCount();
}
