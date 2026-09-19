#include "can_command_binding.h"

#include <limits.h>
#include <math.h>
#include "critical_hw.h"
#include "utils.h"
#include "can_motor_status.h"
#include "can_parameter_format.h"
#include "can_transport.h"
#include "can_binding_commands.h"
#include "can_binding_queries.h"
#include "foc_cogging_calibration.h"

/* 命令入口实现：中断只做取帧、帧级校验、线路解码与入队；2 kHz 服务排空队列并
 * 路由到写路径/读路径。具体状态更新与应答取值分别落在 can_binding_commands /
 * can_binding_queries。 */

/* 索引类查询的越界判定：中断入队前与 2 kHz 派发时共用，避免无效命令占用队列槽位。 */
static bool CanCommand_CoggingIndexInvalid(CAN_PARAM_ID param_id, float data)
{
    return param_id == CAN_GET_COGGING_POINT &&
           (!isfinite(data) || data < 0.0f || data >= (float)COGGING_MAP_POINTS ||
            floorf(data) != data);
}

void CAN_ReceiveMessage_Update(CAN_PARAM_ID param_id, float data)
{
    int data_int;
    uint32_t primask;

    /* Read-only handshake; this query never arms or changes motor settings. */
    if (param_id == CAN_GET_PROTOCOL_REVISION)
    {
        CAN_SendMessage_Update(CAN_GET_PROTOCOL_REVISION, (float)CAN_PARAMETER_FORMAT_REVISION);
        return;
    }
    /* Handle before float-to-int conversion; NaN/fraction/out-of-range commands
     * are rejected atomically without changing the previous stream setting. */
    if (param_id == CAN_SET_STATUS_STREAM)
    {
        bool accepted = CanMotorStatus_Configure(data);
        CAN_SendMessage_Update(CAN_GET_STATUS_STREAM,
                               accepted ? (float)CanMotorStatus_Rate() : -1.0f);
        return;
    }
    if (param_id == CAN_GET_STATUS_STREAM)
    {
        CAN_SendMessage_Update(CAN_GET_STATUS_STREAM, (float)CanMotorStatus_Rate());
        return;
    }
    /* Validate table indexes before the shared float-to-int conversion. */
    if (CanCommand_CoggingIndexInvalid(param_id, data))
        return;
    if (!isfinite(data))
        return;

    data_int = (int)data;

    /* 写路径在短临界区内应用：2 kHz 服务可被 20 kHz 快环抢占，临界区恢复
     * "命令对快环原子"（含成对限幅）；读路径只生成应答，不屏蔽中断。 */
    primask = critical_hw_enter();
    CanBinding_ApplyCommand(param_id, data, data_int);
    critical_hw_exit(primask);
    /* 写路径与读路径的参数 ID 集合互不相交，先后顺序不影响结果。 */
    CanBinding_ApplyQuery(param_id, data, data_int);
}

void CanCommand_ServiceRx(void)
{
    CanQueuedCommand_TypeDef command;

    while (CanTransport_PopRxCommand(&command))
    {
        CAN_ReceiveMessage_Update((CAN_PARAM_ID)command.param_id, command.data);
    }
}

/**
    * @brief  CAN Rx interrupt Handle  
              extract param id and data from mail box
 **/
void CANRxIRQHandler(void)
{
    CommHwCanFrame frame;
    uint8_t node_id;
    uint8_t param_id;
    uint32_t u32_data = 0;
    float decoded_data;
    CanValueEncoding encoding;

    if (!CanTransport_ReceiveFrame(&frame))
        return;

    /*high 3 bits*/
    node_id = frame.identifier >> 8;
    /*low 8 bits*/
    param_id = frame.identifier & 0x0FF;

    /*node id matches*/
    if (node_id == CANMsg.node_id)
    {
        encoding = CanParamWire_CommandEncoding((CAN_PARAM_ID)param_id);
        if (frame.length != CanParamWire_Length(encoding))
            return;
        if (encoding == CAN_VALUE_MILLI_I16)
        {
            int16_t value = (int16_t)(((uint16_t)frame.data[0] << 8) | frame.data[1]);
            if (value == INT16_MIN)
                return;
            decoded_data = (float)value / 1000.0f;
        }
        else
        {
            u32_data |= (uint32_t)frame.data[0] << 24;
            u32_data |= (uint32_t)frame.data[1] << 16;
            u32_data |= (uint32_t)frame.data[2] << 8;
            u32_data |= (uint32_t)frame.data[3];
            if ((encoding == CAN_VALUE_MILLI_I32 || encoding == CAN_VALUE_CENTI_I32) &&
                (int32_t)u32_data == INT32_MIN)
                return;
            if (encoding == CAN_VALUE_MILLI_I32)
                decoded_data = (float)(int32_t)u32_data / 1000.0f;
            else if (encoding == CAN_VALUE_CENTI_I32)
                decoded_data = (float)(int32_t)u32_data / 100.0f;
            else
                decoded_data = IntBitToFloat(u32_data);
        }

        CANMsg.can_rx_en = true;
        CANMsg.can_hb_count = 0;

        /* 链路恢复后的故障清除由运行状态机按恢复矩阵执行（阶段 D）。 */

        CANMsg.rx_param_id = (CAN_PARAM_ID)param_id;
        CANMsg.rx_data = decoded_data;

        /* 派发移出中断：只入队，由 2 kHz 服务统一处理（S5）；越界索引帧在这里拒绝。 */
        if (!CanCommand_CoggingIndexInvalid((CAN_PARAM_ID)param_id, decoded_data))
        {
            (void)CanTransport_PushRxCommand(param_id, decoded_data);
        }
    }
}
