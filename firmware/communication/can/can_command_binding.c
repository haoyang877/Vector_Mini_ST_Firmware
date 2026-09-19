#include "can_command_binding.h"

#include <limits.h>
#include <math.h>
#include "utils.h"
#include "can_motor_status.h"
#include "can_parameter_format.h"
#include "can_transport.h"
#include "can_binding_commands.h"
#include "can_binding_queries.h"
#include "foc_cogging_calibration.h"

/* 命令入口实现：本文件只做取帧、帧级校验、线路解码与路由。
 * 具体状态更新与应答取值分别落在 can_binding_commands / can_binding_queries。 */

void CAN_ReceiveMessage_Update(CAN_PARAM_ID param_id, float data)
{
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
    if (param_id == CAN_GET_COGGING_POINT &&
        (!isfinite(data) || data < 0.0f || data >= (float)COGGING_MAP_POINTS ||
         floorf(data) != data))
        return;
    if (!isfinite(data))
        return;

    int data_int = (int)data;

    /* 写路径与读路径的参数 ID 集合互不相交，先后顺序不影响结果。 */
    CanBinding_ApplyCommand(param_id, data, data_int);
    CanBinding_ApplyQuery(param_id, data, data_int);
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

        CAN_ReceiveMessage_Update(CANMsg.rx_param_id, CANMsg.rx_data);
    }
}
