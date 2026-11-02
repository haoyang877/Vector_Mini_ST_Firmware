#include "yg_protocol_motor_adapter.h"

#include <stddef.h>

/* 停机业务结果 → 内部服务状态；只做映射，不改变业务判定或补造成功。 */
static yg_protocol_service_status_t
map_stop_result(motor_stop_result_t result, uint32_t token, yg_protocol_service_reply_t *reply)
{
    switch (result)
    {
    case MOTOR_STOP_RESULT_OK:
        reply->token = token;
        return YG_PROTOCOL_SERVICE_OK;
    case MOTOR_STOP_RESULT_ACCEPTED:
        reply->token = token;
        return YG_PROTOCOL_SERVICE_ACCEPTED;
    case MOTOR_STOP_RESULT_DENIED:
        return YG_PROTOCOL_SERVICE_DENIED;
    case MOTOR_STOP_RESULT_UNAVAILABLE:
        return YG_PROTOCOL_SERVICE_UNSUPPORTED;
    case MOTOR_STOP_RESULT_FAILED:
    default:
        return YG_PROTOCOL_SERVICE_FAILED;
    }
}

void yg_protocol_motor_adapter_handle(void *context,
                                      const yg_protocol_motor_request_t *request,
                                      yg_protocol_service_reply_t *reply)
{
    yg_protocol_motor_adapter_t *handler = context;
    motor_stop_result_t stop_result;
    uint32_t token = 0U;

    if (reply == NULL)
    {
        return;
    }
    /* 稳定初始化：任何早退路径都给出确定结果，绝不沿用调用方旧值。 */
    *reply = (yg_protocol_service_reply_t){.status = YG_PROTOCOL_SERVICE_FAILED};
    if (handler == NULL || request == NULL)
    {
        reply->status = YG_PROTOCOL_SERVICE_INVALID_ARGUMENT;
        return;
    }
    switch (request->operation)
    {
    case YG_PROTOCOL_MOTOR_STOP:
    case YG_PROTOCOL_MOTOR_DISABLE:
        if (handler->stop_service == NULL)
        {
            reply->status = YG_PROTOCOL_SERVICE_UNSUPPORTED;
            return;
        }
        stop_result = MotorStopService_RequestStop(
            handler->stop_service, request->source_node, request->sequence, &token);
        reply->status = map_stop_result(stop_result, token, reply);
        reply->detail = (uint16_t)stop_result;
        return;
    case YG_PROTOCOL_MOTOR_ENABLE:
    case YG_PROTOCOL_MOTOR_START_CONTROL:
    case YG_PROTOCOL_MOTOR_CLEAR_FAULT:
    case YG_PROTOCOL_MOTOR_SET_CONTROL:
        /* 只有APP注入的生命周期映射可以请求动作；此处不操作功率或故障对象。 */
        if (handler->lifecycle_handler == NULL)
        {
            reply->status = YG_PROTOCOL_SERVICE_UNSUPPORTED;
            return;
        }
        handler->lifecycle_handler(handler->lifecycle_context, request, reply);
        return;
    default:
        reply->status = YG_PROTOCOL_SERVICE_INVALID_ARGUMENT;
        return;
    }
}

bool yg_protocol_motor_decode_mode(uint8_t wire_mode, MotorControlMode *mode)
{
    if (mode == NULL)
    {
        return false;
    }
    switch (wire_mode)
    {
    case 1U:
        *mode = MOTOR_CONTROL_CURRENT;
        return true;
    case 2U:
        *mode = MOTOR_CONTROL_SPEED;
        return true;
    case 3U:
        *mode = MOTOR_CONTROL_POSITION;
        return true;
    case 6U:
        *mode = MOTOR_CONTROL_VQ;
        return true;
    case 9U:
        *mode = MOTOR_CONTROL_OPEN_VOLTAGE;
        return true;
    case 10U:
        *mode = MOTOR_CONTROL_OPEN_CURRENT;
        return true;
    default:
        return false;
    }
}
