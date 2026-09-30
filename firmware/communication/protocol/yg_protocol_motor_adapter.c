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

static yg_protocol_service_status_t map_control_result(MotorControlResult result)
{
    switch (result)
    {
    case MOTOR_CONTROL_ACCEPTED:
        return YG_PROTOCOL_SERVICE_ACCEPTED;
    case MOTOR_CONTROL_INVALID:
        return YG_PROTOCOL_SERVICE_INVALID_ARGUMENT;
    case MOTOR_CONTROL_DENIED:
        return YG_PROTOCOL_SERVICE_DENIED;
    case MOTOR_CONTROL_BUSY:
        return YG_PROTOCOL_SERVICE_BUSY;
    case MOTOR_CONTROL_OUT_OF_RANGE:
        return YG_PROTOCOL_SERVICE_OUT_OF_RANGE;
    case MOTOR_CONTROL_FAULT:
        return YG_PROTOCOL_SERVICE_FAULT;
    default:
        return YG_PROTOCOL_SERVICE_FAILED;
    }
}

static yg_protocol_service_status_t submit_control(yg_protocol_motor_adapter_t *adapter,
                                                   const yg_protocol_motor_request_t *request)
{
    MotorControlCommand command = {.mode = MOTOR_CONTROL_CURRENT};
    if (adapter->submit_control == NULL)
    {
        return YG_PROTOCOL_SERVICE_UNSUPPORTED;
    }
    switch (request->mode)
    {
    case 1U:
        command.mode = MOTOR_CONTROL_CURRENT;
        break;
    case 2U:
        command.mode = MOTOR_CONTROL_SPEED;
        break;
    case 3U:
        command.mode = MOTOR_CONTROL_POSITION;
        break;
    case 4U:
    case 5U:
        /* MIT 增益与低侧制动尚未验证，不映射到历史同编号模式。 */
        return YG_PROTOCOL_SERVICE_UNSUPPORTED;
    case 6U:
        command.mode = MOTOR_CONTROL_VQ;
        break;
    default:
        return YG_PROTOCOL_SERVICE_INVALID_ARGUMENT;
    }
    command.position_rad = (float)request->position_mrad * 0.001f;
    command.speed_rad_s = (float)request->speed_mrad_s * 0.001f;
    command.iq_a = (float)request->iq_mA * 0.001f;
    command.vq_v = (float)request->v_q_mV * 0.001f;
    command.source_node = request->source_node;
    command.sequence = request->sequence;
    return map_control_result(adapter->submit_control(adapter->control_context, &command));
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
        /* 只有APP注入的生命周期映射可以请求动作；此处不操作功率或故障对象。 */
        if (handler->lifecycle_handler == NULL)
        {
            reply->status = YG_PROTOCOL_SERVICE_UNSUPPORTED;
            return;
        }
        handler->lifecycle_handler(handler->lifecycle_context, request, reply);
        return;
    case YG_PROTOCOL_MOTOR_SET_CONTROL:
        reply->status = submit_control(handler, request);
        return;
    default:
        reply->status = YG_PROTOCOL_SERVICE_INVALID_ARGUMENT;
        return;
    }
}
