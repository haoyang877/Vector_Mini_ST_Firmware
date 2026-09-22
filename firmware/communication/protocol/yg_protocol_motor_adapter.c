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
    reply->status = YG_PROTOCOL_SERVICE_FAILED;
    reply->token = 0U;
    reply->revision = 0U;
    reply->value = 0;
    reply->detail = 0U;
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
        stop_result = MotorStopService_RequestStop(handler->stop_service, &token);
        reply->status = map_stop_result(stop_result, token, reply);
        reply->detail = (uint16_t)stop_result;
        return;
    case YG_PROTOCOL_MOTOR_ENABLE:
    case YG_PROTOCOL_MOTOR_SET_MODE:
    case YG_PROTOCOL_MOTOR_SET_TARGET:
        /* 本批不提供使能/模式/目标通道，避免通信侧越权启动；缺口见计划文档。 */
        reply->status = YG_PROTOCOL_SERVICE_UNSUPPORTED;
        return;
    default:
        reply->status = YG_PROTOCOL_SERVICE_INVALID_ARGUMENT;
        return;
    }
}
