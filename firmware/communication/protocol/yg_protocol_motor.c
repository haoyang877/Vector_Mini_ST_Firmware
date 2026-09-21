/* motor 服务边界：显式注入后端，不读取全局状态或访问硬件。 */
#include "yg_protocol_motor.h"

yg_protocol_service_status_t yg_protocol_motor_call(const yg_protocol_motor_service_t *service,
                                                    const yg_protocol_motor_request_t *request,
                                                    yg_protocol_service_reply_t *reply)
{
    if (reply == NULL)
    {
        return YG_PROTOCOL_SERVICE_INVALID_ARGUMENT;
    }
    *reply = (yg_protocol_service_reply_t){0};
    reply->status = YG_PROTOCOL_SERVICE_INVALID_ARGUMENT;
    if (service == NULL || request == NULL || request->operation < YG_PROTOCOL_MOTOR_STOP ||
        request->operation > YG_PROTOCOL_MOTOR_SET_TARGET)
    {
        return reply->status;
    }
    reply->status = YG_PROTOCOL_SERVICE_UNSUPPORTED;
    if (service->handler != NULL)
    {
        /* 后端遗漏结果时不能被误认为成功。 */
        reply->status = YG_PROTOCOL_SERVICE_FAILED;
        service->handler(service->context, request, reply);
    }
    return reply->status;
}
