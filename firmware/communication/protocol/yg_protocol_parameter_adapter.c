/* 参数通信适配器：只做请求/结果转换，不拥有参数状态，也不访问硬件。 */
#include "yg_protocol_parameter_adapter.h"

/* 业务结果到内部服务结果的显式映射；未实现与未就绪都不能被折叠成 SUCCESS。 */
static yg_protocol_service_status_t map_read_status(ParameterReadStatus status)
{
    switch (status)
    {
    case PARAMETER_READ_OK:
        return YG_PROTOCOL_SERVICE_OK;
    case PARAMETER_READ_UNSUPPORTED:
        return YG_PROTOCOL_SERVICE_UNSUPPORTED;
    case PARAMETER_READ_NOT_READY:
        return YG_PROTOCOL_SERVICE_BUSY;
    case PARAMETER_READ_INVALID_ARGUMENT:
        return YG_PROTOCOL_SERVICE_INVALID_ARGUMENT;
    case PARAMETER_READ_INVALID_VALUE:
    default:
        return YG_PROTOCOL_SERVICE_FAILED;
    }
}

void yg_protocol_parameter_adapter_handle(void *context,
                                          const yg_protocol_parameter_request_t *request,
                                          yg_protocol_service_reply_t *reply)
{
    const yg_protocol_parameter_adapter_t *handler = context;
    ParameterReadResult result;
    ParameterReadStatus status;

    if (reply == NULL)
    {
        return;
    }
    *reply = (yg_protocol_service_reply_t){0};
    reply->status = YG_PROTOCOL_SERVICE_INVALID_ARGUMENT;
    if (handler == NULL || request == NULL)
    {
        return;
    }
    if (request->operation != YG_PROTOCOL_PARAMETER_READ)
    {
        reply->status = YG_PROTOCOL_SERVICE_UNSUPPORTED;
        return;
    }
    status = ParameterRead_Get(&handler->source, request->parameter_id, &result);
    reply->status = map_read_status(status);
    if (status == PARAMETER_READ_OK)
    {
        /* 类型来自实际读服务，仅读绑定不宣称拥有写权限。 */
        reply->value = result.value;
        reply->revision = handler->source.config_revision;
        reply->value_type = result.value_type;
        reply->access = YG_PROTOCOL_PARAMETER_ACCESS_READ;
    }
}
