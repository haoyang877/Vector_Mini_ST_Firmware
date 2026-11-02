#include "motor_stop_request.h"
#include <stddef.h>

/* 只有所有者的实际关相确认能产生OK；请求受理不能替代关闭证据。 */

bool MotorStopService_Init(motor_stop_service_t *service,
                           const motor_stop_owner_port_t *port,
                           void *owner_context)
{
    if (service == NULL)
    {
        return false;
    }
    *service = (motor_stop_service_t){0};
    if (port == NULL || port->request_stop == NULL || port->is_power_disabled == NULL)
    {
        return false;
    }
    service->port = port;
    service->owner_context = owner_context;
    return true;
}

motor_stop_result_t MotorStopService_RequestStop(motor_stop_service_t *service,
                                                 uint8_t source_node,
                                                 uint16_t sequence,
                                                 uint32_t *token_out)
{
    if (token_out == NULL)
    {
        return MOTOR_STOP_RESULT_FAILED;
    }
    *token_out = 0U;
    if (service == NULL)
    {
        return MOTOR_STOP_RESULT_FAILED;
    }
    if (service->port == NULL)
    {
        return MOTOR_STOP_RESULT_UNAVAILABLE;
    }
    if (!service->port->request_stop(service->owner_context, source_node, sequence))
    {
        return MOTOR_STOP_RESULT_DENIED;
    }
    bool disabled = service->port->is_power_disabled(service->owner_context);
    if (service->token == 0U || (service->confirmed && !disabled))
    {
        if (++service->token == 0U)
        {
            ++service->token;
        }
    }
    service->confirmed = disabled;
    *token_out = service->token;
    return disabled ? MOTOR_STOP_RESULT_OK : MOTOR_STOP_RESULT_ACCEPTED;
}
