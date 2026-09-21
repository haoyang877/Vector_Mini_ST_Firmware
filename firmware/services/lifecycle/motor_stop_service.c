#include "motor_stop_service.h"

#include <stddef.h>

/* 分配非零单调 token；回绕后从 1 重新开始，0 保留表示“无受理请求”。 */
static uint32_t next_token(motor_stop_service_t *service)
{
    uint32_t token = service->next_token;

    if (token == 0U)
    {
        token = 1U;
    }
    service->next_token = token + 1U;
    if (service->next_token == 0U)
    {
        service->next_token = 1U;
    }
    return token;
}

/* 唯一停机证据来自所有者：只有它明确回报功率输出禁止才算确认。 */
static bool owner_confirmed(const motor_stop_service_t *service)
{
    return service->port->is_power_disabled(service->owner_context);
}

bool MotorStopService_Init(motor_stop_service_t *service,
                           const motor_stop_owner_port_t *port,
                           void *owner_context)
{
    if (service == NULL)
    {
        return false;
    }
    service->port = NULL;
    service->owner_context = NULL;
    service->phase = MOTOR_STOP_PHASE_IDLE;
    service->token = 0U;
    service->next_token = 1U;
    /* 缺少请求或确认回调时不能受理停机：无法确认关断就不允许报告成功。 */
    if (port == NULL || port->request_stop == NULL || port->is_power_disabled == NULL)
    {
        return false;
    }
    service->port = port;
    service->owner_context = owner_context;
    return true;
}

motor_stop_result_t MotorStopService_RequestStop(motor_stop_service_t *service, uint32_t *token_out)
{
    motor_stop_owner_result_t owner_result;

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
    /* 幂等只对仍然关闭的输出成立；旧确认不能掩盖所有者后来重新使能。 */
    if (service->phase == MOTOR_STOP_PHASE_CONFIRMED)
    {
        if (owner_confirmed(service))
        {
            *token_out = service->token;
            return MOTOR_STOP_RESULT_OK;
        }
        service->phase = MOTOR_STOP_PHASE_IDLE;
        service->token = 0U;
    }
    if (service->phase == MOTOR_STOP_PHASE_IDLE)
    {
        owner_result = service->port->request_stop(service->owner_context);
        if (owner_result == MOTOR_STOP_OWNER_DENIED)
        {
            return MOTOR_STOP_RESULT_DENIED;
        }
        if (owner_result != MOTOR_STOP_OWNER_ACCEPTED)
        {
            /* 未知端口返回值按内部错误处理，不假定已停机。 */
            return MOTOR_STOP_RESULT_FAILED;
        }
        service->token = next_token(service);
        service->phase = MOTOR_STOP_PHASE_PENDING;
    }
    /* PENDING 或刚受理：只有所有者确认功率输出禁止才升级为 OK。 */
    if (owner_confirmed(service))
    {
        service->phase = MOTOR_STOP_PHASE_CONFIRMED;
        *token_out = service->token;
        return MOTOR_STOP_RESULT_OK;
    }
    *token_out = service->token;
    return MOTOR_STOP_RESULT_ACCEPTED;
}
