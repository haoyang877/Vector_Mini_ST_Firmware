#ifndef APP_PROTOCOL_H
#define APP_PROTOCOL_H

#include "motor_lifecycle.h"
#include "yg_protocol.h"

/** @brief 前台独占的协议业务状态，端点回调绑定此实例。 */
typedef struct
{
    MotorLifecycle *lifecycle;
    ProtocolCommand last_target;
    uint32_t last_target_epoch, calibration_request_number;
} AppProtocol;

/**
 * @brief 将零初始化的协议适配器绑定到一个端点。
 * @param adapter 此端点独占的业务状态。
 * @param endpoint 已分配的协议端点。
 * @param life 对应轴的生命周期，覆盖端点使用期。
 */
void AppProtocol_Init(AppProtocol *adapter, ProtocolEndpoint *endpoint, MotorLifecycle *life);

#endif
