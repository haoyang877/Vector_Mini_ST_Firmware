#ifndef APP_PROTOCOL_H
#define APP_PROTOCOL_H

#include "motor_lifecycle.h"
#include "yg_protocol.h"

/** @brief 前台独占的协议业务状态，端点回调绑定此实例。 */
typedef struct
{
    MotorLifecycle *lifecycle;
    ProtocolMotorStatus status;
    ProtocolCommand last_target;
    uint32_t target_epoch, calibration_request_number, stop_token;
    bool target_valid, stop_confirmed;
} AppProtocol;

/**
 * @brief 将零初始化的协议适配器绑定到一个端点。
 * @param adapter 此端点独占的业务状态。
 * @param endpoint 已分配的协议端点。
 * @param life 对应轴的生命周期，覆盖端点使用期。
 */
void AppProtocol_Init(AppProtocol *adapter, ProtocolEndpoint *endpoint, MotorLifecycle *life);

/**
 * @brief 更新协议可见的状态、模式、故障与已有测量的单位换算。
 * @param adapter 此节点的协议适配器。
 * @note 仅前台串行调用；不执行控制、硬件操作或总线发送。
 */
void AppProtocol_Refresh(AppProtocol *adapter);
#endif
