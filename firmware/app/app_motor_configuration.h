#ifndef APP_MOTOR_CONFIGURATION_H
#define APP_MOTOR_CONFIGURATION_H
#include "motor_axis.h"
#include "yg_protocol.h"

/** @brief 前台独占的型号配置事务；pending 是待重启配置，不替代当前活动轴配置。 */
typedef struct
{
    MotorAxis *axis;
    ProtocolEndpoint *endpoint;
    MotorConfiguration pending;
    ProtocolMotorConfigurationRequest saved_request;
    uint32_t previous_revision, saved_revision, restart_at_ms, prepare_number;
    bool prepared, saved, storage_failed, restart_requested;
} AppMotorConfiguration;

/**
 * @brief 绑定零初始化事务和端点，不访问硬件。
 * @param service 本端点的配置事务，寿命覆盖应用。
 * @param axis 本机轴和独占参数记录。
 * @param endpoint 已分配的协议端点。
 */
void AppMotorConfiguration_Init(AppMotorConfiguration *service,
                                MotorAxis *axis,
                                ProtocolEndpoint *endpoint);
/**
 * @brief 在显式激活请求的回复发送完成后复位，保存失败或输出未关断时不复位。
 * @param service 前台独占的配置事务。
 * @param now_ms 当前单调毫秒计数。
 * @note 仅在 Protocol_Process 返回后调用；新型号在下一次启动装载，禁止自动使能。
 */
void AppMotorConfiguration_Process(AppMotorConfiguration *service, uint32_t now_ms);
#endif
