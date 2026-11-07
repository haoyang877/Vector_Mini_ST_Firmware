#ifndef APP_CONTEXT_H
#define APP_CONTEXT_H
#include "motor_axis.h"
#include "app_protocol.h"
#include "app_loader.h"
#include "device_identity.h"

/** @brief 由板级静态分配并零初始化，绑定后地址稳定；每个实例独占这些对象。 */
typedef struct
{
    MotorAxis axis;
    AppProtocol protocol;
    ProtocolEndpoint endpoint;
    AppLoader loader;
    DeviceIdentity identity;
    volatile uint32_t fast_samples, supervisor_ticks;
    uint32_t watchdog_last_ms, watchdog_last_fast, watchdog_last_slow;
    bool watchdog_enabled, communication_ready;
    bool initialized;
} AppContext;
#endif
