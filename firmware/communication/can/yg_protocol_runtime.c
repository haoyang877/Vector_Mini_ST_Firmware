#include "yg_protocol_runtime.h"

#include "comm_hw.h"
#include "yg_protocol_link.h"

/* runtime 只拥有硬件调度；link 接收组合层的地址值并独占端点状态。 */
static uint32_t s_last_recovery_ms;
static bool s_initialized;
static bool s_started;

bool YgProtocolRuntime_Init(uint8_t node_id)
{
    /* 不允许前台重复初始化与已经运行的 RX ISR 竞争同一队列。 */
    if (s_initialized || !YgProtocolLink_Init(node_id))
    {
        return false;
    }
    s_last_recovery_ms = 0U;
    s_initialized = true;
    return true;
}

bool YgProtocolRuntime_Start(void)
{
    if (!s_initialized || !YgProtocolLink_IsReady())
    {
        return false;
    }
    if (s_started)
    {
        return true;
    }
    comm_hw_can_start_fd();
    s_started = true;
    return true;
}

void YgProtocolRuntime_Service(uint32_t now_ms)
{
    if (!s_started)
    {
        return;
    }
    if ((uint32_t)(now_ms - s_last_recovery_ms) >= 1000U)
    {
        s_last_recovery_ms = now_ms;
        (void)comm_hw_can_service_bus_off();
    }
    YgProtocolLink_Service(now_ms);
}
