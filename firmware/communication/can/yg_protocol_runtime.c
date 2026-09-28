#include "yg_protocol_runtime.h"

#include "comm_hw.h"
#include "param_comm_bridge.h"
#include "yg_protocol_link.h"

/* runtime 拥有参数桥值与硬件调度；link 独占冻结的地址、队列及业务状态。
 * 参数桥更新只供参数装载/保存，不在线重配端点，也不启用通信看门狗。 */
static uint8_t configured_node_id;
static uint32_t configured_heartbeat_ms;
static uint32_t last_recovery_ms;
static bool initialized;
static bool started;

bool YgProtocolRuntime_Init(uint8_t node_id)
{
    /* 不允许前台重复初始化与已经运行的 RX ISR 竞争同一队列。 */
    if (initialized || !YgProtocolLink_Init(node_id))
    {
        return false;
    }
    configured_node_id = node_id;
    last_recovery_ms = 0U;
    initialized = true;
    return true;
}

bool YgProtocolRuntime_Start(void)
{
    if (!initialized || !YgProtocolLink_IsReady())
    {
        return false;
    }
    if (started)
    {
        return true;
    }
    comm_hw_can_start_fd();
    started = true;
    return true;
}

void YgProtocolRuntime_Service(uint32_t now_ms)
{
    if (!started)
    {
        return;
    }
    if ((uint32_t)(now_ms - last_recovery_ms) >= 1000U)
    {
        last_recovery_ms = now_ms;
        (void)comm_hw_can_service_bus_off();
    }
    YgProtocolLink_Service(now_ms);
}

uint8_t Protocol_NodeId_Get(void)
{
    return configured_node_id;
}

void Protocol_NodeId_Set(uint8_t node_id)
{
    configured_node_id = node_id;
}

uint32_t Protocol_HeartbeatMs_Get(void)
{
    return configured_heartbeat_ms;
}

void Protocol_HeartbeatMs_Set(uint32_t heartbeat_ms)
{
    configured_heartbeat_ms = heartbeat_ms;
}
