#include "yg_protocol_runtime.h"

#include "comm_hw.h"
#include "param_comm_bridge.h"
#include "yg_protocol_link.h"

/* 参数桥保留 Flash 字段，CAN FD 端点单独冻结启动节点号；运行时不更改总线速率。 */
static uint8_t node_id;
static uint32_t heartbeat_ms;
static uint32_t last_recovery_ms;
static bool initialized;

bool YgProtocolRuntime_Init(uint8_t configured_node_id)
{
    /* 不允许前台重复初始化与已经运行的 RX ISR 竞争同一队列。 */
    if (initialized || !YgProtocolLink_Init(configured_node_id))
    {
        return false;
    }
    node_id = configured_node_id;
    last_recovery_ms = 0U;
    comm_hw_can_start_fd();
    initialized = true;
    return true;
}

void YgProtocolRuntime_Service(uint32_t now_ms)
{
    if (!initialized)
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
    return node_id;
}

void Protocol_NodeId_Set(uint8_t configured_node_id)
{
    node_id = configured_node_id;
}

uint32_t Protocol_HeartbeatMs_Get(void)
{
    return heartbeat_ms;
}

void Protocol_HeartbeatMs_Set(uint32_t configured_heartbeat_ms)
{
    heartbeat_ms = configured_heartbeat_ms;
}
