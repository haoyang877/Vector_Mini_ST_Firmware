#include "yg_protocol_runtime.h"

#include "comm_hw.h"
#include "yg_protocol_app_adapter.h"
#include "yg_protocol_link.h"

static uint8_t node_id;
static uint32_t heartbeat_ms;

bool YgProtocolRuntime_Init(uint8_t configured_node_id)
{
    if (!YgProtocolLink_Init(configured_node_id))
    {
        return false;
    }
    if (!YgProtocolAppAdapter_Init())
    {
        return false;
    }
    node_id = configured_node_id;
    comm_hw_can_start_fd(configured_node_id);
    return true;
}

uint8_t YgProtocolRuntime_NodeId(void)
{
    return node_id;
}

void YgProtocolRuntime_SetNodeId(uint8_t configured_node_id)
{
    node_id = configured_node_id;
}

uint32_t YgProtocolRuntime_HeartbeatMs(void)
{
    return heartbeat_ms;
}

void YgProtocolRuntime_SetHeartbeatMs(uint32_t configured_heartbeat_ms)
{
    heartbeat_ms = configured_heartbeat_ms;
}

uint8_t Protocol_NodeId_Get(void)
{
    return YgProtocolRuntime_NodeId();
}

void Protocol_NodeId_Set(uint8_t configured_node_id)
{
    YgProtocolRuntime_SetNodeId(configured_node_id);
}

uint32_t Protocol_HeartbeatMs_Get(void)
{
    return YgProtocolRuntime_HeartbeatMs();
}

void Protocol_HeartbeatMs_Set(uint32_t configured_heartbeat_ms)
{
    YgProtocolRuntime_SetHeartbeatMs(configured_heartbeat_ms);
}
