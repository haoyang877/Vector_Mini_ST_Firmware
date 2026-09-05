#include "can_configuration_service.h"

static CanConfigurationServiceContext *ActiveContext;
#define ConfigurationPort (ActiveContext->port)
#define ConfigurationPortInitialized (ActiveContext != 0 && ActiveContext->is_initialized)

bool CanConfigurationService_Initialize(CanConfigurationServiceContext *context,
	const CanConfigurationPort *port)
{
	if (context == 0 || port == 0 || port->set_node_id == 0 || port->get_node_id == 0 ||
		port->set_bitrate_kbps == 0 || port->get_bitrate_kbps == 0 ||
		port->set_heartbeat_ms == 0 || port->get_heartbeat_ms == 0)
		return false;
	context->port = *port;
	context->is_initialized = true;
	ActiveContext = context;
	return true;
}

bool CanConfigurationService_SetNodeId(uint8_t node_id)
{
	return ConfigurationPortInitialized &&
		ConfigurationPort.set_node_id(ConfigurationPort.context, node_id);
}

uint8_t CanConfigurationService_GetNodeId(void)
{
	return ConfigurationPortInitialized ?
		ConfigurationPort.get_node_id(ConfigurationPort.context) : 0U;
}

bool CanConfigurationService_SetBitrateKbps(uint32_t bitrate_kbps)
{
	return ConfigurationPortInitialized &&
		ConfigurationPort.set_bitrate_kbps(ConfigurationPort.context,
			bitrate_kbps);
}

uint32_t CanConfigurationService_GetBitrateKbps(void)
{
	return ConfigurationPortInitialized ?
		ConfigurationPort.get_bitrate_kbps(ConfigurationPort.context) : 0U;
}

bool CanConfigurationService_SetHeartbeatMs(uint32_t heartbeat_ms)
{
	return ConfigurationPortInitialized &&
		ConfigurationPort.set_heartbeat_ms(ConfigurationPort.context,
			heartbeat_ms);
}

uint32_t CanConfigurationService_GetHeartbeatMs(void)
{
	return ConfigurationPortInitialized ?
		ConfigurationPort.get_heartbeat_ms(ConfigurationPort.context) : 0U;
}
