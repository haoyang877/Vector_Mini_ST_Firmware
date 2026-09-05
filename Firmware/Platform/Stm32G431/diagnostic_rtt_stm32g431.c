#include "diagnostic_rtt_stm32g431.h"

#include "SEGGER_RTT.h"

static bool DiagnosticRttStm32G431_Write(void *context, const void *data,
	uint16_t size_bytes)
{
	(void)context;
	if (data == 0 || size_bytes == 0U)
		return false;
	return SEGGER_RTT_Write(1U, data, size_bytes) != 0U;
}

DiagnosticTransportPort DiagnosticRttStm32G431_CreatePort(void)
{
	DiagnosticTransportPort port;
	port.context = 0;
	port.write = DiagnosticRttStm32G431_Write;
	return port;
}
