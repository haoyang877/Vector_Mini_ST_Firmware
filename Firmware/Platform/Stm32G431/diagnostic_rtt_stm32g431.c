#include "diagnostic_rtt_stm32g431.h"

#include "SEGGER_RTT.h"

static BspResult DiagnosticRttStm32G431_Write(void *context, const void *data,
	size_t size_bytes)
{
	unsigned int written;

	(void)context;
	if (data == 0 || size_bytes == 0U || size_bytes > UINT16_MAX)
		return BSP_RESULT_INVALID_ARGUMENT;
	written = SEGGER_RTT_Write(1U, data, (unsigned int)size_bytes);
	return written == size_bytes ? BSP_RESULT_OK : BSP_RESULT_NOT_READY;
}

BspDiagnosticSinkPort DiagnosticRttStm32G431_CreatePort(void)
{
	BspDiagnosticSinkPort port;
	port.context = 0;
	port.write = DiagnosticRttStm32G431_Write;
	return port;
}
