#include "usb_cdc_transport.h"

#include "usbd_cdc_if.h"

static bool UsbCdcTransport_Transmit(void *context,
	const uint8_t *data, uint16_t length)
{
	(void)context;
	if (data == 0 || length == 0U)
		return false;
	return CDC_Transmit_FS((uint8_t *)data, length) == USBD_OK;
}

static bool UsbCdcTransport_CancelTransmit(void *context)
{
	(void)context;
	return CDC_AbortTransmit_FS() == USBD_OK;
}

ByteTransportPort UsbCdcTransport_CreatePort(void)
{
	ByteTransportPort port;
	port.context = 0;
	port.transmit = UsbCdcTransport_Transmit;
	port.cancel_transmit = UsbCdcTransport_CancelTransmit;
	return port;
}
