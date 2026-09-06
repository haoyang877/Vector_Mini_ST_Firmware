#ifndef PLATFORM_STM32G431_USB_CDC_TRANSPORT_H
#define PLATFORM_STM32G431_USB_CDC_TRANSPORT_H

#include <stdbool.h>
#include <stdint.h>

#include "Bsp/Api/bsp_communication.h"

bool UsbCdcTransport_CreatePort(
	const BspCommunicationEndpointCapabilities *capabilities,
	BspByteStreamPort *port);
void UsbCdcTransport_OnReceiveInterrupt(const uint8_t *data,
	uint32_t length);
void UsbCdcTransport_OnTransmitCompleteInterrupt(void);

#endif
