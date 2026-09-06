#ifndef PLATFORM_STM32G431_CAN_FDCAN1_TRANSPORT_H
#define PLATFORM_STM32G431_CAN_FDCAN1_TRANSPORT_H

#include <stdbool.h>

#include "Bsp/Api/bsp_communication.h"

bool CanFdcan1Transport_CreatePort(
	const BspCommunicationEndpointCapabilities *capabilities,
	BspCanPort *port);
void CanFdcan1Transport_OnReceiveInterrupt(uint32_t interrupt_flags);

#endif
