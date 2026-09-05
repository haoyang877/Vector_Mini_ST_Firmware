#ifndef PLATFORM_STM32G431_CAN_FDCAN1_TRANSPORT_H
#define PLATFORM_STM32G431_CAN_FDCAN1_TRANSPORT_H

#include "can_transport_port.h"

CanTransportPort CanFdcan1Transport_CreatePort(void);

#endif
