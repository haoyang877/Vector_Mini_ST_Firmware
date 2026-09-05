#ifndef PLATFORM_STM32G431_USB_CDC_TRANSPORT_H
#define PLATFORM_STM32G431_USB_CDC_TRANSPORT_H

#include "byte_transport_port.h"

ByteTransportPort UsbCdcTransport_CreatePort(void);

#endif
