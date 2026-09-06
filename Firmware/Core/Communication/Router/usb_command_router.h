#ifndef CORE_COMMUNICATION_ROUTER_USB_COMMAND_ROUTER_H
#define CORE_COMMUNICATION_ROUTER_USB_COMMAND_ROUTER_H

#include <stdbool.h>
#include <stdint.h>

#include "Core/Communication/Protocol/usb_protocol_contract.h"
#include "Core/Communication/Protocol/usb_protocol_v1.h"
#include "Core/Application/Api/application_endpoints.h"

#define USB_COMMAND_ROUTER_RESPONSE_CAPACITY 80U

typedef struct
{
	ApplicationEndpoints *application;
} UsbCommandRouterContext;

typedef struct
{
	bool print_active;
	bool lut_export_active;
	bool friction_export_active;
} UsbCommandRouterState;

typedef enum
{
	USB_COMMAND_ROUTER_ACTION_NONE = 0,
	USB_COMMAND_ROUTER_ACTION_SEND_TEXT,
	USB_COMMAND_ROUTER_ACTION_CONFIGURE_PRINT,
	USB_COMMAND_ROUTER_ACTION_BEGIN_LUT_EXPORT,
	USB_COMMAND_ROUTER_ACTION_BEGIN_FRICTION_EXPORT
} UsbCommandRouterAction;

/*
 * A print channel stores the already-resolved value source rather than the
 * three-character protocol key.  This keeps the 1 kHz stream path independent
 * of the command lookup implementation and avoids a second, large parameter
 * switch in the USB interface.
 */
typedef uint8_t UsbCommandRouterPrintSource;

#define USB_COMMAND_ROUTER_PRINT_INTEGER_FLAG       0x80U
#define USB_COMMAND_ROUTER_PRINT_TELEMETRY_MASK     0x7FU
#define USB_COMMAND_ROUTER_PRINT_ZERO               0x7CU
#define USB_COMMAND_ROUTER_PRINT_CAN_NODE_ID        0x7DU
#define USB_COMMAND_ROUTER_PRINT_CAN_BITRATE        0x7EU
#define USB_COMMAND_ROUTER_PRINT_CAN_HEARTBEAT      0x7FU

typedef struct
{
	UsbCommandRouterAction action;
	char text[USB_COMMAND_ROUTER_RESPONSE_CAPACITY];
	uint8_t print_channel;
	UsbCommandRouterPrintSource print_source;
	float print_scale;
} UsbCommandRouterResponse;

bool UsbCommandRouter_Initialize(UsbCommandRouterContext *context,
	ApplicationEndpoints *application);
UsbCommandError UsbCommandRouter_Handle(UsbCommandRouterContext *context,
	const UsbProtocolV1Command *command,
	const UsbCommandRouterState *state, UsbCommandRouterResponse *response);

#endif
