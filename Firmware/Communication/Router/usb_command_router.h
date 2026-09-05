#ifndef COMMUNICATION_USB_COMMAND_ROUTER_H
#define COMMUNICATION_USB_COMMAND_ROUTER_H

#include <stdbool.h>
#include <stdint.h>

#include "usb_protocol_contract.h"
#include "usb_protocol_v1.h"
#include "application_endpoints.h"

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

typedef struct
{
	UsbCommandRouterAction action;
	char text[USB_COMMAND_ROUTER_RESPONSE_CAPACITY];
	uint8_t print_channel;
	UsbParameterId print_parameter;
	float print_scale;
} UsbCommandRouterResponse;

bool UsbCommandRouter_Initialize(UsbCommandRouterContext *context,
	ApplicationEndpoints *application);
UsbCommandError UsbCommandRouter_Handle(UsbCommandRouterContext *context,
	const UsbProtocolV1Command *command,
	const UsbCommandRouterState *state, UsbCommandRouterResponse *response);

#endif
