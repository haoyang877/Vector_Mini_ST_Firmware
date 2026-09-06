#ifndef CORE_COMMUNICATION_PROTOCOL_USB_PROTOCOL_V1_H
#define CORE_COMMUNICATION_PROTOCOL_USB_PROTOCOL_V1_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    USB_PROTOCOL_V1_OPERATION_WRITE = 0,
    USB_PROTOCOL_V1_OPERATION_READ = 1,
    USB_PROTOCOL_V1_OPERATION_PRINT = 2
} UsbProtocolV1Operation;

typedef enum
{
    USB_PROTOCOL_V1_DECODE_OK = 0,
    USB_PROTOCOL_V1_DECODE_SYNTAX_ERROR,
    USB_PROTOCOL_V1_DECODE_INVALID_NUMBER
} UsbProtocolV1DecodeResult;

typedef struct
{
    UsbProtocolV1Operation operation;
    uint32_t parameter_id;
    float value;
    bool value_is_float;
} UsbProtocolV1Command;

UsbProtocolV1DecodeResult UsbProtocolV1_Decode(const uint8_t *data,
    uint16_t length, UsbProtocolV1Command *command);

#endif
