#include "Core/Communication/Protocol/usb_protocol_v1.h"

#include <math.h>
#include <stddef.h>

UsbProtocolV1DecodeResult UsbProtocolV1_Decode(const uint8_t *data,
    uint16_t length, UsbProtocolV1Command *command)
{
    uint16_t index;
    float integer_part = 0.0f;
    float fractional_part = 0.0f;
    float fractional_scale = 0.1f;
    float sign = 1.0f;
    bool has_digit = false;
    bool has_decimal_point = false;

    if (data == NULL || command == NULL || length < 8U ||
        data[0] != '\\' || data[2] != '_' ||
        data[length - 2U] != '\r' || data[length - 1U] != '\n')
        return USB_PROTOCOL_V1_DECODE_SYNTAX_ERROR;

    if (data[1] == 'w')
        command->operation = USB_PROTOCOL_V1_OPERATION_WRITE;
    else if (data[1] == 'r')
        command->operation = USB_PROTOCOL_V1_OPERATION_READ;
    else if (data[1] == 'p')
        command->operation = USB_PROTOCOL_V1_OPERATION_PRINT;
    else
        return USB_PROTOCOL_V1_DECODE_SYNTAX_ERROR;

    command->parameter_id = ((uint32_t)data[3] << 16) |
        ((uint32_t)data[4] << 8) | data[5];
    command->value = 0.0f;
    command->value_is_float = false;

    if (command->operation == USB_PROTOCOL_V1_OPERATION_READ)
        return length == 8U ? USB_PROTOCOL_V1_DECODE_OK :
            USB_PROTOCOL_V1_DECODE_SYNTAX_ERROR;
    if (length < 10U || data[6] != '=')
        return USB_PROTOCOL_V1_DECODE_SYNTAX_ERROR;

    index = 7U;
    if (data[index] == '-')
    {
        sign = -1.0f;
        index++;
    }
    for (; index < length - 2U; ++index)
    {
        uint8_t character = data[index];
        if (character == '.')
        {
            if (has_decimal_point)
                return USB_PROTOCOL_V1_DECODE_INVALID_NUMBER;
            has_decimal_point = true;
            command->value_is_float = true;
            continue;
        }
        if (character < '0' || character > '9')
            return USB_PROTOCOL_V1_DECODE_INVALID_NUMBER;
        has_digit = true;
        if (!has_decimal_point)
            integer_part = integer_part * 10.0f + (float)(character - '0');
        else
        {
            fractional_part += (float)(character - '0') * fractional_scale;
            fractional_scale *= 0.1f;
        }
    }
    command->value = sign * (integer_part + fractional_part);
    if (!has_digit || !isfinite(command->value))
        return USB_PROTOCOL_V1_DECODE_INVALID_NUMBER;
    return USB_PROTOCOL_V1_DECODE_OK;
}
