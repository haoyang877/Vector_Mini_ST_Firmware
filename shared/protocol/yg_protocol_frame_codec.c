/* 统一应用帧的定长头部、显式小端序列化和双层 CRC 实现。 */
#include "yg_protocol_frame_codec.h"
#include "yg_protocol_byte_order.h"

#include <stdbool.h>

#include "yg_protocol_crc.h"

static bool has_valid_payload(const yg_protocol_frame_view_t *message)
{
    return message->payload_length == 0U || message->payload != NULL;
}

yg_protocol_result_t yg_protocol_frame_encode(const yg_protocol_frame_view_t *message,
                                              uint8_t *buffer,
                                              size_t capacity,
                                              size_t *written)
{
    size_t frame_length;
    uint16_t frame_crc;

    if (message == NULL || buffer == NULL || !has_valid_payload(message))
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    if (message->version != YG_PROTOCOL_VERSION)
    {
        return YG_PROTOCOL_INVALID_VERSION;
    }
    if (message->payload_length > YG_PROTOCOL_MAX_PAYLOAD_SIZE)
    {
        return YG_PROTOCOL_INVALID_LENGTH;
    }

    frame_length =
        YG_PROTOCOL_HEADER_SIZE + (size_t)message->payload_length + YG_PROTOCOL_TRAILER_SIZE;
    if (capacity < frame_length)
    {
        return YG_PROTOCOL_BUFFER_TOO_SMALL;
    }

    YgProtocol_WriteU16Le(&buffer[0], YG_PROTOCOL_MAGIC);
    buffer[2] = message->version;
    buffer[3] = message->flags;
    buffer[4] = message->source_node;
    buffer[5] = message->destination_node;
    YgProtocol_WriteU16Le(&buffer[6], message->message_type);
    YgProtocol_WriteU16Le(&buffer[8], message->sequence);
    YgProtocol_WriteU32Le(&buffer[10], message->payload_length);
    buffer[14] = message->reserved;
    buffer[15] = yg_protocol_crc8(buffer, 15U);

    if (message->payload_length > 0U)
    {
        for (size_t index = 0U; index < message->payload_length; ++index)
        {
            buffer[YG_PROTOCOL_HEADER_SIZE + index] = message->payload[index];
        }
    }

    frame_crc = yg_protocol_crc16(buffer, YG_PROTOCOL_HEADER_SIZE + message->payload_length);
    YgProtocol_WriteU16Le(&buffer[YG_PROTOCOL_HEADER_SIZE + message->payload_length], frame_crc);
    if (written != NULL)
    {
        *written = frame_length;
    }
    return YG_PROTOCOL_OK;
}

yg_protocol_result_t
yg_protocol_frame_decode(const uint8_t *buffer, size_t length, yg_protocol_frame_view_t *message)
{
    uint32_t payload_length;
    size_t expected_length;
    uint16_t received_crc;
    uint16_t calculated_crc;

    if (buffer == NULL || message == NULL)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    if (length < YG_PROTOCOL_HEADER_SIZE + YG_PROTOCOL_TRAILER_SIZE)
    {
        return YG_PROTOCOL_INVALID_LENGTH;
    }
    if (YgProtocol_ReadU16Le(buffer) != YG_PROTOCOL_MAGIC)
    {
        return YG_PROTOCOL_INVALID_MAGIC;
    }
    if (buffer[2] != YG_PROTOCOL_VERSION)
    {
        return YG_PROTOCOL_INVALID_VERSION;
    }
    if (yg_protocol_crc8(buffer, 15U) != buffer[15])
    {
        return YG_PROTOCOL_INVALID_HEADER_CRC;
    }

    payload_length = YgProtocol_ReadU32Le(&buffer[10]);
    if (payload_length > YG_PROTOCOL_MAX_PAYLOAD_SIZE)
    {
        return YG_PROTOCOL_INVALID_LENGTH;
    }
    expected_length = YG_PROTOCOL_HEADER_SIZE + (size_t)payload_length + YG_PROTOCOL_TRAILER_SIZE;
    if (length != expected_length)
    {
        return YG_PROTOCOL_INVALID_LENGTH;
    }

    received_crc = YgProtocol_ReadU16Le(&buffer[YG_PROTOCOL_HEADER_SIZE + payload_length]);
    calculated_crc = yg_protocol_crc16(buffer, YG_PROTOCOL_HEADER_SIZE + payload_length);
    if (received_crc != calculated_crc)
    {
        return YG_PROTOCOL_INVALID_FRAME_CRC;
    }

    message->version = buffer[2];
    message->flags = buffer[3];
    message->source_node = buffer[4];
    message->destination_node = buffer[5];
    message->message_type = YgProtocol_ReadU16Le(&buffer[6]);
    message->sequence = YgProtocol_ReadU16Le(&buffer[8]);
    message->payload_length = payload_length;
    message->reserved = buffer[14];
    message->payload = &buffer[YG_PROTOCOL_HEADER_SIZE];
    return YG_PROTOCOL_OK;
}
