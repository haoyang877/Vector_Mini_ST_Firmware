#include "can_protocol_v1.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

bool CanProtocolV1_Decode(const BspCanFrame *frame,
    uint8_t expected_node_id, CanProtocolV1Command *command)
{
    uint32_t encoded_value;

    if (frame == NULL || command == NULL || frame->length != 4U ||
		(frame->flags & (BSP_CAN_FRAME_EXTENDED_ID |
			BSP_CAN_FRAME_REMOTE)) != 0U ||
		frame->identifier > 0x7FFU ||
        (uint8_t)(frame->identifier >> 8) != expected_node_id)
	{
		return false;
	}

    encoded_value = (uint32_t)frame->data[0] << 24;
    encoded_value |= (uint32_t)frame->data[1] << 16;
    encoded_value |= (uint32_t)frame->data[2] << 8;
    encoded_value |= frame->data[3];
    memcpy(&command->value, &encoded_value, sizeof(command->value));
    if (!isfinite(command->value))
        return false;
    command->parameter_id = (uint8_t)(frame->identifier & 0xFFU);
    return true;
}

bool CanProtocolV1_Encode(uint8_t node_id, uint8_t parameter_id,
    float value, BspCanFrame *frame)
{
    uint32_t encoded_value;

    if (frame == NULL || node_id > 7U || !isfinite(value))
	{
		return false;
	}
	memset(frame, 0, sizeof(*frame));
    memcpy(&encoded_value, &value, sizeof(encoded_value));
	frame->identifier = ((uint32_t)node_id << 8) | parameter_id;
    frame->length = 4U;
    frame->data[0] = (uint8_t)(encoded_value >> 24);
    frame->data[1] = (uint8_t)(encoded_value >> 16);
    frame->data[2] = (uint8_t)(encoded_value >> 8);
    frame->data[3] = (uint8_t)encoded_value;
    return true;
}
