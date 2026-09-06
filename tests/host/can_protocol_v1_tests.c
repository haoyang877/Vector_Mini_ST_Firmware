#include "Core/Communication/Protocol/can_protocol_v1.h"

#include <math.h>
#define TEST_CHECK(condition) do { if (!(condition)) return __LINE__; } while (0)

int CanProtocolV1_RunHostTests(void)
{
	BspCanFrame frame;
	CanProtocolV1Command command;
	float decoded_value;
	uint32_t nan_bits = UINT32_C(0x7FC00000);

	TEST_CHECK(CanProtocolV1_Encode(3U, 0x42U, 1.0f, &frame));
	TEST_CHECK(frame.identifier == 0x342U);
	TEST_CHECK(frame.flags == 0U);
	TEST_CHECK(frame.length == 4U);
	TEST_CHECK(frame.data[0] == 0x3FU && frame.data[1] == 0x80U &&
		frame.data[2] == 0x00U && frame.data[3] == 0x00U);
	TEST_CHECK(CanProtocolV1_Decode(&frame, 3U, &command));
	TEST_CHECK(command.parameter_id == 0x42U);
	decoded_value = command.value;
	TEST_CHECK(decoded_value == 1.0f);

	TEST_CHECK(CanProtocolV1_Encode(7U, 0xFFU, -12.5f, &frame));
	frame.flags = BSP_CAN_FRAME_FD | BSP_CAN_FRAME_BRS;
	TEST_CHECK(CanProtocolV1_Decode(&frame, 7U, &command));
	TEST_CHECK(command.parameter_id == 0xFFU && command.value == -12.5f);

	TEST_CHECK(!CanProtocolV1_Encode(8U, 0U, 0.0f, &frame));
	TEST_CHECK(!CanProtocolV1_Encode(0U, 0U, NAN, &frame));
	TEST_CHECK(!CanProtocolV1_Encode(0U, 0U, 0.0f, NULL));

	TEST_CHECK(CanProtocolV1_Encode(2U, 1U, 2.0f, &frame));
	TEST_CHECK(!CanProtocolV1_Decode(&frame, 1U, &command));
	frame.length = 3U;
	TEST_CHECK(!CanProtocolV1_Decode(&frame, 2U, &command));
	frame.length = 4U;
	frame.flags = BSP_CAN_FRAME_EXTENDED_ID;
	TEST_CHECK(!CanProtocolV1_Decode(&frame, 2U, &command));
	frame.flags = BSP_CAN_FRAME_REMOTE;
	TEST_CHECK(!CanProtocolV1_Decode(&frame, 2U, &command));
	frame.flags = 0U;
	frame.identifier = 0x800U;
	TEST_CHECK(!CanProtocolV1_Decode(&frame, 2U, &command));
	frame.identifier = 0x201U;
	frame.data[0] = (uint8_t)(nan_bits >> 24);
	frame.data[1] = (uint8_t)(nan_bits >> 16);
	frame.data[2] = (uint8_t)(nan_bits >> 8);
	frame.data[3] = (uint8_t)nan_bits;
	TEST_CHECK(!CanProtocolV1_Decode(&frame, 2U, &command));
	TEST_CHECK(!CanProtocolV1_Decode(NULL, 2U, &command));
	TEST_CHECK(!CanProtocolV1_Decode(&frame, 2U, NULL));
	return 0;
}
