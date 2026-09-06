#include "encoder.h"

#include <string.h>

#define TEST_CHECK(condition) do { if (!(condition)) return __LINE__; } while (0)

static int Encoder_CheckInitializationAndCalibrationState(void)
{
	EncoderContext encoder = {0};

	encoder.electrical_zero_q15 = 101U;
	encoder.mechanical_zero_q15 = 202U;
	encoder.linearization_lut_q15[17] = -303;
	encoder.calib_flag = ENC_CALIB_ALL;
	encoder.reverse = 2U;
	encoder.cogging_compensation_map_ma[9] = 404;

	TEST_CHECK(!Encoder_ParamInit(0, 10U, 0.0005f));
	TEST_CHECK(!Encoder_ParamInit(&encoder, 0U, 0.0005f));
	TEST_CHECK(!Encoder_ParamInit(&encoder, 10U, 0.0f));
	TEST_CHECK(Encoder_ParamInit(&encoder, 10U, 0.0005f));
	TEST_CHECK(encoder.electrical_zero_q15 == 101U);
	TEST_CHECK(encoder.mechanical_zero_q15 == 202U);
	TEST_CHECK(encoder.linearization_lut_q15[17] == -303);
	TEST_CHECK(encoder.calib_flag == ENC_CALIB_ALL);
	TEST_CHECK(encoder.reverse == 1U);
	TEST_CHECK(encoder.cogging_compensation_map_ma[9] == 404);
	TEST_CHECK(!encoder.has_valid_sample);
	TEST_CHECK(encoder.read_status == ENCODER_READ_OK);
	TEST_CHECK(encoder.read_status_latched == ENCODER_READ_OK);
	TEST_CHECK(encoder.read_error_count == 0U);
	return 0;
}

static int Encoder_CheckSampleAndDiagnosticSemantics(void)
{
	EncoderContext encoder = {0};
	EncoderSample sample = {ENCODER_READ_OK, 0x1234U, 65530U};
	uint32_t index;

	TEST_CHECK(Encoder_ParamInit(&encoder, 2U, 0.001f));
	Encoder_Update(&encoder, 7U, &sample);
	TEST_CHECK(Encoder_IsOnline(&encoder));
	TEST_CHECK(encoder.sensor_raw_data_word == 0x1234U);
	TEST_CHECK(encoder.raw_q15 == 65530U);
	TEST_CHECK(encoder.directed_q15 == 65530U);
	TEST_CHECK(encoder.linearized_q15 == 65530U);
	TEST_CHECK(encoder.shadow_q15 == 65530);

	sample.raw_data_word = 0x5678U;
	sample.raw_angle_q15 = 10U;
	Encoder_Update(&encoder, 7U, &sample);
	TEST_CHECK(encoder.shadow_q15 == 65546);
	TEST_CHECK(encoder.velocity_sample_count == 0U);

	sample.raw_angle_q15 = 20U;
	Encoder_Update(&encoder, 7U, &sample);
	TEST_CHECK(encoder.shadow_q15 == 65556);
	TEST_CHECK(encoder.velocity_sample_count == 1U);

	sample.status = ENCODER_READ_CRC_MISMATCH;
	for (index = 0U; index < ENCODER_BAD_FRAME_OFFLINE_COUNT; ++index)
		Encoder_Update(&encoder, 7U, &sample);
	TEST_CHECK(!Encoder_IsOnline(&encoder));
	TEST_CHECK(encoder.read_status == ENCODER_READ_CRC_MISMATCH);
	TEST_CHECK(encoder.read_status_latched == ENCODER_READ_CRC_MISMATCH);
	TEST_CHECK(encoder.read_error_count == ENCODER_BAD_FRAME_OFFLINE_COUNT);
	TEST_CHECK(encoder.bad_frame_streak == ENCODER_BAD_FRAME_OFFLINE_COUNT);
	TEST_CHECK(encoder.raw_q15 == 20U);
	TEST_CHECK(encoder.velocity_sample_count == 1U);

	sample.status = ENCODER_READ_OK;
	sample.raw_angle_q15 = 30U;
	Encoder_Update(&encoder, 7U, &sample);
	TEST_CHECK(Encoder_IsOnline(&encoder));
	TEST_CHECK(encoder.bad_frame_streak == 0U);
	TEST_CHECK(encoder.read_error_count == ENCODER_BAD_FRAME_OFFLINE_COUNT);
	TEST_CHECK(encoder.read_status_latched == ENCODER_READ_CRC_MISMATCH);

	Encoder_Update(&encoder, 7U, 0);
	TEST_CHECK(encoder.read_status == ENCODER_READ_TRANSPORT_ERROR);
	TEST_CHECK(encoder.bad_frame_streak == 1U);
	return 0;
}

static int Encoder_CheckDirectionLutZerosAndVelocity(void)
{
	EncoderContext encoder = {0};
	EncoderSample sample = {ENCODER_READ_OK, 0U, 64U};
	uint32_t index;

	TEST_CHECK(Encoder_ParamInit(&encoder, 1U, 0.00005f));
	encoder.linearization_lut_q15[1] = 10;
	encoder.linearization_lut_q15[2] = 30;
	Encoder_Update(&encoder, 2U, &sample);
	TEST_CHECK(encoder.directed_q15 == 64U);
	TEST_CHECK(encoder.linearized_q15 == 54U);

	sample.raw_angle_q15 = 96U;
	Encoder_Update(&encoder, 2U, &sample);
	TEST_CHECK(encoder.linearized_q15 == 76U);
	TEST_CHECK(Encoder_SetElectricalZero(&encoder));
	TEST_CHECK(encoder.electrical_zero_q15 == 76U);
	TEST_CHECK(Encoder_SetMechanicalZero(&encoder));
	TEST_CHECK(encoder.mechanical_zero_q15 == 76U);
	TEST_CHECK(encoder.theta_mech == 0.0f);

	Encoder_SetReverse(&encoder, true);
	TEST_CHECK(encoder.reverse == 1U);
	TEST_CHECK(encoder.calib_flag == 0U);
	TEST_CHECK(!encoder.has_valid_sample);
	TEST_CHECK(encoder.linearization_lut_q15[1] == 0);
	TEST_CHECK(encoder.cogging_compensation_map_ma[0] == 0);
	sample.raw_angle_q15 = 1U;
	Encoder_Update(&encoder, 2U, &sample);
	TEST_CHECK(encoder.directed_q15 == 65535U);

	memset(&encoder, 0, sizeof(encoder));
	TEST_CHECK(Encoder_ParamInit(&encoder, 1U, 0.00005f));
	sample.raw_angle_q15 = 0U;
	Encoder_Update(&encoder, 3U, &sample);
	for (index = 1U; index <= ENCODER_VELOCITY_WINDOW; ++index)
	{
		sample.raw_angle_q15 = (uint16_t)(index * 64U);
		Encoder_Update(&encoder, 3U, &sample);
	}
	TEST_CHECK(encoder.velocity_ready);
	TEST_CHECK(encoder.velocity_sample_count == ENCODER_VELOCITY_WINDOW);
	TEST_CHECK(encoder.velocity_delta_sum ==
		(int32_t)(ENCODER_VELOCITY_WINDOW * 64U));
	TEST_CHECK(Encoder_GetMecVel(&encoder) > 0.0f);
	TEST_CHECK(Encoder_GetEleVel(&encoder) ==
		Encoder_GetMecVel(&encoder) * 3.0f);
	return 0;
}

int Encoder_RunHostTests(void)
{
	int result;

	result = Encoder_CheckInitializationAndCalibrationState();
	if (result != 0)
		return result;
	result = Encoder_CheckSampleAndDiagnosticSemantics();
	if (result != 0)
		return result;
	return Encoder_CheckDirectionLutZerosAndVelocity();
}

#if defined(ENCODER_HOST_TEST_STANDALONE)
int main(void)
{
	return Encoder_RunHostTests();
}
#endif
