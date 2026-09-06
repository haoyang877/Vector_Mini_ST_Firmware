#include "encoder.h"

#include <limits.h>
#include <string.h>

#if defined(__CC_ARM)
#pragma O3
#pragma Ospace
#endif

#define ENCODER_VELOCITY_ZERO_THRESHOLD_Q15 8
#define ENCODER_TWO_PI 6.28318530717958647692f

static void Encoder_MarkReadStatus(EncoderContext *encoder, Encoder_ReadStatus status)
{
	encoder->read_status = status;
	if (status == ENCODER_READ_OK)
	{
		encoder->bad_frame_streak = 0U;
		return;
	}

	encoder->read_status_latched = status;
	encoder->read_error_count++;
	if (encoder->bad_frame_streak < UINT16_MAX)
		encoder->bad_frame_streak++;
}

static bool Encoder_ReadSensorSample(EncoderContext *encoder, uint16_t *raw_q15)
{
	RotorSensorSample sample;
	RotorSensorReadStatus status;

	if (encoder->sensor_port.read_sample == 0)
	{
		Encoder_MarkReadStatus(encoder, ENCODER_READ_TRANSPORT_ERROR);
		return false;
	}
	status = encoder->sensor_port.read_sample(encoder->sensor_port.context,
		&sample);
	if (status != ROTOR_SENSOR_READ_OK)
	{
		Encoder_MarkReadStatus(encoder, status);
		return false;
	}

	encoder->sensor_raw_data_word = sample.raw_data_word;
	*raw_q15 = sample.raw_angle_q15;
	Encoder_MarkReadStatus(encoder, ENCODER_READ_OK);
	return true;
}
static uint16_t Encoder_ApplyDirectionQ15(const EncoderContext *encoder, uint16_t raw_q15)
{
	if (encoder->reverse == 0U)
		return raw_q15;

	return (uint16_t)(0U - raw_q15);
}

static uint16_t Encoder_ApplyLinearizationQ15(const EncoderContext *encoder, uint16_t raw_q15)
{
	uint16_t lut_index = raw_q15 >> 6;
	uint16_t fraction = raw_q15 & 0x003FU;
	int32_t correction_a = encoder->linearization_lut_q15[lut_index];
	int32_t correction_b = encoder->linearization_lut_q15[(lut_index + 1U) & (ENCODER_OFFSET_LUT_SIZE - 1U)];
	int32_t correction = correction_a + (((correction_b - correction_a) * fraction) >> 6);

	return (uint16_t)((int32_t)raw_q15 - correction);
}

void Encoder_ResetVelocity(EncoderContext *encoder)
{
	memset(encoder->velocity_delta_history, 0, sizeof(encoder->velocity_delta_history));
	encoder->velocity_divider = 0U;
	encoder->velocity_history_index = 0U;
	encoder->velocity_sample_count = 0U;
	encoder->velocity_delta_sum = 0;
	encoder->velocity_ready = false;
	encoder->velocity_shadow_q15 = encoder->shadow_q15;
	encoder->vel_mech = 0.0f;
	encoder->vel_elec = 0.0f;
}

void Encoder_SetReverse(EncoderContext *encoder, bool reverse)
{
	uint8_t reverse_value = reverse ? 1U : 0U;
	uint32_t primask;

	primask = encoder->critical_section.enter != 0 ?
		encoder->critical_section.enter(encoder->critical_section.context) : 0U;
	encoder->reverse = reverse_value;
	encoder->electrical_zero_q15 = 0U;
	encoder->mechanical_zero_q15 = 0U;
	encoder->calib_flag = 0U;
	memset(encoder->linearization_lut_q15, 0, sizeof(encoder->linearization_lut_q15));
	memset(encoder->cogging_compensation_map_ma, 0,
		sizeof(encoder->cogging_compensation_map_ma));
	encoder->raw_q15 = 0U;
	encoder->directed_q15 = 0U;
	encoder->linearized_q15 = 0U;
	encoder->previous_linearized_q15 = 0U;
	encoder->shadow_q15 = 0;
	encoder->mechanical_zero_shadow_q15 = 0;
	encoder->has_valid_sample = false;
	encoder->theta_elec = 0.0f;
	encoder->theta_mech = 0.0f;
	Encoder_ResetVelocity(encoder);
	if (encoder->critical_section.exit != 0)
		encoder->critical_section.exit(encoder->critical_section.context, primask);
}

static void Encoder_UpdateVelocity2kHz(EncoderContext *encoder, uint32_t pole_pairs)
{
	int64_t delta64;
	int32_t delta_q15;
	int32_t sum_abs;
	float velocity_scale;

	if (++encoder->velocity_divider < encoder->velocity_update_divider)
		return;
	encoder->velocity_divider = 0U;

	delta64 = encoder->shadow_q15 - encoder->velocity_shadow_q15;
	encoder->velocity_shadow_q15 = encoder->shadow_q15;
	if (delta64 > INT32_MAX)
		delta_q15 = INT32_MAX;
	else if (delta64 < INT32_MIN)
		delta_q15 = INT32_MIN;
	else
		delta_q15 = (int32_t)delta64;

	encoder->velocity_delta_sum -= encoder->velocity_delta_history[encoder->velocity_history_index];
	encoder->velocity_delta_history[encoder->velocity_history_index] = delta_q15;
	encoder->velocity_delta_sum += delta_q15;
	encoder->velocity_history_index = (uint8_t)((encoder->velocity_history_index + 1U) % ENCODER_VELOCITY_WINDOW);
	if (encoder->velocity_sample_count < ENCODER_VELOCITY_WINDOW)
		encoder->velocity_sample_count++;
	encoder->velocity_ready = encoder->velocity_sample_count == ENCODER_VELOCITY_WINDOW;

	sum_abs = encoder->velocity_delta_sum;
	if (sum_abs < 0)
		sum_abs = -sum_abs;
	if (!encoder->velocity_ready || sum_abs <= ENCODER_VELOCITY_ZERO_THRESHOLD_Q15)
	{
		encoder->vel_mech = 0.0f;
	}
	else
	{
		velocity_scale = ENCODER_TWO_PI / ((float)ENCODER_Q15_CPR *
			(float)ENCODER_VELOCITY_WINDOW * encoder->velocity_sample_period_s);
		encoder->vel_mech = (float)encoder->velocity_delta_sum * velocity_scale;
	}
	encoder->vel_elec = encoder->vel_mech * (float)pole_pairs;
}

static void Encoder_UpdateAngles(EncoderContext *encoder, uint32_t pole_pairs)
{
	uint16_t electrical_q15;

	electrical_q15 = (uint16_t)((uint32_t)(uint16_t)(encoder->linearized_q15 - encoder->electrical_zero_q15) * pole_pairs);
	encoder->theta_elec = (float)electrical_q15 *
		(ENCODER_TWO_PI / (float)ENCODER_Q15_CPR);
	encoder->theta_mech = (float)(encoder->shadow_q15 -
		encoder->mechanical_zero_shadow_q15) *
		(ENCODER_TWO_PI / (float)ENCODER_Q15_CPR);
}

bool Encoder_ParamInit(EncoderContext *encoder,
	const RotorSensorPort *sensor_port,
	const CriticalSectionPort *critical_section,
	uint8_t velocity_update_divider,
	float velocity_sample_period_s)
{
	if (encoder == 0 || sensor_port == 0 || critical_section == 0 ||
		sensor_port->initialize == 0 || sensor_port->read_sample == 0 ||
		velocity_update_divider == 0U || velocity_sample_period_s <= 0.0f)
		return false;
	encoder->sensor_port = *sensor_port;
	encoder->critical_section = *critical_section;
	encoder->velocity_update_divider = velocity_update_divider;
	encoder->velocity_sample_period_s = velocity_sample_period_s;

	encoder->reverse = encoder->reverse != 0U ? 1U : 0U;
	encoder->raw_q15 = 0U;
	encoder->directed_q15 = 0U;
	encoder->linearized_q15 = 0U;
	encoder->previous_linearized_q15 = 0U;
	encoder->shadow_q15 = 0;
	encoder->mechanical_zero_shadow_q15 = (int64_t)encoder->mechanical_zero_q15;
	encoder->velocity_shadow_q15 = 0;
	encoder->has_valid_sample = false;
	encoder->theta_elec = 0.0f;
	encoder->theta_mech = 0.0f;
	encoder->read_status = ENCODER_READ_OK;
	encoder->read_status_latched = ENCODER_READ_OK;
	encoder->sensor_raw_data_word = 0U;
	encoder->read_error_count = 0U;
	encoder->bad_frame_streak = 0U;
	Encoder_ResetVelocity(encoder);

	return encoder->sensor_port.initialize(encoder->sensor_port.context);
}

bool Encoder_IsOnline(const EncoderContext *encoder)
{
	return encoder->has_valid_sample &&
	       encoder->bad_frame_streak < ENCODER_BAD_FRAME_OFFLINE_COUNT;
}

bool Encoder_SetElectricalZeroQ15(EncoderContext *encoder, uint16_t electrical_zero_q15)
{
	if (!Encoder_IsOnline(encoder))
		return false;

	encoder->electrical_zero_q15 = electrical_zero_q15;
	encoder->calib_flag |= ENC_CALIB_ELECTRICAL_ZERO;
	return true;
}

bool Encoder_SetElectricalZero(EncoderContext *encoder)
{
	return Encoder_SetElectricalZeroQ15(encoder, encoder->linearized_q15);
}

bool Encoder_SetMechanicalZero(EncoderContext *encoder)
{
	if (!Encoder_IsOnline(encoder))
		return false;

	encoder->mechanical_zero_q15 = encoder->linearized_q15;
	encoder->mechanical_zero_shadow_q15 = encoder->shadow_q15;
	encoder->calib_flag |= ENC_CALIB_MECHANICAL_ZERO;
	encoder->theta_mech = 0.0f;
	return true;
}

void Encoder_Update(EncoderContext *encoder, uint32_t pole_pairs)
{
	uint16_t raw_q15;
	uint16_t directed_q15;
	uint16_t linearized_q15;
	int32_t delta_q15;
	if (encoder == 0)
		return;

	if (!Encoder_ReadSensorSample(encoder, &raw_q15))
		return;

	directed_q15 = Encoder_ApplyDirectionQ15(encoder, raw_q15);
	linearized_q15 = Encoder_ApplyLinearizationQ15(encoder, directed_q15);
	encoder->raw_q15 = raw_q15;
	encoder->directed_q15 = directed_q15;
	encoder->linearized_q15 = linearized_q15;
	if (pole_pairs == 0U)
		pole_pairs = 1U;

	if (!encoder->has_valid_sample)
	{
		encoder->previous_linearized_q15 = linearized_q15;
		encoder->shadow_q15 = linearized_q15;
		if ((encoder->calib_flag & ENC_CALIB_MECHANICAL_ZERO) == 0U)
			encoder->mechanical_zero_shadow_q15 = 0;
		else
			encoder->mechanical_zero_shadow_q15 = (int64_t)encoder->mechanical_zero_q15;
		encoder->has_valid_sample = true;
		Encoder_ResetVelocity(encoder);
		Encoder_UpdateAngles(encoder, pole_pairs);
		return;
	}

	delta_q15 = (int32_t)linearized_q15 - (int32_t)encoder->previous_linearized_q15;
	if (delta_q15 > ENCODER_Q15_HALF_TURN)
		delta_q15 -= (int32_t)ENCODER_Q15_CPR;
	else if (delta_q15 < -ENCODER_Q15_HALF_TURN)
		delta_q15 += (int32_t)ENCODER_Q15_CPR;

	encoder->previous_linearized_q15 = linearized_q15;
	encoder->shadow_q15 += delta_q15;
	Encoder_UpdateVelocity2kHz(encoder, pole_pairs);
	Encoder_UpdateAngles(encoder, pole_pairs);
}

float Encoder_GetElePhase(const EncoderContext *encoder)
{
	return encoder->theta_elec;
}

float Encoder_GetMecPos(const EncoderContext *encoder)
{
	return encoder->theta_mech;
}

float Encoder_GetEleVel(const EncoderContext *encoder)
{
	return encoder->vel_elec;
}

float Encoder_GetMecVel(const EncoderContext *encoder)
{
	return encoder->vel_mech;
}

float Encoder_GetCountInCPR_Ratio(const EncoderContext *encoder)
{
	return (float)encoder->linearized_q15 / (float)ENCODER_Q15_CPR;
}
