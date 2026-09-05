#include "rotor_calibration_port_adapter.h"

static bool RotorCalibrationAdapter_SetReverse(void *context, bool reverse)
{
	EncoderContext *encoder = (EncoderContext *)context;

	if (encoder == 0)
		return false;
	Encoder_SetReverse(encoder, reverse);
	return encoder->reverse == (reverse ? 1U : 0U);
}

static bool RotorCalibrationAdapter_ReadEntry(void *context, uint16_t index,
	RotorCalibrationEntry *entry)
{
	EncoderContext *encoder = (EncoderContext *)context;
	uint16_t directed_q15;
	int16_t directed_error_q15;

	if (encoder == 0 || entry == 0 || index >= ENCODER_OFFSET_LUT_SIZE)
		return false;

	directed_q15 = (uint16_t)(index << 6);
	directed_error_q15 = encoder->linearization_lut_q15[index];
	entry->raw_angle_q15 = encoder->reverse != 0U ?
		(uint16_t)(0U - directed_q15) : directed_q15;
	entry->error_q15 = encoder->reverse != 0U ?
		-(int32_t)directed_error_q15 : (int32_t)directed_error_q15;
	return true;
}

RotorCalibrationPort RotorCalibrationAdapter_CreatePort(EncoderContext *encoder)
{
	RotorCalibrationPort port;

	port.context = encoder;
	port.entry_count = ENCODER_OFFSET_LUT_SIZE;
	port.counts_per_revolution = ENCODER_Q15_CPR;
	port.set_reverse = RotorCalibrationAdapter_SetReverse;
	port.read_entry = RotorCalibrationAdapter_ReadEntry;
	return port;
}
