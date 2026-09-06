#include "rotor_calibration_port_adapter.h"

static bool RotorCalibrationAdapter_SetReverse(void *context, bool reverse)
{
	RotorCalibrationAdapterContext *adapter =
		(RotorCalibrationAdapterContext *)context;
	EncoderContext *encoder;
	uint32_t interrupt_state;

	if (adapter == 0 || adapter->encoder == 0 || adapter->motor == 0)
		return false;
	encoder = adapter->encoder;
	interrupt_state = adapter->critical_section.enter != 0 ?
		adapter->critical_section.enter(adapter->critical_section.context) : 0U;
	if (encoder->reverse != (reverse ? 1U : 0U))
		adapter->motor->configuration.friction_model_valid = false;
	Encoder_SetReverse(encoder, reverse);
	if (adapter->critical_section.exit != 0)
		adapter->critical_section.exit(adapter->critical_section.context,
			interrupt_state);
	return encoder->reverse == (reverse ? 1U : 0U);
}

static bool RotorCalibrationAdapter_ReadEntry(void *context, uint16_t index,
	RotorCalibrationEntry *entry)
{
	RotorCalibrationAdapterContext *adapter =
		(RotorCalibrationAdapterContext *)context;
	EncoderContext *encoder;
	uint16_t directed_q15;
	int16_t directed_error_q15;

	if (adapter == 0 || adapter->encoder == 0 || entry == 0 ||
		index >= ENCODER_OFFSET_LUT_SIZE)
		return false;
	encoder = adapter->encoder;

	directed_q15 = (uint16_t)(index << 6);
	directed_error_q15 = encoder->linearization_lut_q15[index];
	entry->raw_angle_q15 = encoder->reverse != 0U ?
		(uint16_t)(0U - directed_q15) : directed_q15;
	entry->error_q15 = encoder->reverse != 0U ?
		-(int32_t)directed_error_q15 : (int32_t)directed_error_q15;
	return true;
}

RotorCalibrationPort RotorCalibrationAdapter_CreatePort(
	RotorCalibrationAdapterContext *context, EncoderContext *encoder,
	MotorControlContext *motor,
	const CriticalSectionPort *critical_section)
{
	RotorCalibrationPort port = {0};

	if (context == 0 || encoder == 0 || motor == 0 || critical_section == 0)
		return port;
	context->encoder = encoder;
	context->motor = motor;
	context->critical_section = *critical_section;
	port.context = context;
	port.entry_count = ENCODER_OFFSET_LUT_SIZE;
	port.counts_per_revolution = ENCODER_Q15_CPR;
	port.set_reverse = RotorCalibrationAdapter_SetReverse;
	port.read_entry = RotorCalibrationAdapter_ReadEntry;
	return port;
}
