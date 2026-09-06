#ifndef RUNTIME_ROTOR_CALIBRATION_PORT_ADAPTER_H
#define RUNTIME_ROTOR_CALIBRATION_PORT_ADAPTER_H

#include "critical_section_port.h"
#include "encoder.h"
#include "motor_control_types.h"
#include "Core/Application/Contracts/rotor_calibration_port.h"

typedef struct
{
	EncoderContext *encoder;
	MotorControlContext *motor;
	CriticalSectionPort critical_section;
} RotorCalibrationAdapterContext;

RotorCalibrationPort RotorCalibrationAdapter_CreatePort(
	RotorCalibrationAdapterContext *context, EncoderContext *encoder,
	MotorControlContext *motor,
	const CriticalSectionPort *critical_section);

#endif
