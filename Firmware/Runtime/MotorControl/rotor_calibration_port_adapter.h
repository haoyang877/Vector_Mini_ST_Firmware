#ifndef RUNTIME_ROTOR_CALIBRATION_PORT_ADAPTER_H
#define RUNTIME_ROTOR_CALIBRATION_PORT_ADAPTER_H

#include "bsp_system.h"
#include "encoder.h"
#include "motor_control_types.h"
#include "Core/Application/Contracts/rotor_calibration_port.h"

typedef struct
{
	EncoderContext *encoder;
	MotorControlContext *motor;
	BspCriticalSectionPort critical_section;
} RotorCalibrationAdapterContext;

RotorCalibrationPort RotorCalibrationAdapter_CreatePort(
	RotorCalibrationAdapterContext *context, EncoderContext *encoder,
	MotorControlContext *motor,
	const BspCriticalSectionPort *critical_section);

#endif
