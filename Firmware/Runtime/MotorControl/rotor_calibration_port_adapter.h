#ifndef RUNTIME_ROTOR_CALIBRATION_PORT_ADAPTER_H
#define RUNTIME_ROTOR_CALIBRATION_PORT_ADAPTER_H

#include "encoder.h"
#include "motor_control_types.h"
#include "rotor_calibration_port.h"

typedef struct
{
	EncoderContext *encoder;
	MotorControlContext *motor;
} RotorCalibrationAdapterContext;

RotorCalibrationPort RotorCalibrationAdapter_CreatePort(
	RotorCalibrationAdapterContext *context, EncoderContext *encoder,
	MotorControlContext *motor);

#endif
