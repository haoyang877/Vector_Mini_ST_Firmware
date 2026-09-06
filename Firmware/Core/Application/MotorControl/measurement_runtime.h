#ifndef CORE_APPLICATION_MOTOR_CONTROL_MEASUREMENT_RUNTIME_H
#define CORE_APPLICATION_MOTOR_CONTROL_MEASUREMENT_RUNTIME_H

#include "motor_control_types.h"
#include "current_control_runtime.h"
#include "measurement_model.h"

typedef struct MotorStateContext MotorStateContext;

bool Measurement_Capture(CurrentControlContext *CurrentControl);
bool Measurement_Configure(MeasurementModelContext *context,
    const MotorControlContext *motor,
    const MeasurementModelConfig *design_config);
bool Measurement_UpdateCurrentOffsets(MeasurementModelContext *context,
    const MotorControlContext *motor);
bool Measurement_Process(MeasurementModelContext *context,
	CurrentControlContext *CurrentControl, bool protection_is_active,
	MotorStateContext *motor_state);

#endif
