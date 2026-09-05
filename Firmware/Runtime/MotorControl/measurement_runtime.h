#ifndef RUNTIME_MEASUREMENT_H
#define RUNTIME_MEASUREMENT_H

#include "motor_control_types.h"
#include "current_control_runtime.h"
#include "measurement_model.h"
#include "board_profile.h"

bool Measurement_Capture(CurrentControlContext *CurrentControl);
bool Measurement_Configure(MeasurementModelContext *context,
    const MotorControlContext *motor, const BoardProfile *board_profile);
bool Measurement_Process(MeasurementModelContext *context,
	CurrentControlContext *CurrentControl, bool protection_is_active);

#endif
