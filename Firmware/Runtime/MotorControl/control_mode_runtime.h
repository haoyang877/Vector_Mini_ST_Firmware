#ifndef RUNTIME_CONTROL_MODE_H
#define RUNTIME_CONTROL_MODE_H

#include "encoder.h"
#include "current_control_runtime.h"
#include "sensorless_runtime.h"
#include "pi_controller.h"
#include "motor_control_types.h"
#include "position_cascade.h"
#include "position_impedance.h"
#include "motor_profiles.h"
#include "board_profile.h"

typedef struct
{
	PositionCascadeContext cascade;
	PositionImpedanceContext impedance;
	uint16_t speed_loop_count;
} MotionControlContext;

void ControlModeRuntime_RunCurrent(CurrentControlContext *CurrentControl, MotorControlContext *MotorControl, EncoderContext *Encoder, FluxObserverContext *Fluxobserver);
void ControlModeRuntime_RunSpeed(MotionControlContext *motion, CurrentControlContext *CurrentControl,
	MotorControlContext *MotorControl, PiController *controller,
	EncoderContext *Encoder);
void ControlModeRuntime_RunSensorlessSpeed(CurrentControlContext *CurrentControl,
	MotorControlContext *MotorControl, PiController *controller,
	FluxObserverContext *Fluxobserver, SensorlessStartupContext *Startup,
	const SensorlessStartupTuning *tuning);
void ControlModeRuntime_RunPositionCascade(MotionControlContext *motion, CurrentControlContext *CurrentControl,
	MotorControlContext *MotorControl, EncoderContext *Encoder,
	const MotorProfile *motor_profile);
void ControlModeRuntime_RunPositionImpedance(MotionControlContext *motion, CurrentControlContext *CurrentControl,
	MotorControlContext *MotorControl, EncoderContext *Encoder,
	const MotorProfile *motor_profile, const BoardProfile *board_profile);
void ControlModeRuntime_ResetPosition(MotionControlContext *motion);
void ControlModeRuntime_RunVoltageOpenLoop(CurrentControlContext *CurrentControl, MotorControlContext *MotorControl);
void ControlModeRuntime_RunQVoltage(CurrentControlContext *CurrentControl, MotorControlContext *MotorControl, EncoderContext *Encoder);

#endif
