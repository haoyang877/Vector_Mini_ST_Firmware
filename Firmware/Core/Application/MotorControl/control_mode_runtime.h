#ifndef CORE_APPLICATION_MOTOR_CONTROL_CONTROL_MODE_RUNTIME_H
#define CORE_APPLICATION_MOTOR_CONTROL_CONTROL_MODE_RUNTIME_H

#include "encoder.h"
#include "current_control_runtime.h"
#include "sensorless_runtime.h"
#include "pi_controller.h"
#include "motor_control_types.h"
#include "position_cascade.h"
#include "position_impedance.h"
#include "Core/Services/RotorFeedback/feedback_frame.h"

typedef struct
{
	PositionCascadeContext cascade;
	PositionImpedanceContext impedance;
	uint16_t speed_loop_count;
} MotionControlContext;

typedef struct MotorStateContext MotorStateContext;

void ControlModeRuntime_RunCurrent(CurrentControlContext *CurrentControl,
	MotorControlContext *MotorControl, EncoderContext *Encoder,
	FluxObserverContext *Fluxobserver, const RotorFeedbackFrame *feedback);
void ControlModeRuntime_RunSpeed(MotionControlContext *motion, CurrentControlContext *CurrentControl,
	MotorControlContext *MotorControl, PiController *controller,
	EncoderContext *Encoder, const RotorFeedbackFrame *feedback);
void ControlModeRuntime_RunSensorlessSpeed(CurrentControlContext *CurrentControl,
	MotorControlContext *MotorControl, PiController *controller,
	FluxObserverContext *Fluxobserver, SensorlessStartupContext *Startup,
	const ProductSensorlessStartupConfig *tuning, MotorStateContext *motor_state);
void ControlModeRuntime_RunPositionCascade(MotionControlContext *motion, CurrentControlContext *CurrentControl,
	MotorControlContext *MotorControl, EncoderContext *Encoder,
	const RotorFeedbackFrame *feedback,
	MotorStateContext *motor_state);
void ControlModeRuntime_RunPositionImpedance(MotionControlContext *motion, CurrentControlContext *CurrentControl,
	MotorControlContext *MotorControl, EncoderContext *Encoder,
	const RotorFeedbackFrame *feedback,
	float maximum_current_limit_a,
	MotorStateContext *motor_state);
void ControlModeRuntime_ResetPosition(MotionControlContext *motion);
void ControlModeRuntime_RunVoltageOpenLoop(CurrentControlContext *CurrentControl, MotorControlContext *MotorControl);
void ControlModeRuntime_RunQVoltage(CurrentControlContext *CurrentControl,
	MotorControlContext *MotorControl, EncoderContext *Encoder,
	const RotorFeedbackFrame *feedback,
	MotorStateContext *motor_state);

#endif
