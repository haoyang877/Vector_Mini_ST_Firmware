#ifndef RUNTIME_MOTOR_CONTROL_H
#define RUNTIME_MOTOR_CONTROL_H

#include "power_stage.h"
#include "measurement_port.h"
#include "rotor_sensor_port.h"
#include "critical_section_port.h"
#include "rotor_calibration_port.h"
#include "rotor_calibration_port_adapter.h"
#include "motor_command_port.h"
#include "motor_configuration_port.h"
#include "board_profile.h"
#include "motor_profiles.h"
#include "encoder_profiles.h"
#include "motor_control_types.h"
#include "pi_controller.h"
#include "encoder.h"
#include "sensorless_runtime.h"
#include "control_mode_runtime.h"
#include "measurement_model.h"
#include "phase_resistance_runtime.h"
#include "friction_identification_runtime.h"
#include "encoder_calibration_runtime.h"
#include "current_control_runtime.h"
#include "device_lifecycle.h"
#include "motor_state_runtime.h"
#include "motor_service_adapter.h"
#include "parameter_snapshot.h"
#include "control_tuning_profile.h"
#include "mechanical_load_profiles.h"
#include "monotonic_clock_port.h"
#include "execution_timer_port.h"

typedef struct
{
	int16_t channel[9];
} MotorDiagnosticFrame;

typedef struct
{
	uint32_t invocation_count;
	uint32_t maximum_cycles;
	uint32_t deadline_cycles;
	uint32_t deadline_overrun_count;
	uint32_t latest_cycles;
	uint32_t filtered_cycles;
} MotorFastLoopMetrics;

typedef struct
{
	MotorControlContext motor;
	PiController speed_controller;
	EncoderContext encoder;
	FluxObserverContext flux_observer;
	SensorlessStartupContext sensorless_startup;
	MotionControlContext motion_control;
	MeasurementModelContext measurement_model;
	PhaseResistanceRuntimeContext phase_resistance;
	FrictionIdentificationRuntimeContext friction_identification;
	MotorCalibrationContext calibration;
	CurrentOffsetCalibrationContext current_offset_calibration;
	ElectricalZeroCalibrationContext electrical_zero_calibration;
	CurrentControlContext current_control;
	DeviceLifecycleContext lifecycle;
	MotorStateContext motor_state;
	CalibrationServiceContext calibration_service;
	IdentificationServiceContext identification_service;
	MotorConfigurationAdapterContext configuration_adapter;
	MotorCommandAdapterContext command_adapter;
	RotorCalibrationAdapterContext rotor_calibration_adapter;
	ParameterSnapshotContext parameter_snapshot;
	CriticalSectionPort critical_section;
	ExecutionTimerPort execution_timer;
	MotorFastLoopMetrics fast_loop_metrics;
	const BoardProfile *board_profile;
	const MotorProfile *motor_profile;
	const EncoderProfile *encoder_profile;
	const ControlTuningProfile *tuning_profile;
	const MechanicalLoadProfile *mechanical_load_profile;
	bool previous_operation_requires_power;
	ServiceProcedure previous_service_procedure;
} MotorControlRuntimeContext;

void MotorControlRuntime_Initialize(PowerStageContext *power_stage,
	const MeasurementPort *measurement_port,
	const RotorSensorPort *rotor_sensor_port,
	const CriticalSectionPort *critical_section_port);
bool MotorControlRuntime_Prepare(MotorControlRuntimeContext *context,
	const BoardProfile *board_profile,
	const MotorProfile *motor_profile, const EncoderProfile *encoder_profile,
	const ControlTuningProfile *tuning_profile,
	const MechanicalLoadProfile *mechanical_load_profile,
	const MonotonicClockPort *monotonic_clock,
	const ExecutionTimerPort *execution_timer);

void MotorControlRuntime_ExecuteFastLoop(void);
void MotorControlRuntime_PublishTelemetry(void);
bool MotorControlRuntime_ReadDiagnosticFrame(MotorDiagnosticFrame *frame);
bool MotorControlRuntime_ReadFastLoopMetrics(MotorFastLoopMetrics *metrics);
RotorCalibrationPort MotorControlRuntime_CreateRotorCalibrationPort(void);
MotorCommandPort MotorControlRuntime_CreateCommandPort(void);
MotorConfigurationPort MotorControlRuntime_CreateConfigurationPort(void);
FrictionIdentificationPort MotorControlRuntime_CreateFrictionIdentificationPort(void);

#endif
