#ifndef CORE_APPLICATION_MOTOR_CONTROL_MOTOR_CONTROL_RUNTIME_H
#define CORE_APPLICATION_MOTOR_CONTROL_MOTOR_CONTROL_RUNTIME_H

#include "Core/Application/MotorControl/motor_drive_service.h"
#include "Bsp/Api/bsp_angle_sensor.h"
#include "bsp_system.h"
#include "Core/Application/Contracts/rotor_calibration_port.h"
#include "rotor_calibration_port_adapter.h"
#include "Core/Application/Contracts/motor_command_port.h"
#include "Core/Application/Contracts/motor_configuration_port.h"
#include "Core/Config/product_config.h"
#include "motor_control_types.h"
#include "pi_controller.h"
#include "encoder.h"
#include "sensorless_runtime.h"
#include "control_mode_runtime.h"
#include "measurement_model.h"
#include "phase_resistance_runtime.h"
#include "friction_identification_runtime.h"
#include "encoder_calibration_runtime.h"
#include "encoder_direction_calibration_runtime.h"
#include "cogging_identification_runtime.h"
#include "current_control_runtime.h"
#include "Core/Application/device_lifecycle.h"
#include "motor_state_runtime.h"
#include "motor_service_adapter.h"
#include "parameter_snapshot.h"
#include "Core/Application/Communication/can_configuration_service.h"
#include "Core/Infrastructure/Telemetry/telemetry_service.h"
#include "rotor_feedback_runtime.h"

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

/* ProductConfig projection owned by the motor-control composition boundary.
 * Each nested value belongs to the leaf module that consumes it; no board or
 * product descriptor is retained by the runtime. */
typedef struct
{
	uint32_t control_frequency_hz;
	uint32_t current_offset_calibration_sample_count;
	float command_current_limit_a;
	MotorCommissioningStageMask commissioning_stage_mask;
	MotorControlModeMask allowed_control_mode_mask;
	RotorFeedbackRuntimeConfig feedback;
	CalibrationCurrentOffsetLimits current_offset_limits;
	PhaseResistanceRuntimeBoardConfig phase_resistance;
	ParameterSnapshotBoardConfig parameter_snapshot;
} MotorControlRuntimeConfig;

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
	EncoderDirectionCalibrationContext encoder_direction_calibration;
	CoggingIdentificationRuntimeContext cogging_identification;
	CurrentControlContext current_control;
	DeviceLifecycleContext lifecycle;
	MotorStateContext motor_state;
	CalibrationServiceContext calibration_service;
	IdentificationServiceContext identification_service;
	MotorConfigurationAdapterContext configuration_adapter;
	MotorCommandAdapterContext command_adapter;
	RotorCalibrationAdapterContext rotor_calibration_adapter;
	ParameterSnapshotContext parameter_snapshot;
	RotorFeedbackRuntimeContext rotor_feedback;
	BspCriticalSectionPort critical_section;
	BspExecutionTimerPort execution_timer;
	MotorFastLoopMetrics fast_loop_metrics;
	uint32_t current_offset_calibration_sample_count;
	float command_current_limit_a;
	PhaseResistanceRuntimeBoardConfig phase_resistance_config;
	const ProductMotorDesign *motor_design;
	const ProductControlConfig *control_config;
	const ProductCommissioningTuningConfig *commissioning_tuning;
	bool previous_operation_requires_power;
	bool fault_shutdown_complete;
	ServiceProcedure previous_service_procedure;
} MotorControlRuntimeContext;

void MotorControlRuntime_Initialize(MotorControlRuntimeContext *context,
	MotorDriveServiceContext *motor_drive,
	const MeasurementModelConfig *measurement_config,
	const BspAngleSensorPort *angle_sensor_ports,
	uint8_t angle_sensor_count,
	const BspCriticalSectionPort *critical_section_port);
bool MotorControlRuntime_Prepare(MotorControlRuntimeContext *context,
	const MotorControlRuntimeConfig *runtime_config,
	const ProductMotorDesign *motor_design,
	const ProductControlConfig *control_config,
	const ProductCommissioningTuningConfig *commissioning_tuning,
	const ProductMotorAcceptanceConfig *motor_acceptance,
	const BspMonotonicClockPort *monotonic_clock,
	const BspExecutionTimerPort *execution_timer,
	CanConfigurationServiceContext *can_configuration);

void MotorControlRuntime_ExecuteFastLoop(MotorControlRuntimeContext *context);
/* Narrow supervision boundary for the most recent trustworthy temperature.
 * Invalid contexts and non-finite observations are rejected without changing
 * the value subsequently published through the existing telemetry path. */
bool MotorControlRuntime_UpdateSupervisedTemperature(
	MotorControlRuntimeContext *context, float temperature_c);
void MotorControlRuntime_PublishTelemetry(MotorControlRuntimeContext *context,
	TelemetryServiceContext *telemetry);
bool MotorControlRuntime_ReadDiagnosticFrame(
	MotorControlRuntimeContext *context, MotorDiagnosticFrame *frame);
bool MotorControlRuntime_ReadFastLoopMetrics(
	const MotorControlRuntimeContext *context, MotorFastLoopMetrics *metrics);
RotorCalibrationPort MotorControlRuntime_CreateRotorCalibrationPort(
	MotorControlRuntimeContext *context);
MotorCommandPort MotorControlRuntime_CreateCommandPort(
	MotorControlRuntimeContext *context);
MotorConfigurationPort MotorControlRuntime_CreateConfigurationPort(
	MotorControlRuntimeContext *context);
FrictionIdentificationPort MotorControlRuntime_CreateFrictionIdentificationPort(
	MotorControlRuntimeContext *context);

#endif
