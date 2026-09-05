#include "motor_control_runtime.h"

#include <math.h>
#include <stdint.h>
#include "encoder.h"
#include "current_control_runtime.h"
#include "encoder_calibration_runtime.h"
#include "motor_state_runtime.h"
#include "parameter_snapshot.h"
#include "pi_controller.h"
#include "control_mode_runtime.h"
#include "measurement_runtime.h"
#include "sensorless_runtime.h"
#include "fast_math.h"
#include "phase_resistance_runtime.h"
#include "friction_identification_runtime.h"
#include "telemetry_service.h"
#include "rotor_calibration_port_adapter.h"
#include "motor_service_adapter.h"
#include "current_offset_calibration_runtime.h"
#include "electrical_zero_calibration_runtime.h"

static MotorControlRuntimeContext *ActiveRuntime;

#define MotorControl (ActiveRuntime->motor)
#define PI_Speed (ActiveRuntime->speed_controller)
#define OnBoard_Encoder (ActiveRuntime->encoder)
#define Fluxobserver (ActiveRuntime->flux_observer)
#define SensorlessStartup (ActiveRuntime->sensorless_startup)
#define MotionControl (ActiveRuntime->motion_control)
#define MeasurementModel (ActiveRuntime->measurement_model)
#define PhaseResistanceRuntime (ActiveRuntime->phase_resistance)
#define FrictionIdentificationRuntime (ActiveRuntime->friction_identification)
#define MotorCalibration (ActiveRuntime->calibration)
#define CurrentOffsetCalibration (ActiveRuntime->current_offset_calibration)
#define ElectricalZeroCalibration (ActiveRuntime->electrical_zero_calibration)
#define MotorLifecycle (ActiveRuntime->lifecycle)
#define MotorConfigurationAdapter (ActiveRuntime->configuration_adapter)
#define MotorCommandAdapter (ActiveRuntime->command_adapter)
#define RuntimeCriticalSection (ActiveRuntime->critical_section)
#define ActiveBoardProfile (ActiveRuntime->board_profile)
#define ActiveMotorProfile (ActiveRuntime->motor_profile)
#define ActiveEncoderProfile (ActiveRuntime->encoder_profile)
#define ActiveTuningProfile (ActiveRuntime->tuning_profile)
#define ActiveMechanicalLoadProfile (ActiveRuntime->mechanical_load_profile)
#define PreviousOperationRequiresPower (ActiveRuntime->previous_operation_requires_power)
#define PreviousServiceProcedure (ActiveRuntime->previous_service_procedure)
#define CurrentControl (ActiveRuntime->current_control)

bool MotorControlRuntime_Prepare(MotorControlRuntimeContext *context,
	const BoardProfile *board_profile,
	const MotorProfile *motor_profile, const EncoderProfile *encoder_profile,
	const ControlTuningProfile *tuning_profile,
	const MechanicalLoadProfile *mechanical_load_profile,
	const MonotonicClockPort *monotonic_clock,
	const ExecutionTimerPort *execution_timer)
{
	MotorFaultRuntimeBindings bindings;

	if (context == 0 || board_profile == 0 || motor_profile == 0 ||
		encoder_profile == 0 || tuning_profile == 0 ||
		mechanical_load_profile == 0 || monotonic_clock == 0 ||
		monotonic_clock->read_ms == 0 || execution_timer == 0 ||
		execution_timer->read_cycles == 0 ||
		execution_timer->cycles_per_second == 0 ||
		board_profile->control_frequency_hz == 0U ||
		!isfinite(tuning_profile->flux_observer_resistance_scale) ||
		tuning_profile->flux_observer_resistance_scale <= 0.0f)
		return false;
	ActiveRuntime = context;
	ActiveBoardProfile = board_profile;
	ActiveMotorProfile = motor_profile;
	ActiveEncoderProfile = encoder_profile;
	ActiveTuningProfile = tuning_profile;
	ActiveMechanicalLoadProfile = mechanical_load_profile;
	MotorControl.tuning_profile = tuning_profile;
	MotorControl.mechanical_load_profile = mechanical_load_profile;
	ActiveRuntime->execution_timer = *execution_timer;
	ActiveRuntime->fast_loop_metrics.invocation_count = 0U;
	ActiveRuntime->fast_loop_metrics.maximum_cycles = 0U;
	ActiveRuntime->fast_loop_metrics.deadline_overrun_count = 0U;
	ActiveRuntime->fast_loop_metrics.latest_cycles = 0U;
	ActiveRuntime->fast_loop_metrics.filtered_cycles = 0U;
	ActiveRuntime->fast_loop_metrics.deadline_cycles =
		execution_timer->cycles_per_second(execution_timer->context) /
		board_profile->control_frequency_hz;
	PreviousServiceProcedure = SERVICE_PROCEDURE_NONE;

	bindings.motor = &MotorControl;
	bindings.current_control = &CurrentControl;
	bindings.speed_controller = &PI_Speed;
	bindings.encoder = &OnBoard_Encoder;
	bindings.sensorless_startup = &SensorlessStartup;
	bindings.motion_control = &MotionControl;
	bindings.motor_calibration = &MotorCalibration;
	bindings.lifecycle = &MotorLifecycle;
	bindings.state = &ActiveRuntime->motor_state;
	bindings.calibration_service = &ActiveRuntime->calibration_service;
	bindings.identification_service = &ActiveRuntime->identification_service;
	bindings.monotonic_clock = *monotonic_clock;
	return MotorState_Initialize(&bindings) &&
		CalibrationService_Initialize(&ActiveRuntime->calibration_service,
			&MotorLifecycle, board_profile, 120000U) &&
		IdentificationService_Initialize(&ActiveRuntime->identification_service,
			&MotorLifecycle, motor_profile, 120000U) &&
		ParameterSnapshot_Initialize(&ActiveRuntime->parameter_snapshot,
			&MotorControl, &OnBoard_Encoder,
			board_profile, motor_profile, encoder_profile);
}

RotorCalibrationPort MotorControlRuntime_CreateRotorCalibrationPort(void)
{
	return RotorCalibrationAdapter_CreatePort(
		&ActiveRuntime->rotor_calibration_adapter, &OnBoard_Encoder,
		&MotorControl);
}

MotorCommandPort MotorControlRuntime_CreateCommandPort(void)
{
	return MotorServiceAdapter_CreateCommandPort(&MotorCommandAdapter,
		&MotorControl, &RuntimeCriticalSection);
}

MotorConfigurationPort MotorControlRuntime_CreateConfigurationPort(void)
{
	return MotorServiceAdapter_CreateConfigurationPort(&MotorConfigurationAdapter,
		&MotorControl, &RuntimeCriticalSection, ActiveMotorProfile);
}

FrictionIdentificationPort MotorControlRuntime_CreateFrictionIdentificationPort(void)
{
	return FrictionIdentificationRuntime_CreatePort(
		&FrictionIdentificationRuntime, &MotorControl,
		&MotorConfigurationAdapter, &RuntimeCriticalSection);
}

void MotorControlRuntime_PublishTelemetry(void)
{
	MotorTelemetrySnapshot snapshot;
	FaultRecord primary_fault_record = {0U, 0U, 0U};
	MotorFastLoopMetrics fast_loop_metrics = {0};
	uint8_t fault_code;

	snapshot.mode = (uint32_t)MotorLifecycle_GetProtocolActionCode();
	snapshot.primary_error = (uint32_t)MotorControl.runtime.primary_fault;
	snapshot.active_faults = MotorFaults_GetActiveSet();
	snapshot.latched_faults = MotorFaults_GetLatchedSet();
	snapshot.fault_event_sequence = MotorFaults_GetEventSequence();
	for (fault_code = 0U; fault_code < MOTOR_TELEMETRY_FAULT_CODE_COUNT;
		fault_code++)
	{
		FaultRecord record = {0U, 0U, 0U};
		(void)MotorFaults_GetRecord(fault_code, &record);
		snapshot.fault_occurrence_count[fault_code] =
			record.occurrence_count;
	}
	(void)MotorControlRuntime_ReadFastLoopMetrics(&fast_loop_metrics);
	snapshot.fast_loop_invocation_count = fast_loop_metrics.invocation_count;
	snapshot.fast_loop_maximum_cycles = fast_loop_metrics.maximum_cycles;
	snapshot.fast_loop_deadline_cycles = fast_loop_metrics.deadline_cycles;
	snapshot.fast_loop_deadline_overrun_count =
		fast_loop_metrics.deadline_overrun_count;
	(void)MotorFaults_GetRecord((uint8_t)snapshot.primary_error,
		&primary_fault_record);
	snapshot.primary_fault_occurrence_count =
		primary_fault_record.occurrence_count;
	snapshot.primary_fault_first_event_sequence =
		primary_fault_record.first_event_sequence;
	snapshot.primary_fault_latest_event_sequence =
		primary_fault_record.latest_event_sequence;
	snapshot.primary_fault_first_time_ms = primary_fault_record.first_time_ms;
	snapshot.primary_fault_latest_time_ms = primary_fault_record.latest_time_ms;
	snapshot.fault_bus_voltage_v =
		primary_fault_record.latest_observation.bus_voltage_v;
	snapshot.fault_phase_a_current_a =
		primary_fault_record.latest_observation.phase_a_current_a;
	snapshot.fault_phase_b_current_a =
		primary_fault_record.latest_observation.phase_b_current_a;
	snapshot.fault_phase_c_current_a =
		primary_fault_record.latest_observation.phase_c_current_a;
	snapshot.fault_temperature_c =
		primary_fault_record.latest_observation.temperature_c;
	snapshot.fault_mechanical_position_rad =
		primary_fault_record.latest_observation.mechanical_position_rad;
	snapshot.fault_mechanical_speed_rad_s =
		primary_fault_record.latest_observation.mechanical_speed_rad_s;
	snapshot.current_reference_a = MotorControl.command.q_axis_current_reference_a;
	snapshot.speed_reference_rad_s = MotorControl.command.speed_reference_rad_s;
	snapshot.position_reference_rad = MotorControl.command.position_reference_rad;
	snapshot.d_axis_current_target_a = MotorControl.targets.d_axis_current_a;
	snapshot.q_axis_current_target_a = MotorControl.targets.q_axis_current_a;
	snapshot.speed_target_rad_s = MotorControl.targets.speed_rad_s;
	snapshot.position_target_rad = MotorControl.targets.position_rad;
	snapshot.bus_voltage_v = CurrentControl.filtered_bus_voltage_v;
	snapshot.bus_current_a = CurrentControl.filtered_bus_current_a;
	snapshot.phase_a_current_a = CurrentControl.phase_a_current_a;
	snapshot.phase_b_current_a = CurrentControl.phase_b_current_a;
	snapshot.phase_c_current_a = CurrentControl.phase_c_current_a;
	snapshot.d_axis_current_a = CurrentControl.d_axis_current_a;
	snapshot.q_axis_current_a = CurrentControl.q_axis_current_a;
	snapshot.d_axis_current_filtered_a = CurrentControl.filtered_d_axis_current_a;
	snapshot.q_axis_current_filtered_a = CurrentControl.filtered_q_axis_current_a;
	snapshot.mechanical_speed_rad_s = OnBoard_Encoder.vel_mech;
	snapshot.mechanical_position_rad = OnBoard_Encoder.theta_mech;
	snapshot.temperature_c = CurrentControl.temperature_c;
	snapshot.encoder_online = Encoder_IsOnline(&OnBoard_Encoder) ? 1U : 0U;
	snapshot.encoder_reversed = OnBoard_Encoder.reverse != 0U ? 1U : 0U;
	snapshot.pole_pairs = (float)MotorControl.configuration.pole_pairs;
	snapshot.calibration_current_a = MotorControl.configuration.calibration_current_a;
	snapshot.current_limit_a = MotorControl.configuration.current_limit_a;
	snapshot.speed_limit_rad_s = MotorControl.configuration.speed_limit_rad_s;
	snapshot.speed_acceleration_rad_s2 = MotorControl.configuration.speed_acceleration_rad_s2;
	snapshot.speed_deceleration_rad_s2 = MotorControl.configuration.speed_deceleration_rad_s2;
	snapshot.speed_kp = MotorControl.configuration.speed_kp;
	snapshot.speed_ki = MotorControl.configuration.speed_ki;
	snapshot.position_acceleration_rad_s2 = MotorControl.configuration.position_acceleration_rad_s2;
	snapshot.position_deceleration_rad_s2 = MotorControl.configuration.position_deceleration_rad_s2;
	snapshot.position_max_speed_rad_s = MotorControl.configuration.position_max_speed_rad_s;
	snapshot.position_kp_a_per_rad = MotorControl.configuration.position_kp_a_per_rad;
	snapshot.position_kd_a_per_rad_s = MotorControl.configuration.position_kd_a_per_rad_s;
	snapshot.position_ki_a_per_rad_s = MotorControl.configuration.position_ki_a_per_rad_s;
	snapshot.position_integral_limit_a = MotorControl.configuration.position_integral_limit_a;
	snapshot.cascade_position_kp_per_s = MotorControl.configuration.cascade_position_kp_per_s;
	snapshot.cascade_position_kd = MotorControl.configuration.cascade_position_kd;
	snapshot.phase_resistance_ohm = MotorControl.configuration.phase_resistance_ohm;
	snapshot.d_axis_inductance_h = MotorControl.configuration.d_axis_inductance_h;
	snapshot.q_axis_inductance_h = MotorControl.configuration.q_axis_inductance_h;
	snapshot.flux_weber = MotorControl.configuration.flux_weber;

	TelemetryService_Publish(&snapshot);
}

#define RTT_SPEED_SCALE_COUNTS_PER_RAD_S	10000.0f
#define RTT_CURRENT_SCALE_COUNTS_PER_A		1000.0f
#define RTT_VOLTAGE_SCALE_COUNTS_PER_V		1000.0f
#define RTT_ANGLE_Q15_SCALE				(32768.0f / MATH_PI)

static int16_t RTT_EncodeInt16(float value, float scale)
{
	float scaled;

	if (!isfinite(value) || !isfinite(scale))
		return 0;
	scaled = value * scale;
	if (scaled > 32767.0f)
		return 32767;
	if (scaled < -32768.0f)
		return -32768;

	return (int16_t)scaled;
}

static int16_t RTT_EncodeAngleQ15(float angle)
{
	if (!isfinite(angle))
		return 0;

	/* Signed Q15 angle: 0 rad -> 0; wrap occurs at +/-pi. */
	angle = FastMath_NormalizeAngle(angle);
	if (angle >= MATH_PI)
		angle -= MATH_TWO_PI;
	return RTT_EncodeInt16(angle, RTT_ANGLE_Q15_SCALE);
}

bool MotorControlRuntime_ReadDiagnosticFrame(MotorDiagnosticFrame *frame)
{
	PhaseResistanceRuntimeTelemetry phase_rtt;
	float encoder_theta_elec;
	float observer_theta_elec;
	float phase_error;

	int16_t *data;
	if (frame == 0)
		return false;
	data = frame->channel;

	if (MotorLifecycle_GetDeviceState() == DEVICE_STATE_SERVICING &&
		MotorLifecycle_GetServiceProcedure() ==
		SERVICE_PROCEDURE_PHASE_RESISTANCE_IDENTIFICATION)
	{
		if (PhaseResistanceRuntime_GetTelemetry(&PhaseResistanceRuntime, &phase_rtt))
		{
			/* angle[mrad], IdRef/Id/Iq[mA], Vd/Vq[mV], I/U[mA/mV], Vbus[mV]. */
			data[0] = RTT_EncodeInt16(phase_rtt.electrical_angle, 1000.0f);
			data[1] = RTT_EncodeInt16(phase_rtt.id_ref, 1000.0f);
			data[2] = RTT_EncodeInt16(phase_rtt.id, 1000.0f);
			data[3] = RTT_EncodeInt16(phase_rtt.iq, 1000.0f);
			data[4] = RTT_EncodeInt16(phase_rtt.vd, 1000.0f);
			data[5] = RTT_EncodeInt16(phase_rtt.vq, 1000.0f);
			data[6] = RTT_EncodeInt16(phase_rtt.current_magnitude, 1000.0f);
			data[7] = RTT_EncodeInt16(phase_rtt.parallel_voltage, 1000.0f);
			data[8] = RTT_EncodeInt16(phase_rtt.vbus, 1000.0f);
		}
		else
		{
			uint8_t index;
			for (index = 0U; index < 9U; ++index)
				data[index] = 0;
		}
	}
	else if (MotorLifecycle_GetDeviceState() == DEVICE_STATE_SERVICING &&
		MotorLifecycle_GetServiceProcedure() ==
		SERVICE_PROCEDURE_OBSERVER_CALIBRATION)
	{
		/* Mode 13 diagnostics: calibration/startup states, actual/open-loop/
		 * observer/lock speeds, Iq reference/actual and encoder-observer phase.
		 * Electrical speeds use 0.1 rad/s; mechanical speed uses 0.01 rad/s. */
		encoder_theta_elec = FastMath_NormalizeAngle(OnBoard_Encoder.theta_elec);
		observer_theta_elec = FastMath_NormalizeAngle(
			FluxObserver_GetElectricalAngle(&Fluxobserver));
		phase_error = encoder_theta_elec - observer_theta_elec;
		if (phase_error >= MATH_PI)
			phase_error -= MATH_TWO_PI;
		else if (phase_error < -MATH_PI)
			phase_error += MATH_TWO_PI;

		data[0] = (int16_t)MotorCalibration.step;
		data[1] = (int16_t)SensorlessStartup.state;
		data[2] = RTT_EncodeInt16(OnBoard_Encoder.vel_mech, 100.0f);
		data[3] = RTT_EncodeInt16(SensorlessStartup.open_loop_omega, 10.0f);
		data[4] = RTT_EncodeInt16(
			FluxObserver_GetElectricalVelocity(&Fluxobserver), 10.0f);
		data[5] = RTT_EncodeInt16(SensorlessStartup.lock_speed_feedback, 10.0f);
		data[6] = RTT_EncodeInt16(MotorControl.targets.q_axis_current_a, 1000.0f);
		data[7] = RTT_EncodeInt16(CurrentControl.q_axis_current_a, 1000.0f);
		data[8] = RTT_EncodeAngleQ15(phase_error);
	}
	else if (MotorLifecycle_GetDeviceState() == DEVICE_STATE_ACTIVE &&
		(MotorLifecycle_GetControlMode() == MOTOR_CONTROL_MODE_POSITION_CASCADE ||
		 MotorLifecycle_GetControlMode() == MOTOR_CONTROL_MODE_POSITION_IMPEDANCE))
	{
		/*
		 * Position-control telemetry, nine signed 16-bit channels:
		 * position/electrical angles use signed Q15; speed uses 0.0001 rad/s;
		 * current and voltage use mA and mV respectively.
		 */
		data[0] = RTT_EncodeAngleQ15(MotorControl.runtime.position_command_ramp_rad);
		data[1] = RTT_EncodeAngleQ15(OnBoard_Encoder.theta_mech);
		data[2] = RTT_EncodeInt16(MotorControl.runtime.speed_command_ramp_rad_s,
			RTT_SPEED_SCALE_COUNTS_PER_RAD_S);
		data[3] = RTT_EncodeInt16(MotorControl.runtime.position_velocity_filtered_rad_s,
			RTT_SPEED_SCALE_COUNTS_PER_RAD_S);
		data[4] = RTT_EncodeInt16(MotorControl.targets.q_axis_current_a,
			RTT_CURRENT_SCALE_COUNTS_PER_A);
		data[5] = RTT_EncodeInt16(CurrentControl.q_axis_current_a,
			RTT_CURRENT_SCALE_COUNTS_PER_A);
		data[6] = RTT_EncodeInt16(CurrentControl.q_axis_modulation * CurrentControl.filtered_bus_voltage_v / 1.5f,
			RTT_VOLTAGE_SCALE_COUNTS_PER_V);
		data[7] = RTT_EncodeInt16(CurrentControl.d_axis_modulation * CurrentControl.filtered_bus_voltage_v / 1.5f,
			RTT_VOLTAGE_SCALE_COUNTS_PER_V);
		data[8] = RTT_EncodeAngleQ15(OnBoard_Encoder.theta_elec);
	}
	else
	{
		/* Preserve the existing observer diagnostics outside position/calibration modes. */
		encoder_theta_elec = FastMath_NormalizeAngle(OnBoard_Encoder.theta_elec);
		observer_theta_elec = FastMath_NormalizeAngle(FluxObserver_GetElectricalAngle(&Fluxobserver));
		phase_error = encoder_theta_elec - observer_theta_elec;
		if (phase_error >= MATH_PI)
			phase_error -= MATH_TWO_PI;
		else if (phase_error < -MATH_PI)
			phase_error += MATH_TWO_PI;

		data[0] = RTT_EncodeAngleQ15(phase_error);
		data[1] = RTT_EncodeInt16(OnBoard_Encoder.vel_mech, 100.0f);
		data[2] = RTT_EncodeInt16(MotorControl.targets.q_axis_current_a, 1000.0f);
		data[3] = RTT_EncodeInt16(CurrentControl.q_axis_current_a, 1000.0f);
		data[4] = RTT_EncodeInt16(CurrentControl.d_axis_current_a, 1000.0f);
		data[5] = RTT_EncodeInt16(CurrentControl.q_axis_modulation * CurrentControl.filtered_bus_voltage_v / 1.5f, 1000.0f);
		data[6] = RTT_EncodeInt16(CurrentControl.d_axis_modulation * CurrentControl.filtered_bus_voltage_v / 1.5f, 1000.0f);
		data[7] = RTT_EncodeAngleQ15(encoder_theta_elec - MATH_PI);
		data[8] = RTT_EncodeAngleQ15(observer_theta_elec - MATH_PI);
	}
	return true;
}

/**
	* @brief  Initialize motor control parameters
 **/
void MotorControlRuntime_Initialize(PowerStageContext *power_stage,
	const MeasurementPort *measurement_port,
	const RotorSensorPort *rotor_sensor_port,
	const CriticalSectionPort *critical_section_port)
{
	if (!Encoder_ParamInit(&OnBoard_Encoder, rotor_sensor_port,
		critical_section_port, ActiveEncoderProfile->speed_loop_divider,
		ActiveEncoderProfile->speed_sample_period_s))
		MotorState_RaiseFault(MOTOR_FAULT_ENCODER);
	
	FluxObserver_Initialize(&Fluxobserver, ActiveTuningProfile, &MotorControl);
	SensorlessStartup_Reset(&SensorlessStartup);
	CurrentControlRuntime_ResetControllers(&CurrentControl);
	CurrentControlRuntime_ConfigureControllers(&CurrentControl, &MotorControl);
	MeasurementModel_Reset(&MeasurementModel);
	if (!Measurement_Configure(&MeasurementModel, &MotorControl,
		ActiveBoardProfile))
		MotorState_RaiseFault(MOTOR_FAULT_INVALID_PARAMETER);
	MotorCalibration_Reset(&MotorCalibration);
	CurrentOffsetCalibrationRuntime_Reset(&CurrentOffsetCalibration);
	ElectricalZeroCalibrationRuntime_Reset(&ElectricalZeroCalibration);
	CurrentControl.power_stage = power_stage;
	if (critical_section_port != 0)
		RuntimeCriticalSection = *critical_section_port;
	if (measurement_port != 0)
		CurrentControl.measurement_port = *measurement_port;
	PI_Controller_Reset(&PI_Speed);
	
	MotorControl.runtime.position_velocity_filtered_rad_s = 0.0f;
	ControlModeRuntime_ResetPosition(&MotionControl);
	
	/*run current offset calibration automatically at power-up*/
	(void)MotorLifecycle_RequestService(
		SERVICE_PROCEDURE_CURRENT_OFFSET_CALIBRATION);
	MotorControlRuntime_PublishTelemetry();
}

/**
	* @brief  CurrentControl task, motor control related
			  use finite state machine
 **/
static void MotorControlRuntime_SetMechanicalZero(MotorControlContext *motor,
	EncoderContext *encoder)
{
	if (!Encoder_SetMechanicalZero(encoder))
	{
		MotorState_RaiseFault(MOTOR_FAULT_ENCODER);
		return;
	}

	ControlModeRuntime_ResetPosition(&MotionControl);
	MotorLifecycle_ReportServiceComplete(true);
}

static bool Encoder_FeedbackRequired(const MotorControlContext *motor,
	DeviceState device_state, MotorControlMode mode,
	ServiceProcedure procedure)
{
	if (device_state == DEVICE_STATE_ACTIVE && mode == MOTOR_CONTROL_MODE_CURRENT)
		return !motor->configuration.use_sensorless_feedback;

	if (device_state == DEVICE_STATE_ACTIVE)
		return mode == MOTOR_CONTROL_MODE_SPEED ||
			mode == MOTOR_CONTROL_MODE_POSITION_CASCADE ||
			mode == MOTOR_CONTROL_MODE_POSITION_IMPEDANCE ||
			mode == MOTOR_CONTROL_MODE_VQ;

	return device_state == DEVICE_STATE_SERVICING &&
		(procedure == SERVICE_PROCEDURE_ENCODER_LINEARIZATION ||
		 procedure == SERVICE_PROCEDURE_OBSERVER_CALIBRATION ||
		 procedure == SERVICE_PROCEDURE_ELECTRICAL_ZERO_CALIBRATION ||
		 procedure == SERVICE_PROCEDURE_FRICTION_IDENTIFICATION ||
		 procedure == SERVICE_PROCEDURE_SET_MECHANICAL_ZERO);
}

static bool LifecycleRequiresPowerStage(DeviceState device_state,
	ServiceProcedure procedure, ProcedureState procedure_state)
{
	if (device_state == DEVICE_STATE_ACTIVE)
		return true;
	if (device_state != DEVICE_STATE_SERVICING ||
		procedure_state != PROCEDURE_STATE_RUNNING)
		return false;
	return procedure == SERVICE_PROCEDURE_ENCODER_LINEARIZATION ||
		procedure == SERVICE_PROCEDURE_OBSERVER_CALIBRATION ||
		procedure == SERVICE_PROCEDURE_ELECTRICAL_ZERO_CALIBRATION ||
		procedure == SERVICE_PROCEDURE_PHASE_RESISTANCE_IDENTIFICATION ||
		procedure == SERVICE_PROCEDURE_FRICTION_IDENTIFICATION;
}

static void MotorControlRuntime_PrepareControlTargets(DeviceState device_state,
	MotorControlMode control_mode)
{
	if (device_state != DEVICE_STATE_ACTIVE)
		return;

	switch (control_mode)
	{
		case MOTOR_CONTROL_MODE_CURRENT:
			MotorControl.targets.d_axis_current_a =
				MotorControl.command.d_axis_current_reference_a;
			MotorControl.targets.q_axis_current_a =
				MotorControl.command.q_axis_current_reference_a;
			break;
		case MOTOR_CONTROL_MODE_SPEED:
		case MOTOR_CONTROL_MODE_SENSORLESS_SPEED:
			MotorControl.targets.speed_rad_s =
				MotorControl.command.speed_reference_rad_s;
			break;
		case MOTOR_CONTROL_MODE_POSITION_CASCADE:
		case MOTOR_CONTROL_MODE_POSITION_IMPEDANCE:
			MotorControl.targets.position_rad =
				MotorControl.command.position_reference_rad;
			break;
		case MOTOR_CONTROL_MODE_VQ:
			MotorControl.targets.q_axis_voltage_v =
				MotorControl.command.q_axis_voltage_reference_v;
			break;
		default:
			break;
	}
}

static void MotorControlRuntime_EnterFaultedState(void)
{
	/* Hardware output disable is deliberately the first fault-side effect. */
	MotorState_DisablePowerStage();
	PhaseResistanceRuntime_Cancel(&PhaseResistanceRuntime, &CurrentControl, &MotorControl);
	if (FrictionIdentificationRuntime.started)
		FrictionIdentification_Fail(&FrictionIdentificationRuntime.core,
			FRICTION_IDENT_REASON_SAFETY_FAULT);
	FrictionIdentificationRuntime_Cancel(&FrictionIdentificationRuntime,
		&CurrentControl, &MotorControl, &PI_Speed);
	CurrentOffsetCalibrationRuntime_Reset(&CurrentOffsetCalibration);
	ElectricalZeroCalibrationRuntime_Reset(&ElectricalZeroCalibration);
	MotorState_ResetControlState();
	PreviousOperationRequiresPower = false;
	MotorControl.runtime.action_telemetry =
		(float)MotorLifecycle_GetProtocolActionCode();
	MotorControl.runtime.fault_telemetry = (float)MotorControl.runtime.primary_fault;
}

static void MotorControlRuntime_CancelExitedService(DeviceState device_state,
	ServiceProcedure service_procedure)
{
	if (PreviousServiceProcedure == SERVICE_PROCEDURE_NONE ||
		(device_state == DEVICE_STATE_SERVICING &&
		 service_procedure == PreviousServiceProcedure))
		return;
	switch (PreviousServiceProcedure)
	{
		case SERVICE_PROCEDURE_CURRENT_OFFSET_CALIBRATION:
			CurrentOffsetCalibrationRuntime_Reset(&CurrentOffsetCalibration);
			break;
		case SERVICE_PROCEDURE_ELECTRICAL_ZERO_CALIBRATION:
			ElectricalZeroCalibrationRuntime_Reset(&ElectricalZeroCalibration);
			break;
		case SERVICE_PROCEDURE_ENCODER_LINEARIZATION:
		case SERVICE_PROCEDURE_OBSERVER_CALIBRATION:
			MotorCalibration_Reset(&MotorCalibration);
			break;
		case SERVICE_PROCEDURE_PHASE_RESISTANCE_IDENTIFICATION:
			PhaseResistanceRuntime_Cancel(&PhaseResistanceRuntime, &CurrentControl,
				&MotorControl);
			break;
		case SERVICE_PROCEDURE_FRICTION_IDENTIFICATION:
			FrictionIdentificationRuntime_Cancel(&FrictionIdentificationRuntime,
				&CurrentControl, &MotorControl, &PI_Speed);
			break;
		default:
			break;
	}
}

static bool MotorControlRuntime_RequiresFluxObserver(DeviceState device_state,
	MotorControlMode mode, ServiceProcedure service_procedure)
{
	if (device_state == DEVICE_STATE_ACTIVE)
	{
		return mode == MOTOR_CONTROL_MODE_SENSORLESS_SPEED ||
			(mode == MOTOR_CONTROL_MODE_CURRENT &&
			 MotorControl.configuration.use_sensorless_feedback);
	}
	return device_state == DEVICE_STATE_SERVICING &&
		service_procedure == SERVICE_PROCEDURE_OBSERVER_CALIBRATION;
}

static bool MotorControlRuntime_MeasurementProtectionIsActive(
	DeviceState device_state, ServiceProcedure service_procedure)
{
	if (device_state == DEVICE_STATE_ACTIVE)
		return true;
	return device_state == DEVICE_STATE_SERVICING &&
		(service_procedure == SERVICE_PROCEDURE_ENCODER_LINEARIZATION ||
		 service_procedure == SERVICE_PROCEDURE_ELECTRICAL_ZERO_CALIBRATION ||
		 service_procedure == SERVICE_PROCEDURE_OBSERVER_CALIBRATION ||
		 service_procedure == SERVICE_PROCEDURE_PHASE_RESISTANCE_IDENTIFICATION ||
		 service_procedure == SERVICE_PROCEDURE_FRICTION_IDENTIFICATION);
}

static void MotorControlRuntime_ExecuteFastLoopBody(void)
{
	bool operation_requires_power;
	DeviceState device_state;
	MotorControlMode control_mode;
	ServiceProcedure service_procedure;
	ProcedureState procedure_state;
	DeviceState sampled_device_state;
	MotorControlMode sampled_control_mode;
	ServiceProcedure sampled_service_procedure;

	sampled_device_state = MotorLifecycle_GetDeviceState();
	sampled_control_mode = MotorLifecycle_GetControlMode();
	sampled_service_procedure = MotorLifecycle_GetServiceProcedure();

	if (!Measurement_Capture(&CurrentControl))
	{
		MotorState_RaiseFault(MOTOR_FAULT_POWER_STAGE);
		MotorControlRuntime_EnterFaultedState();
		return;
	}
	if (!Measurement_Process(&MeasurementModel, &CurrentControl,
		MotorControlRuntime_MeasurementProtectionIsActive(
			sampled_device_state, sampled_service_procedure)))
	{
		MotorState_RaiseFault(MOTOR_FAULT_POWER_STAGE);
		MotorControlRuntime_EnterFaultedState();
		return;
	}
	CurrentControlRuntime_UpdatePhaseCurrents(&CurrentControl);
	
	Encoder_Update(&OnBoard_Encoder,
		MotorControl.configuration.pole_pairs > 0 ?
		(uint32_t)MotorControl.configuration.pole_pairs : 1U);
	/* Encoder-feedback control and phase-commanded calibration do not consume
	 * observer feedback. Keep the observer off those paths to preserve the
	 * 20 kHz deadline and main-loop/USB bandwidth. */
	if (MotorControlRuntime_RequiresFluxObserver(sampled_device_state,
		sampled_control_mode, sampled_service_procedure))
		FluxObserver_Update(&CurrentControl, &Fluxobserver);

	if (Encoder_FeedbackRequired(&MotorControl, sampled_device_state,
		sampled_control_mode, sampled_service_procedure) &&
		OnBoard_Encoder.bad_frame_streak >= ENCODER_BAD_FRAME_OFFLINE_COUNT)
		MotorState_RaiseFault(MOTOR_FAULT_ENCODER);

	/* Protection owns the cycle: never execute control after a blocking fault. */
	if (MotorFaults_HasActive())
	{
		MotorControlRuntime_EnterFaultedState();
		return;
	}
	if (MotorServiceAdapter_ApplyPendingConfiguration(
		&MotorConfigurationAdapter))
	{
		CurrentControlRuntime_ConfigureControllers(&CurrentControl,
			&MotorControl);
		(void)FluxObserver_ConfigureMotor(&Fluxobserver, &MotorControl);
		(void)Measurement_Configure(&MeasurementModel, &MotorControl,
			ActiveBoardProfile);
	}
	(void)MotorServiceAdapter_ApplyPendingCommand(&MotorCommandAdapter);
	device_state = MotorLifecycle_GetDeviceState();
	control_mode = MotorLifecycle_GetControlMode();
	service_procedure = MotorLifecycle_GetServiceProcedure();
	procedure_state = MotorLifecycle_GetProcedureState();
	MotorControlRuntime_PrepareControlTargets(device_state, control_mode);
	MotorControlRuntime_CancelExitedService(device_state, service_procedure);

	if (device_state == DEVICE_STATE_ACTIVE)
	{
		switch (control_mode)
		{
			case MOTOR_CONTROL_MODE_CURRENT:
				ControlModeRuntime_RunCurrent(&CurrentControl, &MotorControl,
					&OnBoard_Encoder, &Fluxobserver);
				break;
			case MOTOR_CONTROL_MODE_SPEED:
				ControlModeRuntime_RunSpeed(&MotionControl, &CurrentControl,
					&MotorControl, &PI_Speed, &OnBoard_Encoder);
				break;
			case MOTOR_CONTROL_MODE_SENSORLESS_SPEED:
				ControlModeRuntime_RunSensorlessSpeed(&CurrentControl, &MotorControl,
					&PI_Speed, &Fluxobserver, &SensorlessStartup,
					&ActiveTuningProfile->sensorless_startup);
				break;
			case MOTOR_CONTROL_MODE_POSITION_CASCADE:
				ControlModeRuntime_RunPositionCascade(&MotionControl, &CurrentControl,
					&MotorControl, &OnBoard_Encoder, ActiveMotorProfile);
				break;
			case MOTOR_CONTROL_MODE_POSITION_IMPEDANCE:
				ControlModeRuntime_RunPositionImpedance(&MotionControl, &CurrentControl,
					&MotorControl, &OnBoard_Encoder, ActiveMotorProfile,
					ActiveBoardProfile);
				break;
			case MOTOR_CONTROL_MODE_VOLTAGE_OPEN_LOOP:
				ControlModeRuntime_RunVoltageOpenLoop(&CurrentControl, &MotorControl);
				break;
			case MOTOR_CONTROL_MODE_VQ:
				ControlModeRuntime_RunQVoltage(&CurrentControl, &MotorControl,
					&OnBoard_Encoder);
				break;
			default:
				CurrentControlRuntime_ApplyHighSideZeroVector(&CurrentControl);
				break;
		}
	}
	else if (device_state == DEVICE_STATE_SERVICING &&
		procedure_state == PROCEDURE_STATE_RUNNING)
		{
		switch (service_procedure)
		{
			case SERVICE_PROCEDURE_CURRENT_OFFSET_CALIBRATION:
			{
				uint16_t phase_a_offset_adc;
				uint16_t phase_b_offset_adc;
				uint16_t phase_c_offset_adc;
				if (CurrentOffsetCalibrationRuntime_ExecuteStep(
						&CurrentOffsetCalibration, &CurrentControl,
						ActiveBoardProfile) ==
					CURRENT_OFFSET_CALIBRATION_COMPLETE)
				{
					bool accepted = CurrentOffsetCalibrationRuntime_ReadResult(
						&CurrentOffsetCalibration, &phase_a_offset_adc,
						&phase_b_offset_adc, &phase_c_offset_adc) &&
						CalibrationService_AcceptCurrentOffsetResult(
							&ActiveRuntime->calibration_service,
							phase_a_offset_adc, phase_b_offset_adc,
							phase_c_offset_adc) &&
						MotorServiceAdapter_StageCurrentOffsetResult(
							&MotorConfigurationAdapter, phase_a_offset_adc,
							phase_b_offset_adc, phase_c_offset_adc);
					CurrentOffsetCalibrationRuntime_Reset(
						&CurrentOffsetCalibration);
					if (accepted)
						MotorLifecycle_ReportServiceComplete(false);
					else
						MotorState_RaiseFault(MOTOR_FAULT_CURRENT_OFFSET);
				}
				break;
			}
			case SERVICE_PROCEDURE_ENCODER_LINEARIZATION:
				CalibrationRuntime_RunEncoderLinearization(&MotorCalibration,
					&CurrentControl, &MotorControl, &OnBoard_Encoder);
				break;
			case SERVICE_PROCEDURE_ELECTRICAL_ZERO_CALIBRATION:
				ElectricalZeroCalibrationRuntime_ExecuteStep(
					&ElectricalZeroCalibration, &CurrentControl,
					&MotorControl, &OnBoard_Encoder);
				break;
			case SERVICE_PROCEDURE_OBSERVER_CALIBRATION:
				CalibrationRuntime_RunEncoderObserver(&MotorCalibration, &CurrentControl,
					&MotorControl, &PI_Speed, &OnBoard_Encoder, &Fluxobserver,
					&SensorlessStartup);
				break;
			case SERVICE_PROCEDURE_PHASE_RESISTANCE_IDENTIFICATION:
			{
				PhaseResistanceRuntimeStatus status = PhaseResistanceRuntime_Run(
					&PhaseResistanceRuntime, &CurrentControl, &MotorControl,
					ActiveBoardProfile, ActiveMotorProfile);
				if (status == PHASE_RESISTANCE_MODE_DONE)
				{
					float mean_resistance_ohm;
					if (IdentificationService_AcceptPhaseResistanceResult(
							&ActiveRuntime->identification_service,
							MotorControl.runtime.phase_a_resistance_ohm,
							MotorControl.runtime.phase_b_resistance_ohm,
							MotorControl.runtime.phase_c_resistance_ohm,
							MotorControl.runtime.phase_resistance_spread_pct,
							MotorControl.runtime.phase_resistance_balanced,
							&mean_resistance_ohm) &&
						MotorServiceAdapter_StagePhaseResistanceResult(
							&MotorConfigurationAdapter,
							mean_resistance_ohm))
						MotorLifecycle_ReportServiceComplete(false);
					else
						MotorState_RaiseFault(MOTOR_FAULT_PHASE_RESISTANCE);
				}
				else if (status == PHASE_RESISTANCE_MODE_SETTLE_TIMEOUT)
					MotorState_RaiseFault(MOTOR_FAULT_PHASE_RESISTANCE);
				else if (status == PHASE_RESISTANCE_MODE_INVALID_RESULT)
					MotorState_RaiseFault(MOTOR_FAULT_INVALID_PARAMETER);
				else if (status == PHASE_RESISTANCE_MODE_UNDER_VOLTAGE)
					MotorState_RaiseFault(MOTOR_FAULT_UNDER_VOLTAGE);
				else if (status == PHASE_RESISTANCE_MODE_OVER_VOLTAGE)
					MotorState_RaiseFault(MOTOR_FAULT_OVER_VOLTAGE);
				break;
			}
			case SERVICE_PROCEDURE_FRICTION_IDENTIFICATION:
			{
				FrictionIdentificationState status =
					FrictionIdentificationRuntime_Run(
						&FrictionIdentificationRuntime, &MotionControl,
						&CurrentControl, &MotorControl, &PI_Speed,
						&OnBoard_Encoder, ActiveBoardProfile,
						ActiveMechanicalLoadProfile);
				if (status == FRICTION_IDENT_COMPLETE)
					MotorLifecycle_ReportServiceComplete(false);
				else if (status == FRICTION_IDENT_FAILED)
					MotorState_RaiseFault(MOTOR_FAULT_FRICTION_IDENTIFICATION);
				break;
			}
			case SERVICE_PROCEDURE_SET_MECHANICAL_ZERO:
				MotorControlRuntime_SetMechanicalZero(&MotorControl, &OnBoard_Encoder);
				break;
			default:
				CurrentControlRuntime_ApplyHighSideZeroVector(&CurrentControl);
				break;
		}
	}
	else
	{
		PhaseResistanceRuntime_Cancel(&PhaseResistanceRuntime, &CurrentControl,
			&MotorControl);
		FrictionIdentificationRuntime_Cancel(&FrictionIdentificationRuntime,
			&CurrentControl, &MotorControl, &PI_Speed);
		CurrentControlRuntime_ApplyHighSideZeroVector(&CurrentControl);
	}

	if (PowerStage_HasLatchedFault(CurrentControl.power_stage))
		MotorState_RaiseFault(MOTOR_FAULT_POWER_STAGE);

	operation_requires_power = LifecycleRequiresPowerStage(device_state,
		service_procedure, procedure_state);
	if (PreviousOperationRequiresPower && !operation_requires_power)
	{
		MotorState_DisablePowerStage();
		MotorState_ResetControlState();
	}
	if (!PreviousOperationRequiresPower && operation_requires_power)
	{
		if (!MotorState_EnablePowerStage())
			MotorState_RaiseFault(MOTOR_FAULT_POWER_STAGE);
	}
	PreviousOperationRequiresPower = operation_requires_power;
	PreviousServiceProcedure = device_state == DEVICE_STATE_SERVICING ?
		service_procedure : SERVICE_PROCEDURE_NONE;
	MotorControl.runtime.action_telemetry =
		(float)MotorLifecycle_GetProtocolActionCode();
	MotorControl.runtime.fault_telemetry = MotorControl.runtime.primary_fault;
}

void MotorControlRuntime_ExecuteFastLoop(void)
{
	uint32_t start_cycles;
	uint32_t elapsed_cycles;
	MotorFastLoopMetrics *metrics;

	if (ActiveRuntime == 0)
		return;
	start_cycles = ActiveRuntime->execution_timer.read_cycles(
		ActiveRuntime->execution_timer.context);
	MotorControlRuntime_ExecuteFastLoopBody();
	elapsed_cycles = ActiveRuntime->execution_timer.read_cycles(
		ActiveRuntime->execution_timer.context) - start_cycles;
	metrics = &ActiveRuntime->fast_loop_metrics;
	if (metrics->invocation_count != UINT32_MAX)
		metrics->invocation_count++;
	if (elapsed_cycles > metrics->maximum_cycles)
		metrics->maximum_cycles = elapsed_cycles;
	metrics->latest_cycles = elapsed_cycles;
	if (metrics->filtered_cycles == 0U)
		metrics->filtered_cycles = elapsed_cycles;
	else
		metrics->filtered_cycles = (uint32_t)((int32_t)metrics->filtered_cycles +
			((int32_t)elapsed_cycles - (int32_t)metrics->filtered_cycles) / 64);
	if (metrics->deadline_cycles > 0U &&
		elapsed_cycles > metrics->deadline_cycles &&
		metrics->deadline_overrun_count != UINT32_MAX)
		metrics->deadline_overrun_count++;
}

bool MotorControlRuntime_ReadFastLoopMetrics(MotorFastLoopMetrics *metrics)
{
	uint32_t interrupt_state;
	if (ActiveRuntime == 0 || metrics == 0)
		return false;
	interrupt_state = RuntimeCriticalSection.enter(RuntimeCriticalSection.context);
	*metrics = ActiveRuntime->fast_loop_metrics;
	RuntimeCriticalSection.exit(RuntimeCriticalSection.context, interrupt_state);
	return true;
}
