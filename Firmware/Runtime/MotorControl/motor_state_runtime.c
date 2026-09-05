#include "motor_state_runtime.h"

#include <math.h>
#include <stddef.h>

typedef enum
{
	LIFECYCLE_REQUEST_NONE = 0,
	LIFECYCLE_REQUEST_STANDBY,
	LIFECYCLE_REQUEST_CONTROL_MODE,
	LIFECYCLE_REQUEST_SERVICE,
	LIFECYCLE_REQUEST_CLEAR_FAULTS
} LifecycleRequestType;

typedef enum
{
	SERVICE_RESULT_NONE = 0,
	SERVICE_RESULT_COMPLETE,
	SERVICE_RESULT_COMPLETE_AND_SAVE,
	SERVICE_RESULT_FAILED
} ServiceResult;

#define RuntimeBindings (context->bindings)

#define MotorControl       (*RuntimeBindings.motor)
#define CurrentControl     (*RuntimeBindings.current_control)
#define PI_Speed           (*RuntimeBindings.speed_controller)
#define OnBoard_Encoder    (*RuntimeBindings.encoder)
#define SensorlessStartup  (*RuntimeBindings.sensorless_startup)
#define MotionControl      (*RuntimeBindings.motion_control)
#define MotorCalibration   (*RuntimeBindings.motor_calibration)
#define MotorLifecycle     (*RuntimeBindings.lifecycle)
#define State              (*context)
#define MotorFaultManager  (State.fault_manager)
#define PendingLifecycleRequest (State.pending_lifecycle_request)
#define PendingServiceResult (State.pending_service_result)
#define ServiceElapsedTicks (State.service_elapsed_ticks)
#define ServiceTerminalHold (State.service_terminal_hold)
#define MotorStateChanged (State.has_changed)
#define PreviousDeviceState (State.previous_device_state)
#define PreviousControlMode (State.previous_control_mode)
#define PreviousServiceProcedure (State.previous_service_procedure)
#define PreviousFault (State.previous_fault)

#define LIFECYCLE_REQUEST_ENCODE(type, value) \
	(((uint32_t)(type) & 0xFFU) | (((uint32_t)(value) & 0xFFU) << 8U))
#define LIFECYCLE_REQUEST_TYPE(request) ((LifecycleRequestType)((request) & 0xFFU))
#define LIFECYCLE_REQUEST_VALUE(request) ((uint8_t)(((request) >> 8U) & 0xFFU))
#define SERVICE_TIMEOUT_1KHZ_TICKS 120000U

static void MotorFaults_UpdateRuntimeProjection(MotorStateContext *context)
{
	MotorControl.runtime.primary_fault = (MotorFaultCode)
		FaultManager_GetPrimaryFault(&MotorFaultManager);
}

static bool MotorLifecycle_ModeNeedsEncoder(const MotorStateContext *context,
	MotorControlMode mode)
{
	if (mode == MOTOR_CONTROL_MODE_CURRENT)
		return !MotorControl.configuration.use_sensorless_feedback;
	return mode == MOTOR_CONTROL_MODE_SPEED ||
		mode == MOTOR_CONTROL_MODE_POSITION_CASCADE ||
		mode == MOTOR_CONTROL_MODE_POSITION_IMPEDANCE ||
		mode == MOTOR_CONTROL_MODE_VQ;
}

static bool MotorLifecycle_ServiceNeedsEncoder(ServiceProcedure procedure)
{
	return procedure == SERVICE_PROCEDURE_ENCODER_LINEARIZATION ||
		procedure == SERVICE_PROCEDURE_ELECTRICAL_ZERO_CALIBRATION ||
		procedure == SERVICE_PROCEDURE_OBSERVER_CALIBRATION ||
		procedure == SERVICE_PROCEDURE_FRICTION_IDENTIFICATION ||
		procedure == SERVICE_PROCEDURE_ENCODER_DIRECTION_CALIBRATION ||
		procedure == SERVICE_PROCEDURE_COGGING_IDENTIFICATION ||
		procedure == SERVICE_PROCEDURE_SET_MECHANICAL_ZERO;
}

static bool MotorLifecycle_CheckControlPreconditions(
	MotorStateContext *context, MotorControlMode mode)
{
	if (mode <= MOTOR_CONTROL_MODE_NONE || mode > MOTOR_CONTROL_MODE_VQ ||
		MotorFaults_HasActive(context))
		return false;
	if (MotorLifecycle_ModeNeedsEncoder(context, mode) &&
		!Encoder_IsOnline(&OnBoard_Encoder))
	{
		MotorState_RaiseFault(context, MOTOR_FAULT_ENCODER);
		return false;
	}
	if (mode == MOTOR_CONTROL_MODE_SENSORLESS_SPEED &&
		(MotorControl.configuration.pole_pairs <= 0 ||
		 MotorControl.configuration.phase_resistance_ohm <= 0.0f ||
		 MotorControl.configuration.d_axis_inductance_h <= 0.0f ||
		 MotorControl.configuration.q_axis_inductance_h <= 0.0f ||
		 MotorControl.configuration.flux_weber <= 0.0f ||
		 MotorControl.configuration.current_limit_a <= 0.0f))
	{
		MotorState_RaiseFault(context, MOTOR_FAULT_INVALID_PARAMETER);
		return false;
	}
	if (MotorLifecycle_ModeNeedsEncoder(context, mode) &&
		(OnBoard_Encoder.calib_flag & ENC_CALIB_ALL) != ENC_CALIB_ALL)
	{
		MotorState_RaiseFault(context, MOTOR_FAULT_ENCODER_NOT_CALIBRATED);
		return false;
	}
	return MotorLifecycle.device_state == DEVICE_STATE_STANDBY ||
		MotorLifecycle.device_state == DEVICE_STATE_ACTIVE;
}

static bool MotorLifecycle_CheckServicePreconditions(
	MotorStateContext *context, ServiceProcedure procedure)
{
	if (procedure <= SERVICE_PROCEDURE_NONE || procedure >= SERVICE_PROCEDURE_COUNT ||
		MotorFaults_HasActive(context))
		return false;
	if (MotorLifecycle_ServiceNeedsEncoder(procedure) &&
		!Encoder_IsOnline(&OnBoard_Encoder))
	{
		MotorState_RaiseFault(context, MOTOR_FAULT_ENCODER);
		return false;
	}
	if ((procedure == SERVICE_PROCEDURE_FRICTION_IDENTIFICATION ||
		 procedure == SERVICE_PROCEDURE_COGGING_IDENTIFICATION) &&
		(OnBoard_Encoder.calib_flag & ENC_CALIB_ALL) != ENC_CALIB_ALL)
	{
		MotorState_RaiseFault(context, MOTOR_FAULT_ENCODER_NOT_CALIBRATED);
		return false;
	}
	return MotorLifecycle.device_state == DEVICE_STATE_STANDBY ||
		MotorLifecycle.device_state == DEVICE_STATE_SERVICING;
}

static void MotorLifecycle_UpdateChangeFlag(MotorStateContext *context)
{
	if (PreviousDeviceState != MotorLifecycle.device_state ||
		PreviousControlMode != MotorLifecycle.motor_control_mode ||
		PreviousServiceProcedure != MotorLifecycle.service_procedure ||
		PreviousFault != MotorControl.runtime.primary_fault)
		MotorStateChanged = true;
	PreviousDeviceState = MotorLifecycle.device_state;
	PreviousControlMode = MotorLifecycle.motor_control_mode;
	PreviousServiceProcedure = MotorLifecycle.service_procedure;
	PreviousFault = MotorControl.runtime.primary_fault;
}

bool MotorState_Initialize(MotorStateContext *context,
	const MotorFaultRuntimeBindings *bindings)
{
	if (context == NULL || bindings == NULL || bindings->motor == NULL ||
		bindings->current_control == NULL || bindings->speed_controller == NULL ||
		bindings->encoder == NULL || bindings->sensorless_startup == NULL ||
		bindings->motion_control == NULL || bindings->motor_calibration == NULL ||
		bindings->lifecycle == NULL ||
		bindings->calibration_service == NULL ||
		bindings->identification_service == NULL ||
		bindings->monotonic_clock.read_ms == NULL)
		return false;
	RuntimeBindings = *bindings;
	FaultManager_Initialize(&MotorFaultManager);
	DeviceLifecycle_Initialize(&MotorLifecycle);
	(void)DeviceLifecycle_CompleteBoot(&MotorLifecycle);
	PendingLifecycleRequest = LIFECYCLE_REQUEST_ENCODE(LIFECYCLE_REQUEST_NONE, 0U);
	PendingServiceResult = SERVICE_RESULT_NONE;
	ServiceElapsedTicks = 0U;
	ServiceTerminalHold = false;
	MotorStateChanged = true;
	MotorCommissioningWorkflow_Initialize(&State.commissioning);
	PreviousDeviceState = DEVICE_STATE_BOOTING;
	PreviousControlMode = MOTOR_CONTROL_MODE_NONE;
	PreviousServiceProcedure = SERVICE_PROCEDURE_NONE;
	PreviousFault = MOTOR_FAULT_NONE;
	MotorFaults_UpdateRuntimeProjection(context);
	return true;
}

DeviceState MotorLifecycle_GetDeviceState(const MotorStateContext *context) { return MotorLifecycle.device_state; }
MotorControlMode MotorLifecycle_GetControlMode(const MotorStateContext *context) { return MotorLifecycle.motor_control_mode; }
ServiceProcedure MotorLifecycle_GetServiceProcedure(const MotorStateContext *context) { return MotorLifecycle.service_procedure; }
ProcedureState MotorLifecycle_GetProcedureState(const MotorStateContext *context) { return MotorLifecycle.procedure_state; }

uint8_t MotorLifecycle_GetProtocolActionCode(const MotorStateContext *context)
{
	if (State.commissioning.active)
		return 21U;
	if (MotorLifecycle.device_state == DEVICE_STATE_ACTIVE)
	{
		switch (MotorLifecycle.motor_control_mode)
		{
			case MOTOR_CONTROL_MODE_CURRENT: return 1U;
			case MOTOR_CONTROL_MODE_SPEED: return 2U;
			case MOTOR_CONTROL_MODE_POSITION_CASCADE: return 3U;
			case MOTOR_CONTROL_MODE_VOLTAGE_OPEN_LOOP: return 12U;
			case MOTOR_CONTROL_MODE_VQ: return 14U;
			case MOTOR_CONTROL_MODE_SENSORLESS_SPEED: return 16U;
			case MOTOR_CONTROL_MODE_POSITION_IMPEDANCE: return 18U;
			default: return 0U;
		}
	}
	if (MotorLifecycle.device_state == DEVICE_STATE_SERVICING)
	{
		switch (MotorLifecycle.service_procedure)
		{
			case SERVICE_PROCEDURE_ENCODER_LINEARIZATION: return 5U;
			case SERVICE_PROCEDURE_SET_MECHANICAL_ZERO: return 7U;
			case SERVICE_PROCEDURE_RESTORE_DEFAULTS: return 8U;
			case SERVICE_PROCEDURE_PARAMETER_SAVE: return 9U;
			case SERVICE_PROCEDURE_CURRENT_OFFSET_CALIBRATION: return 11U;
			case SERVICE_PROCEDURE_OBSERVER_CALIBRATION: return 13U;
			case SERVICE_PROCEDURE_ELECTRICAL_ZERO_CALIBRATION: return 15U;
			case SERVICE_PROCEDURE_PHASE_RESISTANCE_IDENTIFICATION: return 17U;
			case SERVICE_PROCEDURE_FRICTION_IDENTIFICATION: return 19U;
			case SERVICE_PROCEDURE_ENCODER_DIRECTION_CALIBRATION: return 20U;
			case SERVICE_PROCEDURE_COGGING_IDENTIFICATION: return 6U;
			default: return 0U;
		}
	}
	return 0U;
}

MotorCommissioningStage MotorLifecycle_GetCommissioningStage(
	const MotorStateContext *context)
{
	return State.commissioning.stage;
}

uint8_t MotorLifecycle_GetCommissioningProgressPercent(
	const MotorStateContext *context)
{
	return MotorCommissioningWorkflow_GetProgressPercent(&State.commissioning);
}

MotorCommissioningStage MotorLifecycle_GetCommissioningFailureStage(
	const MotorStateContext *context)
{
	return State.commissioning.failure_stage;
}

bool MotorLifecycle_RequestControlMode(MotorStateContext *context,
	MotorControlMode mode)
{
	if (!MotorLifecycle_CheckControlPreconditions(context, mode))
		return false;
	if (mode == MOTOR_CONTROL_MODE_POSITION_CASCADE ||
		mode == MOTOR_CONTROL_MODE_POSITION_IMPEDANCE)
	{
		float current_position = Encoder_GetMecPos(&OnBoard_Encoder);
		if (!isfinite(current_position))
		{
			MotorState_RaiseFault(context, MOTOR_FAULT_ENCODER);
			return false;
		}
		MotorControl.command.position_reference_rad = current_position;
		MotorControl.runtime.position_command_ramp_rad = current_position;
		MotorControl.runtime.speed_command_ramp_rad_s = 0.0f;
		MotorControl.runtime.has_reached_position = false;
		ControlModeRuntime_ResetPosition(&MotionControl);
	}
	PendingLifecycleRequest = LIFECYCLE_REQUEST_ENCODE(
		LIFECYCLE_REQUEST_CONTROL_MODE, mode);
	return true;
}

bool MotorLifecycle_RequestService(MotorStateContext *context,
	ServiceProcedure procedure)
{
	ServiceProcedure first_procedure;
	if (!MotorLifecycle_CheckServicePreconditions(context, procedure))
		return false;
	if (procedure == SERVICE_PROCEDURE_FULL_COMMISSIONING)
	{
		if (MotorLifecycle.device_state != DEVICE_STATE_STANDBY ||
			!MotorCommissioningWorkflow_Start(&State.commissioning,
				&first_procedure))
			return false;
		procedure = first_procedure;
	}
	PendingLifecycleRequest = LIFECYCLE_REQUEST_ENCODE(
		LIFECYCLE_REQUEST_SERVICE, procedure);
	return true;
}

bool MotorLifecycle_RequestStandby(MotorStateContext *context)
{
	if (MotorLifecycle.device_state == DEVICE_STATE_BOOTING ||
		MotorLifecycle.device_state == DEVICE_STATE_UPDATING)
		return false;
	if (State.commissioning.active)
		MotorCommissioningWorkflow_Fail(&State.commissioning, 0xFFU);
	PendingLifecycleRequest = LIFECYCLE_REQUEST_ENCODE(LIFECYCLE_REQUEST_STANDBY, 0U);
	return true;
}

bool MotorLifecycle_RequestClearFaults(MotorStateContext *context)
{
	if (MotorLifecycle.device_state != DEVICE_STATE_FAULTED)
		return false;
	PendingLifecycleRequest = LIFECYCLE_REQUEST_ENCODE(
		LIFECYCLE_REQUEST_CLEAR_FAULTS, 0U);
	return true;
}

void MotorLifecycle_ReportServiceComplete(MotorStateContext *context,
	bool request_parameter_save)
{
	PendingServiceResult = request_parameter_save ?
		SERVICE_RESULT_COMPLETE_AND_SAVE : SERVICE_RESULT_COMPLETE;
}

void MotorLifecycle_ReportServiceFailed(MotorStateContext *context)
{
	PendingServiceResult = SERVICE_RESULT_FAILED;
}

static void MotorLifecycle_ApplyRequest(MotorStateContext *context,
	uint32_t request)
{
	LifecycleRequestType type = LIFECYCLE_REQUEST_TYPE(request);
	uint8_t value = LIFECYCLE_REQUEST_VALUE(request);

	switch (type)
	{
		case LIFECYCLE_REQUEST_STANDBY:
			if (MotorLifecycle.device_state == DEVICE_STATE_SERVICING)
				DeviceLifecycle_CancelService(&MotorLifecycle);
			(void)DeviceLifecycle_RequestStandby(&MotorLifecycle);
			break;
		case LIFECYCLE_REQUEST_CONTROL_MODE:
			(void)DeviceLifecycle_RequestMotorControl(&MotorLifecycle,
				(MotorControlMode)value);
			break;
		case LIFECYCLE_REQUEST_SERVICE:
			(void)DeviceLifecycle_RequestService(&MotorLifecycle,
				(ServiceProcedure)value);
			ServiceElapsedTicks = 0U;
			ServiceTerminalHold = false;
			break;
		case LIFECYCLE_REQUEST_CLEAR_FAULTS:
			PowerStage_ClearLatchedFault(CurrentControl.power_stage);
			FaultManager_ClearAll(&MotorFaultManager);
			MotorFaults_UpdateRuntimeProjection(context);
			(void)DeviceLifecycle_ClearFault(&MotorLifecycle, true);
			break;
		default:
			break;
	}
}

void MotorLifecycle_Supervise1kHz(MotorStateContext *context)
{
	uint32_t request = PendingLifecycleRequest;
	ServiceResult result = (ServiceResult)PendingServiceResult;
	bool save_after_completion = false;

	PendingLifecycleRequest = LIFECYCLE_REQUEST_ENCODE(LIFECYCLE_REQUEST_NONE, 0U);
	PendingServiceResult = SERVICE_RESULT_NONE;
	if (request != LIFECYCLE_REQUEST_ENCODE(LIFECYCLE_REQUEST_NONE, 0U))
		MotorLifecycle_ApplyRequest(context, request);

	if (MotorLifecycle.device_state == DEVICE_STATE_SERVICING)
	{
		bool specialized_service = false;
		if (CalibrationService_OwnsProcedure(MotorLifecycle.service_procedure))
		{
			specialized_service = true;
			if (!CalibrationService_Supervise1kHz(
				RuntimeBindings.calibration_service))
				result = SERVICE_RESULT_FAILED;
		}
		else if (IdentificationService_OwnsProcedure(
			MotorLifecycle.service_procedure))
		{
			specialized_service = true;
			if (!IdentificationService_Supervise1kHz(
				RuntimeBindings.identification_service))
				result = SERVICE_RESULT_FAILED;
		}
		if (!specialized_service &&
			MotorLifecycle.procedure_state == PROCEDURE_STATE_PRECHECK)
			(void)DeviceLifecycle_BeginServiceRun(&MotorLifecycle);
		else if (!specialized_service &&
			MotorLifecycle.procedure_state == PROCEDURE_STATE_RUNNING)
		{
			if (ServiceElapsedTicks < UINT32_MAX)
				ServiceElapsedTicks++;
			if (ServiceElapsedTicks >= SERVICE_TIMEOUT_1KHZ_TICKS)
				result = SERVICE_RESULT_FAILED;
		}

		if (result == SERVICE_RESULT_COMPLETE ||
			result == SERVICE_RESULT_COMPLETE_AND_SAVE)
		{
			bool commissioning_was_active = State.commissioning.active;
			ServiceProcedure next_procedure = SERVICE_PROCEDURE_NONE;
			bool commissioning_complete = false;
			save_after_completion = result == SERVICE_RESULT_COMPLETE_AND_SAVE &&
				!commissioning_was_active;
			if (DeviceLifecycle_BeginServiceVerification(&MotorLifecycle))
				(void)DeviceLifecycle_CompleteService(&MotorLifecycle);
			if (commissioning_was_active &&
				MotorCommissioningWorkflow_CompleteProcedure(&State.commissioning,
					MotorLifecycle.service_procedure, &next_procedure,
					&commissioning_complete))
			{
				if (commissioning_complete)
					ServiceTerminalHold = true;
				else
					PendingLifecycleRequest = LIFECYCLE_REQUEST_ENCODE(
						LIFECYCLE_REQUEST_SERVICE, next_procedure);
			}
			else if (!commissioning_was_active)
				ServiceTerminalHold = true;
			else
			{
				MotorCommissioningWorkflow_Fail(&State.commissioning,
					(uint8_t)MOTOR_FAULT_INVALID_PARAMETER);
				DeviceLifecycle_FailService(&MotorLifecycle);
				ServiceTerminalHold = true;
			}
		}
		else if (result == SERVICE_RESULT_FAILED)
		{
			MotorCommissioningWorkflow_Fail(&State.commissioning,
				(uint8_t)MotorControl.runtime.primary_fault);
			DeviceLifecycle_FailService(&MotorLifecycle);
			ServiceTerminalHold = true;
		}
		else if (ServiceTerminalHold)
		{
			ServiceTerminalHold = false;
			(void)DeviceLifecycle_RequestStandby(&MotorLifecycle);
		}

		if (save_after_completion)
			PendingLifecycleRequest = LIFECYCLE_REQUEST_ENCODE(
				LIFECYCLE_REQUEST_SERVICE, SERVICE_PROCEDURE_PARAMETER_SAVE);
	}
	MotorLifecycle_UpdateChangeFlag(context);
}

bool MotorFaults_HasActive(const MotorStateContext *context) { return FaultManager_HasFaults(&MotorFaultManager); }
FaultSet MotorFaults_GetActiveSet(const MotorStateContext *context) { return FaultManager_GetActiveFaults(&MotorFaultManager); }
FaultSet MotorFaults_GetLatchedSet(const MotorStateContext *context) { return FaultManager_GetLatchedFaults(&MotorFaultManager); }
uint32_t MotorFaults_GetEventSequence(const MotorStateContext *context) { return FaultManager_GetEventSequence(&MotorFaultManager); }
bool MotorFaults_GetRecord(const MotorStateContext *context,
	uint8_t fault_code, FaultRecord *record)
{
	return FaultManager_GetRecord(&MotorFaultManager, fault_code, record);
}

MotorFaultCode MotorState_GetPrimaryFault(const MotorStateContext *context) { return MotorControl.runtime.primary_fault; }

void MotorState_RaiseFault(MotorStateContext *context, MotorFaultCode fault)
{
	FaultObservation observation;
	if (fault == MOTOR_FAULT_NONE)
		return;
	else
	{
		observation.time_ms = RuntimeBindings.monotonic_clock.read_ms(
			RuntimeBindings.monotonic_clock.context);
		observation.bus_voltage_v = CurrentControl.filtered_bus_voltage_v;
		observation.phase_a_current_a = CurrentControl.phase_a_current_a;
		observation.phase_b_current_a = CurrentControl.phase_b_current_a;
		observation.phase_c_current_a = CurrentControl.phase_c_current_a;
		observation.temperature_c = CurrentControl.temperature_c;
		observation.mechanical_position_rad = OnBoard_Encoder.theta_mech;
		observation.mechanical_speed_rad_s = OnBoard_Encoder.vel_mech;
		(void)FaultManager_Raise(&MotorFaultManager, (uint8_t)fault,
			&observation);
		DeviceLifecycle_NotifyFault(&MotorLifecycle);
		MotorCommissioningWorkflow_Fail(&State.commissioning, (uint8_t)fault);
		PendingLifecycleRequest = LIFECYCLE_REQUEST_ENCODE(LIFECYCLE_REQUEST_NONE, 0U);
	}
	MotorFaults_UpdateRuntimeProjection(context);
}

void MotorState_ClearFault(MotorStateContext *context,
	MotorFaultCode error_to_clear)
{
	(void)FaultManager_ClearActive(&MotorFaultManager, (uint8_t)error_to_clear);
	MotorFaults_UpdateRuntimeProjection(context);
}

void MotorState_ClearAllFaults(MotorStateContext *context)
{
	FaultManager_ClearAll(&MotorFaultManager);
	MotorFaults_UpdateRuntimeProjection(context);
}

void MotorState_ResetControlState(MotorStateContext *context)
{
	MotorControl.command.d_axis_current_reference_a = 0.0f;
	MotorControl.command.q_axis_current_reference_a = 0.0f;
	MotorControl.command.q_axis_voltage_reference_v = 0.0f;
	MotorControl.command.speed_reference_rad_s = 0.0f;
	MotorControl.targets.d_axis_current_a = 0.0f;
	MotorControl.targets.q_axis_current_a = 0.0f;
	MotorControl.targets.q_axis_voltage_v = 0.0f;
	MotorControl.targets.speed_rad_s = 0.0f;
	MotorControl.targets.position_rad = 0.0f;
	MotorControl.runtime.speed_command_ramp_rad_s = 0.0f;
	MotorControl.runtime.position_command_ramp_rad = 0.0f;
	MotorControl.runtime.has_reached_position = false;
	MotorControl.runtime.position_velocity_filtered_rad_s = 0.0f;
	ControlModeRuntime_ResetPosition(&MotionControl);
	MotorCalibration_Reset(&MotorCalibration);
	CurrentControl.d_axis_current_a = 0.0f;
	CurrentControl.q_axis_current_a = 0.0f;
	CurrentControlRuntime_ResetControllers(&CurrentControl);
	PI_Controller_Reset(&PI_Speed);
	SensorlessStartup_Reset(&SensorlessStartup);
}

bool MotorState_HasChanged(const MotorStateContext *context) { return MotorStateChanged; }
void MotorState_ClearChangeFlag(MotorStateContext *context) { MotorStateChanged = false; }
void MotorState_DisablePowerStage(MotorStateContext *context) { PowerStage_ForceDisable(CurrentControl.power_stage); }
bool MotorState_EnablePowerStage(MotorStateContext *context)
{
	return PowerStage_RequestEnable(CurrentControl.power_stage,
		!MotorFaults_HasActive(context));
}
