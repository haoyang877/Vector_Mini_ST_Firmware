#include "firmware_composition.h"

#include "board_runtime.h"
#include "can_fdcan1_transport.h"
#include "motor_state_runtime.h"
#include "parameter_persistence_adapter.h"
#include "motor_control_runtime.h"
#include "interface_can.h"
#include "interface_usb.h"
#include "measurement_adc12.h"
#include "indicator_stm32g431.h"
#include "led.h"
#include "power_stage_tim1.h"
#include "rotor_sensor_tle5012b.h"
#include "rotor_calibration_service.h"
#include "motor_command_service.h"
#include "parameter_service.h"
#include "rgb.h"
#include "usb_cdc_transport.h"
#include "fault_port_adapter.h"
#include "communication_watchdog_service.h"
#include "can_configuration_service.h"
#include "can_response_service.h"
#include "board_profile.h"
#include "motor_profiles.h"
#include "encoder_profiles.h"
#include "parameter_transaction_service.h"
#include "parameter_transaction_adapter.h"
#include "telemetry_service.h"
#include "supervisor_task.h"
#include "control_tuning_profile.h"
#include "mechanical_load_profiles.h"
#include "friction_identification_service.h"
#include "monotonic_clock_stm32g431.h"
#include "diagnostic_rtt_stm32g431.h"
#include "execution_timer_stm32g431.h"
#include "reset_reason_stm32g431.h"
#include "diagnostic_service.h"
#include "device_identity_stm32g431.h"
#include "parameter_store_flash.h"
#include "can_command_router.h"
#include "usb_command_router.h"
#include "application_endpoints.h"

static PowerStageContext MotorPowerStage;
static MotorControlRuntimeContext MotorControlRuntime;
static CriticalSectionPort BoardCriticalSection;
static ParameterTransactionServiceContext ParameterTransactionService;
static ParameterTransactionAdapterContext ParameterTransactionAdapter;
static MotorCommandServiceContext MotorCommandService;
static ParameterServiceContext ParameterService;
static TelemetryServiceContext TelemetryService;
static CanConfigurationServiceContext CanConfigurationService;
static CanResponseServiceContext CanResponseService;
static CommunicationWatchdogServiceContext CommunicationWatchdogService;
static RotorCalibrationServiceContext RotorCalibrationService;
static LedServiceContext LedService;
static RgbServiceContext RgbService;
static SupervisorTaskContext SupervisorTask;
static ParameterPersistenceAdapterContext ParameterPersistenceAdapter;
static CanInterfaceContext CanInterface;
static UsbInterfaceContext UsbInterface;
static DiagnosticServiceContext DiagnosticService;
static FrictionIdentificationServiceContext FrictionIdentificationService;
static CanCommandRouterContext CanCommandRouter;
static UsbCommandRouterContext UsbCommandRouter;
static ApplicationEndpoints ApplicationEndpointSet;
static bool FirmwareIsInitialized;

/**
	* @brief  Initialize board peripherals and application modules
 **/
void FirmwareComposition_Initialize(void)
{
	const BoardProfile *board_profile = BoardProfile_GetActive();
	PowerStagePort power_stage_port = PowerStageTim1_CreatePort();
	MeasurementPort measurement_port = MeasurementAdc12_CreatePort();
	RotorSensorPort rotor_sensor_port = RotorSensorTle5012b_CreatePort();
	CanTransportPort can_transport = CanFdcan1Transport_CreatePort(
		board_profile->can_fd_enabled, board_profile->can_brs_enabled);
	ByteTransportPort usb_transport = UsbCdcTransport_CreatePort();
	IndicatorPort indicator_port = IndicatorStm32G431_CreatePort();
	RotorCalibrationPort rotor_calibration_port;
	MotorCommandPort motor_command_port;
	MotorConfigurationPort motor_configuration_port;
	FrictionIdentificationPort friction_identification_port;
	FaultCommandPort fault_command_port;
	CanConfigurationPort can_configuration_port;
	CanResponsePort can_response_port;
	ParameterTransactionPort parameter_transaction_port;
	MonotonicClockPort monotonic_clock = MonotonicClockStm32G431_CreatePort();
	ExecutionTimerPort execution_timer = ExecutionTimerStm32G431_CreatePort();
	ResetReasonPort reset_reason_port = ResetReasonStm32G431_CreatePort();
	DeviceIdentityPort device_identity_port =
		DeviceIdentityStm32G431_CreatePort();
	DiagnosticTransportPort diagnostic_transport =
		DiagnosticRttStm32G431_CreatePort();
	ParameterStorePort parameter_store = ParameterStoreFlash_CreatePort();
	const MotorProfile *motor_profile = MotorProfile_GetActive();
	const EncoderProfile *encoder_profile = EncoderProfile_GetActive();
	const ControlTuningProfile *tuning_profile = ControlTuningProfile_GetActive();
	const MechanicalLoadProfile *mechanical_load_profile =
		MechanicalLoadProfile_GetActive();
	FirmwareIsInitialized = false;
	BoardCriticalSection = BoardRuntimeStm32G431_CreateCriticalSectionPort();
	if (!TelemetryService_Initialize(&TelemetryService))
		return;
	if (!DiagnosticService_Initialize(&DiagnosticService, &reset_reason_port,
		&device_identity_port, &TelemetryService))
		return;

	/* Power outputs are disabled before application state is initialized. */
	PowerStage_Initialize(&MotorPowerStage, &power_stage_port);
	if (!CanInterface_Initialize(&CanInterface, &can_transport))
		return;
	can_configuration_port = CanInterface_CreateConfigurationPort(&CanInterface);
	can_response_port = CanInterface_CreateResponsePort(&CanInterface);
	if (!CanConfigurationService_Initialize(&CanConfigurationService,
			&can_configuration_port) ||
		!CanResponseService_Initialize(&CanResponseService, &can_response_port))
	{
		return;
	}
	if (!MotorControlRuntime_Prepare(&MotorControlRuntime, board_profile,
		motor_profile, encoder_profile, tuning_profile, mechanical_load_profile,
		&monotonic_clock, &execution_timer, &CanConfigurationService))
		return;
	if (!ParameterPersistenceAdapter_Initialize(&ParameterPersistenceAdapter,
		&parameter_store, &MotorControlRuntime.parameter_snapshot))
		return;

	/*read parameters and calibration data from flash*/
	/*if magic word invalid or not calibrated, fall back to code defaults*/
	ParameterPersistenceAdapter_Load(&ParameterPersistenceAdapter);
	
	/*motor control related parameters initialize*/
	MotorControlRuntime_Initialize(&MotorControlRuntime, &MotorPowerStage,
		&measurement_port,
		&rotor_sensor_port, &BoardCriticalSection);
	rotor_calibration_port = MotorControlRuntime_CreateRotorCalibrationPort(
		&MotorControlRuntime);
	if (!RotorCalibrationService_Initialize(&RotorCalibrationService,
		&rotor_calibration_port))
		MotorState_RaiseFault(&MotorControlRuntime.motor_state,
			MOTOR_FAULT_ENCODER);
	motor_command_port = MotorControlRuntime_CreateCommandPort(
		&MotorControlRuntime);
	motor_configuration_port = MotorControlRuntime_CreateConfigurationPort(
		&MotorControlRuntime);
	friction_identification_port =
		MotorControlRuntime_CreateFrictionIdentificationPort(
			&MotorControlRuntime);
	if (!MotorCommandService_Initialize(&MotorCommandService, &motor_command_port) ||
		!ParameterService_Initialize(&ParameterService,
			&motor_configuration_port, board_profile,
			motor_profile) ||
		!FrictionIdentificationService_Initialize(
			&FrictionIdentificationService, &friction_identification_port))
		MotorState_RaiseFault(&MotorControlRuntime.motor_state,
			MOTOR_FAULT_PARAMETER_STORE);
	parameter_transaction_port = ParameterTransactionAdapter_CreatePort(
		&ParameterTransactionAdapter, &BoardCriticalSection,
		&ParameterPersistenceAdapter, &MotorControlRuntime.parameter_snapshot,
		&MotorControlRuntime.motor_state);
	if (!ParameterTransactionService_Initialize(&ParameterTransactionService,
		&parameter_transaction_port))
		MotorState_RaiseFault(&MotorControlRuntime.motor_state,
			MOTOR_FAULT_PARAMETER_STORE);
	fault_command_port = FaultApplicationAdapter_CreatePort(
		&MotorControlRuntime.motor_state);
	if (!CommunicationWatchdogService_Initialize(&CommunicationWatchdogService,
		&fault_command_port,
		(uint32_t)MOTOR_FAULT_CAN_DISCONNECTED))
		MotorState_RaiseFault(&MotorControlRuntime.motor_state,
			MOTOR_FAULT_CAN_DISCONNECTED);
	
	if (!LED_Initialize(&LedService, &indicator_port) ||
		!RGB_Initialize(&RgbService, &indicator_port))
		MotorState_RaiseFault(&MotorControlRuntime.motor_state,
			MOTOR_FAULT_POWER_STAGE);
	/* Apply the node ID loaded from the parameter snapshot. */
	CanInterface_ApplyConfiguredBitrate(&CanInterface,
		&CommunicationWatchdogService);
	if (!UsbInterface_Initialize(&UsbInterface, &usb_transport,
		&monotonic_clock))
	{
		MotorState_RaiseFault(&MotorControlRuntime.motor_state,
			MOTOR_FAULT_POWER_STAGE);
		return;
	}
	if (!ApplicationEndpoints_Initialize(&ApplicationEndpointSet,
			&CanConfigurationService, &FrictionIdentificationService,
			&MotorCommandService, &ParameterService, &RotorCalibrationService,
			&TelemetryService) ||
		!CanCommandRouter_Initialize(&CanCommandRouter, &ApplicationEndpointSet,
			&CanResponseService) ||
		!UsbCommandRouter_Initialize(&UsbCommandRouter, &ApplicationEndpointSet))
	{
		MotorState_RaiseFault(&MotorControlRuntime.motor_state,
			MOTOR_FAULT_PARAMETER_STORE);
		return;
	}
	if (!SupervisorTask_Initialize(&SupervisorTask, &diagnostic_transport,
		&MotorControlRuntime, &TelemetryService, &CanInterface, &UsbInterface,
		&CommunicationWatchdogService, &CanConfigurationService, &LedService,
		&RgbService))
		MotorState_RaiseFault(&MotorControlRuntime.motor_state,
			MOTOR_FAULT_POWER_STAGE);

	/* Enable periodic interrupts only after every ISR-visible context exists. */
	FirmwareIsInitialized = true;
	if (!BoardRuntimeStm32G431_Start())
	{
		FirmwareIsInitialized = false;
		MotorState_RaiseFault(&MotorControlRuntime.motor_state,
			MOTOR_FAULT_POWER_STAGE);
	}
}

void FirmwareComposition_ExecuteFastLoop(void)
{
	if (FirmwareIsInitialized)
		MotorControlRuntime_ExecuteFastLoop(&MotorControlRuntime);
}

void FirmwareComposition_ExecuteSupervisor1kHz(void)
{
	if (FirmwareIsInitialized)
		SupervisorTask_Execute1kHz(&SupervisorTask);
}

void FirmwareComposition_OnCanReceiveInterrupt(void)
{
	if (FirmwareIsInitialized)
		CanInterface_OnReceiveInterrupt(&CanInterface);
}

void FirmwareComposition_OnUsbReceiveInterrupt(const uint8_t *data,
	uint32_t size_bytes)
{
	if (FirmwareIsInitialized)
		UsbInterface_OnReceiveInterrupt(&UsbInterface, data, size_bytes);
}

void FirmwareComposition_OnUsbTransmitCompleteInterrupt(void)
{
	if (FirmwareIsInitialized)
		UsbInterface_OnTransmitCompleteInterrupt(&UsbInterface);
}

void FirmwareComposition_RunBackground(void)
{
	if (!FirmwareIsInitialized)
		return;
	UsbInterface_ProcessReceivedCommands(&UsbInterface, &UsbCommandRouter);
	UsbInterface_FlushTransmit(&UsbInterface, &FrictionIdentificationService,
		&RotorCalibrationService);
	CanInterface_ProcessReceivedFrames(&CanInterface, &CanCommandRouter,
		&CommunicationWatchdogService);
	CanInterface_FlushTransmit(&CanInterface);
	SupervisorTask_RunBackground(&SupervisorTask);
	ParameterTransactionService_RunBackground(&ParameterTransactionService);
}
