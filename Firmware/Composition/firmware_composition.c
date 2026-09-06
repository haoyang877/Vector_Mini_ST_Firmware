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
#include "tle5012b_rotor_sensor_adapter.h"
#include "rotor_calibration_service.h"
#include "motor_command_service.h"
#include "parameter_service.h"
#include "rgb.h"
#include "usb_cdc_transport.h"
#include "fault_port_adapter.h"
#include "communication_watchdog_service.h"
#include "can_configuration_service.h"
#include "can_response_service.h"
#include "product_variant.h"
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
#include "product_config_bridge.h"
#include "product_runtime_selection.h"
#include "vector_mini_st_bsp.h"
#include "vector_mini_st_angle_serial.h"
#include "vector_mini_st_storage.h"

#if defined(__CC_ARM)
#pragma O3
#pragma Ospace
#endif

static PowerStageContext MotorPowerStage;
static MotorControlRuntimeContext MotorControlRuntime;
static BspCriticalSectionPort BoardCriticalSection;
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
static ControlAuthorityServiceContext ControlAuthorityService;
static AngleSerialStm32g431Context AngleSerialContext;
static ParameterStoreFlashContext ParameterStoreContext;
static Tle5012bRotorSensorAdapterContext RotorSensorAdapter;
static bool FirmwareIsInitialized;
static volatile ProductConfigBridgeStatus ProductConfigBridgeStartupStatus;

/**
	* @brief  Initialize board peripherals and application modules
 **/
void FirmwareComposition_Initialize(void)
{
	const ProductVariant *product;
	ProductVariant product_storage;
	const BoardProfile *board_profile;
	const MotorProfile *motor_profile;
	const EncoderProfile *encoder_profile;
	const ControlTuningProfile *tuning_profile;
	const MechanicalLoadProfile *mechanical_load_profile;
	PowerStagePort power_stage_port;
	MeasurementPort measurement_port;
	RotorSensorPort rotor_sensor_port;
	BspSynchronousSerialPort angle_serial_port;
	const AngleSerialStm32g431ResourceConfig *angle_serial_resources;
	CanTransportPort can_transport;
	ByteTransportPort usb_transport;
	BspIndicatorPort indicator_port;
	RotorCalibrationPort rotor_calibration_port;
	MotorCommandPort motor_command_port;
	MotorConfigurationPort motor_configuration_port;
	FrictionIdentificationPort friction_identification_port;
	FaultCommandPort fault_command_port;
	CanConfigurationPort can_configuration_port;
	CanResponsePort can_response_port;
	ParameterTransactionPort parameter_transaction_port;
	BspMonotonicClockPort monotonic_clock;
	BspExecutionTimerPort execution_timer;
	BspResetReasonPort reset_reason_port;
	BspUniqueIdPort device_identity_port;
	BspDiagnosticSinkPort diagnostic_transport;
	BspNonvolatileStoragePort parameter_store;
	ProductConfigBridgeStatus product_bridge_status;

	FirmwareIsInitialized = false;
	product = &product_storage;
	if (!ProductVariant_GetActive(&product_storage))
	{
		ProductConfigBridgeStartupStatus =
			PRODUCT_CONFIG_BRIDGE_RUNTIME_CONFIG_INVALID;
		return;
	}
	if (!ProductConfigBridge_ValidateRuntime(
		&ProductCatalog_CurrentRuntimeSelection,
		&BspVectorMiniSt_RuntimeIdentity,
		product->configuration_fingerprint, &product_bridge_status))
	{
		ProductConfigBridgeStartupStatus = product_bridge_status;
		return;
	}
	board_profile = product->board;
	motor_profile = product->motor;
	encoder_profile = product->encoder;
	tuning_profile = product->control_tuning;
	mechanical_load_profile = product->mechanical_load;
	power_stage_port = PowerStageTim1_CreatePort();
	measurement_port = MeasurementAdc12_CreatePort();
	angle_serial_resources = BspVectorMiniSt_FindAngleSerialResources(
		BSP_VECTOR_MINI_ST_ANGLE_ENDPOINT_ONBOARD);
	if (!AngleSerialStm32g431_CreatePort(&AngleSerialContext,
		angle_serial_resources, &angle_serial_port))
	{
		ProductConfigBridgeStartupStatus =
			PRODUCT_CONFIG_BRIDGE_BINDING_MISMATCH;
		return;
	}
	rotor_sensor_port = Tle5012bRotorSensorAdapter_CreatePort(
		&RotorSensorAdapter, &angle_serial_port);
	ProductConfigBridgeStartupStatus = PRODUCT_CONFIG_BRIDGE_OK;
	can_transport = CanFdcan1Transport_CreatePort(board_profile->can_fd_enabled,
		board_profile->can_brs_enabled);
	usb_transport = UsbCdcTransport_CreatePort();
	indicator_port = IndicatorStm32G431_CreatePort();
	monotonic_clock = MonotonicClockStm32G431_CreatePort();
	execution_timer = ExecutionTimerStm32G431_CreatePort();
	reset_reason_port = ResetReasonStm32G431_CreatePort();
	device_identity_port = DeviceIdentityStm32G431_CreatePort();
	diagnostic_transport = DiagnosticRttStm32G431_CreatePort();
	if (!ParameterStoreFlash_CreatePort(&ParameterStoreContext,
		&BspVectorMiniSt_ParameterStorageResources, &parameter_store))
	{
		ProductConfigBridgeStartupStatus =
			PRODUCT_CONFIG_BRIDGE_BINDING_MISMATCH;
		return;
	}
	BoardCriticalSection = BoardRuntimeStm32G431_CreateCriticalSectionPort();
	if (!TelemetryService_Initialize(&TelemetryService))
		return;
	if (!DiagnosticService_Initialize(&DiagnosticService, &reset_reason_port,
		&device_identity_port, &TelemetryService))
		return;

	/* Power outputs are disabled before application state is initialized. */
	PowerStage_Initialize(&MotorPowerStage, &power_stage_port);
	if (!ControlAuthorityService_Initialize(&ControlAuthorityService) ||
		!CanInterface_Initialize(&CanInterface, &can_transport,
			&ControlAuthorityService, board_profile->default_can_bitrate_kbps,
			board_profile->minimum_can_heartbeat_ms,
			board_profile->maximum_can_heartbeat_ms))
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
			&TelemetryService, &ControlAuthorityService) ||
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
