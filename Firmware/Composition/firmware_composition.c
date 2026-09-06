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
#include "Core/Application/rotor_calibration_service.h"
#include "Core/Application/motor_command_service.h"
#include "parameter_service.h"
#include "rgb.h"
#include "usb_cdc_transport.h"
#include "fault_port_adapter.h"
#include "Core/Application/communication_watchdog_service.h"
#include "can_configuration_service.h"
#include "Core/Communication/Can/can_response_service.h"
#include "product_catalog.h"
#include "Core/Application/parameter_transaction_service.h"
#include "parameter_transaction_adapter.h"
#include "Core/Infrastructure/Telemetry/telemetry_service.h"
#include "supervisor_task.h"
#include "control_tuning_profile.h"
#include "mechanical_load_profiles.h"
#include "Core/Application/friction_identification_service.h"
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

static void FirmwareComposition_ProjectProductConfig(
	const ProductConfig *product_config,
	MotorControlRuntimeConfig *motor_runtime_config,
	MeasurementModelConfig *measurement_config,
	ParameterServiceLimits *parameter_limits)
{
	const ProductCurrentSenseConfig *current =
		&product_config->board->current_sense;
	const ProductTemperatureSensorInstanceConfig *temperature =
		&product_config->temperature_sensors[0];

	measurement_config->minimum_valid_offset_adc =
		current->minimum_valid_offset_count;
	measurement_config->maximum_valid_offset_adc =
		current->maximum_valid_offset_count;
	measurement_config->current_a_per_count = current->current_a_per_count;
	motor_runtime_config->current_offset_limits.minimum_current_offset_adc =
		current->minimum_valid_offset_count;
	motor_runtime_config->current_offset_limits.maximum_current_offset_adc =
		current->maximum_valid_offset_count;
	motor_runtime_config->parameter_snapshot.default_current_offset_adc =
		current->default_offset_count;
	motor_runtime_config->parameter_snapshot.minimum_current_offset_adc =
		current->minimum_valid_offset_count;
	motor_runtime_config->parameter_snapshot.maximum_current_offset_adc =
		current->maximum_valid_offset_count;
	measurement_config->bus_voltage_v_per_count =
		product_config->board->bus_voltage_v_per_count;
	measurement_config->bus_voltage_filter_alpha =
		product_config->safety.bus_voltage_filter_alpha;
	measurement_config->overcurrent_trip_a =
		product_config->safety.software_overcurrent_trip_a;
	measurement_config->overvoltage_trip_v =
		product_config->safety.overvoltage_trip_v;
	measurement_config->undervoltage_trip_v =
		product_config->safety.undervoltage_trip_v;
	measurement_config->maximum_temperature_c = temperature->protection_limit_c;
	measurement_config->temperature_protection_enabled =
		temperature->protection_enabled;
	measurement_config->temperature_invalid_is_fault =
		product_config->safety.temperature_invalid_is_fault;
	measurement_config->overcurrent_confirm_cycles =
		product_config->safety.overcurrent_confirm_cycles;
	measurement_config->voltage_confirm_cycles =
		product_config->safety.voltage_confirm_cycles;
	measurement_config->temperature_sample_divider = temperature->sample_divider;

	motor_runtime_config->control_frequency_hz =
		product_config->board->control_frequency_hz;
	motor_runtime_config->current_offset_calibration_sample_count =
		current->offset_calibration_sample_count;
	motor_runtime_config->command_current_limit_a =
		product_config->board->command_phase_current_limit_a;
	motor_runtime_config->phase_resistance.control_frequency_hz =
		product_config->board->control_frequency_hz;
	motor_runtime_config->phase_resistance.path_compensation_ohm =
		product_config->board->phase_resistance_path_compensation_ohm;
	motor_runtime_config->phase_resistance.undervoltage_trip_v =
		product_config->safety.undervoltage_trip_v;
	motor_runtime_config->phase_resistance.overvoltage_trip_v =
		product_config->safety.overvoltage_trip_v;
	motor_runtime_config->parameter_snapshot.current_sense_shunt_milliohm =
		current->nominal_shunt_milliohm;
	motor_runtime_config->parameter_snapshot.default_can_node_id =
		product_config->can.default_node_id;
	motor_runtime_config->parameter_snapshot.default_can_heartbeat_ms =
		product_config->can.heartbeat_ms;
	parameter_limits->command_current_limit_a =
		product_config->board->command_phase_current_limit_a;
	parameter_limits->calibration_current_limit_a =
		product_config->board->calibration_phase_current_limit_a;
}

/**
	* @brief  Initialize board peripherals and application modules
 **/
void FirmwareComposition_Initialize(void)
{
	const ProductConfig *product_config;
	const MotorProfile *motor_profile;
	const EncoderProfile *encoder_profile;
	const ControlTuningProfile *tuning_profile;
	const MechanicalLoadProfile *mechanical_load_profile;
	PowerStagePort power_stage_port;
	MeasurementPort measurement_port;
	RotorSensorPort rotor_sensor_port;
	BspSynchronousSerialPort angle_serial_port;
	const AngleSerialStm32g431ResourceConfig *angle_serial_resources;
	BspCanPort can_transport;
	BspByteStreamPort usb_transport;
	const BspCommunicationEndpointCapabilities *can_capabilities;
	const BspCommunicationEndpointCapabilities *usb_capabilities;
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
	MotorControlRuntimeConfig motor_runtime_config = {0};
	MeasurementModelConfig measurement_config = {0};
	ParameterServiceLimits parameter_limits = {0};

	FirmwareIsInitialized = false;
	product_config = &ProductCatalog_CurrentConfig;
	if (!ProductConfigBridge_ValidateRuntime(
		product_config, &BspVectorMiniSt_RuntimeIdentity,
		&product_bridge_status))
	{
		ProductConfigBridgeStartupStatus = product_bridge_status;
		return;
	}
	motor_profile = MotorProfile_GetActive();
	encoder_profile = EncoderProfile_GetActive();
	tuning_profile = ControlTuningProfile_GetActive();
	mechanical_load_profile = MechanicalLoadProfile_GetActive();
	if (motor_profile == 0 || encoder_profile == 0 || tuning_profile == 0 ||
		mechanical_load_profile == 0)
	{
		ProductConfigBridgeStartupStatus = PRODUCT_CONFIG_BRIDGE_PRODUCT_INVALID;
		return;
	}
	FirmwareComposition_ProjectProductConfig(product_config,
		&motor_runtime_config, &measurement_config, &parameter_limits);
	power_stage_port = PowerStageTim1_CreatePort();
	measurement_port = MeasurementAdc12_CreatePort();
	angle_serial_resources = BspVectorMiniSt_FindAngleSerialResources(
		product_config->angle_sensors[0].endpoint);
	if (!AngleSerialStm32g431_CreatePort(&AngleSerialContext,
		angle_serial_resources, &angle_serial_port))
	{
		ProductConfigBridgeStartupStatus =
			PRODUCT_CONFIG_BRIDGE_BINDING_MISMATCH;
		return;
	}
	rotor_sensor_port = Tle5012bRotorSensorAdapter_CreatePort(
		&RotorSensorAdapter, &angle_serial_port);
	can_capabilities = BspBoard_FindCommunicationEndpoint(
		&BspVectorMiniSt_Capabilities,
		product_config->can.endpoint);
	usb_capabilities = BspBoard_FindCommunicationEndpoint(
		&BspVectorMiniSt_Capabilities,
		product_config->service_stream.endpoint);
	if (!CanFdcan1Transport_CreatePort(can_capabilities, &can_transport) ||
		!UsbCdcTransport_CreatePort(usb_capabilities, &usb_transport))
	{
		ProductConfigBridgeStartupStatus =
			PRODUCT_CONFIG_BRIDGE_BINDING_MISMATCH;
		return;
	}
	ProductConfigBridgeStartupStatus = PRODUCT_CONFIG_BRIDGE_OK;
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
			&ControlAuthorityService,
			product_config->can.mode == PRODUCT_CAN_MODE_FD,
			product_config->can.bit_rate_switching,
			product_config->can.nominal_bitrate_kbps,
			product_config->can.data_bitrate_kbps,
			product_config->can.minimum_heartbeat_ms,
			product_config->can.maximum_heartbeat_ms))
		return;
	can_configuration_port = CanInterface_CreateConfigurationPort(&CanInterface);
	can_response_port = CanInterface_CreateResponsePort(&CanInterface);
	if (!CanConfigurationService_Initialize(&CanConfigurationService,
			&can_configuration_port) ||
		!CanResponseService_Initialize(&CanResponseService, &can_response_port))
	{
		return;
	}
	if (!MotorControlRuntime_Prepare(&MotorControlRuntime, &motor_runtime_config,
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
		&measurement_port, &measurement_config,
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
			&motor_configuration_port, &parameter_limits,
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
