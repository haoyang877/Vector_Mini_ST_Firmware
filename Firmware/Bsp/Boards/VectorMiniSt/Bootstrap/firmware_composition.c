#include "Bsp/Boards/VectorMiniSt/Bootstrap/firmware_composition.h"

#include <stdint.h>

#include "board_runtime.h"
#include "can_fdcan1_transport.h"
#include "motor_state_runtime.h"
#include "parameter_persistence_adapter.h"
#include "motor_control_runtime.h"
#include "Core/Communication/Interfaces/interface_can.h"
#include "Core/Communication/Interfaces/interface_usb.h"
#include "motor_drive_tim1_adc2_stm32g431.h"
#include "temperature_adc1_stm32g431.h"
#include "indicator_stm32g431.h"
#include "Core/Application/Indicators/led.h"
#include "Drivers/Angle/Tle5012b/tle5012b_angle_sensor_adapter.h"
#include "Core/Application/rotor_calibration_service.h"
#include "Core/Application/motor_command_service.h"
#include "Core/Application/Parameters/parameter_service.h"
#include "Core/Application/Indicators/rgb.h"
#include "Core/Application/Supervision/temperature_supervision.h"
#include "usb_cdc_transport.h"
#include "fault_port_adapter.h"
#include "Core/Application/communication_watchdog_service.h"
#include "Core/Application/Communication/can_configuration_service.h"
#include "Core/Communication/Can/can_response_service.h"
#include "product_catalog.h"
#include "Core/Application/parameter_transaction_service.h"
#include "parameter_transaction_adapter.h"
#include "Core/Infrastructure/Telemetry/telemetry_service.h"
#include "Core/Application/Supervision/supervisor_task.h"
#include "Core/Application/friction_identification_service.h"
#include "monotonic_clock_stm32g431.h"
#include "diagnostic_rtt_stm32g431.h"
#include "execution_timer_stm32g431.h"
#include "reset_reason_stm32g431.h"
#include "Core/Application/Diagnostics/diagnostic_service.h"
#include "device_identity_stm32g431.h"
#include "parameter_store_flash.h"
#include "Core/Communication/Router/can_command_router.h"
#include "Core/Communication/Router/usb_command_router.h"
#include "Core/Application/Api/application_endpoints.h"
#include "Bsp/Boards/VectorMiniSt/Bootstrap/product_config_bridge.h"
#include "vector_mini_st_bsp.h"
#include "vector_mini_st_angle_serial.h"
#include "vector_mini_st_storage.h"

#if defined(__CC_ARM)
#pragma O3
#pragma Ospace
#endif

static MotorDriveServiceContext MotorDriveService;
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
static TemperatureSupervisionContext TemperatureSupervision;
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
static AngleSerialStm32g431Context
	AngleSerialContexts[PRODUCT_CONFIG_MAX_ANGLE_SENSORS];
static ParameterStoreFlashContext ParameterStoreContext;
static Tle5012bAngleSensorAdapterContext
	AngleSensorAdapters[PRODUCT_CONFIG_MAX_ANGLE_SENSORS];
static BspAngleSensorPort AngleSensorPorts[PRODUCT_CONFIG_MAX_ANGLE_SENSORS];
static bool FirmwareIsInitialized;
static volatile ProductConfigBridgeStatus ProductConfigBridgeStartupStatus;

static void FirmwareComposition_SuperviseCommunication1kHz(void *context,
	const CommunicationSupervisionInput *input)
{
	(void)context;
	if (input == 0)
		return;
	UsbInterface_UpdateTelemetryStream(&UsbInterface, &TelemetryService,
		&CanConfigurationService);
	CanInterface_Supervise1kHz(&CanInterface,
		input->can_motion_mode_active, &CommunicationWatchdogService);
}

static bool FirmwareComposition_CreateAngleSensorPorts(
	const ProductConfig *product_config)
{
	uint8_t index;

	if (product_config == 0 ||
		product_config->angle_sensor_count > PRODUCT_CONFIG_MAX_ANGLE_SENSORS)
		return false;
	for (index = 0U; index < product_config->angle_sensor_count; index++)
	{
		const ProductAngleSensorInstanceConfig *sensor =
			&product_config->angle_sensors[index];
		const BspAngleSensorEndpointCapabilities *capabilities =
			BspBoard_FindAngleSensorEndpoint(&BspVectorMiniSt_Capabilities,
				sensor->endpoint);
		const AngleSerialStm32g431ResourceConfig *resources;
		BspSynchronousSerialPort transport;

		if (sensor->design == 0)
			return false;
		switch (sensor->design->design_id)
		{
			case PRODUCT_CATALOG_ANGLE_TLE5012B:
				if (sensor->design->source !=
						PRODUCT_ANGLE_SENSOR_SOURCE_ABSOLUTE_SERIAL ||
					sensor->design->counts_per_turn != UINT32_C(65536) ||
					sensor->source !=
						PRODUCT_ANGLE_SENSOR_SOURCE_ABSOLUTE_SERIAL)
				{
					return false;
				}
				break;
			default:
				return false;
		}
		if (
			capabilities == 0 ||
			capabilities->availability != BSP_ENDPOINT_AVAILABLE ||
			capabilities->kind != BSP_ANGLE_ENDPOINT_SYNCHRONOUS_SERIAL ||
			capabilities->maximum_transfer_word_bits < 16U ||
			!capabilities->has_dedicated_select)
			return false;
		resources = BspVectorMiniSt_FindAngleSerialResources(sensor->endpoint);
		if (!AngleSerialStm32g431_CreatePort(&AngleSerialContexts[index],
			resources, &transport) ||
			!Tle5012bAngleSensorAdapter_CreatePort(&AngleSensorAdapters[index],
				&transport, &AngleSensorPorts[index]))
			return false;
	}
	return true;
}

static bool FirmwareComposition_ProjectCurrentSense(
	const ProductCurrentSenseConfig *current,
	const BspMotorDriveEndpointCapabilities *motor_drive_capabilities,
	MotorControlRuntimeConfig *motor_runtime_config,
	MeasurementModelConfig *measurement_config)
{
	ProductCurrentSenseProjection projection;
	uint8_t channel;
	uint8_t flash_slot;

	if (current == 0 || motor_runtime_config == 0 || measurement_config == 0 ||
		!ProductConfigBridge_ProjectCurrentSense(current,
			motor_drive_capabilities, &projection))
		return false;
	measurement_config->current_sense = projection.measurement;

	/* Keep the legacy phase-indexed view coherent while downstream callers
	 * migrate. The formal acquisition pipeline above is authoritative. */
	for (channel = 0U; channel < current->physical_channel_count; channel++)
	{
		uint8_t phase;

		switch (projection.measurement.channels[channel].role)
		{
			case MEASUREMENT_CURRENT_CHANNEL_ROLE_PHASE_A: phase = 0U; break;
			case MEASUREMENT_CURRENT_CHANNEL_ROLE_PHASE_B: phase = 1U; break;
			case MEASUREMENT_CURRENT_CHANNEL_ROLE_PHASE_C: phase = 2U; break;
			case MEASUREMENT_CURRENT_CHANNEL_ROLE_DC_LINK:
				continue;
			default:
				return false;
		}
		measurement_config->phase_channel_index[phase] =
			projection.measurement.channels[channel].acquisition_index;
		measurement_config->minimum_valid_offset_adc[phase] =
			projection.measurement.channels[channel].minimum_valid_offset_adc;
		measurement_config->maximum_valid_offset_adc[phase] =
			projection.measurement.channels[channel].maximum_valid_offset_adc;
		measurement_config->current_a_per_count[phase] =
			projection.measurement.channels[channel].current_a_per_count;
	}
	for (flash_slot = 0U; flash_slot < MEASUREMENT_MODEL_PHASE_COUNT;
		flash_slot++)
	{
		motor_runtime_config->current_offset_limits.
			minimum_current_offset_adc[flash_slot] =
			projection.minimum_offset_adc[flash_slot];
		motor_runtime_config->current_offset_limits.
			maximum_current_offset_adc[flash_slot] =
			projection.maximum_offset_adc[flash_slot];
		motor_runtime_config->parameter_snapshot.
			default_current_offset_adc[flash_slot] =
			projection.default_offset_adc[flash_slot];
		motor_runtime_config->parameter_snapshot.
			minimum_current_offset_adc[flash_slot] =
			projection.minimum_offset_adc[flash_slot];
		motor_runtime_config->parameter_snapshot.
			maximum_current_offset_adc[flash_slot] =
			projection.maximum_offset_adc[flash_slot];
	}
	return true;
}

static bool FirmwareComposition_ProjectMotorDrive(
	const ProductConfig *product_config,
	BspMotorDriveConfiguration *configuration)
{
	const ProductCurrentSenseConfig *current;
	uint8_t channel;
	BspMotorPhaseSet phase_mask = 0U;

	if (product_config == 0 || product_config->board == 0 ||
		configuration == 0)
		return false;
	current = &product_config->board->current_sense;
	switch (current->topology)
	{
		case PRODUCT_CURRENT_SENSE_TOPOLOGY_INLINE_3_SHUNT:
			configuration->current_sense_topology =
				BSP_CURRENT_SENSE_PHASE_INLINE_THREE_SENSOR;
			break;
		case PRODUCT_CURRENT_SENSE_TOPOLOGY_LOW_SIDE_3_SHUNT:
			configuration->current_sense_topology =
				BSP_CURRENT_SENSE_LOW_SIDE_THREE_SHUNT;
			break;
		case PRODUCT_CURRENT_SENSE_TOPOLOGY_LOW_SIDE_2_SHUNT:
			configuration->current_sense_topology =
				BSP_CURRENT_SENSE_LOW_SIDE_TWO_SHUNT;
			break;
		case PRODUCT_CURRENT_SENSE_TOPOLOGY_DC_LINK_1_SHUNT:
			configuration->current_sense_topology =
				BSP_CURRENT_SENSE_DC_LINK_SINGLE_SHUNT;
			break;
		default:
			return false;
	}
	configuration->pwm_frequency_hz =
		product_config->board->control_frequency_hz;
	configuration->sampling_mode =
		current->topology ==
			PRODUCT_CURRENT_SENSE_TOPOLOGY_DC_LINK_1_SHUNT ?
			BSP_CURRENT_SAMPLING_MODE_DYNAMIC :
			BSP_CURRENT_SAMPLING_MODE_FIXED;
	if (configuration->sampling_mode == BSP_CURRENT_SAMPLING_MODE_DYNAMIC)
	{
		configuration->fixed_sample_count = 0U;
		configuration->fixed_direct_phase_currents = 0U;
		return true;
	}
	for (channel = 0U; channel < current->physical_channel_count; channel++)
	{
		switch (current->channel_roles[channel])
		{
			case PRODUCT_CURRENT_CHANNEL_ROLE_PHASE_A:
				phase_mask |= BSP_MOTOR_PHASE_A; break;
			case PRODUCT_CURRENT_CHANNEL_ROLE_PHASE_B:
				phase_mask |= BSP_MOTOR_PHASE_B; break;
			case PRODUCT_CURRENT_CHANNEL_ROLE_PHASE_C:
				phase_mask |= BSP_MOTOR_PHASE_C; break;
			default:
				return false;
		}
	}
	configuration->fixed_sample_count = current->physical_channel_count;
	configuration->fixed_direct_phase_currents = phase_mask;
	return true;
}

static bool FirmwareComposition_ProjectProductConfig(
	const ProductConfig *product_config,
	const BspBoardCapabilities *board_capabilities,
	const BspMotorDriveEndpointCapabilities *motor_drive_capabilities,
	MotorControlRuntimeConfig *motor_runtime_config,
	MeasurementModelConfig *measurement_config,
	ParameterServiceLimits *parameter_limits,
	TemperatureSupervisionConfig *temperature_config,
	ProductTemperatureRuntimeProjection *temperature_projection)
{
	const ProductCurrentSenseConfig *current =
		&product_config->board->current_sense;

	if (board_capabilities == 0 || temperature_config == 0 ||
		temperature_projection == 0 ||
		!ProductConfigBridge_ProjectFeedback(product_config,
			&motor_runtime_config->feedback) ||
		!ProductConfigBridge_ProjectControlModes(product_config,
			&motor_runtime_config->feedback,
			&motor_runtime_config->allowed_control_mode_mask) ||
		!ProductConfigBridge_ProjectTemperature(product_config,
			board_capabilities, temperature_projection) ||
		!ProductConfigBridge_ProjectCommissioning(product_config,
			&motor_runtime_config->commissioning_stage_mask) ||
		!FirmwareComposition_ProjectCurrentSense(current,
		motor_drive_capabilities,
		motor_runtime_config, measurement_config))
		return false;
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
	measurement_config->overcurrent_confirm_cycles =
		product_config->safety.overcurrent_confirm_cycles;
	measurement_config->voltage_confirm_cycles =
		product_config->safety.voltage_confirm_cycles;

	if (temperature_projection->supervision_enabled)
	{
		const ProductTemperatureSensorInstanceConfig *temperature =
			&product_config->temperature_sensors[
				temperature_projection->sensor_index];

		temperature_config->sample_period_ms = temperature->sample_period_ms;
		temperature_config->pending_timeout_ms =
			temperature->pending_timeout_ms;
		temperature_config->monitor.protection_enabled =
			temperature_projection->protection_enabled;
		temperature_config->monitor.trip_temperature_c =
			temperature->protection_limit_c;
		temperature_config->monitor.trip_on_invalid_sample =
			product_config->safety.temperature_invalid_is_fault;
		temperature_config->monitor.trip_on_stale_sample =
			product_config->safety.temperature_invalid_is_fault;
		temperature_config->monitor.trip_on_sensor_open =
			product_config->safety.temperature_invalid_is_fault;
		temperature_config->monitor.trip_on_sensor_short =
			product_config->safety.temperature_invalid_is_fault;
		temperature_config->monitor.trip_on_sensor_fault =
			product_config->safety.temperature_invalid_is_fault;
	}

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
	motor_runtime_config->phase_resistance.tuning =
		product_config->commissioning_tuning.phase_resistance;
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
	parameter_limits->speed_limit_max_rad_s =
		product_config->control.parameter_limits.speed_limit_max_rad_s;
	parameter_limits->speed_ramp_max_rad_s2 =
		product_config->control.parameter_limits.speed_ramp_max_rad_s2;
	parameter_limits->position_ramp_max_rad_s2 =
		product_config->control.parameter_limits.position_ramp_max_rad_s2;
	parameter_limits->position_speed_limit_rad_s =
		product_config->control.parameter_limits.position_speed_limit_rad_s;
	parameter_limits->position_kp_limit_a_per_rad =
		product_config->control.parameter_limits.position_kp_limit_a_per_rad;
	parameter_limits->position_kd_limit_a_per_rad_s =
		product_config->control.parameter_limits.position_kd_limit_a_per_rad_s;
	parameter_limits->position_ki_limit_a_per_rad_s =
		product_config->control.parameter_limits.position_ki_limit_a_per_rad_s;
	parameter_limits->cascade_position_kp_limit_per_s =
		product_config->control.parameter_limits.cascade_position_kp_limit_per_s;
	parameter_limits->cascade_position_kd_limit =
		product_config->control.parameter_limits.cascade_position_kd_limit;
	parameter_limits->phase_resistance_min_ohm =
		product_config->motor_acceptance.phase_resistance_min_ohm;
	parameter_limits->phase_resistance_max_ohm =
		product_config->motor_acceptance.phase_resistance_max_ohm;
	parameter_limits->inductance_min_h =
		product_config->motor_acceptance.inductance_min_h;
	parameter_limits->inductance_max_h =
		product_config->motor_acceptance.inductance_max_h;
	parameter_limits->flux_min_weber =
		product_config->motor_acceptance.flux_min_weber;
	parameter_limits->flux_max_weber =
		product_config->motor_acceptance.flux_max_weber;
	return true;
}

/**
	* @brief  Initialize board peripherals and application modules
 **/
void FirmwareComposition_Initialize(void)
{
	const ProductConfig *product_config;
	const ProductCatalogEntry *catalog_entry;
	BspMotorDrivePort motor_drive_port;
	BspMotorDriveConfiguration motor_drive_configuration;
	const BspMotorDriveEndpointCapabilities *motor_drive_capabilities;
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
	BspTemperaturePort temperature_port;
	const BspTemperatureEndpointCapabilities *temperature_capabilities;
	TemperatureSupervisionContext *active_temperature_supervision = 0;
	BspResetReasonPort reset_reason_port;
	BspUniqueIdPort device_identity_port;
	BspDiagnosticSinkPort diagnostic_transport;
	BspNonvolatileStoragePort parameter_store;
	CommunicationSupervisionPort communication_supervision_port;
	ProductConfigBridgeStatus product_bridge_status;
	ProductTemperatureRuntimeProjection temperature_projection;
	MotorControlRuntimeConfig motor_runtime_config = {0};
	MeasurementModelConfig measurement_config = {0};
	ParameterServiceLimits parameter_limits = {0};
	ParameterPersistenceRuntimeConfig persistence_config = {0};
	TemperatureSupervisionConfig temperature_config = {0};

	FirmwareIsInitialized = false;
	catalog_entry = ProductCatalog_GetCurrent();
	product_config = catalog_entry != 0 ? catalog_entry->config : 0;
	if (!ProductConfigBridge_ValidateRuntime(
		catalog_entry, &BspVectorMiniSt_RuntimeIdentity,
		&BspVectorMiniSt_Capabilities,
		&product_bridge_status))
	{
		ProductConfigBridgeStartupStatus = product_bridge_status;
		return;
	}
	motor_drive_capabilities = BspBoard_FindMotorDriveEndpoint(
		&BspVectorMiniSt_Capabilities,
		product_config->board->motor_drive_endpoint);
	if (!FirmwareComposition_ProjectProductConfig(product_config,
			&BspVectorMiniSt_Capabilities,
			motor_drive_capabilities,
			&motor_runtime_config, &measurement_config, &parameter_limits,
			&temperature_config, &temperature_projection) ||
		!FirmwareComposition_ProjectMotorDrive(product_config,
			&motor_drive_configuration))
	{
		ProductConfigBridgeStartupStatus =
			PRODUCT_CONFIG_BRIDGE_RUNTIME_UNSUPPORTED;
		return;
	}
	if (!MotorDriveTim1Adc2Stm32g431_CreatePort(motor_drive_capabilities,
			&motor_drive_port) ||
		!MotorDriveService_Initialize(&MotorDriveService, &motor_drive_port,
			&motor_drive_configuration))
	{
		ProductConfigBridgeStartupStatus =
			PRODUCT_CONFIG_BRIDGE_BINDING_MISMATCH;
		return;
	}
	if (!FirmwareComposition_CreateAngleSensorPorts(product_config))
	{
		ProductConfigBridgeStartupStatus =
			PRODUCT_CONFIG_BRIDGE_BINDING_MISMATCH;
		return;
	}
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
	if (temperature_projection.supervision_enabled)
	{
		const ProductTemperatureSensorInstanceConfig *temperature =
			&product_config->temperature_sensors[
				temperature_projection.sensor_index];

		temperature_capabilities = BspBoard_FindTemperatureEndpoint(
			&BspVectorMiniSt_Capabilities,
			temperature->endpoint);
		if (!TemperatureAdc1Stm32g431_CreatePort(temperature_capabilities,
				&monotonic_clock, &temperature_port) ||
			TemperatureSupervision_Initialize(&TemperatureSupervision,
				&temperature_port, &temperature_config) !=
				TEMPERATURE_SUPERVISION_STATUS_OK)
		{
			ProductConfigBridgeStartupStatus =
				PRODUCT_CONFIG_BRIDGE_BINDING_MISMATCH;
			return;
		}
		active_temperature_supervision = &TemperatureSupervision;
	}
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
		&device_identity_port, &catalog_entry->manifest, &TelemetryService))
		return;

	if (!ControlAuthorityService_Initialize(&ControlAuthorityService) ||
		!CanInterface_Initialize(&CanInterface, &can_transport,
			&monotonic_clock, &BoardCriticalSection,
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
		product_config->motor, &product_config->control,
		&product_config->commissioning_tuning,
		&product_config->motor_acceptance,
		&monotonic_clock, &execution_timer, &CanConfigurationService))
		return;
	persistence_config.product_id = catalog_entry->manifest.product_id;
	persistence_config.hardware_compatibility_id =
		catalog_entry->persistence.hardware_compatibility_id;
	persistence_config.motor_compatibility_id =
		catalog_entry->persistence.motor_compatibility_id;
	persistence_config.parameter_schema_version =
		catalog_entry->manifest.parameter_schema_version;
	persistence_config.configuration_fingerprint =
		product_config->identity.configuration_fingerprint;
	persistence_config.allow_erased_fingerprint_migration =
		catalog_entry->persistence.allow_erased_fingerprint_migration;
	if (!ParameterPersistenceAdapter_Initialize(&ParameterPersistenceAdapter,
		&parameter_store, &MotorControlRuntime.parameter_snapshot,
		&persistence_config))
		return;

	/*read parameters and calibration data from flash*/
	/*if magic word invalid or not calibrated, fall back to code defaults*/
	ParameterPersistenceAdapter_Load(&ParameterPersistenceAdapter);

	/*motor control related parameters initialize*/
	MotorControlRuntime_Initialize(&MotorControlRuntime, &MotorDriveService,
		&measurement_config,
		AngleSensorPorts, product_config->angle_sensor_count,
		&BoardCriticalSection);
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
			&motor_configuration_port, &parameter_limits) ||
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
	communication_supervision_port.context = 0;
	communication_supervision_port.execute_1khz =
		FirmwareComposition_SuperviseCommunication1kHz;
	if (!SupervisorTask_Initialize(&SupervisorTask, &diagnostic_transport,
		&communication_supervision_port, active_temperature_supervision,
		&MotorControlRuntime, &TelemetryService, &LedService, &RgbService))
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
	/* The static context is zero-initialized before CAN can run. The interface
	 * itself gates reception until its transport has started. */
	CanInterface_OnReceiveInterrupt(&CanInterface);
}

void FirmwareComposition_RunBackground(void)
{
	if (!FirmwareIsInitialized)
		return;
	UsbInterface_ProcessReceivedCommands(&UsbInterface, &UsbCommandRouter);
	UsbInterface_FlushTransmit(&UsbInterface, &FrictionIdentificationService,
		&RotorCalibrationService);
	CanInterface_RunBackground(&CanInterface, &CanCommandRouter,
		&CommunicationWatchdogService);
	SupervisorTask_RunBackground(&SupervisorTask);
	ParameterTransactionService_RunBackground(&ParameterTransactionService);
}
