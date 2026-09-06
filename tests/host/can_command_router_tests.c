#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "Core/Communication/Router/can_command_router.h"

/*
 * The interface fault suite supplies a router spy with the production symbol.
 * Compile the real router here under suite-local names so both suites exercise
 * their intended seam in the same host executable.
 */
#define CanCommandRouter_Initialize CanCommandRouter_TestInitialize
#define CanCommandRouter_Handle CanCommandRouter_TestHandle
#include "../../Firmware/Core/Communication/Router/can_command_router.c"
#undef CanCommandRouter_Handle
#undef CanCommandRouter_Initialize

#define TEST_TWO_PI      6.2831853072f
#define TEST_ONE_BY_2PI  0.15915494309f
#define TEST_CLOSE(a_, b_) (fabsf((a_) - (b_)) <= 0.0001f)

typedef struct
{
	float values[MOTOR_PARAMETER_FLUX_WEBER + 1];
	MotorParameterId staged_parameter;
	float staged_value;
	unsigned int stage_count;
} FakeMotorConfiguration;

typedef struct
{
	uint8_t parameter_id;
	float value;
	unsigned int count;
} FakeCanResponse;

typedef struct
{
	CanParameterId set_id;
	CanParameterId get_id;
	MotorParameterId parameter;
	bool revolutions_unit;
	float write_value;
} ExpectedParameterRoute;

typedef struct
{
	CanParameterId response_id;
	MotorTelemetryId telemetry;
	bool revolutions_unit;
} ExpectedTelemetryRoute;

static const ExpectedParameterRoute ExpectedParameterRoutes[] =
{
	{ CAN_SET_POLEPARIS, CAN_GET_POLEPARIS, MOTOR_PARAMETER_POLE_PAIRS, false, 2.0f },
	{ CAN_SET_CURRENT_CAL, CAN_GET_CURRENT_CAL, MOTOR_PARAMETER_CALIBRATION_CURRENT_A, false, 1.0f },
	{ CAN_SET_CURRENT_LIMIT, CAN_GET_CURRENT_LIMIT, MOTOR_PARAMETER_CURRENT_LIMIT_A, false, 1.0f },
	{ CAN_SET_SPEED_LIMIT, CAN_GET_SPEED_LIMIT, MOTOR_PARAMETER_SPEED_LIMIT_RAD_S, true, 1.0f },
	{ CAN_SET_SPEED_ACC, CAN_GET_SPEED_ACC, MOTOR_PARAMETER_SPEED_ACCELERATION_RAD_S2, true, 1.0f },
	{ CAN_SET_SPEED_DEC, CAN_GET_SPEED_DEC, MOTOR_PARAMETER_SPEED_DECELERATION_RAD_S2, true, 1.0f },
	{ CAN_SET_SPEED_KP, CAN_GET_SPEED_KP, MOTOR_PARAMETER_SPEED_KP, false, 1.0f },
	{ CAN_SET_SPEED_KI, CAN_GET_SPEED_KI, MOTOR_PARAMETER_SPEED_KI, false, 1.0f },
	{ CAN_SET_POS_ACC, CAN_GET_POS_ACC, MOTOR_PARAMETER_POSITION_ACCELERATION_RAD_S2, true, 1.0f },
	{ CAN_SET_POS_DEC, CAN_GET_POS_DEC, MOTOR_PARAMETER_POSITION_DECELERATION_RAD_S2, true, 1.0f },
	{ CAN_SET_POS_MAXSPEED, CAN_GET_POS_MAXSPEED, MOTOR_PARAMETER_POSITION_MAX_SPEED_RAD_S, true, 1.0f },
	{ CAN_SET_POS_KP, CAN_GET_POS_KP, MOTOR_PARAMETER_POSITION_KP_A_PER_RAD, false, 1.0f },
	{ CAN_SET_POS_KD, CAN_GET_POS_KD, MOTOR_PARAMETER_POSITION_KD_A_PER_RAD_S, false, 1.0f },
	{ CAN_SET_POS_KI, CAN_GET_POS_KI, MOTOR_PARAMETER_POSITION_KI_A_PER_RAD_S, false, 1.0f },
	{ CAN_SET_POS_INTEGRAL_LIMIT, CAN_GET_POS_INTEGRAL_LIMIT, MOTOR_PARAMETER_POSITION_INTEGRAL_LIMIT_A, false, 1.0f },
	{ CAN_SET_CASCADE_POS_KP, CAN_GET_CASCADE_POS_KP, MOTOR_PARAMETER_CASCADE_POSITION_KP_PER_S, false, 1.0f },
	{ CAN_SET_CASCADE_POS_KD, CAN_GET_CASCADE_POS_KD, MOTOR_PARAMETER_CASCADE_POSITION_KD, false, 1.0f },
	{ CAN_SET_RS, CAN_GET_RS, MOTOR_PARAMETER_PHASE_RESISTANCE_OHM, false, 1.0f },
	{ CAN_SET_LD, CAN_GET_LD, MOTOR_PARAMETER_D_AXIS_INDUCTANCE_H, false, 1.0f },
	{ CAN_SET_LQ, CAN_GET_LQ, MOTOR_PARAMETER_Q_AXIS_INDUCTANCE_H, false, 1.0f },
	{ CAN_SET_FLUX, CAN_GET_FLUX, MOTOR_PARAMETER_FLUX_WEBER, false, 1.0f }
};

static const ExpectedTelemetryRoute ExpectedTelemetryRoutes[] =
{
	{ CAN_GET_MODE, MOTOR_TELEMETRY_MODE, false },
	{ CAN_GET_CURRENT_SET, MOTOR_TELEMETRY_CURRENT_REFERENCE_A, false },
	{ CAN_GET_SPEED_SET, MOTOR_TELEMETRY_SPEED_REFERENCE_RAD_S, true },
	{ CAN_GET_POS_SET, MOTOR_TELEMETRY_POSITION_REFERENCE_RAD, true },
	{ CAN_GET_ENCODER_REVERSE, MOTOR_TELEMETRY_ENCODER_REVERSED, false },
	{ CAN_GET_COMMISSIONING_STAGE, MOTOR_TELEMETRY_COMMISSIONING_STAGE, false },
	{ CAN_GET_COMMISSIONING_PROGRESS, MOTOR_TELEMETRY_COMMISSIONING_PROGRESS_PERCENT, false },
	{ CAN_GET_RESISTANCE_SPREAD, MOTOR_TELEMETRY_PHASE_RESISTANCE_SPREAD_PERCENT, false },
	{ CAN_GET_RESISTANCE_DESIGN_ERROR, MOTOR_TELEMETRY_PHASE_RESISTANCE_DESIGN_ERROR_PERCENT, false },
	{ CAN_GET_VBUS, MOTOR_TELEMETRY_BUS_VOLTAGE_V, false },
	{ CAN_GET_IBUS, MOTOR_TELEMETRY_BUS_CURRENT_A, false },
	{ CAN_GET_IA, MOTOR_TELEMETRY_PHASE_A_CURRENT_A, false },
	{ CAN_GET_IB, MOTOR_TELEMETRY_PHASE_B_CURRENT_A, false },
	{ CAN_GET_IC, MOTOR_TELEMETRY_PHASE_C_CURRENT_A, false },
	{ CAN_GET_ID, MOTOR_TELEMETRY_D_AXIS_CURRENT_A, false },
	{ CAN_GET_IQ, MOTOR_TELEMETRY_Q_AXIS_CURRENT_A, false },
	{ CAN_GET_SPEED2_FILT, MOTOR_TELEMETRY_MECHANICAL_SPEED_RAD_S, false },
	{ CAN_GET_POS2_FILT, MOTOR_TELEMETRY_MECHANICAL_POSITION_RAD, false },
	{ CAN_GET_TEMP, MOTOR_TELEMETRY_TEMPERATURE_C, false },
	{ CAN_GET_ERROR, MOTOR_TELEMETRY_PRIMARY_ERROR, false }
};

static bool FakeMotorCanStage(void *context)
{
	(void)context;
	return true;
}

static bool FakeMotorRead(void *context, MotorParameterId parameter, float *value)
{
	FakeMotorConfiguration *fake = (FakeMotorConfiguration *)context;
	if (fake == 0 || value == 0 || parameter > MOTOR_PARAMETER_FLUX_WEBER)
		return false;
	*value = fake->values[parameter];
	return true;
}

static bool FakeMotorStage(void *context, MotorParameterId parameter, float value)
{
	FakeMotorConfiguration *fake = (FakeMotorConfiguration *)context;
	if (fake == 0 || parameter > MOTOR_PARAMETER_FLUX_WEBER)
		return false;
	fake->values[parameter] = value;
	fake->staged_parameter = parameter;
	fake->staged_value = value;
	++fake->stage_count;
	return true;
}

static bool FakeQueueResponse(void *context, uint8_t parameter_id, float value)
{
	FakeCanResponse *fake = (FakeCanResponse *)context;
	if (fake == 0)
		return false;
	fake->parameter_id = parameter_id;
	fake->value = value;
	++fake->count;
	return true;
}

static void SetTelemetryValue(MotorTelemetrySnapshot *snapshot,
	MotorTelemetryId telemetry, float value)
{
	switch (telemetry)
	{
		case MOTOR_TELEMETRY_MODE: snapshot->mode = (uint32_t)value; break;
		case MOTOR_TELEMETRY_PRIMARY_ERROR: snapshot->primary_error = (uint32_t)value; break;
		case MOTOR_TELEMETRY_CURRENT_REFERENCE_A: snapshot->current_reference_a = value; break;
		case MOTOR_TELEMETRY_SPEED_REFERENCE_RAD_S: snapshot->speed_reference_rad_s = value; break;
		case MOTOR_TELEMETRY_POSITION_REFERENCE_RAD: snapshot->position_reference_rad = value; break;
		case MOTOR_TELEMETRY_BUS_VOLTAGE_V: snapshot->bus_voltage_v = value; break;
		case MOTOR_TELEMETRY_BUS_CURRENT_A: snapshot->bus_current_a = value; break;
		case MOTOR_TELEMETRY_PHASE_A_CURRENT_A: snapshot->phase_a_current_a = value; break;
		case MOTOR_TELEMETRY_PHASE_B_CURRENT_A: snapshot->phase_b_current_a = value; break;
		case MOTOR_TELEMETRY_PHASE_C_CURRENT_A: snapshot->phase_c_current_a = value; break;
		case MOTOR_TELEMETRY_D_AXIS_CURRENT_A: snapshot->d_axis_current_a = value; break;
		case MOTOR_TELEMETRY_Q_AXIS_CURRENT_A: snapshot->q_axis_current_a = value; break;
		case MOTOR_TELEMETRY_MECHANICAL_SPEED_RAD_S: snapshot->mechanical_speed_rad_s = value; break;
		case MOTOR_TELEMETRY_MECHANICAL_POSITION_RAD: snapshot->mechanical_position_rad = value; break;
		case MOTOR_TELEMETRY_TEMPERATURE_C: snapshot->temperature_c = value; break;
		case MOTOR_TELEMETRY_ENCODER_REVERSED: snapshot->encoder_reversed = (uint32_t)value; break;
		case MOTOR_TELEMETRY_COMMISSIONING_STAGE: snapshot->commissioning_stage = (uint32_t)value; break;
		case MOTOR_TELEMETRY_COMMISSIONING_PROGRESS_PERCENT: snapshot->commissioning_progress_percent = (uint32_t)value; break;
		case MOTOR_TELEMETRY_PHASE_RESISTANCE_SPREAD_PERCENT: snapshot->phase_resistance_spread_percent = value; break;
		case MOTOR_TELEMETRY_PHASE_RESISTANCE_DESIGN_ERROR_PERCENT: snapshot->phase_resistance_design_error_percent = value; break;
		default: break;
	}
}

static int VerifyParameterRoutes(CanCommandRouterContext *router,
	FakeMotorConfiguration *configuration, FakeCanResponse *response)
{
	unsigned int index;
	if (sizeof(ExpectedParameterRoutes) / sizeof(ExpectedParameterRoutes[0]) !=
		sizeof(g_motor_parameter_routes) / sizeof(g_motor_parameter_routes[0]))
		return __LINE__;
	for (index = 0U; index < sizeof(ExpectedParameterRoutes) / sizeof(ExpectedParameterRoutes[0]); ++index)
	{
		const ExpectedParameterRoute *route = &ExpectedParameterRoutes[index];
		const float expected_write = route->revolutions_unit ?
			route->write_value * TEST_TWO_PI : route->write_value;
		const unsigned int old_stage_count = configuration->stage_count;
		if ((int)g_motor_parameter_routes[index].set_id != (int)route->set_id ||
			(int)g_motor_parameter_routes[index].get_id != (int)route->get_id ||
			(int)g_motor_parameter_routes[index].parameter != (int)route->parameter ||
			(g_motor_parameter_routes[index].revolutions_unit != 0U) !=
				route->revolutions_unit)
			return __LINE__;
		CanCommandRouter_TestHandle(router, route->set_id, route->write_value);
		if (configuration->stage_count != old_stage_count + 1U)
			return __LINE__;
		if (configuration->staged_parameter != route->parameter ||
			!TEST_CLOSE(configuration->staged_value, expected_write))
			return __LINE__;

		configuration->values[route->parameter] = route->revolutions_unit ?
			2.0f * TEST_TWO_PI : 3.25f;
		response->count = 0U;
		CanCommandRouter_TestHandle(router, route->get_id, 0.0f);
		if (response->count != 1U || response->parameter_id != (uint8_t)route->get_id)
			return __LINE__;
		if (!TEST_CLOSE(response->value, route->revolutions_unit ? 2.0f : 3.25f))
			return __LINE__;
	}
	return 0;
}

static int VerifyTelemetryRoutes(CanCommandRouterContext *router,
	TelemetryServiceContext *telemetry, FakeCanResponse *response)
{
	unsigned int index;
	if (sizeof(ExpectedTelemetryRoutes) / sizeof(ExpectedTelemetryRoutes[0]) !=
		sizeof(g_telemetry_routes) / sizeof(g_telemetry_routes[0]))
		return __LINE__;
	for (index = 0U; index < sizeof(ExpectedTelemetryRoutes) / sizeof(ExpectedTelemetryRoutes[0]); ++index)
	{
		MotorTelemetrySnapshot snapshot;
		const ExpectedTelemetryRoute *route = &ExpectedTelemetryRoutes[index];
		const float core_value = route->revolutions_unit ? 2.0f * TEST_TWO_PI : 7.0f;
		if ((int)g_telemetry_routes[index].response_id != (int)route->response_id ||
			(int)g_telemetry_routes[index].telemetry != (int)route->telemetry ||
			(g_telemetry_routes[index].revolutions_unit != 0U) !=
				route->revolutions_unit)
			return __LINE__;
		(void)memset(&snapshot, 0, sizeof(snapshot));
		SetTelemetryValue(&snapshot, route->telemetry, core_value);
		TelemetryService_Publish(telemetry, &snapshot);
		response->count = 0U;
		CanCommandRouter_TestHandle(router, route->response_id, 0.0f);
		if (response->count != 1U || response->parameter_id != (uint8_t)route->response_id)
			return __LINE__;
		if (!TEST_CLOSE(response->value, route->revolutions_unit ? 2.0f : 7.0f))
			return __LINE__;
	}
	return 0;
}

int CanCommandRouter_RunHostTests(void)
{
	FakeMotorConfiguration fake_configuration;
	FakeCanResponse fake_response;
	MotorConfigurationPort configuration_port;
	CanResponsePort response_port;
	ParameterServiceLimits limits;
	ParameterServiceContext parameters;
	TelemetryServiceContext telemetry;
	CanResponseServiceContext response;
	ApplicationEndpoints endpoints;
	CanCommandRouterContext router;
	int result;

	(void)memset(&fake_configuration, 0, sizeof(fake_configuration));
	(void)memset(&fake_response, 0, sizeof(fake_response));
	(void)memset(&limits, 0, sizeof(limits));
	(void)memset(&endpoints, 0, sizeof(endpoints));
	configuration_port.context = &fake_configuration;
	configuration_port.can_stage = FakeMotorCanStage;
	configuration_port.read = FakeMotorRead;
	configuration_port.stage = FakeMotorStage;
	response_port.context = &fake_response;
	response_port.queue_response = FakeQueueResponse;
	limits.command_current_limit_a = 1000.0f;
	limits.calibration_current_limit_a = 1000.0f;
	limits.speed_limit_max_rad_s = 1000.0f;
	limits.speed_ramp_max_rad_s2 = 1000.0f;
	limits.position_ramp_max_rad_s2 = 1000.0f;
	limits.position_speed_limit_rad_s = 1000.0f;
	limits.position_kp_limit_a_per_rad = 1000.0f;
	limits.position_kd_limit_a_per_rad_s = 1000.0f;
	limits.position_ki_limit_a_per_rad_s = 1000.0f;
	limits.cascade_position_kp_limit_per_s = 1000.0f;
	limits.cascade_position_kd_limit = 1000.0f;
	limits.phase_resistance_min_ohm = 0.0f;
	limits.phase_resistance_max_ohm = 1000.0f;
	limits.inductance_min_h = 0.0f;
	limits.inductance_max_h = 1000.0f;
	limits.flux_min_weber = 0.0f;
	limits.flux_max_weber = 1000.0f;
	fake_configuration.values[MOTOR_PARAMETER_SPEED_LIMIT_RAD_S] = 1000.0f;

	if (!ParameterService_Initialize(&parameters, &configuration_port, &limits) ||
		!TelemetryService_Initialize(&telemetry) ||
		!CanResponseService_Initialize(&response, &response_port))
		return __LINE__;
	endpoints.parameters = &parameters;
	endpoints.telemetry = &telemetry;
	if (!CanCommandRouter_TestInitialize(&router, &endpoints, &response))
		return __LINE__;

	result = VerifyParameterRoutes(&router, &fake_configuration, &fake_response);
	if (result != 0)
		return result;
	result = VerifyTelemetryRoutes(&router, &telemetry, &fake_response);
	if (result != 0)
		return result;

	return 0;
}
