#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "Core/Communication/Router/usb_command_router.h"

/* Exercise the production implementation while avoiding a public-symbol clash. */
#define UsbCommandRouter_Initialize UsbCommandRouter_TestInitialize
#define UsbCommandRouter_Handle UsbCommandRouter_TestHandle
#include "../../Firmware/Core/Communication/Router/usb_command_router.c"
#undef UsbCommandRouter_Handle
#undef UsbCommandRouter_Initialize

#define TEST_TWO_PI      6.2831853072f
#define TEST_ONE_BY_2PI  0.15915494309f
#define TEST_CLOSE(a_, b_) (fabsf((a_) - (b_)) <= 0.0001f)

typedef struct
{
	float values[MOTOR_PARAMETER_FLUX_WEBER + 1];
	MotorParameterId staged_parameter;
	float staged_value;
	unsigned int stage_count;
	bool allow_can_stage;
	bool allow_stage;
} FakeMotorConfiguration;

typedef struct
{
	MotorPortMode mode;
	float current_reference_a;
	float speed_reference_rad_s;
	float position_reference_rad;
} FakeMotorCommand;

typedef struct
{
	bool reverse;
} FakeRotorCalibration;

typedef struct
{
	uint8_t node_id;
	uint32_t bitrate_kbps;
	uint32_t heartbeat_ms;
} FakeCanConfiguration;

typedef struct
{
	FrictionIdentificationPortStatus status;
	bool status_available;
	bool apply_allowed;
} FakeFrictionIdentification;

typedef struct
{
	UsbParameterId usb_parameter;
	MotorParameterId motor_parameter;
	uint8_t write_scale;
	uint8_t read_scale;
	uint8_t format;
	uint8_t precision;
	bool integer_write;
	const char *prefix;
	const char *suffix;
	float write_value;
	float core_read_value;
	const char *expected_text;
} ExpectedMotorRoute;

typedef struct
{
	UsbParameterId usb_parameter;
	MotorTelemetryId telemetry;
	uint8_t scale;
	uint8_t format;
	uint8_t precision;
	const char *prefix;
	const char *suffix;
	float core_value;
	const char *expected_text;
} ExpectedTelemetryRoute;

static const ExpectedMotorRoute ExpectedMotorRoutes[] =
{
	{ USB_POLEPARIS, MOTOR_PARAMETER_POLE_PAIRS, USB_ROUTE_SCALE_ONE,
		USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_I32, 0, true, "pol=", "\r\n",
		7.0f, 7.0f, "pol=7\r\n" },
	{ USB_CURRENT_CAL, MOTOR_PARAMETER_CALIBRATION_CURRENT_A, USB_ROUTE_SCALE_ONE,
		USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_FIXED, 2, false, "i_cal=", "A\r\n",
		1.25f, 1.25f, "i_cal=1.25A\r\n" },
	{ USB_CURRENT_LIMIT, MOTOR_PARAMETER_CURRENT_LIMIT_A, USB_ROUTE_SCALE_ONE,
		USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_FIXED, 2, false, "i_lim=", "A\r\n",
		1.25f, 1.25f, "i_lim=1.25A\r\n" },
	{ USB_SPEED_LIMIT, MOTOR_PARAMETER_SPEED_LIMIT_RAD_S, USB_ROUTE_SCALE_TWO_PI,
		USB_ROUTE_SCALE_ONE_BY_2PI, USB_ROUTE_FORMAT_FIXED, 2, false,
		"spd_lim=", "r/s\r\n", 1.0f, TEST_TWO_PI, "spd_lim=1.00r/s\r\n" },
	{ USB_SPEED_ACC, MOTOR_PARAMETER_SPEED_ACCELERATION_RAD_S2,
		USB_ROUTE_SCALE_TWO_PI, USB_ROUTE_SCALE_ONE_BY_2PI, USB_ROUTE_FORMAT_FIXED,
		2, false, "spd_acc=", "r/s2\r\n", 1.0f, TEST_TWO_PI,
		"spd_acc=1.00r/s2\r\n" },
	{ USB_SPEED_DEC, MOTOR_PARAMETER_SPEED_DECELERATION_RAD_S2,
		USB_ROUTE_SCALE_TWO_PI, USB_ROUTE_SCALE_ONE_BY_2PI, USB_ROUTE_FORMAT_FIXED,
		2, false, "spd_dec=", "r/s2\r\n", 1.0f, TEST_TWO_PI,
		"spd_dec=1.00r/s2\r\n" },
	{ USB_SPEED_KP, MOTOR_PARAMETER_SPEED_KP, USB_ROUTE_SCALE_ONE,
		USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_FIXED, 2, false, "spd_kp=", "\r\n",
		1.25f, 1.25f, "spd_kp=1.25\r\n" },
	{ USB_SPEED_KI, MOTOR_PARAMETER_SPEED_KI, USB_ROUTE_SCALE_ONE,
		USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_FIXED, 2, false, "spd_ki=", "\r\n",
		1.25f, 1.25f, "spd_ki=1.25\r\n" },
	{ USB_POS_ACC, MOTOR_PARAMETER_POSITION_ACCELERATION_RAD_S2,
		USB_ROUTE_SCALE_TWO_PI, USB_ROUTE_SCALE_ONE_BY_2PI, USB_ROUTE_FORMAT_FIXED,
		3, false, "pos_acc=", "r/s2\r\n", 1.0f, TEST_TWO_PI,
		"pos_acc=1.000r/s2\r\n" },
	{ USB_POS_DEC, MOTOR_PARAMETER_POSITION_DECELERATION_RAD_S2,
		USB_ROUTE_SCALE_TWO_PI, USB_ROUTE_SCALE_ONE_BY_2PI, USB_ROUTE_FORMAT_FIXED,
		3, false, "pos_dec=", "r/s2\r\n", 1.0f, TEST_TWO_PI,
		"pos_dec=1.000r/s2\r\n" },
	{ USB_POS_MAXSPEED, MOTOR_PARAMETER_POSITION_MAX_SPEED_RAD_S,
		USB_ROUTE_SCALE_TWO_PI, USB_ROUTE_SCALE_ONE_BY_2PI, USB_ROUTE_FORMAT_FIXED,
		3, false, "pos_maxspd=", "r/s\r\n", 1.0f, TEST_TWO_PI,
		"pos_maxspd=1.000r/s\r\n" },
	{ USB_POS_KP, MOTOR_PARAMETER_POSITION_KP_A_PER_RAD, USB_ROUTE_SCALE_ONE,
		USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_FIXED, 2, false, "pos_kp=", "\r\n",
		1.25f, 1.25f, "pos_kp=1.25\r\n" },
	{ USB_POS_KD, MOTOR_PARAMETER_POSITION_KD_A_PER_RAD_S, USB_ROUTE_SCALE_ONE,
		USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_FIXED, 2, false, "pos_kd=", "\r\n",
		1.25f, 1.25f, "pos_kd=1.25\r\n" },
	{ USB_POS_KI, MOTOR_PARAMETER_POSITION_KI_A_PER_RAD_S, USB_ROUTE_SCALE_ONE,
		USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_FIXED, 3, false, "pos_ki=", "\r\n",
		1.25f, 1.25f, "pos_ki=1.250\r\n" },
	{ USB_POS_INTEGRAL_LIMIT, MOTOR_PARAMETER_POSITION_INTEGRAL_LIMIT_A,
		USB_ROUTE_SCALE_ONE, USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_FIXED, 3, false,
		"pos_i_limit=", "A\r\n", 1.25f, 1.25f, "pos_i_limit=1.250A\r\n" },
	{ USB_CASCADE_POS_KP, MOTOR_PARAMETER_CASCADE_POSITION_KP_PER_S,
		USB_ROUTE_SCALE_ONE, USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_FIXED, 3, false,
		"cascade_pos_kp=", "\r\n", 1.25f, 1.25f,
		"cascade_pos_kp=1.250\r\n" },
	{ USB_CASCADE_POS_KD, MOTOR_PARAMETER_CASCADE_POSITION_KD,
		USB_ROUTE_SCALE_ONE, USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_FIXED, 3, false,
		"cascade_pos_kd=", "\r\n", 1.25f, 1.25f,
		"cascade_pos_kd=1.250\r\n" },
	{ USB_RS, MOTOR_PARAMETER_PHASE_RESISTANCE_OHM, USB_ROUTE_SCALE_ONE,
		USB_ROUTE_SCALE_MILLI, USB_ROUTE_FORMAT_FIXED, 2, false, "Rs=", "mohm\r\n",
		1.25f, 0.00125f, "Rs=1.25mohm\r\n" },
	{ USB_LD, MOTOR_PARAMETER_D_AXIS_INDUCTANCE_H, USB_ROUTE_SCALE_ONE,
		USB_ROUTE_SCALE_MICRO, USB_ROUTE_FORMAT_FIXED, 2, false, "Ld=", "uH\r\n",
		1.25f, 0.00000125f, "Ld=1.25uH\r\n" },
	{ USB_LQ, MOTOR_PARAMETER_Q_AXIS_INDUCTANCE_H, USB_ROUTE_SCALE_ONE,
		USB_ROUTE_SCALE_MICRO, USB_ROUTE_FORMAT_FIXED, 2, false, "Lq=", "uH\r\n",
		1.25f, 0.00000125f, "Lq=1.25uH\r\n" },
	{ USB_FLUX, MOTOR_PARAMETER_FLUX_WEBER, USB_ROUTE_SCALE_ONE,
		USB_ROUTE_SCALE_MILLI, USB_ROUTE_FORMAT_FIXED, 2, false, "Flux=", "mWb\r\n",
		1.25f, 0.00125f, "Flux=1.25mWb\r\n" }
};

static const ExpectedTelemetryRoute ExpectedTelemetryRoutes[] =
{
	{ USB_MODE, MOTOR_TELEMETRY_MODE, USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_I32,
		0, "mode=", "\r\n", 7.0f, "mode=7\r\n" },
	{ USB_CURRENT_SET, MOTOR_TELEMETRY_CURRENT_REFERENCE_A, USB_ROUTE_SCALE_ONE,
		USB_ROUTE_FORMAT_FIXED, 2, "i_set=", "A\r\n", 1.25f,
		"i_set=1.25A\r\n" },
	{ USB_SPEED_SET, MOTOR_TELEMETRY_SPEED_REFERENCE_RAD_S,
		USB_ROUTE_SCALE_ONE_BY_2PI, USB_ROUTE_FORMAT_FIXED, 2, "spd_set=",
		"r/s\r\n", TEST_TWO_PI, "spd_set=1.00r/s\r\n" },
	{ USB_POS_SET, MOTOR_TELEMETRY_POSITION_REFERENCE_RAD,
		USB_ROUTE_SCALE_ONE_BY_2PI, USB_ROUTE_FORMAT_FIXED, 2, "pos_set=",
		"r\r\n", TEST_TWO_PI, "pos_set=1.00r\r\n" },
	{ USB_ENCODER_REVERSE, MOTOR_TELEMETRY_ENCODER_REVERSED, USB_ROUTE_SCALE_ONE,
		USB_ROUTE_FORMAT_U32, 0, "erv=", "\r\n", 1.0f, "erv=1\r\n" },
	{ USB_VBUS, MOTOR_TELEMETRY_BUS_VOLTAGE_V, USB_ROUTE_SCALE_ONE,
		USB_ROUTE_FORMAT_FIXED, 2, "vbus=", "V\r\n", 1.25f,
		"vbus=1.25V\r\n" },
	{ USB_IBUS, MOTOR_TELEMETRY_BUS_CURRENT_A, USB_ROUTE_SCALE_ONE,
		USB_ROUTE_FORMAT_FIXED, 2, "ibus=", "A\r\n", 1.25f,
		"ibus=1.25A\r\n" },
	{ USB_IA, MOTOR_TELEMETRY_PHASE_A_CURRENT_A, USB_ROUTE_SCALE_ONE,
		USB_ROUTE_FORMAT_FIXED, 2, "ia=", "A\r\n", 1.25f, "ia=1.25A\r\n" },
	{ USB_IB, MOTOR_TELEMETRY_PHASE_B_CURRENT_A, USB_ROUTE_SCALE_ONE,
		USB_ROUTE_FORMAT_FIXED, 2, "ib=", "A\r\n", 1.25f, "ib=1.25A\r\n" },
	{ USB_IC, MOTOR_TELEMETRY_PHASE_C_CURRENT_A, USB_ROUTE_SCALE_ONE,
		USB_ROUTE_FORMAT_FIXED, 2, "ic=", "A\r\n", 1.25f, "ic=1.25A\r\n" },
	{ USB_ID, MOTOR_TELEMETRY_D_AXIS_CURRENT_A, USB_ROUTE_SCALE_ONE,
		USB_ROUTE_FORMAT_FIXED, 2, "id=", "A\r\n", 1.25f, "id=1.25A\r\n" },
	{ USB_IQ, MOTOR_TELEMETRY_Q_AXIS_CURRENT_A, USB_ROUTE_SCALE_ONE,
		USB_ROUTE_FORMAT_FIXED, 2, "iq=", "A\r\n", 1.25f, "iq=1.25A\r\n" },
	{ USB_SPEED2_FILT, MOTOR_TELEMETRY_MECHANICAL_SPEED_RAD_S,
		USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_FIXED, 2, "spd2_filt=", "r/s\r\n",
		1.25f, "spd2_filt=1.25r/s\r\n" },
	{ USB_POS2_FILT, MOTOR_TELEMETRY_MECHANICAL_POSITION_RAD,
		USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_FIXED, 3, "pos2_filt=", "r\r\n",
		1.25f, "pos2_filt=1.250r\r\n" },
	{ USB_TEMP, MOTOR_TELEMETRY_TEMPERATURE_C, USB_ROUTE_SCALE_ONE,
		USB_ROUTE_FORMAT_FIXED, 2, "temp=", "C\r\n", 1.25f,
		"temp=1.25C\r\n" },
	{ USB_ERROR, MOTOR_TELEMETRY_PRIMARY_ERROR, USB_ROUTE_SCALE_ONE,
		USB_ROUTE_FORMAT_I32, 0, "error=", "\r\n", 7.0f, "error=7\r\n" },
	{ USB_COMMISSIONING_STAGE, MOTOR_TELEMETRY_COMMISSIONING_STAGE,
		USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_U32, 0, "cst=", "\r\n", 7.0f,
		"cst=7\r\n" },
	{ USB_COMMISSIONING_PROGRESS, MOTOR_TELEMETRY_COMMISSIONING_PROGRESS_PERCENT,
		USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_U32, 0, "cpr=", "%\r\n", 7.0f,
		"cpr=7%\r\n" },
	{ USB_RESISTANCE_SPREAD, MOTOR_TELEMETRY_PHASE_RESISTANCE_SPREAD_PERCENT,
		USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_FIXED, 2, "rsp=", "%\r\n", 1.25f,
		"rsp=1.25%\r\n" },
	{ USB_RESISTANCE_DESIGN_ERROR,
		MOTOR_TELEMETRY_PHASE_RESISTANCE_DESIGN_ERROR_PERCENT,
		USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_FIXED, 2, "rde=", "%\r\n", 1.25f,
		"rde=1.25%\r\n" }
};

static bool FakeCanStage(void *context)
{
	return ((FakeMotorConfiguration *)context)->allow_can_stage;
}

static bool FakeRead(void *context, MotorParameterId parameter, float *value)
{
	FakeMotorConfiguration *fake = (FakeMotorConfiguration *)context;
	if (fake == NULL || value == NULL || parameter > MOTOR_PARAMETER_FLUX_WEBER)
		return false;
	*value = fake->values[parameter];
	return true;
}

static bool FakeStage(void *context, MotorParameterId parameter, float value)
{
	FakeMotorConfiguration *fake = (FakeMotorConfiguration *)context;
	if (fake == NULL || parameter > MOTOR_PARAMETER_FLUX_WEBER || !fake->allow_stage)
		return false;
	fake->values[parameter] = value;
	fake->staged_parameter = parameter;
	fake->staged_value = value;
	++fake->stage_count;
	return true;
}

static MotorPortMode FakeGetMode(void *context)
{
	return ((FakeMotorCommand *)context)->mode;
}

static bool FakeRequestMode(void *context, MotorPortMode mode)
{
	((FakeMotorCommand *)context)->mode = mode;
	return true;
}

static bool FakeRequestService(void *context, MotorPortService service)
{
	(void)context;
	(void)service;
	return true;
}

static bool FakeRequestStandby(void *context)
{
	((FakeMotorCommand *)context)->mode = MOTOR_PORT_MODE_NONE;
	return true;
}

static bool FakeRequestClearFaults(void *context)
{
	(void)context;
	return true;
}

static float FakeGetCurrentLimit(void *context)
{
	(void)context;
	return 100.0f;
}

static float FakeGetSpeedLimit(void *context)
{
	(void)context;
	return 100.0f;
}

static void FakeSetCurrent(void *context, float value)
{
	((FakeMotorCommand *)context)->current_reference_a = value;
}

static void FakeSetSpeed(void *context, float value)
{
	((FakeMotorCommand *)context)->speed_reference_rad_s = value;
}

static void FakeSetPosition(void *context, float value)
{
	((FakeMotorCommand *)context)->position_reference_rad = value;
}

static bool FakeSetReverse(void *context, bool reverse)
{
	((FakeRotorCalibration *)context)->reverse = reverse;
	return true;
}

static bool FakeReadEntry(void *context, uint16_t index,
	RotorCalibrationEntry *entry)
{
	(void)context;
	(void)index;
	(void)entry;
	return true;
}

static bool FakeCanSetNodeId(void *context, uint8_t node_id)
{
	((FakeCanConfiguration *)context)->node_id = node_id;
	return true;
}

static uint8_t FakeCanGetNodeId(void *context)
{
	return ((FakeCanConfiguration *)context)->node_id;
}

static bool FakeCanSetBitrate(void *context, uint32_t bitrate_kbps)
{
	if (bitrate_kbps == 0U)
		return false;
	((FakeCanConfiguration *)context)->bitrate_kbps = bitrate_kbps;
	return true;
}

static uint32_t FakeCanGetBitrate(void *context)
{
	return ((FakeCanConfiguration *)context)->bitrate_kbps;
}

static bool FakeCanSetHeartbeat(void *context, uint32_t heartbeat_ms)
{
	if (heartbeat_ms == 0U)
		return false;
	((FakeCanConfiguration *)context)->heartbeat_ms = heartbeat_ms;
	return true;
}

static uint32_t FakeCanGetHeartbeat(void *context)
{
	return ((FakeCanConfiguration *)context)->heartbeat_ms;
}

static bool FakeFrictionReadStatus(void *context,
	FrictionIdentificationPortStatus *status)
{
	FakeFrictionIdentification *fake = (FakeFrictionIdentification *)context;
	if (!fake->status_available || status == NULL)
		return false;
	*status = fake->status;
	return true;
}

static bool FakeFrictionReadSample(void *context, uint8_t index,
	FrictionIdentificationPortSample *sample)
{
	(void)context;
	(void)index;
	(void)sample;
	return true;
}

static bool FakeFrictionApply(void *context)
{
	return ((FakeFrictionIdentification *)context)->apply_allowed;
}

static UsbCommandError Handle(UsbCommandRouterContext *router,
	UsbCommandRouterState *state, UsbCommandRouterResponse *response,
	UsbProtocolV1Operation operation, UsbParameterId parameter, float value,
	bool value_is_float)
{
	UsbProtocolV1Command command;
	command.operation = operation;
	command.parameter_id = (uint32_t)parameter;
	command.value = value;
	command.value_is_float = value_is_float;
	return UsbCommandRouter_TestHandle(router, &command, state, response);
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

static int VerifyMotorRoutes(UsbCommandRouterContext *router,
	UsbCommandRouterState *state, UsbCommandRouterResponse *response,
	FakeMotorConfiguration *configuration)
{
	unsigned int index;

	for (index = 0U; index < sizeof(ExpectedMotorRoutes) /
		sizeof(ExpectedMotorRoutes[0]); ++index)
	{
		const ExpectedMotorRoute *expected = &ExpectedMotorRoutes[index];
		const UsbScalarRoute *actual = UsbCommandRouter_FindScalar(
			expected->usb_parameter);
		const unsigned int old_stage_count = configuration->stage_count;
		float expected_write = UsbCommandRouter_ApplyScale(expected->write_value,
			expected->write_scale);
		if (actual == NULL ||
			USB_ROUTE_KIND(actual->details) != USB_ROUTE_KIND_MOTOR ||
			USB_ROUTE_SOURCE(actual->details) != (uint8_t)expected->motor_parameter ||
			USB_ROUTE_WRITE_SCALE(actual->details) != expected->write_scale ||
			USB_ROUTE_READ_SCALE(actual->details) != expected->read_scale ||
			USB_ROUTE_FORMAT(actual->details) != expected->format ||
			USB_ROUTE_INTEGER_WRITE(actual->details) != expected->integer_write ||
			USB_ROUTE_PRECISION(actual->details) != expected->precision ||
			strcmp(&g_route_suffixes[g_route_suffix_offsets[
				USB_ROUTE_SUFFIX(actual->details)]], expected->suffix) != 0)
			return __LINE__;

		if (Handle(router, state, response, USB_PROTOCOL_V1_OPERATION_WRITE,
			expected->usb_parameter, expected->write_value,
			!expected->integer_write) != USB_NO_ERROR ||
			configuration->stage_count != old_stage_count + 1U ||
			configuration->staged_parameter != expected->motor_parameter ||
			!TEST_CLOSE(configuration->staged_value, expected_write) ||
			response->action != USB_COMMAND_ROUTER_ACTION_SEND_TEXT ||
			strcmp(response->text, "Write Success!\r\n") != 0)
			return __LINE__;

		configuration->values[expected->motor_parameter] = expected->core_read_value;
		if (Handle(router, state, response, USB_PROTOCOL_V1_OPERATION_READ,
			expected->usb_parameter, 0.0f, false) != USB_NO_ERROR ||
			response->action != USB_COMMAND_ROUTER_ACTION_SEND_TEXT ||
			strcmp(response->text, expected->expected_text) != 0)
			return __LINE__;

		if (Handle(router, state, response, USB_PROTOCOL_V1_OPERATION_PRINT,
			expected->usb_parameter, 3.0f, false) != USB_NO_ERROR ||
			response->action != USB_COMMAND_ROUTER_ACTION_CONFIGURE_PRINT ||
			response->print_channel != 3U ||
			response->print_source != (UsbCommandRouterPrintSource)
				(MOTOR_TELEMETRY_POLE_PAIRS + expected->motor_parameter) ||
			!TEST_CLOSE(response->print_scale,
				UsbCommandRouter_ApplyScale(1.0f, expected->read_scale)))
			return __LINE__;
	}
	return 0;
}

static int VerifyTelemetryRoutes(UsbCommandRouterContext *router,
	UsbCommandRouterState *state, UsbCommandRouterResponse *response,
	TelemetryServiceContext *telemetry)
{
	unsigned int index;

	for (index = 0U; index < sizeof(ExpectedTelemetryRoutes) /
		sizeof(ExpectedTelemetryRoutes[0]); ++index)
	{
		MotorTelemetrySnapshot snapshot;
		const ExpectedTelemetryRoute *expected = &ExpectedTelemetryRoutes[index];
		const UsbScalarRoute *actual = UsbCommandRouter_FindScalar(
			expected->usb_parameter);
		UsbCommandRouterPrintSource expected_source =
			(UsbCommandRouterPrintSource)expected->telemetry;
		if (expected->usb_parameter == USB_ID)
			expected_source = MOTOR_TELEMETRY_D_AXIS_CURRENT_FILTERED_A;
		else if (expected->usb_parameter == USB_IQ)
			expected_source = MOTOR_TELEMETRY_Q_AXIS_CURRENT_FILTERED_A;
		if (expected->format != USB_ROUTE_FORMAT_FIXED)
			expected_source |= USB_COMMAND_ROUTER_PRINT_INTEGER_FLAG;
		if (actual == NULL ||
			USB_ROUTE_KIND(actual->details) != USB_ROUTE_KIND_TELEMETRY ||
			USB_ROUTE_SOURCE(actual->details) != (uint8_t)expected->telemetry ||
			USB_ROUTE_READ_SCALE(actual->details) != expected->scale ||
			USB_ROUTE_FORMAT(actual->details) != expected->format ||
			USB_ROUTE_PRECISION(actual->details) != expected->precision ||
			strcmp(&g_route_suffixes[g_route_suffix_offsets[
				USB_ROUTE_SUFFIX(actual->details)]], expected->suffix) != 0)
			return __LINE__;

		(void)memset(&snapshot, 0, sizeof(snapshot));
		SetTelemetryValue(&snapshot, expected->telemetry, expected->core_value);
		TelemetryService_Publish(telemetry, &snapshot);
		if (Handle(router, state, response, USB_PROTOCOL_V1_OPERATION_READ,
			expected->usb_parameter, 0.0f, false) != USB_NO_ERROR ||
			response->action != USB_COMMAND_ROUTER_ACTION_SEND_TEXT ||
			strcmp(response->text, expected->expected_text) != 0)
			return __LINE__;
		if (expected->usb_parameter != USB_MODE &&
			expected->usb_parameter != USB_CURRENT_SET &&
			expected->usb_parameter != USB_SPEED_SET &&
			expected->usb_parameter != USB_POS_SET &&
			expected->usb_parameter != USB_ENCODER_REVERSE &&
			Handle(router, state, response, USB_PROTOCOL_V1_OPERATION_WRITE,
				expected->usb_parameter, 1.0f, false) != USB_WRITE_INVALID)
			return __LINE__;
		if (Handle(router, state, response, USB_PROTOCOL_V1_OPERATION_PRINT,
			expected->usb_parameter, 4.0f, false) != USB_NO_ERROR ||
			response->print_channel != 4U ||
			response->print_source != expected_source ||
			!TEST_CLOSE(response->print_scale,
				UsbCommandRouter_ApplyScale(1.0f, expected->scale)))
			return __LINE__;
	}
	return 0;
}

static int VerifyConfigurationAndFrictionRoutes(UsbCommandRouterContext *router,
	UsbCommandRouterState *state, UsbCommandRouterResponse *response,
	FakeCanConfiguration *can_configuration,
	FakeFrictionIdentification *friction)
{
	typedef struct
	{
		UsbParameterId parameter;
		const char *text;
	} ExpectedRead;
	static const ExpectedRead friction_reads[] =
	{
		{ USB_FRICTION_COULOMB_POS, "friction_coulomb_pos=1.250000A\r\n" },
		{ USB_FRICTION_COULOMB_NEG, "friction_coulomb_neg=-1.500000A\r\n" },
		{ USB_FRICTION_VISCOUS_POS, "friction_viscous_pos=0.125000A_per_rad_s\r\n" },
		{ USB_FRICTION_VISCOUS_NEG, "friction_viscous_neg=-0.250000A_per_rad_s\r\n" },
		{ USB_FRICTION_RMSE_POS, "friction_rmse_pos=0.010000A\r\n" },
		{ USB_FRICTION_RMSE_NEG, "friction_rmse_neg=0.020000A\r\n" },
		{ USB_FRICTION_VALID, "friction_model_valid=1\r\n" },
		{ USB_ACTIVE_COULOMB_POS, "active_coulomb_pos=1.000000A\r\n" },
		{ USB_ACTIVE_COULOMB_NEG, "active_coulomb_neg=-1.000000A\r\n" },
		{ USB_ACTIVE_VISCOUS_POS, "active_viscous_pos=0.100000A_per_rad_s\r\n" },
		{ USB_ACTIVE_VISCOUS_NEG, "active_viscous_neg=-0.100000A_per_rad_s\r\n" }
	};
	unsigned int index;

	can_configuration->node_id = 3U;
	can_configuration->bitrate_kbps = 1000U;
	can_configuration->heartbeat_ms = 100U;
	if (Handle(router, state, response, USB_PROTOCOL_V1_OPERATION_READ,
		USB_NODE_ID, 0.0f, false) != USB_NO_ERROR ||
		strcmp(response->text, "can_id=3\r\n") != 0 ||
		Handle(router, state, response, USB_PROTOCOL_V1_OPERATION_READ,
			USB_CAN_BR, 0.0f, false) != USB_NO_ERROR ||
		strcmp(response->text, "can_br=1000kbps\r\n") != 0 ||
		Handle(router, state, response, USB_PROTOCOL_V1_OPERATION_READ,
			USB_CAN_HB, 0.0f, false) != USB_NO_ERROR ||
		strcmp(response->text, "can_hb=100ms\r\n") != 0)
		return __LINE__;
	if (Handle(router, state, response, USB_PROTOCOL_V1_OPERATION_WRITE,
		USB_NODE_ID, 7.0f, false) != USB_NO_ERROR ||
		can_configuration->node_id != 7U ||
		Handle(router, state, response, USB_PROTOCOL_V1_OPERATION_WRITE,
			USB_CAN_BR, 500.0f, false) != USB_NO_ERROR ||
		can_configuration->bitrate_kbps != 500U ||
		Handle(router, state, response, USB_PROTOCOL_V1_OPERATION_WRITE,
			USB_CAN_HB, 250.0f, false) != USB_NO_ERROR ||
		can_configuration->heartbeat_ms != 250U)
		return __LINE__;
	if (Handle(router, state, response, USB_PROTOCOL_V1_OPERATION_PRINT,
		USB_NODE_ID, 0.0f, false) != USB_NO_ERROR ||
		response->print_source != USB_COMMAND_ROUTER_PRINT_CAN_NODE_ID ||
		Handle(router, state, response, USB_PROTOCOL_V1_OPERATION_PRINT,
			USB_CAN_BR, 1.0f, false) != USB_NO_ERROR ||
		response->print_source != USB_COMMAND_ROUTER_PRINT_CAN_BITRATE ||
		Handle(router, state, response, USB_PROTOCOL_V1_OPERATION_PRINT,
			USB_CAN_HB, 2.0f, false) != USB_NO_ERROR ||
		response->print_source != USB_COMMAND_ROUTER_PRINT_CAN_HEARTBEAT)
		return __LINE__;

	friction->status.state = 2U;
	friction->status.reason = 3U;
	friction->status.point_index = 4U;
	friction->status.sample_count = 5U;
	friction->status.progress_percent = 50.0f;
	friction->status.candidate_coulomb_pos_a = 1.25f;
	friction->status.candidate_coulomb_neg_a = -1.5f;
	friction->status.candidate_viscous_pos_a_per_rad_s = 0.125f;
	friction->status.candidate_viscous_neg_a_per_rad_s = -0.25f;
	friction->status.candidate_rmse_pos_a = 0.01f;
	friction->status.candidate_rmse_neg_a = 0.02f;
	friction->status.candidate_valid = true;
	friction->status.active_coulomb_pos_a = 1.0f;
	friction->status.active_coulomb_neg_a = -1.0f;
	friction->status.active_viscous_pos_a_per_rad_s = 0.1f;
	friction->status.active_viscous_neg_a_per_rad_s = -0.1f;
	friction->status.active_model_valid = true;
	friction->status_available = true;
	friction->apply_allowed = true;
	for (index = 0U; index < sizeof(friction_reads) /
		sizeof(friction_reads[0]); ++index)
	{
		if (Handle(router, state, response, USB_PROTOCOL_V1_OPERATION_READ,
			friction_reads[index].parameter, 0.0f, false) != USB_NO_ERROR ||
			strcmp(response->text, friction_reads[index].text) != 0 ||
			Handle(router, state, response, USB_PROTOCOL_V1_OPERATION_WRITE,
				friction_reads[index].parameter, 1.0f, false) !=
					USB_WRITE_INVALID ||
			Handle(router, state, response, USB_PROTOCOL_V1_OPERATION_PRINT,
				friction_reads[index].parameter, 0.0f, false) !=
					USB_UNKNOWNED_PARAM)
			return __LINE__;
	}
	if (Handle(router, state, response, USB_PROTOCOL_V1_OPERATION_READ,
		USB_FRICTION_STATUS, 0.0f, false) != USB_NO_ERROR ||
		strcmp(response->text,
			"friction_state=2,reason=3,point=4,progress=50.0,candidate=1\r\n") != 0 ||
		Handle(router, state, response, USB_PROTOCOL_V1_OPERATION_READ,
			USB_FRICTION_DATA_EXPORT, 0.0f, false) != USB_NO_ERROR ||
		response->action != USB_COMMAND_ROUTER_ACTION_BEGIN_FRICTION_EXPORT ||
		strcmp(response->text, "friction_begin,count=5\r\n") != 0 ||
		Handle(router, state, response, USB_PROTOCOL_V1_OPERATION_WRITE,
			USB_FRICTION_APPLY, 1.0f, false) != USB_NO_ERROR)
		return __LINE__;

	friction->status_available = false;
	if (Handle(router, state, response, USB_PROTOCOL_V1_OPERATION_READ,
		USB_FRICTION_COULOMB_POS, 0.0f, false) != USB_WRITE_INVALID)
		return __LINE__;
	return 0;
}

int UsbCommandRouter_RunHostTests(void)
{
	FakeMotorConfiguration fake_configuration;
	FakeMotorCommand fake_motor_command;
	FakeRotorCalibration fake_rotor;
	FakeCanConfiguration fake_can_configuration;
	FakeFrictionIdentification fake_friction;
	MotorConfigurationPort configuration_port;
	MotorCommandPort motor_command_port;
	RotorCalibrationPort rotor_port;
	CanConfigurationPort can_configuration_port;
	FrictionIdentificationPort friction_port;
	ParameterServiceLimits limits;
	ParameterServiceContext parameters;
	TelemetryServiceContext telemetry;
	FrictionIdentificationServiceContext friction;
	CanConfigurationServiceContext can_configuration;
	MotorCommandServiceContext motor_command;
	RotorCalibrationServiceContext rotor_calibration;
	ControlAuthorityServiceContext control_authority;
	ApplicationEndpoints endpoints;
	UsbCommandRouterContext router;
	UsbCommandRouterState state;
	UsbCommandRouterResponse response;
	int result;

	(void)memset(&fake_configuration, 0, sizeof(fake_configuration));
	(void)memset(&fake_motor_command, 0, sizeof(fake_motor_command));
	(void)memset(&fake_rotor, 0, sizeof(fake_rotor));
	(void)memset(&fake_can_configuration, 0, sizeof(fake_can_configuration));
	(void)memset(&fake_friction, 0, sizeof(fake_friction));
	(void)memset(&limits, 0, sizeof(limits));
	(void)memset(&endpoints, 0, sizeof(endpoints));
	(void)memset(&state, 0, sizeof(state));
	fake_configuration.allow_can_stage = true;
	fake_configuration.allow_stage = true;
	fake_configuration.values[MOTOR_PARAMETER_SPEED_LIMIT_RAD_S] = 1000.0f;
	configuration_port.context = &fake_configuration;
	configuration_port.can_stage = FakeCanStage;
	configuration_port.read = FakeRead;
	configuration_port.stage = FakeStage;
	motor_command_port.context = &fake_motor_command;
	motor_command_port.get_mode = FakeGetMode;
	motor_command_port.request_mode = FakeRequestMode;
	motor_command_port.request_service = FakeRequestService;
	motor_command_port.request_standby = FakeRequestStandby;
	motor_command_port.request_clear_faults = FakeRequestClearFaults;
	motor_command_port.get_current_limit_a = FakeGetCurrentLimit;
	motor_command_port.get_speed_limit_rad_s = FakeGetSpeedLimit;
	motor_command_port.set_current_reference_a = FakeSetCurrent;
	motor_command_port.set_speed_reference_rad_s = FakeSetSpeed;
	motor_command_port.set_position_reference_rad = FakeSetPosition;
	rotor_port.context = &fake_rotor;
	rotor_port.entry_count = 1U;
	rotor_port.counts_per_revolution = 32768U;
	rotor_port.set_reverse = FakeSetReverse;
	rotor_port.read_entry = FakeReadEntry;
	can_configuration_port.context = &fake_can_configuration;
	can_configuration_port.set_node_id = FakeCanSetNodeId;
	can_configuration_port.get_node_id = FakeCanGetNodeId;
	can_configuration_port.set_bitrate_kbps = FakeCanSetBitrate;
	can_configuration_port.get_bitrate_kbps = FakeCanGetBitrate;
	can_configuration_port.set_heartbeat_ms = FakeCanSetHeartbeat;
	can_configuration_port.get_heartbeat_ms = FakeCanGetHeartbeat;
	friction_port.context = &fake_friction;
	friction_port.read_status = FakeFrictionReadStatus;
	friction_port.read_sample = FakeFrictionReadSample;
	friction_port.apply_candidate = FakeFrictionApply;
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
	limits.phase_resistance_max_ohm = 1000.0f;
	limits.inductance_max_h = 1000.0f;
	limits.flux_max_weber = 1000.0f;

	if (!ParameterService_Initialize(&parameters, &configuration_port, &limits) ||
		!TelemetryService_Initialize(&telemetry) ||
		!MotorCommandService_Initialize(&motor_command, &motor_command_port) ||
		!RotorCalibrationService_Initialize(&rotor_calibration, &rotor_port) ||
		!CanConfigurationService_Initialize(&can_configuration,
			&can_configuration_port) ||
		!FrictionIdentificationService_Initialize(&friction, &friction_port) ||
		!ControlAuthorityService_Initialize(&control_authority))
		return __LINE__;
	endpoints.can_configuration = &can_configuration;
	endpoints.parameters = &parameters;
	endpoints.telemetry = &telemetry;
	endpoints.friction_identification = &friction;
	endpoints.motor_command = &motor_command;
	endpoints.rotor_calibration = &rotor_calibration;
	endpoints.control_authority = &control_authority;
	if (!UsbCommandRouter_TestInitialize(&router, &endpoints))
		return __LINE__;

	result = VerifyMotorRoutes(&router, &state, &response, &fake_configuration);
	if (result != 0)
		return result;
	result = VerifyTelemetryRoutes(&router, &state, &response, &telemetry);
	if (result != 0)
		return result;
	result = VerifyConfigurationAndFrictionRoutes(&router, &state, &response,
		&fake_can_configuration, &fake_friction);
	if (result != 0)
		return result;

	/* Telemetry-backed reads that also have explicit write behavior stay writable. */
	if (Handle(&router, &state, &response, USB_PROTOCOL_V1_OPERATION_WRITE,
		USB_CURRENT_SET, 0.5f, true) != USB_NO_ERROR ||
		!TEST_CLOSE(fake_motor_command.current_reference_a, 0.5f) ||
		fake_motor_command.mode != MOTOR_PORT_MODE_CURRENT ||
		!ControlAuthorityService_IsOwner(&control_authority, CONTROL_AUTHORITY_USB))
		return __LINE__;
	if (Handle(&router, &state, &response, USB_PROTOCOL_V1_OPERATION_WRITE,
		USB_SPEED_SET, 1.0f, true) != USB_NO_ERROR ||
		!TEST_CLOSE(fake_motor_command.speed_reference_rad_s, TEST_TWO_PI))
		return __LINE__;
	if (Handle(&router, &state, &response, USB_PROTOCOL_V1_OPERATION_WRITE,
		USB_POS_SET, 1.0f, true) != USB_NO_ERROR ||
		!TEST_CLOSE(fake_motor_command.position_reference_rad, TEST_TWO_PI))
		return __LINE__;
	if (Handle(&router, &state, &response, USB_PROTOCOL_V1_OPERATION_WRITE,
		USB_MODE, 0.0f, false) != USB_NO_ERROR ||
		fake_motor_command.mode != MOTOR_PORT_MODE_NONE ||
		!ControlAuthorityService_IsOwner(&control_authority, CONTROL_AUTHORITY_NONE))
		return __LINE__;
	if (Handle(&router, &state, &response, USB_PROTOCOL_V1_OPERATION_WRITE,
		USB_ENCODER_REVERSE, 1.0f, false) != USB_NO_ERROR || !fake_rotor.reverse)
		return __LINE__;

	if (Handle(&router, &state, &response, USB_PROTOCOL_V1_OPERATION_WRITE,
		USB_POLEPARIS, 7.0f, true) != USB_DATA_INVALID)
		return __LINE__;
	if (Handle(&router, &state, &response, USB_PROTOCOL_V1_OPERATION_WRITE,
		USB_CURRENT_LIMIT, 1001.0f, true) != USB_DATA_OUT_OF_RANGE)
		return __LINE__;
	fake_configuration.allow_can_stage = false;
	if (Handle(&router, &state, &response, USB_PROTOCOL_V1_OPERATION_WRITE,
		USB_CURRENT_LIMIT, 1.0f, true) != USB_WRITE_INVALID)
		return __LINE__;
	fake_configuration.allow_can_stage = true;
	fake_configuration.allow_stage = false;
	if (Handle(&router, &state, &response, USB_PROTOCOL_V1_OPERATION_WRITE,
		USB_CURRENT_LIMIT, 1.0f, true) != USB_WRITE_INVALID)
		return __LINE__;
	fake_configuration.allow_stage = true;

	if (Handle(&router, &state, &response, USB_PROTOCOL_V1_OPERATION_PRINT,
		(UsbParameterId)0x787878U, 0.0f, false) != USB_NO_ERROR ||
		!TEST_CLOSE(response.print_scale, 1.0f) ||
		response.print_source != USB_COMMAND_ROUTER_PRINT_ZERO)
		return __LINE__;
	if (Handle(&router, &state, &response, USB_PROTOCOL_V1_OPERATION_PRINT,
		USB_FRICTION_STATUS, 0.0f, false) != USB_UNKNOWNED_PARAM)
		return __LINE__;
	if (Handle(&router, &state, &response, USB_PROTOCOL_V1_OPERATION_READ,
		(UsbParameterId)0x787878U, 0.0f, false) != USB_UNKNOWNED_PARAM)
		return __LINE__;
	if (Handle(&router, &state, &response, USB_PROTOCOL_V1_OPERATION_WRITE,
		(UsbParameterId)0x787878U, 0.0f, false) != USB_UNKNOWNED_PARAM)
		return __LINE__;

	return 0;
}
