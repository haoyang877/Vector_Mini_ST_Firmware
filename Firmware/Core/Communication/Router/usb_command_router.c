#include "Core/Communication/Router/usb_command_router.h"

#include "Core/Application/Communication/can_configuration_service.h"
#include "Core/Application/motor_command_service.h"
#include "Core/Application/Parameters/parameter_service.h"
#include "Core/Application/rotor_calibration_service.h"
#include "Core/Infrastructure/Telemetry/telemetry_service.h"
#include "Core/Application/friction_identification_service.h"
#include "Core/Communication/Formatting/text_writer.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#if defined(__CC_ARM)
#pragma O3
#pragma Ospace
#endif

#define USB_COMMAND_ROUTER_TWO_PI                6.2831853072f
#define USB_COMMAND_ROUTER_ONE_BY_2PI            0.15915494309f
#define USB_COMMAND_ROUTER_DISABLED_MODE         0U
#define USB_COMMAND_ROUTER_MAX_MODE_VALUE        255

typedef enum
{
	USB_ROUTE_SCALE_ONE = 0U,
	USB_ROUTE_SCALE_TWO_PI,
	USB_ROUTE_SCALE_ONE_BY_2PI,
	USB_ROUTE_SCALE_MILLI,
	USB_ROUTE_SCALE_MICRO
} UsbRouteScale;

typedef enum
{
	USB_ROUTE_FORMAT_FIXED = 0U,
	USB_ROUTE_FORMAT_U32,
	USB_ROUTE_FORMAT_I32
} UsbRouteFormat;

typedef enum
{
	USB_ROUTE_KIND_MOTOR = 0U,
	USB_ROUTE_KIND_TELEMETRY,
	USB_ROUTE_KIND_CAN_CONFIGURATION,
	USB_ROUTE_KIND_FRICTION
} UsbRouteKind;

typedef enum
{
	USB_ROUTE_SUFFIX_CRLF = 0U,
	USB_ROUTE_SUFFIX_A,
	USB_ROUTE_SUFFIX_RPS,
	USB_ROUTE_SUFFIX_RPS2,
	USB_ROUTE_SUFFIX_REV,
	USB_ROUTE_SUFFIX_MOHM,
	USB_ROUTE_SUFFIX_UH,
	USB_ROUTE_SUFFIX_MWB,
	USB_ROUTE_SUFFIX_V,
	USB_ROUTE_SUFFIX_C,
	USB_ROUTE_SUFFIX_PERCENT,
	USB_ROUTE_SUFFIX_A_PER_RAD_S,
	USB_ROUTE_SUFFIX_KBPS,
	USB_ROUTE_SUFFIX_MS
} UsbRouteSuffix;

typedef struct
{
	uint32_t usb_parameter;
	uint32_t details;
} UsbScalarRoute;

#define USB_ROUTE_SOURCE(details_)          ((uint8_t)((details_) & 0x3FU))
#define USB_ROUTE_KIND(details_)            ((uint8_t)(((details_) >> 6U) & 0x03U))
#define USB_ROUTE_READ_SCALE(details_)      ((uint8_t)(((details_) >> 8U) & 0x07U))
#define USB_ROUTE_WRITE_SCALE(details_)     \
	((((details_) >> 11U) & 0x01U) != 0U ? USB_ROUTE_SCALE_TWO_PI : \
		USB_ROUTE_SCALE_ONE)
#define USB_ROUTE_FORMAT(details_)          ((uint8_t)(((details_) >> 12U) & 0x03U))
#define USB_ROUTE_INTEGER_WRITE(details_)   (((details_) & (1UL << 14U)) != 0U)
#define USB_ROUTE_PRECISION(details_)       ((uint8_t)(((details_) >> 15U) & 0x07U))
#define USB_ROUTE_PREFIX_OFFSET(details_)   ((uint16_t)(((details_) >> 18U) & 0x03FFU))
#define USB_ROUTE_SUFFIX(details_)          ((uint8_t)(((details_) >> 28U) & 0x0FU))

#define USB_SCALAR_ROUTE(usb_, kind_, source_, write_scale_, read_scale_, \
	format_, integer_write_, precision_, prefix_, suffix_) \
	{ (uint32_t)(usb_), \
		((uint32_t)(source_) & 0x3FU) | (((uint32_t)(kind_) & 0x03U) << 6U) | \
		(((uint32_t)(read_scale_) & 0x07U) << 8U) | \
		(((write_scale_) == USB_ROUTE_SCALE_TWO_PI ? 1UL : 0UL) << 11U) | \
		(((uint32_t)(format_) & 0x03U) << 12U) | \
		(((integer_write_) != 0 ? 1UL : 0UL) << 14U) | \
		(((uint32_t)(precision_) & 0x07U) << 15U) | \
		(((uint32_t)(prefix_) & 0x03FFU) << 18U) | \
		(((uint32_t)(suffix_) & 0x0FU) << 28U) }

enum
{
	USB_PREFIX_POL = 0U,
	USB_PREFIX_I_CAL = USB_PREFIX_POL + sizeof("pol="),
	USB_PREFIX_I_LIM = USB_PREFIX_I_CAL + sizeof("i_cal="),
	USB_PREFIX_SPD_LIM = USB_PREFIX_I_LIM + sizeof("i_lim="),
	USB_PREFIX_SPD_ACC = USB_PREFIX_SPD_LIM + sizeof("spd_lim="),
	USB_PREFIX_SPD_DEC = USB_PREFIX_SPD_ACC + sizeof("spd_acc="),
	USB_PREFIX_SPD_KP = USB_PREFIX_SPD_DEC + sizeof("spd_dec="),
	USB_PREFIX_SPD_KI = USB_PREFIX_SPD_KP + sizeof("spd_kp="),
	USB_PREFIX_POS_ACC = USB_PREFIX_SPD_KI + sizeof("spd_ki="),
	USB_PREFIX_POS_DEC = USB_PREFIX_POS_ACC + sizeof("pos_acc="),
	USB_PREFIX_POS_MAXSPD = USB_PREFIX_POS_DEC + sizeof("pos_dec="),
	USB_PREFIX_POS_KP = USB_PREFIX_POS_MAXSPD + sizeof("pos_maxspd="),
	USB_PREFIX_POS_KD = USB_PREFIX_POS_KP + sizeof("pos_kp="),
	USB_PREFIX_POS_KI = USB_PREFIX_POS_KD + sizeof("pos_kd="),
	USB_PREFIX_POS_I_LIMIT = USB_PREFIX_POS_KI + sizeof("pos_ki="),
	USB_PREFIX_CASCADE_POS_KP = USB_PREFIX_POS_I_LIMIT + sizeof("pos_i_limit="),
	USB_PREFIX_CASCADE_POS_KD = USB_PREFIX_CASCADE_POS_KP + sizeof("cascade_pos_kp="),
	USB_PREFIX_RS = USB_PREFIX_CASCADE_POS_KD + sizeof("cascade_pos_kd="),
	USB_PREFIX_LD = USB_PREFIX_RS + sizeof("Rs="),
	USB_PREFIX_LQ = USB_PREFIX_LD + sizeof("Ld="),
	USB_PREFIX_FLUX = USB_PREFIX_LQ + sizeof("Lq="),
	USB_PREFIX_MODE = USB_PREFIX_FLUX + sizeof("Flux="),
	USB_PREFIX_I_SET = USB_PREFIX_MODE + sizeof("mode="),
	USB_PREFIX_SPD_SET = USB_PREFIX_I_SET + sizeof("i_set="),
	USB_PREFIX_POS_SET = USB_PREFIX_SPD_SET + sizeof("spd_set="),
	USB_PREFIX_ERV = USB_PREFIX_POS_SET + sizeof("pos_set="),
	USB_PREFIX_VBUS = USB_PREFIX_ERV + sizeof("erv="),
	USB_PREFIX_IBUS = USB_PREFIX_VBUS + sizeof("vbus="),
	USB_PREFIX_IA = USB_PREFIX_IBUS + sizeof("ibus="),
	USB_PREFIX_IB = USB_PREFIX_IA + sizeof("ia="),
	USB_PREFIX_IC = USB_PREFIX_IB + sizeof("ib="),
	USB_PREFIX_ID = USB_PREFIX_IC + sizeof("ic="),
	USB_PREFIX_IQ = USB_PREFIX_ID + sizeof("id="),
	USB_PREFIX_SPD2_FILT = USB_PREFIX_IQ + sizeof("iq="),
	USB_PREFIX_POS2_FILT = USB_PREFIX_SPD2_FILT + sizeof("spd2_filt="),
	USB_PREFIX_TEMP = USB_PREFIX_POS2_FILT + sizeof("pos2_filt="),
	USB_PREFIX_ERROR = USB_PREFIX_TEMP + sizeof("temp="),
	USB_PREFIX_CST = USB_PREFIX_ERROR + sizeof("error="),
	USB_PREFIX_CPR = USB_PREFIX_CST + sizeof("cst="),
	USB_PREFIX_RSP = USB_PREFIX_CPR + sizeof("cpr="),
	USB_PREFIX_RDE = USB_PREFIX_RSP + sizeof("rsp="),
	USB_PREFIX_CAN_ID = USB_PREFIX_RDE + sizeof("rde="),
	USB_PREFIX_CAN_BR = USB_PREFIX_CAN_ID + sizeof("can_id="),
	USB_PREFIX_CAN_HB = USB_PREFIX_CAN_BR + sizeof("can_br="),
	USB_PREFIX_FRICTION_COULOMB_POS = USB_PREFIX_CAN_HB + sizeof("can_hb="),
	USB_PREFIX_FRICTION_COULOMB_NEG = USB_PREFIX_FRICTION_COULOMB_POS + sizeof("coulomb_pos="),
	USB_PREFIX_FRICTION_VISCOUS_POS = USB_PREFIX_FRICTION_COULOMB_NEG + sizeof("coulomb_neg="),
	USB_PREFIX_FRICTION_VISCOUS_NEG = USB_PREFIX_FRICTION_VISCOUS_POS + sizeof("viscous_pos="),
	USB_PREFIX_FRICTION_RMSE_POS = USB_PREFIX_FRICTION_VISCOUS_NEG + sizeof("viscous_neg="),
	USB_PREFIX_FRICTION_RMSE_NEG = USB_PREFIX_FRICTION_RMSE_POS + sizeof("rmse_pos="),
	USB_PREFIX_FRICTION_MODEL_VALID = USB_PREFIX_FRICTION_RMSE_NEG + sizeof("rmse_neg="),
	USB_PREFIX_ACTIVE_COULOMB_POS = USB_PREFIX_FRICTION_COULOMB_POS,
	USB_PREFIX_ACTIVE_COULOMB_NEG = USB_PREFIX_FRICTION_COULOMB_NEG,
	USB_PREFIX_ACTIVE_VISCOUS_POS = USB_PREFIX_FRICTION_VISCOUS_POS,
	USB_PREFIX_ACTIVE_VISCOUS_NEG = USB_PREFIX_FRICTION_VISCOUS_NEG
};

/* Descriptor fields are deliberately narrow; fail the build instead of
 * silently truncating a future catalog extension. */
typedef char UsbRouteSourceMustFitSixBits[
	MOTOR_PARAMETER_FLUX_WEBER < 64 &&
	MOTOR_TELEMETRY_PHASE_RESISTANCE_DESIGN_ERROR_PERCENT < 64 &&
	offsetof(FrictionIdentificationPortStatus, active_model_valid) < 64 ? 1 : -1];
typedef char UsbRoutePrefixMustFitTenBits[
	USB_PREFIX_FRICTION_MODEL_VALID < 1024 ? 1 : -1];
typedef char UsbMotorTelemetryProjectionMustRemainAligned[
	MOTOR_TELEMETRY_POLE_PAIRS + MOTOR_PARAMETER_FLUX_WEBER ==
		MOTOR_TELEMETRY_FLUX_WEBER ? 1 : -1];

static const char g_route_prefixes[] =
	"pol=\0i_cal=\0i_lim=\0spd_lim=\0spd_acc=\0spd_dec=\0spd_kp=\0spd_ki=\0"
	"pos_acc=\0pos_dec=\0pos_maxspd=\0pos_kp=\0pos_kd=\0pos_ki=\0"
	"pos_i_limit=\0cascade_pos_kp=\0cascade_pos_kd=\0Rs=\0Ld=\0Lq=\0Flux=\0"
	"mode=\0i_set=\0spd_set=\0pos_set=\0erv=\0vbus=\0ibus=\0ia=\0ib=\0ic=\0"
	"id=\0iq=\0spd2_filt=\0pos2_filt=\0temp=\0error=\0cst=\0cpr=\0rsp=\0rde=\0"
	"can_id=\0can_br=\0can_hb=\0coulomb_pos=\0coulomb_neg=\0viscous_pos=\0"
	"viscous_neg=\0rmse_pos=\0rmse_neg=\0model_valid=";

static const uint8_t g_route_suffix_offsets[] =
{
	0U, 3U, 7U, 13U, 20U, 24U, 31U, 36U, 42U, 46U, 50U, 54U,
	68U, 75U
};

static const char g_route_suffixes[] =
	"\r\n\0A\r\n\0r/s\r\n\0r/s2\r\n\0r\r\n\0mohm\r\n\0uH\r\n\0mWb\r\n\0"
	"V\r\n\0C\r\n\0%\r\n\0A_per_rad_s\r\n\0kbps\r\n\0ms\r\n";

/* All scalar read routes share one pointer-free, eight-byte descriptor. */
static const UsbScalarRoute g_scalar_routes[] =
{
	USB_SCALAR_ROUTE(USB_POLEPARIS, USB_ROUTE_KIND_MOTOR, MOTOR_PARAMETER_POLE_PAIRS, USB_ROUTE_SCALE_ONE, USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_I32, 1, 0, USB_PREFIX_POL, USB_ROUTE_SUFFIX_CRLF),
	USB_SCALAR_ROUTE(USB_CURRENT_CAL, USB_ROUTE_KIND_MOTOR, MOTOR_PARAMETER_CALIBRATION_CURRENT_A, USB_ROUTE_SCALE_ONE, USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_FIXED, 0, 2, USB_PREFIX_I_CAL, USB_ROUTE_SUFFIX_A),
	USB_SCALAR_ROUTE(USB_CURRENT_LIMIT, USB_ROUTE_KIND_MOTOR, MOTOR_PARAMETER_CURRENT_LIMIT_A, USB_ROUTE_SCALE_ONE, USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_FIXED, 0, 2, USB_PREFIX_I_LIM, USB_ROUTE_SUFFIX_A),
	USB_SCALAR_ROUTE(USB_SPEED_LIMIT, USB_ROUTE_KIND_MOTOR, MOTOR_PARAMETER_SPEED_LIMIT_RAD_S, USB_ROUTE_SCALE_TWO_PI, USB_ROUTE_SCALE_ONE_BY_2PI, USB_ROUTE_FORMAT_FIXED, 0, 2, USB_PREFIX_SPD_LIM, USB_ROUTE_SUFFIX_RPS),
	USB_SCALAR_ROUTE(USB_SPEED_ACC, USB_ROUTE_KIND_MOTOR, MOTOR_PARAMETER_SPEED_ACCELERATION_RAD_S2, USB_ROUTE_SCALE_TWO_PI, USB_ROUTE_SCALE_ONE_BY_2PI, USB_ROUTE_FORMAT_FIXED, 0, 2, USB_PREFIX_SPD_ACC, USB_ROUTE_SUFFIX_RPS2),
	USB_SCALAR_ROUTE(USB_SPEED_DEC, USB_ROUTE_KIND_MOTOR, MOTOR_PARAMETER_SPEED_DECELERATION_RAD_S2, USB_ROUTE_SCALE_TWO_PI, USB_ROUTE_SCALE_ONE_BY_2PI, USB_ROUTE_FORMAT_FIXED, 0, 2, USB_PREFIX_SPD_DEC, USB_ROUTE_SUFFIX_RPS2),
	USB_SCALAR_ROUTE(USB_SPEED_KP, USB_ROUTE_KIND_MOTOR, MOTOR_PARAMETER_SPEED_KP, USB_ROUTE_SCALE_ONE, USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_FIXED, 0, 2, USB_PREFIX_SPD_KP, USB_ROUTE_SUFFIX_CRLF),
	USB_SCALAR_ROUTE(USB_SPEED_KI, USB_ROUTE_KIND_MOTOR, MOTOR_PARAMETER_SPEED_KI, USB_ROUTE_SCALE_ONE, USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_FIXED, 0, 2, USB_PREFIX_SPD_KI, USB_ROUTE_SUFFIX_CRLF),
	USB_SCALAR_ROUTE(USB_POS_ACC, USB_ROUTE_KIND_MOTOR, MOTOR_PARAMETER_POSITION_ACCELERATION_RAD_S2, USB_ROUTE_SCALE_TWO_PI, USB_ROUTE_SCALE_ONE_BY_2PI, USB_ROUTE_FORMAT_FIXED, 0, 3, USB_PREFIX_POS_ACC, USB_ROUTE_SUFFIX_RPS2),
	USB_SCALAR_ROUTE(USB_POS_DEC, USB_ROUTE_KIND_MOTOR, MOTOR_PARAMETER_POSITION_DECELERATION_RAD_S2, USB_ROUTE_SCALE_TWO_PI, USB_ROUTE_SCALE_ONE_BY_2PI, USB_ROUTE_FORMAT_FIXED, 0, 3, USB_PREFIX_POS_DEC, USB_ROUTE_SUFFIX_RPS2),
	USB_SCALAR_ROUTE(USB_POS_MAXSPEED, USB_ROUTE_KIND_MOTOR, MOTOR_PARAMETER_POSITION_MAX_SPEED_RAD_S, USB_ROUTE_SCALE_TWO_PI, USB_ROUTE_SCALE_ONE_BY_2PI, USB_ROUTE_FORMAT_FIXED, 0, 3, USB_PREFIX_POS_MAXSPD, USB_ROUTE_SUFFIX_RPS),
	USB_SCALAR_ROUTE(USB_POS_KP, USB_ROUTE_KIND_MOTOR, MOTOR_PARAMETER_POSITION_KP_A_PER_RAD, USB_ROUTE_SCALE_ONE, USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_FIXED, 0, 2, USB_PREFIX_POS_KP, USB_ROUTE_SUFFIX_CRLF),
	USB_SCALAR_ROUTE(USB_POS_KD, USB_ROUTE_KIND_MOTOR, MOTOR_PARAMETER_POSITION_KD_A_PER_RAD_S, USB_ROUTE_SCALE_ONE, USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_FIXED, 0, 2, USB_PREFIX_POS_KD, USB_ROUTE_SUFFIX_CRLF),
	USB_SCALAR_ROUTE(USB_POS_KI, USB_ROUTE_KIND_MOTOR, MOTOR_PARAMETER_POSITION_KI_A_PER_RAD_S, USB_ROUTE_SCALE_ONE, USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_FIXED, 0, 3, USB_PREFIX_POS_KI, USB_ROUTE_SUFFIX_CRLF),
	USB_SCALAR_ROUTE(USB_POS_INTEGRAL_LIMIT, USB_ROUTE_KIND_MOTOR, MOTOR_PARAMETER_POSITION_INTEGRAL_LIMIT_A, USB_ROUTE_SCALE_ONE, USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_FIXED, 0, 3, USB_PREFIX_POS_I_LIMIT, USB_ROUTE_SUFFIX_A),
	USB_SCALAR_ROUTE(USB_CASCADE_POS_KP, USB_ROUTE_KIND_MOTOR, MOTOR_PARAMETER_CASCADE_POSITION_KP_PER_S, USB_ROUTE_SCALE_ONE, USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_FIXED, 0, 3, USB_PREFIX_CASCADE_POS_KP, USB_ROUTE_SUFFIX_CRLF),
	USB_SCALAR_ROUTE(USB_CASCADE_POS_KD, USB_ROUTE_KIND_MOTOR, MOTOR_PARAMETER_CASCADE_POSITION_KD, USB_ROUTE_SCALE_ONE, USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_FIXED, 0, 3, USB_PREFIX_CASCADE_POS_KD, USB_ROUTE_SUFFIX_CRLF),
	USB_SCALAR_ROUTE(USB_RS, USB_ROUTE_KIND_MOTOR, MOTOR_PARAMETER_PHASE_RESISTANCE_OHM, USB_ROUTE_SCALE_ONE, USB_ROUTE_SCALE_MILLI, USB_ROUTE_FORMAT_FIXED, 0, 2, USB_PREFIX_RS, USB_ROUTE_SUFFIX_MOHM),
	USB_SCALAR_ROUTE(USB_LD, USB_ROUTE_KIND_MOTOR, MOTOR_PARAMETER_D_AXIS_INDUCTANCE_H, USB_ROUTE_SCALE_ONE, USB_ROUTE_SCALE_MICRO, USB_ROUTE_FORMAT_FIXED, 0, 2, USB_PREFIX_LD, USB_ROUTE_SUFFIX_UH),
	USB_SCALAR_ROUTE(USB_LQ, USB_ROUTE_KIND_MOTOR, MOTOR_PARAMETER_Q_AXIS_INDUCTANCE_H, USB_ROUTE_SCALE_ONE, USB_ROUTE_SCALE_MICRO, USB_ROUTE_FORMAT_FIXED, 0, 2, USB_PREFIX_LQ, USB_ROUTE_SUFFIX_UH),
	USB_SCALAR_ROUTE(USB_FLUX, USB_ROUTE_KIND_MOTOR, MOTOR_PARAMETER_FLUX_WEBER, USB_ROUTE_SCALE_ONE, USB_ROUTE_SCALE_MILLI, USB_ROUTE_FORMAT_FIXED, 0, 2, USB_PREFIX_FLUX, USB_ROUTE_SUFFIX_MWB),
	USB_SCALAR_ROUTE(USB_MODE, USB_ROUTE_KIND_TELEMETRY, MOTOR_TELEMETRY_MODE, USB_ROUTE_SCALE_ONE, USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_I32, 0, 0, USB_PREFIX_MODE, USB_ROUTE_SUFFIX_CRLF),
	USB_SCALAR_ROUTE(USB_CURRENT_SET, USB_ROUTE_KIND_TELEMETRY, MOTOR_TELEMETRY_CURRENT_REFERENCE_A, USB_ROUTE_SCALE_ONE, USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_FIXED, 0, 2, USB_PREFIX_I_SET, USB_ROUTE_SUFFIX_A),
	USB_SCALAR_ROUTE(USB_SPEED_SET, USB_ROUTE_KIND_TELEMETRY, MOTOR_TELEMETRY_SPEED_REFERENCE_RAD_S, USB_ROUTE_SCALE_ONE, USB_ROUTE_SCALE_ONE_BY_2PI, USB_ROUTE_FORMAT_FIXED, 0, 2, USB_PREFIX_SPD_SET, USB_ROUTE_SUFFIX_RPS),
	USB_SCALAR_ROUTE(USB_POS_SET, USB_ROUTE_KIND_TELEMETRY, MOTOR_TELEMETRY_POSITION_REFERENCE_RAD, USB_ROUTE_SCALE_ONE, USB_ROUTE_SCALE_ONE_BY_2PI, USB_ROUTE_FORMAT_FIXED, 0, 2, USB_PREFIX_POS_SET, USB_ROUTE_SUFFIX_REV),
	USB_SCALAR_ROUTE(USB_ENCODER_REVERSE, USB_ROUTE_KIND_TELEMETRY, MOTOR_TELEMETRY_ENCODER_REVERSED, USB_ROUTE_SCALE_ONE, USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_U32, 0, 0, USB_PREFIX_ERV, USB_ROUTE_SUFFIX_CRLF),
	USB_SCALAR_ROUTE(USB_VBUS, USB_ROUTE_KIND_TELEMETRY, MOTOR_TELEMETRY_BUS_VOLTAGE_V, USB_ROUTE_SCALE_ONE, USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_FIXED, 0, 2, USB_PREFIX_VBUS, USB_ROUTE_SUFFIX_V),
	USB_SCALAR_ROUTE(USB_IBUS, USB_ROUTE_KIND_TELEMETRY, MOTOR_TELEMETRY_BUS_CURRENT_A, USB_ROUTE_SCALE_ONE, USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_FIXED, 0, 2, USB_PREFIX_IBUS, USB_ROUTE_SUFFIX_A),
	USB_SCALAR_ROUTE(USB_IA, USB_ROUTE_KIND_TELEMETRY, MOTOR_TELEMETRY_PHASE_A_CURRENT_A, USB_ROUTE_SCALE_ONE, USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_FIXED, 0, 2, USB_PREFIX_IA, USB_ROUTE_SUFFIX_A),
	USB_SCALAR_ROUTE(USB_IB, USB_ROUTE_KIND_TELEMETRY, MOTOR_TELEMETRY_PHASE_B_CURRENT_A, USB_ROUTE_SCALE_ONE, USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_FIXED, 0, 2, USB_PREFIX_IB, USB_ROUTE_SUFFIX_A),
	USB_SCALAR_ROUTE(USB_IC, USB_ROUTE_KIND_TELEMETRY, MOTOR_TELEMETRY_PHASE_C_CURRENT_A, USB_ROUTE_SCALE_ONE, USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_FIXED, 0, 2, USB_PREFIX_IC, USB_ROUTE_SUFFIX_A),
	USB_SCALAR_ROUTE(USB_ID, USB_ROUTE_KIND_TELEMETRY, MOTOR_TELEMETRY_D_AXIS_CURRENT_A, USB_ROUTE_SCALE_ONE, USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_FIXED, 0, 2, USB_PREFIX_ID, USB_ROUTE_SUFFIX_A),
	USB_SCALAR_ROUTE(USB_IQ, USB_ROUTE_KIND_TELEMETRY, MOTOR_TELEMETRY_Q_AXIS_CURRENT_A, USB_ROUTE_SCALE_ONE, USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_FIXED, 0, 2, USB_PREFIX_IQ, USB_ROUTE_SUFFIX_A),
	USB_SCALAR_ROUTE(USB_SPEED2_FILT, USB_ROUTE_KIND_TELEMETRY, MOTOR_TELEMETRY_MECHANICAL_SPEED_RAD_S, USB_ROUTE_SCALE_ONE, USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_FIXED, 0, 2, USB_PREFIX_SPD2_FILT, USB_ROUTE_SUFFIX_RPS),
	USB_SCALAR_ROUTE(USB_POS2_FILT, USB_ROUTE_KIND_TELEMETRY, MOTOR_TELEMETRY_MECHANICAL_POSITION_RAD, USB_ROUTE_SCALE_ONE, USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_FIXED, 0, 3, USB_PREFIX_POS2_FILT, USB_ROUTE_SUFFIX_REV),
	USB_SCALAR_ROUTE(USB_TEMP, USB_ROUTE_KIND_TELEMETRY, MOTOR_TELEMETRY_TEMPERATURE_C, USB_ROUTE_SCALE_ONE, USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_FIXED, 0, 2, USB_PREFIX_TEMP, USB_ROUTE_SUFFIX_C),
	USB_SCALAR_ROUTE(USB_ERROR, USB_ROUTE_KIND_TELEMETRY, MOTOR_TELEMETRY_PRIMARY_ERROR, USB_ROUTE_SCALE_ONE, USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_I32, 0, 0, USB_PREFIX_ERROR, USB_ROUTE_SUFFIX_CRLF),
	USB_SCALAR_ROUTE(USB_COMMISSIONING_STAGE, USB_ROUTE_KIND_TELEMETRY, MOTOR_TELEMETRY_COMMISSIONING_STAGE, USB_ROUTE_SCALE_ONE, USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_U32, 0, 0, USB_PREFIX_CST, USB_ROUTE_SUFFIX_CRLF),
	USB_SCALAR_ROUTE(USB_COMMISSIONING_PROGRESS, USB_ROUTE_KIND_TELEMETRY, MOTOR_TELEMETRY_COMMISSIONING_PROGRESS_PERCENT, USB_ROUTE_SCALE_ONE, USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_U32, 0, 0, USB_PREFIX_CPR, USB_ROUTE_SUFFIX_PERCENT),
	USB_SCALAR_ROUTE(USB_RESISTANCE_SPREAD, USB_ROUTE_KIND_TELEMETRY, MOTOR_TELEMETRY_PHASE_RESISTANCE_SPREAD_PERCENT, USB_ROUTE_SCALE_ONE, USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_FIXED, 0, 2, USB_PREFIX_RSP, USB_ROUTE_SUFFIX_PERCENT),
	USB_SCALAR_ROUTE(USB_RESISTANCE_DESIGN_ERROR, USB_ROUTE_KIND_TELEMETRY, MOTOR_TELEMETRY_PHASE_RESISTANCE_DESIGN_ERROR_PERCENT, USB_ROUTE_SCALE_ONE, USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_FIXED, 0, 2, USB_PREFIX_RDE, USB_ROUTE_SUFFIX_PERCENT),
	USB_SCALAR_ROUTE(USB_NODE_ID, USB_ROUTE_KIND_CAN_CONFIGURATION, 0, USB_ROUTE_SCALE_ONE, USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_U32, 0, 0, USB_PREFIX_CAN_ID, USB_ROUTE_SUFFIX_CRLF),
	USB_SCALAR_ROUTE(USB_CAN_BR, USB_ROUTE_KIND_CAN_CONFIGURATION, 1, USB_ROUTE_SCALE_ONE, USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_U32, 0, 0, USB_PREFIX_CAN_BR, USB_ROUTE_SUFFIX_KBPS),
	USB_SCALAR_ROUTE(USB_CAN_HB, USB_ROUTE_KIND_CAN_CONFIGURATION, 2, USB_ROUTE_SCALE_ONE, USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_U32, 0, 0, USB_PREFIX_CAN_HB, USB_ROUTE_SUFFIX_MS),
	USB_SCALAR_ROUTE(USB_FRICTION_COULOMB_POS, USB_ROUTE_KIND_FRICTION, offsetof(FrictionIdentificationPortStatus, candidate_coulomb_pos_a), USB_ROUTE_SCALE_ONE, USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_FIXED, 0, 6, USB_PREFIX_FRICTION_COULOMB_POS, USB_ROUTE_SUFFIX_A),
	USB_SCALAR_ROUTE(USB_FRICTION_COULOMB_NEG, USB_ROUTE_KIND_FRICTION, offsetof(FrictionIdentificationPortStatus, candidate_coulomb_neg_a), USB_ROUTE_SCALE_ONE, USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_FIXED, 0, 6, USB_PREFIX_FRICTION_COULOMB_NEG, USB_ROUTE_SUFFIX_A),
	USB_SCALAR_ROUTE(USB_FRICTION_VISCOUS_POS, USB_ROUTE_KIND_FRICTION, offsetof(FrictionIdentificationPortStatus, candidate_viscous_pos_a_per_rad_s), USB_ROUTE_SCALE_ONE, USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_FIXED, 0, 6, USB_PREFIX_FRICTION_VISCOUS_POS, USB_ROUTE_SUFFIX_A_PER_RAD_S),
	USB_SCALAR_ROUTE(USB_FRICTION_VISCOUS_NEG, USB_ROUTE_KIND_FRICTION, offsetof(FrictionIdentificationPortStatus, candidate_viscous_neg_a_per_rad_s), USB_ROUTE_SCALE_ONE, USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_FIXED, 0, 6, USB_PREFIX_FRICTION_VISCOUS_NEG, USB_ROUTE_SUFFIX_A_PER_RAD_S),
	USB_SCALAR_ROUTE(USB_FRICTION_RMSE_POS, USB_ROUTE_KIND_FRICTION, offsetof(FrictionIdentificationPortStatus, candidate_rmse_pos_a), USB_ROUTE_SCALE_ONE, USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_FIXED, 0, 6, USB_PREFIX_FRICTION_RMSE_POS, USB_ROUTE_SUFFIX_A),
	USB_SCALAR_ROUTE(USB_FRICTION_RMSE_NEG, USB_ROUTE_KIND_FRICTION, offsetof(FrictionIdentificationPortStatus, candidate_rmse_neg_a), USB_ROUTE_SCALE_ONE, USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_FIXED, 0, 6, USB_PREFIX_FRICTION_RMSE_NEG, USB_ROUTE_SUFFIX_A),
	USB_SCALAR_ROUTE(USB_FRICTION_VALID, USB_ROUTE_KIND_FRICTION, offsetof(FrictionIdentificationPortStatus, active_model_valid), USB_ROUTE_SCALE_ONE, USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_U32, 0, 0, USB_PREFIX_FRICTION_MODEL_VALID, USB_ROUTE_SUFFIX_CRLF),
	USB_SCALAR_ROUTE(USB_ACTIVE_COULOMB_POS, USB_ROUTE_KIND_FRICTION, offsetof(FrictionIdentificationPortStatus, active_coulomb_pos_a), USB_ROUTE_SCALE_ONE, USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_FIXED, 0, 6, USB_PREFIX_ACTIVE_COULOMB_POS, USB_ROUTE_SUFFIX_A),
	USB_SCALAR_ROUTE(USB_ACTIVE_COULOMB_NEG, USB_ROUTE_KIND_FRICTION, offsetof(FrictionIdentificationPortStatus, active_coulomb_neg_a), USB_ROUTE_SCALE_ONE, USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_FIXED, 0, 6, USB_PREFIX_ACTIVE_COULOMB_NEG, USB_ROUTE_SUFFIX_A),
	USB_SCALAR_ROUTE(USB_ACTIVE_VISCOUS_POS, USB_ROUTE_KIND_FRICTION, offsetof(FrictionIdentificationPortStatus, active_viscous_pos_a_per_rad_s), USB_ROUTE_SCALE_ONE, USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_FIXED, 0, 6, USB_PREFIX_ACTIVE_VISCOUS_POS, USB_ROUTE_SUFFIX_A_PER_RAD_S),
	USB_SCALAR_ROUTE(USB_ACTIVE_VISCOUS_NEG, USB_ROUTE_KIND_FRICTION, offsetof(FrictionIdentificationPortStatus, active_viscous_neg_a_per_rad_s), USB_ROUTE_SCALE_ONE, USB_ROUTE_SCALE_ONE, USB_ROUTE_FORMAT_FIXED, 0, 6, USB_PREFIX_ACTIVE_VISCOUS_NEG, USB_ROUTE_SUFFIX_A_PER_RAD_S)
};

static const float g_route_scales[] =
{
	1.0f,
	USB_COMMAND_ROUTER_TWO_PI,
	USB_COMMAND_ROUTER_ONE_BY_2PI,
	1000.0f,
	1000000.0f
};

static float UsbCommandRouter_ApplyScale(float value, uint8_t scale)
{
	return value * g_route_scales[scale];
}

static const UsbScalarRoute *UsbCommandRouter_FindScalar(
	UsbParameterId parameter)
{
	unsigned int index;
	for (index = 0U; index < sizeof(g_scalar_routes) /
		sizeof(g_scalar_routes[0]); ++index)
	{
		if (g_scalar_routes[index].usb_parameter == (uint32_t)parameter)
			return &g_scalar_routes[index];
	}
	return NULL;
}

bool UsbCommandRouter_Initialize(UsbCommandRouterContext *context,
	ApplicationEndpoints *application)
{
	if (context == NULL || application == NULL)
		return false;
	context->application = application;
	return true;
}

static UsbCommandError UsbCommandRouter_MapParameterResult(
	ParameterServiceResult result)
{
	if (result == PARAMETER_SERVICE_ACCEPTED)
		return USB_NO_ERROR;
	if (result == PARAMETER_SERVICE_INVALID_VALUE)
		return USB_DATA_INVALID;
	if (result == PARAMETER_SERVICE_OUT_OF_RANGE)
		return USB_DATA_OUT_OF_RANGE;
	return USB_WRITE_INVALID;
}

static UsbCommandError UsbCommandRouter_WriteParameter(
	UsbCommandRouterContext *context, MotorParameterId parameter, float value)
{
	return UsbCommandRouter_MapParameterResult(
		ParameterService_WriteMotorParameter(context->application->parameters,
			parameter, value));
}

static float UsbCommandRouter_ReadParameter(UsbCommandRouterContext *context,
	MotorParameterId parameter)
{
	float value = 0.0f;
	(void)ParameterService_ReadMotorParameter(context->application->parameters,
		parameter, &value);
	return value;
}

static float UsbCommandRouter_ReadTelemetry(UsbCommandRouterContext *context,
	MotorTelemetryId telemetry)
{
	float value = 0.0f;
	(void)TelemetryService_ReadValue(context->application->telemetry, telemetry, &value);
	return value;
}

static UsbCommandError UsbCommandRouter_Write(UsbCommandRouterContext *context,
	UsbParameterId parameter, float value, uint8_t value_is_float)
{
	int integer_value = (int)value;
	MotorCommandResult command_result;
	const UsbScalarRoute *route = UsbCommandRouter_FindScalar(parameter);

	if (route != NULL && USB_ROUTE_KIND(route->details) == USB_ROUTE_KIND_MOTOR)
	{
		if (USB_ROUTE_INTEGER_WRITE(route->details) &&
			value_is_float != 0U)
			return USB_DATA_INVALID;
		return UsbCommandRouter_WriteParameter(context,
			(MotorParameterId)USB_ROUTE_SOURCE(route->details),
			UsbCommandRouter_ApplyScale(value,
				USB_ROUTE_WRITE_SCALE(route->details)));
	}
	switch (parameter)
	{
		case USB_MODE:
			if (value_is_float != 0U)
				return USB_DATA_INVALID;
			if (integer_value < 0 || integer_value >= USB_COMMAND_ROUTER_MAX_MODE_VALUE)
				return USB_DATA_OUT_OF_RANGE;
			command_result = MotorCommandService_RequestActionCode(
				context->application->motor_command, (uint8_t)integer_value);
			if (command_result != MOTOR_COMMAND_ACCEPTED)
				return USB_WRITE_INVALID;
			if (integer_value == USB_COMMAND_ROUTER_DISABLED_MODE)
				ControlAuthorityService_Release(
					context->application->control_authority, CONTROL_AUTHORITY_USB);
			else
				ControlAuthorityService_Claim(
					context->application->control_authority, CONTROL_AUTHORITY_USB);
			return USB_NO_ERROR;
		case USB_CURRENT_SET:
			command_result = MotorCommandService_SetCurrentReferenceA(
				context->application->motor_command, value);
			break;
		case USB_SPEED_SET:
			command_result = MotorCommandService_SetSpeedReferenceRps(
				context->application->motor_command, value);
			break;
		case USB_POS_SET:
			command_result = MotorCommandService_SetPositionReferenceRevolutions(
				context->application->motor_command, value);
			break;
		case USB_NODE_ID:
			if (value_is_float != 0U)
				return USB_DATA_INVALID;
			return integer_value >= 0 && integer_value <= 7 &&
				CanConfigurationService_SetNodeId(context->application->can_configuration,
					(uint8_t)integer_value) ?
				USB_NO_ERROR : USB_DATA_OUT_OF_RANGE;
		case USB_ENCODER_STATE:
			return value_is_float != 0U ? USB_DATA_INVALID : USB_DATA_OUT_OF_RANGE;
		case USB_ENCODER_REVERSE:
			if (value_is_float != 0U)
				return USB_DATA_INVALID;
			if ((int)UsbCommandRouter_ReadTelemetry(context,
				MOTOR_TELEMETRY_MODE) !=
				(int)USB_COMMAND_ROUTER_DISABLED_MODE)
				return USB_WRITE_INVALID;
			if ((integer_value != 0 && integer_value != 1) ||
				!RotorCalibrationService_SetReverse(context->application->rotor_calibration,
					integer_value != 0))
				return USB_DATA_OUT_OF_RANGE;
			return USB_NO_ERROR;
		case USB_CAN_BR:
			if (value_is_float != 0U)
				return USB_DATA_INVALID;
			return integer_value >= 0 &&
				CanConfigurationService_SetBitrateKbps(context->application->can_configuration,
					(uint32_t)integer_value) ?
				USB_NO_ERROR : USB_DATA_OUT_OF_RANGE;
		case USB_CAN_HB:
			if (value_is_float != 0U)
				return USB_DATA_INVALID;
			return integer_value >= 0 &&
				CanConfigurationService_SetHeartbeatMs(context->application->can_configuration,
					(uint32_t)integer_value) ?
				USB_NO_ERROR : USB_DATA_OUT_OF_RANGE;
		case USB_COGGING:
			return USB_NO_ERROR;
		case USB_FRICTION_APPLY:
			if (value_is_float != 0U || integer_value != 1)
				return USB_DATA_INVALID;
			return FrictionIdentificationService_ApplyCandidate(
				context->application->friction_identification) ?
				USB_NO_ERROR : USB_WRITE_INVALID;
		case USB_LUT_EXPORT:
		case USB_FRICTION_STATUS:
		case USB_FRICTION_DATA_EXPORT:
			return USB_WRITE_INVALID;
		default:
			return route != NULL ? USB_WRITE_INVALID : USB_UNKNOWNED_PARAM;
	}

	if (command_result == MOTOR_COMMAND_INVALID_VALUE)
		return USB_DATA_INVALID;
	if (command_result == MOTOR_COMMAND_OUT_OF_RANGE)
		return USB_DATA_OUT_OF_RANGE;
	if (command_result != MOTOR_COMMAND_ACCEPTED)
		return USB_WRITE_INVALID;
	ControlAuthorityService_Claim(context->application->control_authority,
		CONTROL_AUTHORITY_USB);
	return USB_NO_ERROR;
}

static void UsbCommandRouter_FormatLiteral(UsbCommandRouterResponse *response,
	const char *literal)
{
	TextWriter writer;

	TextWriter_Initialize(&writer, response->text, sizeof(response->text));
	(void)TextWriter_AppendLiteral(&writer, literal);
}

static void UsbCommandRouter_FormatValue(UsbCommandRouterResponse *response,
	const char *leading, const char *prefix, const char *suffix, uint8_t format,
	uint8_t precision, float value)
{
	TextWriter writer;

	TextWriter_Initialize(&writer, response->text, sizeof(response->text));
	if (leading != NULL)
		(void)TextWriter_AppendLiteral(&writer, leading);
	(void)TextWriter_AppendLiteral(&writer, prefix);
	if ((UsbRouteFormat)format == USB_ROUTE_FORMAT_U32)
		(void)TextWriter_AppendU32(&writer, (uint32_t)value);
	else if ((UsbRouteFormat)format == USB_ROUTE_FORMAT_I32)
		(void)TextWriter_AppendI32(&writer, (int32_t)value);
	else
		(void)TextWriter_AppendFixedF32(&writer, value, precision);
	(void)TextWriter_AppendLiteral(&writer, suffix);
}

static void UsbCommandRouter_FormatScalar(UsbCommandRouterResponse *response,
	const UsbScalarRoute *route, float value)
{
	const char *prefix = &g_route_prefixes[
		USB_ROUTE_PREFIX_OFFSET(route->details)];
	const char *suffix = &g_route_suffixes[g_route_suffix_offsets[
		USB_ROUTE_SUFFIX(route->details)]];
	const char *leading = NULL;

	if (USB_ROUTE_KIND(route->details) == USB_ROUTE_KIND_FRICTION)
		leading = (route->usb_parameter >> 16U) == (uint32_t)'a' ?
			"active_" : "friction_";

	UsbCommandRouter_FormatValue(response, leading, prefix, suffix,
		USB_ROUTE_FORMAT(route->details), USB_ROUTE_PRECISION(route->details),
		value);
}

static UsbCommandError UsbCommandRouter_Read(UsbCommandRouterContext *context,
	const UsbCommandRouterState *state, UsbCommandRouterResponse *response,
	UsbParameterId parameter)
{
	float value;
	TextWriter writer;
	FrictionIdentificationPortStatus friction;
	const UsbScalarRoute *route = UsbCommandRouter_FindScalar(parameter);
	uint8_t source;
	uint8_t kind;

	if (route != NULL)
	{
		source = USB_ROUTE_SOURCE(route->details);
		kind = USB_ROUTE_KIND(route->details);
		if (kind == USB_ROUTE_KIND_MOTOR)
			value = UsbCommandRouter_ReadParameter(context,
				(MotorParameterId)source);
		else if (kind == USB_ROUTE_KIND_TELEMETRY)
			value = UsbCommandRouter_ReadTelemetry(context,
				(MotorTelemetryId)source);
		else if (kind == USB_ROUTE_KIND_CAN_CONFIGURATION)
		{
			if (source == 0U)
				value = (float)CanConfigurationService_GetNodeId(
					context->application->can_configuration);
			else if (source == 1U)
				value = (float)CanConfigurationService_GetBitrateKbps(
					context->application->can_configuration);
			else
				value = (float)CanConfigurationService_GetHeartbeatMs(
					context->application->can_configuration);
		}
		else
		{
			if (!FrictionIdentificationService_ReadStatus(
				context->application->friction_identification, &friction))
				return USB_WRITE_INVALID;
			if (USB_ROUTE_FORMAT(route->details) == USB_ROUTE_FORMAT_U32)
				value = (float)*((const uint8_t *)(const void *)&friction + source);
			else
				(void)memcpy(&value,
					(const uint8_t *)(const void *)&friction + source,
					sizeof(value));
		}
		value = UsbCommandRouter_ApplyScale(value,
			USB_ROUTE_READ_SCALE(route->details));
		UsbCommandRouter_FormatScalar(response, route, value);
		return USB_NO_ERROR;
	}

	switch (parameter)
	{
		case USB_ENCODER_STATE:
			TextWriter_Initialize(&writer, response->text, sizeof(response->text));
			(void)TextWriter_AppendLiteral(&writer, "encoder=");
			(void)TextWriter_AppendLiteral(&writer,
				UsbCommandRouter_ReadTelemetry(context,
					MOTOR_TELEMETRY_ENCODER_ONLINE) != 0.0f ?
					"Online" : "Offline");
			(void)TextWriter_AppendLiteral(&writer, ".\r\n"); break;
		case USB_LUT_EXPORT:
			if (state->print_active || state->lut_export_active ||
				state->friction_export_active)
				return USB_WRITE_INVALID;
			TextWriter_Initialize(&writer, response->text, sizeof(response->text));
			(void)TextWriter_AppendLiteral(&writer, "lut_begin,count=");
			(void)TextWriter_AppendU32(&writer,
				RotorCalibrationService_GetEntryCount(
					context->application->rotor_calibration));
			(void)TextWriter_AppendLiteral(&writer, ",reverse=");
			(void)TextWriter_AppendU32(&writer, (uint32_t)
				UsbCommandRouter_ReadTelemetry(context,
					MOTOR_TELEMETRY_ENCODER_REVERSED));
			(void)TextWriter_AppendLiteral(&writer, "\r\n");
			response->action = USB_COMMAND_ROUTER_ACTION_BEGIN_LUT_EXPORT;
			break;
		case USB_FRICTION_STATUS:
			if (!FrictionIdentificationService_ReadStatus(
				context->application->friction_identification, &friction))
				return USB_WRITE_INVALID;
			TextWriter_Initialize(&writer, response->text, sizeof(response->text));
			(void)TextWriter_AppendLiteral(&writer, "friction_state=");
			(void)TextWriter_AppendU32(&writer, friction.state);
			(void)TextWriter_AppendLiteral(&writer, ",reason=");
			(void)TextWriter_AppendU32(&writer, friction.reason);
			(void)TextWriter_AppendLiteral(&writer, ",point=");
			(void)TextWriter_AppendU32(&writer, friction.point_index);
			(void)TextWriter_AppendLiteral(&writer, ",progress=");
			(void)TextWriter_AppendFixedF32(&writer, friction.progress_percent, 1U);
			(void)TextWriter_AppendLiteral(&writer, ",candidate=");
			(void)TextWriter_AppendU32(&writer, friction.candidate_valid);
			(void)TextWriter_AppendLiteral(&writer, "\r\n"); break;
		case USB_FRICTION_DATA_EXPORT:
			if (!FrictionIdentificationService_ReadStatus(
				context->application->friction_identification, &friction) ||
				!friction.candidate_valid || state->print_active ||
				state->lut_export_active || state->friction_export_active)
				return USB_WRITE_INVALID;
			UsbCommandRouter_FormatValue(response, NULL, "friction_begin,count=", "\r\n",
				USB_ROUTE_FORMAT_U32, 0U, (float)friction.sample_count);
			response->action = USB_COMMAND_ROUTER_ACTION_BEGIN_FRICTION_EXPORT;
			break;
		default:
			return USB_UNKNOWNED_PARAM;
	}
	return USB_NO_ERROR;
}

static bool UsbCommandRouter_ResolvePrint(UsbParameterId parameter, float *scale,
	UsbCommandRouterPrintSource *print_source)
{
	const UsbScalarRoute *route;
	uint8_t source;
	uint8_t kind;

	if (scale == NULL || print_source == NULL)
		return false;
	*scale = 1.0f;
	*print_source = USB_COMMAND_ROUTER_PRINT_ZERO;
	route = UsbCommandRouter_FindScalar(parameter);
	if (route != NULL)
	{
		kind = USB_ROUTE_KIND(route->details);
		source = USB_ROUTE_SOURCE(route->details);
		if (kind == USB_ROUTE_KIND_FRICTION)
			return false;
		*scale = UsbCommandRouter_ApplyScale(1.0f,
			USB_ROUTE_READ_SCALE(route->details));
		if (kind == USB_ROUTE_KIND_MOTOR)
			*print_source = (UsbCommandRouterPrintSource)
				(MOTOR_TELEMETRY_POLE_PAIRS + source);
		else if (kind == USB_ROUTE_KIND_TELEMETRY)
		{
			/* The binary stream historically exposes filtered d/q currents. */
			if (parameter == USB_ID)
				source = MOTOR_TELEMETRY_D_AXIS_CURRENT_FILTERED_A;
			else if (parameter == USB_IQ)
				source = MOTOR_TELEMETRY_Q_AXIS_CURRENT_FILTERED_A;
			*print_source = source;
			if (USB_ROUTE_FORMAT(route->details) != USB_ROUTE_FORMAT_FIXED)
				*print_source |= USB_COMMAND_ROUTER_PRINT_INTEGER_FLAG;
		}
		else if (source == 0U)
			*print_source = USB_COMMAND_ROUTER_PRINT_CAN_NODE_ID;
		else if (source == 1U)
			*print_source = USB_COMMAND_ROUTER_PRINT_CAN_BITRATE;
		else
			*print_source = USB_COMMAND_ROUTER_PRINT_CAN_HEARTBEAT;
		return true;
	}
	switch (parameter)
	{
		case USB_ENCODER_STATE:
			*print_source = MOTOR_TELEMETRY_ENCODER_ONLINE |
				USB_COMMAND_ROUTER_PRINT_INTEGER_FLAG;
			return true;
		case USB_COGGING:
		case USB_LUT_EXPORT:
		case USB_FRICTION_STATUS:
		case USB_FRICTION_APPLY:
		case USB_FRICTION_DATA_EXPORT:
			return false;
		default:
			break;
	}
	return true;
}

UsbCommandError UsbCommandRouter_Handle(UsbCommandRouterContext *context,
	const UsbProtocolV1Command *command,
	const UsbCommandRouterState *state, UsbCommandRouterResponse *response)
{
	UsbCommandError result;
	int channel;
	UsbParameterId parameter_id;

	if (context == NULL || command == NULL || state == NULL || response == NULL ||
		!isfinite(command->value) || command->value >= (float)INT_MAX ||
		command->value <= (float)INT_MIN)
		return USB_DATA_INVALID;
	response->action = USB_COMMAND_ROUTER_ACTION_NONE;
	response->text[0] = '\0';
	parameter_id = (UsbParameterId)command->parameter_id;
	if (command->operation == USB_PROTOCOL_V1_OPERATION_WRITE)
	{
		result = UsbCommandRouter_Write(context, parameter_id, command->value,
			command->value_is_float ? 1U : 0U);
		if (result == USB_NO_ERROR)
		{
			UsbCommandRouter_FormatLiteral(response, "Write Success!\r\n");
			response->action = USB_COMMAND_ROUTER_ACTION_SEND_TEXT;
		}
		return result;
	}
	if (command->operation == USB_PROTOCOL_V1_OPERATION_READ)
	{
		result = UsbCommandRouter_Read(context, state, response, parameter_id);
		if (result == USB_NO_ERROR && response->action == USB_COMMAND_ROUTER_ACTION_NONE)
			response->action = USB_COMMAND_ROUTER_ACTION_SEND_TEXT;
		return result;
	}
	if (command->operation != USB_PROTOCOL_V1_OPERATION_PRINT)
		return USB_SYNTAX_ERROR;

	channel = (int)command->value;
	if (command->value_is_float || channel < 0 || channel >= 5)
		return USB_DATA_INVALID;
	if (!UsbCommandRouter_ResolvePrint(parameter_id,
		&response->print_scale, &response->print_source))
		return USB_UNKNOWNED_PARAM;
	response->print_channel = (uint8_t)channel;
	response->action = USB_COMMAND_ROUTER_ACTION_CONFIGURE_PRINT;
	return USB_NO_ERROR;
}
