#ifndef CORE_SERVICES_ROTOR_FEEDBACK_ENCODER_H
#define CORE_SERVICES_ROTOR_FEEDBACK_ENCODER_H

#include <stdbool.h>
#include <stdint.h>

/* Normalized single-turn angle representation: unsigned 16-bit turn count. */
#define ENCODER_Q15_CPR                 65536UL
#define ENCODER_Q15_HALF_TURN           32768
#define ENCODER_OFFSET_LUT_SIZE          1024U
#define ENCODER_OFFSET_LUT_BITS          10U
#define ENCODER_VELOCITY_WINDOW          16U
#define ENCODER_BAD_FRAME_OFFLINE_COUNT  100U
#define ENCODER_COGGING_MAP_SIZE          128U

/*
 * Sensor-independent sample status. Numeric values remain stable for the
 * runtime boundary adapter, but this service does not depend on a hardware
 * transport contract.
 */
typedef enum
{
	ENCODER_READ_OK = 0,
	ENCODER_READ_TRANSPORT_ERROR = 1,
	ENCODER_READ_CRC_MISMATCH = 2,
	ENCODER_READ_FIELD_TOO_STRONG = 3,
	ENCODER_READ_FIELD_TOO_WEAK = 4,
	ENCODER_READ_FIELD_INVALID = 5,
	ENCODER_READ_OVERSPEED = 6,
	ENCODER_READ_DEVICE_RESET = 7,
	ENCODER_READ_DEVICE_SYSTEM_ERROR = 8,
	ENCODER_READ_DEVICE_INTERFACE_ERROR = 9,
	ENCODER_READ_INVALID_ANGLE = 10
} Encoder_ReadStatus;

typedef struct
{
	Encoder_ReadStatus status;
	uint16_t raw_data_word;
	uint16_t raw_angle_q15;
} EncoderSample;

/* Calibration flags stored in flash. Mechanical zero is optional feedback state. */
#define ENC_CALIB_LINEARIZED        (1U << 0)
#define ENC_CALIB_ELECTRICAL_ZERO   (1U << 1)
#define ENC_CALIB_MECHANICAL_ZERO   (1U << 2)
#define ENC_CALIB_COGGING           (1U << 3)
#define ENC_CALIB_ZERO_POS          ENC_CALIB_ELECTRICAL_ZERO
#define ENC_CALIB_ALL               (ENC_CALIB_LINEARIZED | ENC_CALIB_ELECTRICAL_ZERO)

typedef struct
{
	/* Persisted direction configuration and calibration data. */
	uint16_t electrical_zero_q15;
	uint16_t mechanical_zero_q15;
	int16_t linearization_lut_q15[ENCODER_OFFSET_LUT_SIZE];
	uint8_t calib_flag;
	uint8_t reverse;
	int16_t cogging_compensation_map_ma[ENCODER_COGGING_MAP_SIZE];

	/* Raw sensor reading, direction-corrected input, and LUT-corrected angle. */
	uint16_t raw_q15;
	uint16_t directed_q15;
	uint16_t linearized_q15;
	uint16_t previous_linearized_q15;
	int64_t shadow_q15;
	int64_t mechanical_zero_shadow_q15;
	int64_t velocity_shadow_q15;
	bool has_valid_sample;

	/* Mechanical and electrical feedback exposed to the control interfaces. */
	float theta_elec;
	float vel_elec;
	float theta_mech;
	float vel_mech;

	/* Schedule-driven moving-average mechanical velocity estimator. */
	uint16_t velocity_divider;
	uint16_t velocity_update_divider;
	float velocity_sample_period_s;
	uint8_t velocity_history_index;
	uint8_t velocity_sample_count;
	bool velocity_ready;
	int32_t velocity_delta_history[ENCODER_VELOCITY_WINDOW];
	int32_t velocity_delta_sum;

	/* Sample diagnostics and online state. */
	Encoder_ReadStatus read_status;
	Encoder_ReadStatus read_status_latched;
	uint16_t sensor_raw_data_word;
	uint32_t read_error_count;
	uint16_t bad_frame_streak;
} EncoderContext;

bool Encoder_ParamInit(EncoderContext *encoder,
	uint16_t velocity_update_divider,
	float velocity_sample_period_s);
/* The caller owns sensor acquisition and calls this once per fast-loop tick. */
void Encoder_Update(EncoderContext *encoder, uint32_t pole_pairs,
	const EncoderSample *sample);

bool Encoder_IsOnline(const EncoderContext *encoder);
/* Calibration mutations are deliberately unsynchronized; the caller owns the
 * critical section when these APIs can race with the fast loop. */
bool Encoder_SetElectricalZero(EncoderContext *encoder);
bool Encoder_SetElectricalZeroQ15(EncoderContext *encoder,
	uint16_t electrical_zero_q15);
bool Encoder_SetMechanicalZero(EncoderContext *encoder);
void Encoder_SetReverse(EncoderContext *encoder, bool reverse);
void Encoder_ResetVelocity(EncoderContext *encoder);

float Encoder_GetEleVel(const EncoderContext *encoder);
float Encoder_GetMecVel(const EncoderContext *encoder);
float Encoder_GetElePhase(const EncoderContext *encoder);
float Encoder_GetMecPos(const EncoderContext *encoder);
float Encoder_GetCountInCPR_Ratio(const EncoderContext *encoder);

#endif
