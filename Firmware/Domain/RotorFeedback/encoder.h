#ifndef DOMAIN_ENCODER_H
#define DOMAIN_ENCODER_H

#include <stdbool.h>
#include <stdint.h>
#include "critical_section_port.h"
#include "rotor_sensor_port.h"

/* TLE5012B single-turn angle representation: unsigned Q15, [0, 65535]. */
#define ENCODER_Q15_CPR                 65536UL
#define ENCODER_Q15_HALF_TURN           32768
#define ENCODER_OFFSET_LUT_SIZE          1024U
#define ENCODER_OFFSET_LUT_BITS          10U
#define ENCODER_VELOCITY_WINDOW          16U
#define ENCODER_BAD_FRAME_OFFLINE_COUNT  100U
#define ENCODER_COGGING_MAP_SIZE          128U

typedef enum
{
	ENCODER_READ_OK = 0,
	ENCODER_READ_SPI_TIMEOUT = 1,
	ENCODER_READ_CRC_MISMATCH = 2,
	ENCODER_READ_MAGNET_TOO_STRONG = 3,
	ENCODER_READ_MAGNET_TOO_WEAK = 4,
	ENCODER_READ_MAGNET_INVALID = 5,
	ENCODER_READ_OVERSPEED = 6,
	ENCODER_READ_TLE_RESET = 7,
	ENCODER_READ_TLE_SYSTEM_ERROR = 8,
	ENCODER_READ_TLE_INTERFACE_ERROR = 9,
	ENCODER_READ_TLE_INVALID_ANGLE = 10
} Encoder_ReadStatus;

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

	/* Raw TLE5012B reading, direction-corrected input, and LUT-corrected angle. */
	uint16_t raw_q15;
	uint16_t directed_q15;
	uint16_t linearized_q15;
	uint16_t previous_linearized_q15;
	int64_t shadow_q15;
	int64_t mechanical_zero_shadow_q15;
	int64_t velocity_shadow_q15;
	bool has_valid_sample;

	/* Mechanical and electrical feedback exposed to the CurrentControl/control interfaces. */
	float theta_elec;
	float vel_elec;
	float theta_mech;
	float vel_mech;

	/* 2 kHz moving-average mechanical velocity estimator. */
	uint8_t velocity_divider;
	uint8_t velocity_update_divider;
	float velocity_sample_period_s;
	uint8_t velocity_history_index;
	uint8_t velocity_sample_count;
	bool velocity_ready;
	int32_t velocity_delta_history[ENCODER_VELOCITY_WINDOW];
	int32_t velocity_delta_sum;

	/* TLE5012B SSC frame diagnostics and online state. */
	Encoder_ReadStatus read_status;
	Encoder_ReadStatus read_status_latched;
	uint16_t tle5012_angle_word;
	uint16_t tle5012_safety_word;
	uint8_t tle5012_crc_received;
	uint8_t tle5012_crc_calculated;
	uint32_t tle5012_crc_error_count;
	uint32_t read_error_count;
	uint16_t bad_frame_streak;
	RotorSensorPort sensor_port;
	CriticalSectionPort critical_section;
} EncoderContext;

bool Encoder_ParamInit(EncoderContext *Encoder,
	const RotorSensorPort *sensor_port,
	const CriticalSectionPort *critical_section,
	uint8_t velocity_update_divider,
	float velocity_sample_period_s);
void Encoder_Update(EncoderContext *Encoder, uint32_t pole_pairs);

bool Encoder_IsOnline(const EncoderContext *Encoder);
bool Encoder_SetElectricalZero(EncoderContext *Encoder);
bool Encoder_SetElectricalZeroQ15(EncoderContext *Encoder, uint16_t electrical_zero_q15);
bool Encoder_SetMechanicalZero(EncoderContext *Encoder);
void Encoder_SetReverse(EncoderContext *Encoder, bool reverse);
void Encoder_ResetVelocity(EncoderContext *Encoder);

float Encoder_GetEleVel(const EncoderContext *Encoder);
float Encoder_GetMecVel(const EncoderContext *Encoder);
float Encoder_GetElePhase(const EncoderContext *Encoder);
float Encoder_GetMecPos(const EncoderContext *Encoder);
float Encoder_GetCountInCPR_Ratio(const EncoderContext *Encoder);

#endif
