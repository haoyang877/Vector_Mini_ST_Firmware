#ifndef __ENCODER_H__
#define __ENCODER_H__

#include <stdbool.h>
#include <stdint.h>
#include "main.h"
#include "data_type.h"

/* MT6701 single-turn angle representation: unsigned Q15, [0, 65535]. */
#define ENCODER_Q15_CPR                 65536UL
#define ENCODER_Q15_HALF_TURN           32768
#define ENCODER_OFFSET_LUT_SIZE          1024U
#define ENCODER_OFFSET_LUT_BITS          10U
#define ENCODER_VELOCITY_WINDOW          16U
#define ENCODER_BAD_FRAME_OFFLINE_COUNT  100U

typedef enum
{
	ENCODER_READ_OK = 0,
	ENCODER_READ_SPI_TIMEOUT = 1,
	ENCODER_READ_CRC_MISMATCH = 2,
	ENCODER_READ_MAGNET_TOO_STRONG = 3,
	ENCODER_READ_MAGNET_TOO_WEAK = 4,
	ENCODER_READ_MAGNET_INVALID = 5,
	ENCODER_READ_OVERSPEED = 6
} Encoder_ReadStatus;

/* Calibration flags stored in flash. Mechanical zero is optional feedback state. */
#define ENC_CALIB_LINEARIZED        (1U << 0)
#define ENC_CALIB_ELECTRICAL_ZERO   (1U << 1)
#define ENC_CALIB_MECHANICAL_ZERO   (1U << 2)
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

	/* Raw MT6701 reading, direction-corrected input, and LUT-corrected angle. */
	uint16_t raw_q15;
	uint16_t directed_q15;
	uint16_t linearized_q15;
	uint16_t previous_linearized_q15;
	int64_t shadow_q15;
	int64_t mechanical_zero_shadow_q15;
	int64_t velocity_shadow_q15;
	bool has_valid_sample;

	/* Mechanical and electrical feedback exposed to the FOC/control interfaces. */
	float theta_elec;
	float vel_elec;
	float theta_mech;
	float vel_mech;

	/* 2 kHz moving-average mechanical velocity estimator. */
	uint8_t velocity_divider;
	uint8_t velocity_history_index;
	uint8_t velocity_sample_count;
	bool velocity_ready;
	int32_t velocity_delta_history[ENCODER_VELOCITY_WINDOW];
	int32_t velocity_delta_sum;

	/* MT6701 frame diagnostics and online state. */
	Encoder_ReadStatus read_status;
	Encoder_ReadStatus read_status_latched;
	uint8_t mt6701_status_bits;
	uint8_t mt6701_crc_received;
	uint8_t mt6701_crc_calculated;
	uint32_t mt6701_crc_error_count;
	uint32_t read_error_count;
	uint16_t bad_frame_streak;
} Encoder_TypeDef;

#define ext_enc_spi  hspi1
#define EXT_ENC_SPI  SPI1

#define EXT_ENC_CS_ENABLE  EXT_ENC_CS_GPIO_Port->BSRR = (uint32_t)EXT_ENC_CS_Pin << 16U
#define EXT_ENC_CS_DISABLE EXT_ENC_CS_GPIO_Port->BSRR = EXT_ENC_CS_Pin

void Encoder_ParamInit(Encoder_TypeDef *Encoder);
void Encoder_Update(MotorControl_TypeDef *MotorControl, Encoder_TypeDef *Encoder);

bool Encoder_IsOnline(const Encoder_TypeDef *Encoder);
bool Encoder_SetElectricalZero(Encoder_TypeDef *Encoder);
bool Encoder_SetElectricalZeroQ15(Encoder_TypeDef *Encoder, uint16_t electrical_zero_q15);
bool Encoder_SetMechanicalZero(Encoder_TypeDef *Encoder);
void Encoder_SetReverse(Encoder_TypeDef *Encoder, bool reverse);
void Encoder_ResetVelocity(Encoder_TypeDef *Encoder);

float Encoder_GetEleVel(const Encoder_TypeDef *Encoder);
float Encoder_GetMecVel(const Encoder_TypeDef *Encoder);
float Encoder_GetElePhase(const Encoder_TypeDef *Encoder);
float Encoder_GetMecPos(const Encoder_TypeDef *Encoder);
float Encoder_GetCountInCPR_Ratio(const Encoder_TypeDef *Encoder);


#endif
