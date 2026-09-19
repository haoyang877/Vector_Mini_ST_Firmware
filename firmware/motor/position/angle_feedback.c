#include "angle_feedback.h"

#include <limits.h>
#include <string.h>
#include "control_config.h"
#include "critical_hw.h"
#include "encoder_sensor.h"
#include "utils.h"

/* 角度反馈（motor 层拥有）：帧读取编排、方向/线性化/电零位/多圈累积与速度估计。
 * 传感器通道见 platform/api/encoder_sensor.h，SPI 传输见
 * platform/stm32g4/ports/motor/encoder_spi_stm32g4.c；本文件不访问寄存器、片选、
 * SPI 句柄或板级宏。 */

#define ENCODER_VELOCITY_ZERO_THRESHOLD_Q15 8

/* 记录一次读取结果；出现有效帧时清零连续坏帧计数。 */
static void Encoder_MarkReadStatus(Encoder_TypeDef *encoder, Encoder_ReadStatus status)
{
    encoder->read_status = status;
    if (status == ENCODER_READ_OK)
    {
        encoder->bad_frame_streak = 0U;
        return;
    }

    encoder->read_status_latched = status;
    encoder->read_error_count++;
    if (encoder->bad_frame_streak < UINT16_MAX)
    {
        encoder->bad_frame_streak++;
    }
}

/* 非阻塞发起一次传感器采样；总线异常时保留同步兜底路径。 */
bool Encoder_BeginSample(void)
{
    return encoder_sensor_begin();
}

/* 通道状态映射到既有对外读取状态；旧枚举保留以兼容遥测。 */
static Encoder_ReadStatus Encoder_MapSensorStatus(EncoderSensorStatus status)
{
    switch (status)
    {
    case ENCODER_SENSOR_FRAME_ERROR:
        return ENCODER_READ_CRC_MISMATCH;
    case ENCODER_SENSOR_SENSOR_FAULT:
        return ENCODER_READ_TLE_SYSTEM_ERROR;
    case ENCODER_SENSOR_BUS_TIMEOUT:
    default:
        return ENCODER_READ_SPI_TIMEOUT;
    }
}

/* 采样一帧并取出归一化 Q15 角度；失败时登记映射后的读取状态。 */
static bool Encoder_ReadFrame(Encoder_TypeDef *encoder, uint16_t *raw_q15, bool started)
{
    EncoderSensorSample sample;

    if (encoder_sensor_complete(started, &sample) != ENCODER_SENSOR_OK)
    {
        Encoder_MarkReadStatus(encoder, Encoder_MapSensorStatus(sample.status));
        return false;
    }

    encoder->frame_word = sample.frame_word;
    encoder->safety_word = sample.safety_word;
    encoder->crc_received = 0U;
    encoder->crc_calculated = 0U;
    *raw_q15 = sample.angle_q15;
    Encoder_MarkReadStatus(encoder, ENCODER_READ_OK);
    return true;
}

/* 按方向配置取反单圈角度。 */
static uint16_t Encoder_ApplyDirectionQ15(const Encoder_TypeDef *encoder, uint16_t raw_q15)
{
    if (encoder->reverse == 0U)
    {
        return raw_q15;
    }

    return (uint16_t)(0U - raw_q15);
}

/* 用 1024 点 Q15 表做线性插值校正。 */
static uint16_t Encoder_ApplyLinearizationQ15(const Encoder_TypeDef *encoder, uint16_t raw_q15)
{
    uint16_t lut_index = raw_q15 >> 6;
    uint16_t fraction = raw_q15 & 0x003FU;
    int32_t correction_a = encoder->linearization_lut_q15[lut_index];
    int32_t correction_b =
        encoder->linearization_lut_q15[(lut_index + 1U) & (ENCODER_OFFSET_LUT_SIZE - 1U)];
    int32_t correction = correction_a + (((correction_b - correction_a) * fraction) >> 6);

    return (uint16_t)((int32_t)raw_q15 - correction);
}

void Encoder_ResetVelocity(Encoder_TypeDef *encoder)
{
    memset(encoder->velocity_delta_history, 0, sizeof(encoder->velocity_delta_history));
    encoder->velocity_divider = 0U;
    encoder->velocity_history_index = 0U;
    encoder->velocity_sample_count = 0U;
    encoder->velocity_delta_sum = 0;
    encoder->velocity_ready = false;
    encoder->velocity_shadow_q15 = encoder->shadow_q15;
    encoder->vel_mech = 0.0f;
    encoder->vel_mech_continuous = 0.0f;
    encoder->vel_elec = 0.0f;
}

void Encoder_SetReverse(Encoder_TypeDef *encoder, bool reverse)
{
    uint8_t reverse_value = reverse ? 1U : 0U;
    uint32_t primask;

    if (encoder->reverse == reverse_value)
    {
        return;
    }

    primask = critical_hw_enter();
    encoder->reverse = reverse_value;
    encoder->electrical_zero_q15 = 0U;
    encoder->mechanical_zero_q15 = 0U;
    encoder->calib_flag = 0U;
    memset(encoder->linearization_lut_q15, 0, sizeof(encoder->linearization_lut_q15));
    encoder->raw_q15 = 0U;
    encoder->directed_q15 = 0U;
    encoder->linearized_q15 = 0U;
    encoder->previous_linearized_q15 = 0U;
    encoder->shadow_q15 = 0;
    encoder->mechanical_zero_shadow_q15 = 0;
    encoder->has_valid_sample = false;
    encoder->theta_elec = 0.0f;
    encoder->theta_mech = 0.0f;
    Encoder_ResetVelocity(encoder);
    critical_hw_exit(primask);
}

/* 2 kHz 分频的 16 样本滑动平均速度估计。 */
static void Encoder_UpdateVelocity2kHz(Encoder_TypeDef *encoder, uint32_t pole_pairs)
{
    int64_t delta64;
    int32_t delta_q15;
    int32_t sum_abs;
    float velocity_scale;

    if (++encoder->velocity_divider < SPEED_LOOP_DIVIDER)
    {
        return;
    }
    encoder->velocity_divider = 0U;

    delta64 = encoder->shadow_q15 - encoder->velocity_shadow_q15;
    encoder->velocity_shadow_q15 = encoder->shadow_q15;
    if (delta64 > INT32_MAX)
    {
        delta_q15 = INT32_MAX;
    }
    else if (delta64 < INT32_MIN)
    {
        delta_q15 = INT32_MIN;
    }
    else
    {
        delta_q15 = (int32_t)delta64;
    }

    encoder->velocity_delta_sum -= encoder->velocity_delta_history[encoder->velocity_history_index];
    encoder->velocity_delta_history[encoder->velocity_history_index] = delta_q15;
    encoder->velocity_delta_sum += delta_q15;
    encoder->velocity_history_index =
        (uint8_t)((encoder->velocity_history_index + 1U) % ENCODER_VELOCITY_WINDOW);
    if (encoder->velocity_sample_count < ENCODER_VELOCITY_WINDOW)
    {
        encoder->velocity_sample_count++;
    }
    encoder->velocity_ready = encoder->velocity_sample_count == ENCODER_VELOCITY_WINDOW;
    velocity_scale = _2PI / ((float)ENCODER_Q15_CPR * (float)ENCODER_VELOCITY_WINDOW * Speed_Ts);
    encoder->vel_mech_continuous =
        encoder->velocity_ready ? (float)encoder->velocity_delta_sum * velocity_scale : 0.0f;

    sum_abs = encoder->velocity_delta_sum;
    if (sum_abs < 0)
    {
        sum_abs = -sum_abs;
    }
    if (!encoder->velocity_ready || sum_abs <= ENCODER_VELOCITY_ZERO_THRESHOLD_Q15)
    {
        encoder->vel_mech = 0.0f;
    }
    else
    {
        encoder->vel_mech = encoder->vel_mech_continuous;
    }
    encoder->vel_elec = encoder->vel_mech * (float)pole_pairs;
}

/* 由线性化角度与电零位得到电角度；机械角用多圈 shadow 与机械零位得到。 */
static void Encoder_UpdateAngles(Encoder_TypeDef *encoder, uint32_t pole_pairs)
{
    uint16_t electrical_q15;

    electrical_q15 =
        (uint16_t)((uint32_t)(uint16_t)(encoder->linearized_q15 - encoder->electrical_zero_q15) *
                   pole_pairs);
    encoder->theta_elec = (float)electrical_q15 * (_2PI / (float)ENCODER_Q15_CPR);
    encoder->theta_mech = (float)(encoder->shadow_q15 - encoder->mechanical_zero_shadow_q15) *
                          (_2PI / (float)ENCODER_Q15_CPR);
}

void Encoder_ParamInit(Encoder_TypeDef *encoder)
{
    encoder->reverse = encoder->reverse != 0U ? 1U : 0U;
    encoder->raw_q15 = 0U;
    encoder->directed_q15 = 0U;
    encoder->linearized_q15 = 0U;
    encoder->previous_linearized_q15 = 0U;
    encoder->shadow_q15 = 0;
    encoder->mechanical_zero_shadow_q15 = (int64_t)encoder->mechanical_zero_q15;
    encoder->velocity_shadow_q15 = 0;
    encoder->has_valid_sample = false;
    encoder->theta_elec = 0.0f;
    encoder->theta_mech = 0.0f;
    encoder->read_status = ENCODER_READ_OK;
    encoder->read_status_latched = ENCODER_READ_OK;
    encoder->frame_word = 0U;
    encoder->safety_word = 0U;
    encoder->crc_received = 0U;
    encoder->crc_calculated = 0U;
    encoder->crc_error_count = 0U;
    encoder->read_error_count = 0U;
    encoder->bad_frame_streak = 0U;
    Encoder_ResetVelocity(encoder);

    encoder_sensor_init();
}

bool Encoder_IsOnline(const Encoder_TypeDef *encoder)
{
    return encoder->has_valid_sample && encoder->bad_frame_streak < ENCODER_BAD_FRAME_OFFLINE_COUNT;
}

bool Encoder_SetElectricalZeroQ15(Encoder_TypeDef *encoder, uint16_t electrical_zero_q15)
{
    if (!Encoder_IsOnline(encoder))
    {
        return false;
    }

    encoder->electrical_zero_q15 = electrical_zero_q15;
    encoder->calib_flag |= ENC_CALIB_ELECTRICAL_ZERO;
    return true;
}

bool Encoder_SetElectricalZero(Encoder_TypeDef *encoder)
{
    return Encoder_SetElectricalZeroQ15(encoder, encoder->linearized_q15);
}

bool Encoder_SetMechanicalZero(Encoder_TypeDef *encoder)
{
    if (!Encoder_IsOnline(encoder))
    {
        return false;
    }

    encoder->mechanical_zero_q15 = encoder->linearized_q15;
    encoder->mechanical_zero_shadow_q15 = encoder->shadow_q15;
    encoder->calib_flag |= ENC_CALIB_MECHANICAL_ZERO;
    encoder->theta_mech = 0.0f;
    return true;
}

/* 完成一帧读取：方向校正、LUT 校正、多圈累积、速度与角度更新。 */
void Encoder_CompleteSample(MotorControl_TypeDef *MotorControl,
                            Encoder_TypeDef *encoder,
                            bool sample_started)
{
    uint16_t raw_q15;
    uint16_t directed_q15;
    uint16_t linearized_q15;
    int32_t delta_q15;
    uint32_t pole_pairs;

    if (!Encoder_ReadFrame(encoder, &raw_q15, sample_started))
    {
        return;
    }

    directed_q15 = Encoder_ApplyDirectionQ15(encoder, raw_q15);
    linearized_q15 = Encoder_ApplyLinearizationQ15(encoder, directed_q15);
    encoder->raw_q15 = raw_q15;
    encoder->directed_q15 = directed_q15;
    encoder->linearized_q15 = linearized_q15;
    pole_pairs = MotorControl->motor_pole_pairs > 0 ? (uint32_t)MotorControl->motor_pole_pairs : 1U;

    if (!encoder->has_valid_sample)
    {
        encoder->previous_linearized_q15 = linearized_q15;
        encoder->shadow_q15 = linearized_q15;
        if ((encoder->calib_flag & ENC_CALIB_MECHANICAL_ZERO) == 0U)
        {
            encoder->mechanical_zero_shadow_q15 = 0;
        }
        else
        {
            encoder->mechanical_zero_shadow_q15 = (int64_t)encoder->mechanical_zero_q15;
            /* 只解析首样本的圈数；便携轴策略保留已标定零位并保持行程检查有效。 */
            encoder->shadow_q15 =
                encoder->mechanical_zero_shadow_q15 +
                MotorAxisProfile_InitialEncoderOffsetQ15(&MotorControl->axis_profile,
                                                         MotorControl->axis_profile_valid,
                                                         (int32_t)linearized_q15 -
                                                             (int32_t)encoder->mechanical_zero_q15);
        }
        encoder->has_valid_sample = true;
        Encoder_ResetVelocity(encoder);
        Encoder_UpdateAngles(encoder, pole_pairs);
        return;
    }

    delta_q15 = (int32_t)linearized_q15 - (int32_t)encoder->previous_linearized_q15;
    if (delta_q15 > ENCODER_Q15_HALF_TURN)
    {
        delta_q15 -= (int32_t)ENCODER_Q15_CPR;
    }
    else if (delta_q15 < -ENCODER_Q15_HALF_TURN)
    {
        delta_q15 += (int32_t)ENCODER_Q15_CPR;
    }

    encoder->previous_linearized_q15 = linearized_q15;
    encoder->shadow_q15 += delta_q15;
    Encoder_UpdateVelocity2kHz(encoder, pole_pairs);
    Encoder_UpdateAngles(encoder, pole_pairs);
}

void Encoder_Update(MotorControl_TypeDef *MotorControl, Encoder_TypeDef *encoder)
{
    Encoder_CompleteSample(MotorControl, encoder, false);
}

float Encoder_GetElePhase(const Encoder_TypeDef *encoder)
{
    return encoder->theta_elec;
}

float Encoder_GetMecPos(const Encoder_TypeDef *encoder)
{
    return encoder->theta_mech;
}

float Encoder_GetEleVel(const Encoder_TypeDef *encoder)
{
    return encoder->vel_elec;
}

float Encoder_GetMecVel(const Encoder_TypeDef *encoder)
{
    return encoder->vel_mech;
}

float Encoder_GetMecVelContinuous(const Encoder_TypeDef *encoder)
{
    return encoder->vel_mech_continuous;
}

bool Encoder_DidUpdateVelocity(const Encoder_TypeDef *encoder)
{
    return encoder->bad_frame_streak == 0U && encoder->velocity_sample_count > 0U &&
           encoder->velocity_divider == 0U;
}

float Encoder_GetCountInCPR_Ratio(const Encoder_TypeDef *encoder)
{
    return (float)encoder->linearized_q15 / (float)ENCODER_Q15_CPR;
}

uint8_t Encoder_GetCalibFlag(const Encoder_TypeDef *encoder)
{
    return encoder->calib_flag;
}

uint16_t Encoder_GetBadFrameStreak(const Encoder_TypeDef *encoder)
{
    return encoder->bad_frame_streak;
}

uint8_t Encoder_GetReverse(const Encoder_TypeDef *encoder)
{
    return encoder->reverse;
}
