/* 编码器快环接口回归：校准借用、双实例、动态校准和极对数边界。 */
#include <assert.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "encoder_feedback.h"
#include "control_math.h"
#include "motor_config.h"

static uint16_t s_raw_q15;

uint16_t BspEncoder_ReadAngle(void)
{
    return s_raw_q15;
}

static void update(MotorConfiguration *configuration,
                   EncoderState *encoder,
                   int32_t pole_pairs,
                   uint16_t raw_q15)
{
    s_raw_q15 = raw_q15;
    configuration->model.motor_pole_pairs = pole_pairs;
    Encoder_Update(encoder);
    Encoder_UpdateSlowEstimate(encoder);
    /* 只输出行为字段，供修改前后逐拍对照；不含借用指针或结构填充。 */
    printf("%u %u %u %u %u %lld %lld %a %a %a %a %a %a\n",
           encoder->raw_q15,
           encoder->directed_q15,
           encoder->linearized_q15,
           encoder->sample_epoch,
           encoder->slow_sample_epoch,
           (long long)encoder->shadow_q15,
           (long long)encoder->mechanical_zero_shadow_q15,
           (double)encoder->electrical_angle_rad,
           (double)encoder->mechanical_position_rad,
           (double)encoder->electrical_velocity_rad_s,
           (double)encoder->mechanical_velocity_rad_s,
           (double)encoder->pll_theta_rad,
           (double)encoder->pll_omega_rad_s);
}

static void test_slow_estimate(void)
{
    MotorConfiguration configuration = {0};
    EncoderState encoder = {0};
    EncoderCalibration *calibration = &configuration.device.encoder;
    MotorAxisProfile *profile = &configuration.application.axis_profile;
    profile->magic = MOTOR_AXIS_PROFILE_NUMERIC_MAGIC;
    profile->version = MOTOR_AXIS_PROFILE_NUMERIC_VERSION;
    profile->identity.numeric.joint_type = MOTOR_JOINT_ROLL;
    profile->minimum_position_rad = -1.5f;
    profile->maximum_position_rad = 1.5f;
    configuration.application.axis_profile_valid = true;
    calibration->mechanical_zero_q15 = 65000U;
    calibration->calib_flag = ENC_CALIB_MECHANICAL_ZERO;
    Encoder_InitFeedback(calibration,
                         &configuration.model.motor_pole_pairs,
                         profile,
                         &configuration.application.axis_profile_valid,
                         &encoder);

    /* 有效轴在首帧选择机械零位附近的分支。 */
    update(&configuration, &encoder, 7, 4000U);
    assert(encoder.shadow_q15 - encoder.mechanical_zero_shadow_q15 == 4536);

    /* 原位修改有效标志与轴类型，只在下一次请求重定基准时改变分支。 */
    configuration.application.axis_profile_valid = false;
    encoder.rebase_requested = true;
    update(&configuration, &encoder, 7, 4000U);
    assert(encoder.shadow_q15 - encoder.mechanical_zero_shadow_q15 == -61000);
    configuration.application.axis_profile_valid = true;
    profile->identity.numeric.joint_type = MOTOR_JOINT_UNKNOWN;
    encoder.rebase_requested = true;
    update(&configuration, &encoder, 7, 4000U);
    assert(encoder.shadow_q15 - encoder.mechanical_zero_shadow_q15 == -61000);
    profile->identity.numeric.joint_type = MOTOR_JOINT_PITCH;
    encoder.rebase_requested = true;
    update(&configuration, &encoder, 7, 4000U);
    assert(encoder.shadow_q15 - encoder.mechanical_zero_shadow_q15 == 4536);

    /* 运行途中修改轴范围不会隐式重定位置；显式请求才读取新范围。 */
    profile->minimum_position_rad = -4.0f;
    update(&configuration, &encoder, 7, 4000U);
    assert(encoder.shadow_q15 - encoder.mechanical_zero_shadow_q15 == 4536);
    encoder.rebase_requested = true;
    update(&configuration, &encoder, 7, 4000U);
    assert(encoder.shadow_q15 - encoder.mechanical_zero_shadow_q15 == -61000);

    profile->minimum_position_rad = -1.5f;
    calibration->mechanical_zero_q15 = 500U;
    encoder.rebase_requested = true;
    update(&configuration, &encoder, 7, 65000U);
    assert(encoder.shadow_q15 - encoder.mechanical_zero_shadow_q15 == -1036);
    update(&configuration, &encoder, 7, 65530U);
    int64_t before_wrap = encoder.shadow_q15;
    update(&configuration, &encoder, 7, 10U);
    assert(encoder.shadow_q15 - before_wrap == 16);

    /* 没有新快拍也须消费归零请求，但不更新采样代际或速度。 */
    uint32_t epoch = encoder.slow_sample_epoch;
    float velocity_rad_s = encoder.mechanical_velocity_rad_s;
    encoder.zero_requested = true;
    Encoder_UpdateSlowEstimate(&encoder);
    assert(!encoder.zero_requested && encoder.mechanical_position_rad == 0.0f);
    assert(encoder.slow_sample_epoch == epoch);
    assert(encoder.mechanical_velocity_rad_s == velocity_rad_s);
    Encoder_UpdateSlowEstimate(&encoder);
    assert(encoder.slow_sample_epoch == epoch && encoder.mechanical_position_rad == 0.0f);

    calibration->calib_flag = 0U;
    encoder.rebase_requested = true;
    update(&configuration, &encoder, 7, 65000U);
    assert(encoder.shadow_q15 == 65000 && encoder.mechanical_zero_shadow_q15 == 0);
}

int main(void)
{
    MotorConfiguration first_configuration = {0};
    MotorConfiguration second_configuration = {0};
    EncoderState first = {0};
    EncoderState second = {0};
    EncoderCalibration *calibration = &first_configuration.device.encoder;

    calibration->electrical_zero_q15 = 32U;
    calibration->mechanical_zero_q15 = 65000U;
    calibration->calib_flag = ENC_CALIB_ALL | ENC_CALIB_MECHANICAL_ZERO;
    second_configuration.device.encoder.reverse = 2U;
    for (unsigned index = 0; index < ENCODER_OFFSET_LUT_SIZE; ++index)
    {
        second_configuration.device.encoder.linearization_lut_q15[index] = 4;
    }
    Encoder_InitFeedback(calibration,
                         &first_configuration.model.motor_pole_pairs,
                         &first_configuration.application.axis_profile,
                         &first_configuration.application.axis_profile_valid,
                         &first);
    Encoder_InitFeedback(&second_configuration.device.encoder,
                         &second_configuration.model.motor_pole_pairs,
                         &second_configuration.application.axis_profile,
                         &second_configuration.application.axis_profile_valid,
                         &second);
    assert(first.calibration == calibration);
    assert(second.calibration == &second_configuration.device.encoder);
    assert(first.pole_pairs == &first_configuration.model.motor_pole_pairs);
    assert(second.pole_pairs == &second_configuration.model.motor_pole_pairs);
    assert(first.axis_profile == &first_configuration.application.axis_profile);
    assert(first.axis_profile_valid == &first_configuration.application.axis_profile_valid);
    assert(second.axis_profile == &second_configuration.application.axis_profile);
    assert(second.axis_profile_valid == &second_configuration.application.axis_profile_valid);
    assert(second_configuration.device.encoder.reverse == 1U);

    update(&first_configuration, &first, 7, 64U);
    assert(first.linearized_q15 == 64U && first.velocity_ready);
    assert(first.electrical_angle_rad == 224.0f * (_2PI / 65536.0f));
    assert(first.mechanical_zero_shadow_q15 == 65000);
    assert(first.mechanical_velocity_rad_s == 0.0f);
    update(&second_configuration, &second, 21, 64U);
    assert(second.directed_q15 == 65472U && second.linearized_q15 == 65468U);
    assert(first.linearized_q15 == 64U);

    /* 初始化后原位修改校准，下拍直接读取新值。 */
    calibration->electrical_zero_q15 = 60U;
    for (unsigned index = 0; index < ENCODER_OFFSET_LUT_SIZE; ++index)
    {
        calibration->linearization_lut_q15[index] = 8;
    }
    update(&first_configuration, &first, 7, 72U);
    assert(first.linearized_q15 == 64U);
    assert(first.electrical_angle_rad == 28.0f * (_2PI / 65536.0f));
    assert(Encoder_SetElectricalZero(calibration, &first));
    update(&first_configuration, &first, 7, 72U);
    assert(first.electrical_angle_rad == 0.0f);

    for (uint16_t angle = 100U; angle < 900U; angle += 20U)
    {
        update(&first_configuration, &first, 7, angle);
        assert(first.electrical_velocity_rad_s == first.mechanical_velocity_rad_s * 7.0f);
    }
    assert(first.mechanical_velocity_rad_s > 0.0f);

    /* 无效极对数保持旧的 1 对极回退，全部反馈及请求状态均一致。 */
    const int32_t invalid_poles[] = {0, -7, INT32_MIN};
    for (unsigned index = 0; index < sizeof(invalid_poles) / sizeof(invalid_poles[0]); ++index)
    {
        EncoderState expected = first;
        EncoderState actual = first;
        update(&first_configuration, &expected, 1, 900U);
        update(&first_configuration, &actual, invalid_poles[index], 900U);
        assert(memcmp(&actual, &expected, sizeof(actual)) == 0);
    }
    update(&first_configuration, &first, INT32_MAX, 900U);

    Encoder_ResetVelocity(&first);
    update(&first_configuration, &first, 7, 1000U);
    assert(first.mechanical_velocity_rad_s == 0.0f && !first.velocity_restart_requested);
    assert(Encoder_SetMechanicalZero(calibration, &first));
    update(&first_configuration, &first, 7, 1000U);
    assert(first.mechanical_position_rad == 0.0f);

    Encoder_SetReverse(calibration, &first, true);
    assert(first.calibration == calibration);
    update(&first_configuration, &first, 7, 64U);
    assert(first.directed_q15 == 65472U && first.linearized_q15 == 65472U);
    assert(first.mechanical_velocity_rad_s == 0.0f);

    /* 重新初始化时改绑另一实例的校准，旧校准不再参与后续采样。 */
    Encoder_InitFeedback(&second_configuration.device.encoder,
                         &second_configuration.model.motor_pole_pairs,
                         &second_configuration.application.axis_profile,
                         &second_configuration.application.axis_profile_valid,
                         &first);
    update(&second_configuration, &first, 21, 64U);
    assert(first.linearized_q15 == second.linearized_q15);
    assert(first.calibration == second.calibration);
    assert(first.pole_pairs == second.pole_pairs);
    assert(first.axis_profile == second.axis_profile);
    assert(first.axis_profile_valid == second.axis_profile_valid);
    test_slow_estimate();
    puts("PASS encoder feedback: live bindings, poles, rebase, wrap, zero, reset and rebind");
    return 0;
}
