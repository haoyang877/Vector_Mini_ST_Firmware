/* 完整生产 Execution 与原速度 PI/斜坡；桩只模拟位置核心和反馈/FOC 边界。
 * 位置核心数学由 I 的独立夹具验证，此处检查配置、目标边界及发布/故障/复位合同。 */
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "motor_execution.h"
#include "foc_speed.h"
#include "position_cascade_config.h"
#include "control_config.h"

static MotorControlState s_motor;
static PiController s_pi;
static EncoderState s_encoder;
static PositionCascadeConfig s_config;
static float s_position_rad, s_speed_rad_s;
static float s_output_iq_a = 99.0f;
static bool s_core_valid = true;
static unsigned s_updates, s_resets, s_current_calls;
static float s_phase_rad, s_phase_velocity_rad_s;
static unsigned s_failures;

#define CHECK(condition)                                                                           \
    do                                                                                             \
    {                                                                                              \
        if (!(condition))                                                                          \
        {                                                                                          \
            ++s_failures;                                                                          \
            printf("FAIL %u: %s\n", __LINE__, #condition);                                         \
        }                                                                                          \
    } while (0)

float Encoder_GetMecPos(const EncoderState *encoder)
{
    (void)encoder;
    return s_position_rad;
}
float Encoder_GetMecVel(const EncoderState *encoder)
{
    (void)encoder;
    return s_speed_rad_s;
}
float Encoder_GetMecVelContinuous(const EncoderState *encoder)
{
    (void)encoder;
    return s_speed_rad_s;
}
float Encoder_GetElePhase(const EncoderState *encoder)
{
    (void)encoder;
    return 0.75f;
}
float Encoder_GetEleVel(const EncoderState *encoder)
{
    (void)encoder;
    return 2.5f;
}
bool Encoder_IsOnline(const EncoderState *encoder)
{
    return encoder->has_valid_sample;
}
uint8_t Encoder_GetCalibFlag(const EncoderState *encoder)
{
    return encoder->calib_flag;
}
float Observer_GetElePhase(FluxObserverState *observer)
{
    return observer->electrical_angle_rad;
}
float Observer_GetEleVel(FluxObserverState *observer)
{
    return observer->electrical_velocity_rad_s;
}
void MotorFault_Report(MotorControlState *motor, MotorFaultCode fault)
{
    if (motor->fault_code == MOTOR_FAULT_NONE)
    {
        motor->fault_code = fault;
    }
}
float FOC_MaximumVoltage(float voltage_v)
{
    return isfinite(voltage_v) && voltage_v > 0.0f ? voltage_v * 0.5f : 0.0f;
}
void FOC_Current(FocState *foc, MotorControlState *motor, float phase, float speed)
{
    (void)foc;
    (void)motor;
    ++s_current_calls;
    s_phase_rad = phase;
    s_phase_velocity_rad_s = speed;
}
void FOC_Vq_Mode(FocState *foc, MotorControlState *motor, float phase, float speed)
{
    FOC_Current(foc, motor, phase, speed);
}
void PositionCascade_Reset(void)
{
    ++s_resets;
}
bool PositionCascade_UpdateControl(const PositionCascadeConfig *config,
                                   float position,
                                   float speed,
                                   PositionCascadeControlOutput *output)
{
    ++s_updates;
    s_config = *config;
    if (!s_core_valid)
    {
        return false;
    }
    *output =
        (PositionCascadeControlOutput){config->target_position, 0.2f, speed, s_output_iq_a, false};
    CHECK(position == s_position_rad);
    return true;
}

static void setup(MotorControlMode mode)
{
    memset(&s_motor, 0, sizeof(s_motor));
    memset(&s_pi, 0, sizeof(s_pi));
    s_position_rad = 0.3f;
    s_speed_rad_s = 0.0f;
    s_core_valid = true;
    s_motor.control_mode = mode;
    s_motor.application.axis_profile_valid = true;
    s_motor.application.current_limit_a = 4.0f;
    s_motor.application.speed_kp = 0.5f;
    s_motor.application.speed_ki = 2.0f;
    s_motor.application.speed_acceleration_rad_s2 = 0.8f;
    s_motor.application.speed_deceleration_rad_s2 = 0.5f;
    s_motor.application.position_error_window_rad = 0.001f;
    s_motor.application.position_maximum_speed_rad_s = 0.6f;
    s_motor.application.speed_limit_rad_s = 0.8f;
    s_motor.application.cascade_position_kp = 8.0f;
    s_motor.application.cascade_position_kd = 2.0f;
    s_motor.position_target_rad = 0.3f;
    s_motor.speed_target_rad_s = 0.2f;
    MotorOuterLoop_RequestReset();
}
static void tick(void)
{
    MotorOuterLoop_SlowTick(&s_motor, &s_pi, &s_encoder);
}

static void check_configuration(void)
{
    setup(MOTOR_CONTROL_POSITION);
    tick();
    CHECK(MotorOuterLoop_IsReady() && s_motor.iq_target_a == 4.0f);
    CHECK(s_config.call_divider == 1U && s_config.update_period_s == Speed_Ts);
    CHECK(s_config.target_position == 0.3f &&
          s_config.velocity_filter_hz == POSITION_SERVO_VELOCITY_FILTER_HZ);
    s_motor.position_target_rad = -0.4f;
    s_motor.application.current_limit_a = 1.0f;
    s_motor.application.speed_kp = 0.7f;
    s_motor.application.axis_profile.magic = 1U;
    s_motor.application.axis_profile.minimum_position_rad = -1.0f;
    s_motor.application.axis_profile.maximum_position_rad = 1.0f;
    s_motor.application.axis_profile.maximum_speed_rad_s = 0.4f;
    tick();
    CHECK(s_config.target_position == -0.4f && s_config.speed_kp == 0.7f);
    CHECK(s_config.maximum_speed == 0.4f && s_motor.iq_target_a == 1.0f);
    CHECK(s_motor.position_reference_rad == -0.4f &&
          isnan(s_motor.position_trajectory_speed_rad_s));
}
static void check_validation(void)
{
    const size_t offsets[] = {offsetof(MotorControlState, application.current_limit_a),
                              offsetof(MotorControlState, application.speed_kp),
                              offsetof(MotorControlState, application.speed_ki),
                              offsetof(MotorControlState, application.position_error_window_rad),
                              offsetof(MotorControlState, application.position_maximum_speed_rad_s),
                              offsetof(MotorControlState, application.speed_limit_rad_s),
                              offsetof(MotorControlState, application.cascade_position_kp),
                              offsetof(MotorControlState, application.cascade_position_kd)};
    for (unsigned i = 0U; i < sizeof(offsets) / sizeof(offsets[0]); ++i)
    {
        setup(MOTOR_CONTROL_POSITION);
        tick();
        float invalid = NAN;
        memcpy((char *)&s_motor + offsets[i], &invalid, sizeof(invalid));
        tick();
        CHECK(s_motor.fault_code == MOTOR_FAULT_INVALID_PARAMETERS);
        CHECK(s_motor.iq_target_a == 0.0f && !MotorOuterLoop_IsReady());
    }
    setup(MOTOR_CONTROL_POSITION);
    s_core_valid = false;
    tick();
    CHECK(s_motor.fault_code == MOTOR_FAULT_INVALID_PARAMETERS && s_motor.iq_target_a == 0.0f);
    setup(MOTOR_CONTROL_POSITION);
    s_output_iq_a = NAN;
    tick();
    CHECK(s_motor.fault_code == MOTOR_FAULT_INVALID_PARAMETERS && s_motor.iq_target_a == 0.0f);
    s_output_iq_a = 99.0f;
    setup(MOTOR_CONTROL_SPEED);
    FocState foc = {0};
    MotorControlCommand command = {.mode = MOTOR_CONTROL_SPEED, .speed_rad_s = 0.2f};
    CHECK(MotorExecution_ValidateTarget(&command, &s_motor, &foc, &s_encoder) ==
          MOTOR_CONTROL_ACCEPTED);
    command.iq_a = 0.01f;
    CHECK(MotorExecution_ValidateTarget(&command, &s_motor, &foc, &s_encoder) ==
          MOTOR_CONTROL_INVALID);
    command.iq_a = 0.0f;
    command.speed_rad_s = 1.0f;
    CHECK(MotorExecution_ValidateTarget(&command, &s_motor, &foc, &s_encoder) ==
          MOTOR_CONTROL_OUT_OF_RANGE);
    command.speed_rad_s = NAN;
    CHECK(MotorExecution_ValidateTarget(&command, &s_motor, &foc, &s_encoder) ==
          MOTOR_CONTROL_INVALID);
    CHECK(MotorExecution_ValidateTarget(NULL, &s_motor, &foc, &s_encoder) == MOTOR_CONTROL_INVALID);
}
static void check_runtime(void)
{
    setup(MOTOR_CONTROL_SPEED);
    tick();
    MotorControlState expected = s_motor;
    PiController expected_pi = s_pi;
    for (unsigned i = 0U; i < 64U; ++i)
    {
        if (i % 7U == 0U)
        {
            s_motor.speed_target_rad_s = expected.speed_target_rad_s = -s_motor.speed_target_rad_s;
        }
        SpeedMode_UpdateControl(&expected, &expected_pi, s_speed_rad_s);
        tick();
        CHECK(s_motor.speed_ramped_rad_s == expected.speed_ramped_rad_s &&
              s_motor.iq_target_a == expected.iq_target_a);
        CHECK(memcmp(&s_pi, &expected_pi, sizeof(s_pi)) == 0);
    }
    for (unsigned mode = MOTOR_CONTROL_SPEED; mode <= MOTOR_CONTROL_POSITION; ++mode)
    {
        setup((MotorControlMode)mode);
        tick();
        unsigned resets = s_resets;
        MotorOuterLoop_RequestReset();
        CHECK(!MotorOuterLoop_IsReady());
        tick();
        CHECK(s_resets > resets && MotorOuterLoop_IsReady());
        s_motor.fault_code = MOTOR_FAULT_OVERCURRENT;
        s_motor.iq_target_a = 0.0f;
        tick();
        CHECK(s_motor.iq_target_a == 0.0f);
        setup((MotorControlMode)mode);
        s_speed_rad_s = NAN;
        tick();
        CHECK(s_motor.fault_code == MOTOR_FAULT_INVALID_PARAMETERS && s_motor.iq_target_a == 0.0f);
    }
    setup(MOTOR_CONTROL_CURRENT);
    FocState foc = {0};
    FluxObserverState observer = {0};
    Task_Current_Mode(&foc, &s_motor, &s_encoder, &observer);
    CHECK(s_current_calls == 1U && s_phase_rad == 0.75f && s_phase_velocity_rad_s == 2.5f);
    s_motor.uses_sensorless_feedback = true;
    observer.electrical_angle_rad = 1.25f;
    observer.electrical_velocity_rad_s = 5.0f;
    Task_Current_Mode(&foc, &s_motor, &s_encoder, &observer);
    CHECK(s_current_calls == 2U && s_phase_rad == 1.25f && s_phase_velocity_rad_s == 5.0f);
}

/** @brief 执行单项核心组合合同；失败返回非零，不接触硬件。 */
int main(int argc, char **argv)
{
    if (argc != 2)
    {
        return 2;
    }
    if (strcmp(argv[1], "configuration") == 0)
    {
        check_configuration();
    }
    else if (strcmp(argv[1], "validation") == 0)
    {
        check_validation();
    }
    else
    {
        check_runtime();
    }
    printf("core outer %s: %s\n", argv[1], s_failures == 0U ? "PASS" : "FAIL");
    return s_failures == 0U ? 0 : 1;
}
