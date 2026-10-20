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
#include "application_profile_defaults.h"

static MotorControlState s_motor;
static PiController s_pi;
static EncoderState s_encoder;
static PositionCascadeConfig s_config;
static float s_position_rad, s_speed_rad_s;
static float s_output_iq_a = 99.0f;
static bool s_core_valid = true;
static unsigned s_updates, s_resets, s_current_calls;
static float s_phase_rad, s_phase_velocity_rad_s;
static float s_available_current_limit_a;
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
float MotorThermalControl_CurrentLimit(float configured_a)
{
    return fminf(configured_a, s_available_current_limit_a);
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
bool PositionCascade_UpdateControlLimited(const PositionCascadeConfig *config,
                                          float position,
                                          float speed,
                                          float available_current_a,
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
    if (isfinite(output->iq_reference))
    {
        float limit_a = fminf(config->current_limit_a, available_current_a);
        output->iq_reference = fminf(fmaxf(output->iq_reference, -limit_a), limit_a);
    }
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
    s_available_current_limit_a = INFINITY;
    s_motor.control_mode = mode;
    s_motor.application.axis_profile_valid = true;
    s_motor.application.current_limit_a = 4.0f;
    s_motor.application.speed_kp = 0.5f;
    s_motor.application.speed_ki = 2.0f;
    s_motor.application.speed_acceleration_rad_s2 = 0.8f;
    s_motor.application.speed_deceleration_rad_s2 = 0.5f;
    s_motor.application.speed_feedforward =
        (SpeedFeedforwardConfig)PARAM_APP_FEEDFORWARD_INITIALIZER;
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
    s_available_current_limit_a = 0.5f;
    tick();
    CHECK(s_config.current_limit_a == 4.0f && s_motor.iq_target_a == 0.5f);
    s_available_current_limit_a = INFINITY;
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
    setup(MOTOR_CONTROL_POSITION);
    s_motor.position_target_rad = 0.6f;
    s_available_current_limit_a = 0.5f;
    tick();
    CHECK(MotorOuterLoop_IsReady() && s_motor.iq_target_a == 0.5f);
    CHECK(s_motor.application.current_limit_a == 4.0f);
    s_available_current_limit_a = 0.0f;
    tick();
    CHECK(MotorOuterLoop_IsReady() && s_motor.iq_target_a == 0.0f);
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

/* 复用冻结 5665ff87 的纯核合同：正反转、收敛、acc/jerk 与前馈限幅。 */
static void check_feedforward_core(void)
{
    SpeedFeedforwardConfig config;
    SpeedFeedforwardState state, previous;
    const float dt_s = 0.0005f;
    config = (SpeedFeedforwardConfig)PARAM_APP_FEEDFORWARD_INITIALIZER;
    CHECK(!config.enabled && SpeedFeedforward_IsValid(&config));
    config.enabled = true;
    config.gain_a_per_rad_s2 = 0.2f;
    SpeedFeedforward_Reset(&state);
    float peak_a = 0.0f;
    for (unsigned tick = 0U; tick < 40000U; ++tick)
    {
        float target = tick < 10000U ? 20.0f : tick < 20000U ? 0.0f : tick < 30000U ? -20.0f : 0.0f;
        previous = state;
        CHECK(SpeedFeedforward_Update(&state, &config, target, 0.0f, dt_s, 6.0f));
        CHECK(fabsf(state.acceleration_rad_s2) <= 20.0001f);
        CHECK(fabsf(state.acceleration_rad_s2 - previous.acceleration_rad_s2) <= 0.10001f);
        CHECK(fabsf(state.current_a) <= 2.00001f && isfinite(state.speed_rad_s));
        peak_a = fmaxf(peak_a, fabsf(state.current_a));
        if (tick % 10000U == 9999U)
        {
            CHECK(fabsf(state.speed_rad_s - target) < 0.001f);
            CHECK(fabsf(state.acceleration_rad_s2) < 0.001f);
        }
    }
    CHECK(peak_a > 1.9f);
    for (unsigned tick = 0U; tick < 2000U; ++tick)
    {
        previous = state;
        CHECK(SpeedFeedforward_Update(
            &state, &config, tick < 1200U ? 20.0f : -20.0f, 0.0f, dt_s, 0.5f));
        CHECK(fabsf(state.current_a) <= 0.25001f);
        CHECK(fabsf(state.acceleration_rad_s2 - previous.acceleration_rad_s2) <= 0.10001f);
    }
    previous = state;
    CHECK(!SpeedFeedforward_Update(&state, &config, NAN, 0.0f, dt_s, 6.0f));
    CHECK(!SpeedFeedforward_Update(&state, &config, 0.0f, 0.0f, 0.01f, 6.0f));
    CHECK(!SpeedFeedforward_Update(&state, &config, 0.0f, 0.0f, dt_s, 0.0f));
    CHECK(memcmp(&state, &previous, sizeof(state)) == 0);
}

static void start_feedforward(void)
{
    setup(MOTOR_CONTROL_SPEED);
    s_motor.application.speed_feedforward.enabled = true;
    s_motor.application.speed_feedforward.gain_a_per_rad_s2 = 0.2f;
    s_motor.speed_target_rad_s = 20.0f;
}

static void check_feedforward_runtime(void)
{
    /* 关闭时逐拍比原速度路径，包含 PI 全部状态；标定入口也不读取前馈配置。 */
    setup(MOTOR_CONTROL_SPEED);
    tick();
    MotorControlState expected = s_motor;
    PiController expected_pi = s_pi;
    for (unsigned tick_index = 0U; tick_index < 10000U; ++tick_index)
    {
        s_speed_rad_s = sinf((float)tick_index * 0.01f) * 0.1f;
        s_motor.speed_target_rad_s = expected.speed_target_rad_s =
            tick_index % 2000U < 1000U ? 0.8f : -0.7f;
        SpeedMode_UpdateControl(&expected, &expected_pi, s_speed_rad_s);
        tick();
        CHECK(s_motor.speed_ramped_rad_s == expected.speed_ramped_rad_s);
        CHECK(s_motor.iq_target_a == expected.iq_target_a);
        CHECK(memcmp(&s_pi, &expected_pi, sizeof(s_pi)) == 0);
    }
    s_motor.application.speed_feedforward.enabled = true;
    SpeedMode_UpdateControl(&s_motor, &s_pi, s_speed_rad_s);
    SpeedMode_UpdateControl(&expected, &expected_pi, s_speed_rad_s);
    CHECK(s_motor.iq_target_a == expected.iq_target_a);
    CHECK(memcmp(&s_pi, &expected_pi, sizeof(s_pi)) == 0);

    start_feedforward();
    tick();
    CHECK(MotorOuterLoop_IsReady() && s_motor.uses_speed_ramp);
    CHECK(fabsf(s_motor.speed_ramped_rad_s - 0.00005f) < 0.00000001f);
    CHECK(fabsf(s_motor.iq_target_a - 0.0201002f) < 0.0000001f);
    float first_speed = s_motor.speed_ramped_rad_s;
    float first_current_a = s_motor.iq_target_a;
    for (unsigned tick_index = 0U; tick_index < 20U; ++tick_index)
    {
        tick();
    }
    MotorOuterLoop_RequestReset();
    CHECK(!MotorOuterLoop_IsReady());
    tick();
    CHECK(s_motor.speed_ramped_rad_s == first_speed && s_motor.iq_target_a == first_current_a);

    /* STOP/NONE、故障、切入位置再返回，均不能沿用旧规划与积分。 */
    s_motor.control_mode = MOTOR_CONTROL_NONE;
    tick();
    CHECK(!MotorOuterLoop_IsReady());
    s_motor.control_mode = MOTOR_CONTROL_SPEED;
    tick();
    CHECK(s_motor.speed_ramped_rad_s == first_speed && s_motor.iq_target_a == first_current_a);
    s_motor.fault_code = MOTOR_FAULT_OVERCURRENT;
    s_motor.iq_target_a = 0.0f;
    tick();
    CHECK(!MotorOuterLoop_IsReady() && s_motor.iq_target_a == 0.0f);
    s_motor.fault_code = MOTOR_FAULT_NONE;
    tick();
    CHECK(s_motor.speed_ramped_rad_s == first_speed && s_motor.iq_target_a == first_current_a);
    s_motor.control_mode = MOTOR_CONTROL_POSITION;
    tick();
    s_motor.control_mode = MOTOR_CONTROL_SPEED;
    tick();
    CHECK(s_motor.speed_ramped_rad_s == first_speed && s_motor.iq_target_a == first_current_a);

    start_feedforward();
    s_available_current_limit_a = 0.5f;
    SpeedFeedforwardState planned = {0};
    for (unsigned tick_index = 0U; tick_index < 10000U; ++tick_index)
    {
        CHECK(SpeedFeedforward_Update(
            &planned, &s_motor.application.speed_feedforward, 20.0f, 0.0f, Speed_Ts, 0.5f));
        s_speed_rad_s = tick_index == 0U     ? 0.0f
                        : tick_index < 1000U ? -100.0f
                                             : planned.speed_rad_s;
        tick();
        CHECK(MotorOuterLoop_IsReady() && s_motor.speed_ramped_rad_s == planned.speed_rad_s);
        CHECK(fabsf(s_motor.iq_target_a) <= 0.500001f && isfinite(s_pi.integral_output));
        CHECK(fabsf(planned.current_a) <= 0.250001f);
        CHECK(s_pi.proportional_gain == 0.5f && s_pi.integral_gain == 2.0f);
        CHECK(fabsf(s_pi.output_max - (0.5f - planned.current_a) / 4.0f) < 0.0000001f);
        if (tick_index > 8000U)
        {
            CHECK(fabsf(s_motor.iq_target_a) < 0.01f);
        }
    }
    s_available_current_limit_a = 0.0f;
    tick();
    CHECK(MotorOuterLoop_IsReady() && s_motor.fault_code == MOTOR_FAULT_NONE);
    CHECK(s_motor.iq_target_a == 0.0f && s_pi.integral_output == 0.0f);
    s_speed_rad_s = 0.0f;
    s_available_current_limit_a = 0.5f;
    tick();
    CHECK(s_motor.speed_ramped_rad_s == first_speed && s_motor.iq_target_a == first_current_a);

    start_feedforward();
    s_motor.application.speed_feedforward.gain_a_per_rad_s2 = NAN;
    tick();
    CHECK(s_motor.fault_code == MOTOR_FAULT_INVALID_PARAMETERS);
    CHECK(!MotorOuterLoop_IsReady() && s_motor.id_target_a == 0.0f && s_motor.iq_target_a == 0.0f);
    start_feedforward();
    tick();
    s_pi.integral_output = NAN;
    tick();
    CHECK(s_motor.fault_code == MOTOR_FAULT_INVALID_PARAMETERS);
    CHECK(!MotorOuterLoop_IsReady() && s_motor.iq_target_a == 0.0f);
    setup(MOTOR_CONTROL_SPEED);
    s_available_current_limit_a = 0.5f;
    s_speed_rad_s = 100.0f;
    tick();
    CHECK(MotorOuterLoop_IsReady() && s_motor.iq_target_a == -0.5f);
    CHECK(s_pi.proportional_gain == 0.5f && s_pi.integral_gain == 2.0f);
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
    else if (strcmp(argv[1], "feedforward") == 0)
    {
        check_feedforward_core();
        check_feedforward_runtime();
    }
    else
    {
        check_runtime();
    }
    printf("core outer %s: %s\n", argv[1], s_failures == 0U ? "PASS" : "FAIL");
    return s_failures == 0U ? 0 : 1;
}
