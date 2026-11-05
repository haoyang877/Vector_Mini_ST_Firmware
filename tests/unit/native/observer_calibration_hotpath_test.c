/* 仅内存的生产观测器/标定准备路径检查；不访问平台、PWM或Flash。 */
#undef NDEBUG
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "flux_observer.c"
#include "encoder_calibration.c"

static bool leave_align;
static unsigned current_calls;
static bool simulate_motion;
static bool unstable_motion;
static bool reset_observer_speed;
static unsigned motion_ticks;
static uint32_t trace_hash = 2166136261U;
static uint32_t control_hash = 2166136261U;

static void trace_control(uint32_t tag, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    control_hash = ((control_hash ^ tag) * 16777619U ^ bits) * 16777619U;
}
const SensorlessStartupConfig SensorlessStartup_EncoderCalibConfig = {
    .align_current_ramp_time_s = SENSORLESS_ENCODER_CALIB_ALIGN_CURRENT_RAMP_TIME_S,
    .align_hold_time_s = SENSORLESS_ENCODER_CALIB_ALIGN_HOLD_TIME_S,
    .minimum_electrical_velocity_rad_s = 1.0f,
};

void MotorFault_Report(MotorControlState *motor, MotorFaultCode fault)
{
    trace_control(1U, (float)fault);
    motor->fault_code = fault;
}
void SensorlessStartup_Reset(SensorlessStartupState *startup)
{
    trace_control(2U, startup->speed_pi_output_max);
    memset(startup, 0, sizeof(*startup));
}
void SensorlessStartup_Run(const MotorConfiguration *configuration,
                           FocState *foc,
                           MotorControlState *motor,
                           PiController *controller,
                           FluxObserverState *observer,
                           SensorlessStartupState *startup,
                           const SensorlessStartupConfig *config)
{
    (void)foc;
    (void)configuration;
    (void)foc;
    (void)motor;
    (void)controller;
    (void)observer;
    assert(config == &SensorlessStartup_EncoderCalibConfig);
    trace_control(3U, motor->speed_target_rad_s);
    trace_control(4U, motor->speed_ramped_rad_s);
    trace_control(5U, startup->speed_pi_output_max);
    if (simulate_motion)
    {
        ++motion_ticks;
        startup->state =
            motion_ticks < 4096U ? SENSORLESS_STARTUP_ALIGN : SENSORLESS_STARTUP_CLOSED_LOOP;
        startup->state_ticks = motion_ticks < 4096U ? UINT32_MAX : 0U;
        startup->speed_feedback_rad_s =
            unstable_motion ? 0.0f : SENSORLESS_ENCODER_CALIB_SPEED_MEC_RAD_S;
        if (startup->speed_pi_output_max > 0.0f)
        {
            motor->speed_ramped_rad_s = SENSORLESS_ENCODER_CALIB_SPEED_MEC_RAD_S;
        }
        observer->electrical_velocity_rad_s =
            reset_observer_speed
                ? 0.0f
                : motor->speed_ramped_rad_s * (float)configuration->model.motor_pole_pairs;
        return;
    }
    startup->state = leave_align ? SENSORLESS_STARTUP_OPEN_LOOP : SENSORLESS_STARTUP_ALIGN;
    ++startup->state_ticks;
}
void Foc_ResetCurrent(FocState *foc)
{
    trace_control(6U, foc->id_pi.integral_output);
    PiController_Reset(&foc->id_pi);
    PiController_Reset(&foc->iq_pi);
}
void Foc_RunCurrent(const MotorConfiguration *configuration,
                    FocState *foc,
                    MotorControlState *motor,
                    float phase,
                    float velocity)
{
    (void)foc;
    (void)configuration;
    (void)foc;
    (void)motor;
    (void)phase;
    (void)velocity;
    ++current_calls;
    trace_control(7U, phase);
    trace_control(8U, velocity);
    trace_control(9U, motor->id_target_a);
    trace_control(10U, motor->iq_target_a);
}
bool Encoder_IsOnline(const EncoderState *encoder)
{
    return encoder->has_valid_sample;
}
float Encoder_GetMechVelContinuous(const EncoderState *encoder)
{
    return encoder->mechanical_velocity_rad_s;
}
void Encoder_ResetVelocity(EncoderState *encoder)
{
    encoder->velocity_restart_requested = true;
}
bool Encoder_SetElectricalZeroQ15(EncoderCalibration *calibration,
                                  EncoderState *encoder,
                                  uint16_t zero)
{
    (void)encoder;
    calibration->electrical_zero_q15 = zero;
    trace_control(11U, (float)zero);
    return true;
}

/* 测试显式绑定控制对象，运动采样与任务执行分开。 */
static MotorTaskResult step_lut(MotorConfiguration *configuration,
                                FocState *foc,
                                MotorControlState *motor,
                                EncoderState *encoder,
                                FluxObserverState *observer,
                                SensorlessStartupState *startup,
                                EncoderCalibrationWorkspace *workspace)
{
    const EncoderCalibrationControl control = {
        configuration, foc, motor, encoder, observer, startup};
    return EncoderCalibration_StepLut(&control, workspace);
}

static MotorTaskResult step_electrical_zero(MotorConfiguration *configuration,
                                            EncoderCalibration *calibration,
                                            FocState *foc,
                                            MotorControlState *motor,
                                            EncoderState *encoder)
{
    assert(calibration == &configuration->device.encoder);
    const EncoderCalibrationControl control = {
        .configuration = configuration, .foc = foc, .motor = motor, .encoder = encoder};
    return EncoderCalibration_StepElecZero(&control);
}

static void check_long_sweep_position(void)
{
    const MotorModelConfig model = {.motor_flux_wb = 0.015f};
    const AlphaBetaVector zero = {0};
    /* 14极对、420电rad/s下连续100电机圈，参考角应保持在单精度表示误差内。 */
    for (int direction = -1; direction <= 1; direction += 2)
    {
        FluxObserverState observer = {0};
        FluxObserver_Init(&observer);
        for (unsigned i = 0; i <= 418880U; ++i)
        {
            double expected = direction * (double)i * 420.0 / 20000.0;
            observer.x1_last = model.motor_flux_wb * (float)cos(expected);
            observer.x2_last = model.motor_flux_wb * (float)sin(expected);
            FluxObserver_Update(&model, &zero, &zero, &observer);
            assert(fabs(observer.electrical_position_rad - expected) < 0.002);
        }
    }
}

static void check_finite(void)
{
    /* 指数、符号和尾数跨全binary32范围；含无穷、正负零及各种NaN。 */
    for (uint32_t i = 0U; i < 65536U; ++i)
    {
        uint32_t bits = i * UINT32_C(65537);
        float value;
        memcpy(&value, &bits, sizeof(value));
        assert(fluxobserver_is_finite(value) == !!isfinite(value));
    }
    const uint32_t edge[] = {0U,
                             0x80000000U,
                             1U,
                             0x807FFFFFU,
                             0x7F7FFFFFU,
                             0x7F800000U,
                             0xFF800000U,
                             0x7F800001U,
                             0xFFC12345U};
    for (unsigned i = 0U; i < sizeof(edge) / sizeof(edge[0]); ++i)
    {
        float value;
        memcpy(&value, &edge[i], sizeof(value));
        assert(fluxobserver_is_finite(value) == !!isfinite(value));
    }
}

static void check_observer(void)
{
    MotorModelConfig model = {0};
    FluxObserverState observer = {0};
    model.motor_phase_resistance_ohm = 0.2f;
    model.motor_d_inductance_h = 0.0002f;
    model.motor_q_inductance_h = 0.0003f;
    model.motor_flux_wb = 0.015f;
    FluxObserver_Init(&observer);
    for (unsigned i = 0U; i < 8192U; ++i)
    {
        float phase = (float)i * 0.017f;
        const AlphaBetaVector current_ab_a = {
            0.1f * sinf(phase),
            (0.1f * sinf(phase - 2.0943951f) - 0.1f * sinf(phase + 2.0943951f)) * 0.5773502692f};
        const AlphaBetaVector voltage_ab_v = {0.01f * cosf(phase) * (33.0f / 1.5f),
                                              0.01f * sinf(phase) * (33.0f / 1.5f)};
        FluxObserver_Update(&model, &current_ab_a, &voltage_ab_v, &observer);
        assert(memcmp(&observer.current_ab_a, &current_ab_a, sizeof(current_ab_a)) == 0);
        assert(memcmp(&observer.voltage_ab_v, &voltage_ab_v, sizeof(voltage_ab_v)) == 0);
        float reference = Math_NormalizeAngle(Math_Atan2(observer.sin, observer.cos));
        assert(memcmp(&reference, &observer.electrical_angle_rad, sizeof(reference)) == 0);
        assert(isfinite(observer.electrical_velocity_rad_s));
    }
    const float invalid[] = {0.0f, -1.0f, INFINITY, NAN};
    const AlphaBetaVector zero = {0.0f, 0.0f};
    for (unsigned i = 0U; i < sizeof(invalid) / sizeof(invalid[0]); ++i)
    {
        uint32_t epoch = observer.position_epoch;
        model.motor_flux_wb = invalid[i];
        FluxObserver_Update(&model, &zero, &zero, &observer);
        assert(observer.position_epoch == epoch + 1U);
        assert(observer.electrical_angle_rad == 0.0f && observer.x1_last == 0.0f);
    }
}

static void check_prepare(void)
{
    FocState foc = {0};
    MotorConfiguration configuration = {0};
    MotorControlState motor = {0};
    EncoderState encoder = {0};
    FluxObserverState observer = {0};
    SensorlessStartupState startup = {0};
    EncoderCalibrationWorkspace workspace;
    configuration.model.motor_pole_pairs = 21;
    configuration.model.motor_phase_resistance_ohm = 0.006f;
    configuration.model.motor_d_inductance_h = 0.0002f;
    configuration.model.motor_q_inductance_h = 0.0003f;
    configuration.model.motor_flux_wb = 0.015f;
    configuration.application.current_limit_a = 6.0f;
    encoder.has_valid_sample = true;
    configuration.device.encoder.calib_flag = ENC_CALIB_LINEARIZED | ENC_CALIB_ELECTRICAL_ZERO;
    memset(configuration.device.encoder.linearization_lut_q15,
           0x5A,
           sizeof(configuration.device.encoder.linearization_lut_q15));
    EncoderCalibration_Cancel();
    memset(&workspace, 0xA5, sizeof(workspace));
    leave_align = false;
    motor.speed_target_rad_s = 3.0f;
    motor.speed_ramped_rad_s = 2.0f;
    motor.id_target_a = 1.0f;
    motor.iq_target_a = -1.0f;
    MotorTaskResult result =
        step_lut(&configuration, &foc, &motor, &encoder, &observer, &startup, &workspace);
    assert(result.result == MOTOR_TASK_RUNNING && s_lut.stage.startup.clear_bin == 0U);
    assert(motor.speed_target_rad_s == 3.0f && motor.speed_ramped_rad_s == 2.0f);
    assert(motor.id_target_a == 1.0f && motor.iq_target_a == -1.0f);
    motor = (MotorControlState){0};
    assert(startup.state == SENSORLESS_STARTUP_IDLE && startup.state_ticks == 0U);
    assert(workspace.bins.samples[0] == 0xA5A5U && current_calls == 0U);
    result = step_lut(&configuration, &foc, &motor, &encoder, &observer, &startup, &workspace);
    assert(result.result == MOTOR_TASK_RUNNING &&
           s_lut.stage.startup.clear_bin == ENCODER_CALIBRATION_PREPARE_BINS_PER_TICK);
    assert(workspace.error_sum[0] == 0 &&
           workspace.bins.samples[ENCODER_CALIBRATION_PREPARE_BINS_PER_TICK - 1U] == 0U);
    assert(workspace.bins.samples[ENCODER_CALIBRATION_PREPARE_BINS_PER_TICK] ==
           0xA5A5U); /* 首拍不再清6 KiB。 */
    s_lut.speed_pi.integral_output = 0.4f;
    EncoderCalibration_Cancel();
    assert(EncoderCalibration_IsIdle() && s_lut.stage.startup.clear_bin == 0U);
    assert(s_lut.speed_pi.integral_output == 0.0f);
    memset(&workspace, 0xA5, sizeof(workspace));
    for (unsigned i = 0U; i <= ENCODER_OFFSET_LUT_SIZE / ENCODER_CALIBRATION_PREPARE_BINS_PER_TICK;
         ++i)
    {
        result = step_lut(&configuration, &foc, &motor, &encoder, &observer, &startup, &workspace);
        assert(result.result == MOTOR_TASK_RUNNING);
        assert(s_lut.stage.startup.clear_bin == i * ENCODER_CALIBRATION_PREPARE_BINS_PER_TICK);
    }
    for (unsigned i = 0U; i < ENCODER_OFFSET_LUT_SIZE; ++i)
    {
        assert(workspace.error_sum[i] == 0 && workspace.bins.samples[i] == 0U);
        assert((uint16_t)configuration.device.encoder.linearization_lut_q15[i] == 0x5A5AU);
    }
    leave_align = true;
    result = step_lut(&configuration, &foc, &motor, &encoder, &observer, &startup, &workspace);
    assert(result.result == MOTOR_TASK_RUNNING);
    assert(EncoderCalibration_GetPhase() == CS_OBS_STARTUP);
    simulate_motion = true;
    motion_ticks = 4096U;
    result = step_lut(&configuration, &foc, &motor, &encoder, &observer, &startup, &workspace);
    assert(result.result == MOTOR_TASK_RUNNING);
    assert(EncoderCalibration_GetPhase() == CS_OBS_STARTUP);
    assert(s_lut.stage.startup.stable_ticks == 1U);
    simulate_motion = false;
    EncoderCalibration_Cancel();
    result = step_lut(&configuration, &foc, &motor, &encoder, &observer, &startup, &workspace);
    assert(result.result == MOTOR_TASK_RUNNING && startup.state_ticks == 0U);
    result = step_lut(&configuration, &foc, &motor, &encoder, &observer, &startup, &workspace);
    assert(result.result == MOTOR_TASK_RUNNING);
    assert(EncoderCalibration_GetPhase() == CS_OBS_STARTUP);
    EncoderCalibration_Cancel();
    assert(EncoderCalibration_IsIdle() && s_lut.stage.startup.clear_bin == 0U);
    assert(configuration.device.encoder.calib_flag ==
           (ENC_CALIB_LINEARIZED | ENC_CALIB_ELECTRICAL_ZERO));
    assert(current_calls == 0U); /* 准备检查没有调用功率控制。 */
}

static void check_commit(void)
{
    union
    {
        uint32_t alignment;
        uint8_t bytes[sizeof(EncoderCalibration) + 8U];
    } storage;
    EncoderCalibrationWorkspace workspace;
    for (unsigned offset = 0U; offset <= 2U; offset += 2U)
    {
        memset(&storage, 0x5A, sizeof(storage));
        EncoderCalibration *calibration = (EncoderCalibration *)&storage.bytes[offset];
        calibration->calib_flag = ENC_CALIB_ALL | ENC_CALIB_MECHANICAL_ZERO;
        calibration->reverse = 1U;
        EncoderState encoder = {0};
        encoder.directed_q15 = 32767U;
        for (unsigned i = 0U; i < ENCODER_OFFSET_LUT_SIZE; ++i)
        {
            workspace.bins.candidate[i] = (int16_t)((i * 73U) ^ 0x8421U);
        }
        uint16_t expected = apply_lut(workspace.bins.candidate, encoder.directed_q15);
        s_lut.workspace = &workspace;
        s_lut.phase = CS_OBS_BUILD_LUT;
        assert(!EncoderCalibration_PublishLut(calibration, &encoder));
        s_lut.phase = CS_OBS_STOP_DECEL;
        FocState foc = {0};
        MotorControlState motor = {0};
        SensorlessStartupState startup = {0};
        assert(finish_lut(
                   &(EncoderCalibrationControl){.foc = &foc, .motor = &motor, .startup = &startup})
                   .result == MOTOR_TASK_FINALIZE);
        assert(!EncoderCalibration_IsIdle());
        assert(calibration->calib_flag == (ENC_CALIB_ALL | ENC_CALIB_MECHANICAL_ZERO));
        assert(EncoderCalibration_PublishLut(calibration, &encoder));
        assert(EncoderCalibration_IsIdle());
        assert(!EncoderCalibration_PublishLut(calibration, &encoder));
        assert(memcmp(calibration->linearization_lut_q15,
                      workspace.bins.candidate,
                      sizeof(calibration->linearization_lut_q15)) == 0);
        assert(calibration->calib_flag == ENC_CALIB_LINEARIZED && calibration->reverse == 1U);
        assert(calibration->electrical_zero_q15 == 0U && calibration->mechanical_zero_q15 == 0U);
        assert(encoder.linearized_q15 == expected && encoder.rebase_requested &&
               encoder.velocity_restart_requested);
        for (unsigned i = 0U; i < offset; ++i)
        {
            assert(storage.bytes[i] == 0x5A);
        }
        for (unsigned i = offset + sizeof(*calibration); i < sizeof(storage.bytes); ++i)
        {
            assert(storage.bytes[i] == 0x5A);
        }
        s_lut.workspace = &workspace;
        s_lut.phase = CS_LUT_PUBLISH_PENDING;
        EncoderCalibration before_cancel = *calibration;
        EncoderCalibration_Cancel();
        assert(!EncoderCalibration_PublishLut(calibration, &encoder));
        assert(memcmp(calibration, &before_cancel, sizeof(*calibration)) == 0);
    }
}

/* 用公开入口走完整运动序列；逐拍摘要同时冻结目标、电流、阶段和结果。 */
static void trace_word(uint32_t value)
{
    trace_hash = (trace_hash ^ value) * 16777619U;
}

static void trace_cycle(MotorTaskResult result, const MotorControlState *motor)
{
    uint32_t bits;
    trace_word((uint32_t)result.result);
    trace_word((uint32_t)result.error);
    trace_word((uint32_t)EncoderCalibration_GetPhase());
    memcpy(&bits, &motor->speed_target_rad_s, sizeof(bits));
    trace_word(bits);
    memcpy(&bits, &motor->id_target_a, sizeof(bits));
    trace_word(bits);
    memcpy(&bits, &motor->iq_target_a, sizeof(bits));
    trace_word(bits);
}

typedef struct
{
    int32_t pole_pairs;
    float initial_position_rad;
    float offset_q15;
    float nonlinear_amplitude_q15;
} CalibrationMotion;

/* 合成编码器的机械周期误差，参考输入与LUT算法独立计算。 */
static uint16_t motion_angle_q15(float mechanical_q15, const CalibrationMotion *motion)
{
    float nonlinear_q15 =
        motion->nonlinear_amplitude_q15 * sinf(mechanical_q15 * (2.0f * _2PI / 65536.0f));
    return (uint16_t)(uint32_t)(mechanical_q15 + motion->offset_q15 + nonlinear_q15);
}

static void
sample_motion(const CalibrationMotion *motion, FluxObserverState *observer, EncoderState *encoder)
{
    if (motion_ticks + 1U >= 4096U)
    {
        observer->electrical_position_rad += _2PI * (float)motion->pole_pairs / 8192.0f;
    }
    float mechanical_q15 =
        observer->electrical_position_rad * (65536.0f / (_2PI * (float)motion->pole_pairs));
    encoder->directed_q15 = motion_angle_q15(mechanical_q15, motion);
}

/* 模拟每圈8192拍，额外留出启动、构表和停止时间。 */
#define LUT_FLOW_MAX_TICKS (SENSORLESS_ENCODER_CALIB_MECH_TURNS * 8192U + 200000U)

static void check_lut_flow(const CalibrationMotion *motion)
{
    MotorConfiguration configuration = {0};
    FocState foc = {0};
    MotorControlState motor = {0};
    EncoderState encoder = {.has_valid_sample = true};
    FluxObserverState observer = {.electrical_position_rad = motion->initial_position_rad};
    SensorlessStartupState startup = {0};
    EncoderCalibrationWorkspace workspace;
    configuration.model.motor_pole_pairs = motion->pole_pairs;
    configuration.model.motor_phase_resistance_ohm = 0.1f;
    configuration.model.motor_d_inductance_h = 0.0002f;
    configuration.model.motor_q_inductance_h = 0.0002f;
    configuration.model.motor_flux_wb = 0.015f;
    configuration.application.current_limit_a = 6.0f;
    simulate_motion = true;
    motion_ticks = 0U;
    EncoderCalibration_Cancel();
    MotorTaskResult result = {MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};
    uint32_t phases = 0U;
    unsigned ticks = 0U;
    float reference_offset_q15 = 0.0f;
    while (result.result == MOTOR_TASK_RUNNING && ticks < LUT_FLOW_MAX_TICKS)
    {
        sample_motion(motion, &observer, &encoder);
        EncoderCalibrationPhase previous_phase = EncoderCalibration_GetPhase();
        result = step_lut(&configuration, &foc, &motor, &encoder, &observer, &startup, &workspace);
        EncoderCalibrationPhase phase = EncoderCalibration_GetPhase();
        if (phase >= CS_OBS_STARTUP && phase <= CS_OBS_STOP_CURRENT)
        {
            phases |= 1U << (phase - CS_OBS_STARTUP);
        }
        if (previous_phase == CS_OBS_STARTUP && phase == CS_OBS_SAMPLE_CW)
        {
            float mechanical_q15 =
                observer.electrical_position_rad * (65536.0f / (_2PI * (float)motion->pole_pairs));
            reference_offset_q15 =
                motion->offset_q15 +
                motion->nonlinear_amplitude_q15 * sinf(mechanical_q15 * (2.0f * _2PI / 65536.0f));
        }
        trace_cycle(result, &motor);
        ++ticks;
    }
    const uint32_t expected_phases =
        (1U << (CS_OBS_STARTUP - CS_OBS_STARTUP)) | (1U << (CS_OBS_SAMPLE_CW - CS_OBS_STARTUP)) |
        (1U << (CS_OBS_BUILD_LUT - CS_OBS_STARTUP)) | (1U << (CS_OBS_STOP_DECEL - CS_OBS_STARTUP)) |
        (1U << (CS_OBS_STOP_CURRENT - CS_OBS_STARTUP));
    assert(result.result == MOTOR_TASK_FINALIZE && ticks < LUT_FLOW_MAX_TICKS);
    assert(phases == expected_phases && !EncoderCalibration_IsIdle());
    assert(configuration.device.encoder.calib_flag == 0U);
    assert(EncoderCalibration_PublishLut(&configuration.device.encoder, &encoder));
    assert(configuration.device.encoder.calib_flag == ENC_CALIB_LINEARIZED);
    assert(EncoderCalibration_IsIdle());
    int32_t peak_residual_q15 = 0;
    for (unsigned ideal_q15 = 0U; ideal_q15 < 65536U; ideal_q15 += 16U)
    {
        uint16_t raw_q15 = motion_angle_q15((float)ideal_q15, motion);
        uint16_t linearized_q15 =
            apply_lut(configuration.device.encoder.linearization_lut_q15, raw_q15);
        uint16_t expected_q15 = (uint16_t)(uint32_t)((float)ideal_q15 + reference_offset_q15);
        int32_t residual_q15 = delta_q15(linearized_q15, expected_q15);
        if (residual_q15 < 0)
        {
            residual_q15 = -residual_q15;
        }
        if (residual_q15 > peak_residual_q15)
        {
            peak_residual_q15 = residual_q15;
        }
    }
    /* 16 Q15计数覆盖分箱中心偏差、取整和float累计误差，远小于注入误差。 */
    assert(peak_residual_q15 <= 16);
    for (unsigned i = 0U; i < ENCODER_OFFSET_LUT_SIZE; ++i)
    {
        trace_word((uint16_t)configuration.device.encoder.linearization_lut_q15[i]);
    }
    simulate_motion = false;
    printf("PASS LUT flow: poles=%ld residual=%ld ticks=%u\n",
           (long)motion->pole_pairs,
           (long)peak_residual_q15,
           ticks);
}

static void check_flow(void)
{
    const CalibrationMotion motions[] = {
        {1, 0.0f, 0.0f, 0.0f},
        {7, 123.0f, 65000.0f, 768.0f},
        {21, 246.0f, 17000.0f, 512.0f},
    };
    for (unsigned i = 0U; i < sizeof(motions) / sizeof(motions[0]); ++i)
    {
        check_lut_flow(&motions[i]);
    }

    /* 电零位覆盖斜坡、环形均值和完成后的重入与取消。 */
    MotorConfiguration configuration = {0};
    FocState foc = {0};
    MotorControlState motor = {0};
    EncoderState encoder = {.has_valid_sample = true, .linearized_q15 = 65530U};
    configuration.device.encoder.calib_flag = ENC_CALIB_LINEARIZED;
    configuration.model.calibration_current_a = 2.0f;
    configuration.application.current_limit_a = 6.0f;
    unsigned ticks = 0U;
    MotorTaskResult result;
    do
    {
        encoder.linearized_q15 = ticks % 2U == 0U ? 65530U : 10U;
        result = step_electrical_zero(
            &configuration, &configuration.device.encoder, &foc, &motor, &encoder);
        trace_cycle(result, &motor);
        ++ticks;
    } while (result.result == MOTOR_TASK_RUNNING && ticks < 40000U);
    assert(result.result == MOTOR_TASK_COMPLETED && ticks == 30001U);
    assert(configuration.device.encoder.electrical_zero_q15 <= 2U);
    assert(motor.id_target_a == 0.0f && motor.iq_target_a == 0.0f);
    assert(EncoderCalibration_IsIdle());
    result =
        step_electrical_zero(&configuration, &configuration.device.encoder, &foc, &motor, &encoder);
    assert(result.result == MOTOR_TASK_RUNNING && !EncoderCalibration_IsIdle());
    EncoderCalibration_Cancel();
    assert(EncoderCalibration_IsIdle());
    EncoderCalibration_Cancel();
    printf("PASS calibration flow: trace=%08x\n", (unsigned)trace_hash);
}

typedef struct
{
    MotorConfiguration configuration;
    FocState foc;
    MotorControlState motor;
    EncoderState encoder;
    FluxObserverState observer;
    SensorlessStartupState startup;
    EncoderCalibrationWorkspace workspace;
} CalibrationFixture;

static void reset_fixture(CalibrationFixture *fixture)
{
    memset(fixture, 0, sizeof(*fixture));
    fixture->configuration.model.motor_pole_pairs = 1;
    fixture->configuration.model.motor_phase_resistance_ohm = 0.1f;
    fixture->configuration.model.motor_d_inductance_h = 0.0002f;
    fixture->configuration.model.motor_q_inductance_h = 0.0002f;
    fixture->configuration.model.motor_flux_wb = 0.015f;
    fixture->configuration.application.current_limit_a = 6.0f;
    fixture->encoder.has_valid_sample = true;
    simulate_motion = true;
    unstable_motion = false;
    reset_observer_speed = false;
    leave_align = false;
    motion_ticks = 0U;
    EncoderCalibration_Cancel();
}

static MotorTaskResult step_fixture(CalibrationFixture *fixture)
{
    const CalibrationMotion motion = {1, 0.0f, 0.0f, 0.0f};
    sample_motion(&motion, &fixture->observer, &fixture->encoder);
    return step_lut(&fixture->configuration,
                    &fixture->foc,
                    &fixture->motor,
                    &fixture->encoder,
                    &fixture->observer,
                    &fixture->startup,
                    &fixture->workspace);
}

static void reach_phase(CalibrationFixture *fixture, EncoderCalibrationPhase target)
{
    unsigned ticks = 0U;
    while (EncoderCalibration_GetPhase() != target && ticks++ < LUT_FLOW_MAX_TICKS)
    {
        MotorTaskResult result = step_fixture(fixture);
        assert(result.result == MOTOR_TASK_RUNNING || EncoderCalibration_GetPhase() == target);
    }
    assert(EncoderCalibration_GetPhase() == target && ticks < LUT_FLOW_MAX_TICKS);
}

static void check_cancel(void)
{
    const EncoderCalibrationPhase phases[] = {CS_OBS_STARTUP,
                                              CS_OBS_SAMPLE_CW,
                                              CS_OBS_BUILD_LUT,
                                              CS_OBS_STOP_DECEL,
                                              CS_OBS_STOP_CURRENT,
                                              CS_LUT_PUBLISH_PENDING};
    for (unsigned index = 0U; index < sizeof(phases) / sizeof(phases[0]); ++index)
    {
        CalibrationFixture fixture;
        reset_fixture(&fixture);
        reach_phase(&fixture, phases[index]);
        EncoderCalibration previous = fixture.configuration.device.encoder;
        EncoderCalibration_Cancel();
        EncoderCalibration_Cancel();
        assert(EncoderCalibration_IsIdle());
        assert(!EncoderCalibration_PublishLut(&fixture.configuration.device.encoder,
                                              &fixture.encoder));
        assert(memcmp(&previous, &fixture.configuration.device.encoder, sizeof(previous)) == 0);
        assert(step_fixture(&fixture).result == MOTOR_TASK_RUNNING);
        assert(EncoderCalibration_GetPhase() == CS_OBS_STARTUP);
        assert(s_lut.stage.startup.clear_bin == 0U && s_lut.stage.startup.ticks == 0U &&
               s_lut.stage.startup.stable_ticks == 0U);
    }
    EncoderCalibration_Cancel();
    puts("PASS calibration cancel and reentry in every phase");
}

static void check_epoch_failure(void)
{
    const EncoderCalibrationPhase phases[] = {CS_OBS_SAMPLE_CW, CS_OBS_BUILD_LUT};
    for (unsigned index = 0U; index < sizeof(phases) / sizeof(phases[0]); ++index)
    {
        CalibrationFixture fixture;
        reset_fixture(&fixture);
        reach_phase(&fixture, phases[index]);
        ++fixture.observer.position_epoch;
        MotorTaskResult result = step_fixture(&fixture);
        assert(result.result == MOTOR_TASK_FAILED && result.error == MOTOR_FAULT_SENSORLESS);
        assert(EncoderCalibration_IsIdle());
        assert(!EncoderCalibration_PublishLut(&fixture.configuration.device.encoder,
                                              &fixture.encoder));
        assert(fixture.motor.id_target_a == 0.0f && fixture.motor.iq_target_a == 0.0f);
        assert(fixture.motor.speed_target_rad_s == 0.0f &&
               fixture.motor.speed_ramped_rad_s == 0.0f);
    }
    puts("PASS calibration observer epoch failure in sampling and build");
}

static void check_startup_window(void)
{
    CalibrationFixture fixture;
    reset_fixture(&fixture);
    reach_phase(&fixture, CS_OBS_STARTUP);
    while (fixture.startup.state != SENSORLESS_STARTUP_CLOSED_LOOP)
    {
        assert(step_fixture(&fixture).result == MOTOR_TASK_RUNNING);
    }
    for (unsigned tick = 0U; tick < 100U; ++tick)
    {
        assert(step_fixture(&fixture).result == MOTOR_TASK_RUNNING);
    }
    assert(s_lut.stage.startup.stable_ticks > 0U);
    uint32_t elapsed_ticks = s_lut.stage.startup.ticks;
    unstable_motion = true;
    assert(step_fixture(&fixture).result == MOTOR_TASK_RUNNING);
    assert(s_lut.stage.startup.stable_ticks == 0U);
    assert(s_lut.stage.startup.ticks == elapsed_ticks + 1U);
    unstable_motion = false;
    uint32_t required_ticks = (uint32_t)(SENSORLESS_ENCODER_CALIB_SPEED_STABLE_TIME_S / FOC_TS_S);
    for (uint32_t tick = 1U; tick < required_ticks; ++tick)
    {
        assert(step_fixture(&fixture).result == MOTOR_TASK_RUNNING);
        assert(EncoderCalibration_GetPhase() == CS_OBS_STARTUP);
        assert(s_lut.stage.startup.stable_ticks == tick);
    }
    /* 稳速最后一拍观测器复位，旧滤波速度不能放行新采样参考。 */
    reset_observer_speed = true;
    assert(step_fixture(&fixture).result == MOTOR_TASK_RUNNING);
    assert(fixture.startup.state == SENSORLESS_STARTUP_CLOSED_LOOP);
    assert(fixture.startup.speed_feedback_rad_s == SENSORLESS_ENCODER_CALIB_SPEED_MEC_RAD_S);
    assert(fixture.motor.speed_ramped_rad_s == SENSORLESS_ENCODER_CALIB_SPEED_MEC_RAD_S);
    assert(fixture.observer.electrical_velocity_rad_s == 0.0f);
    assert(EncoderCalibration_GetPhase() == CS_OBS_STARTUP);
    assert(s_lut.stage.startup.stable_ticks == 0U);
    reset_observer_speed = false;
    for (uint32_t tick = 1U; tick < required_ticks; ++tick)
    {
        assert(step_fixture(&fixture).result == MOTOR_TASK_RUNNING);
        assert(EncoderCalibration_GetPhase() == CS_OBS_STARTUP);
    }
    assert(step_fixture(&fixture).result == MOTOR_TASK_RUNNING);
    assert(EncoderCalibration_GetPhase() == CS_OBS_SAMPLE_CW);

    /* 开环等待与闭环失稳共用启动总时限，闭环交接不会重新计时。 */
    uint32_t timeout_ticks = (uint32_t)(SENSORLESS_ENCODER_CALIB_STARTUP_TIMEOUT_S / FOC_TS_S);
    for (unsigned closed_loop = 0U; closed_loop < 2U; ++closed_loop)
    {
        reset_fixture(&fixture);
        simulate_motion = closed_loop != 0U;
        unstable_motion = true;
        leave_align = true;
        MotorTaskResult result = {MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};
        uint32_t ticks = 0U;
        while (result.result == MOTOR_TASK_RUNNING && ticks < timeout_ticks + 2U)
        {
            result = step_fixture(&fixture);
            ++ticks;
        }
        assert(result.result == MOTOR_TASK_FAILED && result.error == MOTOR_FAULT_SENSORLESS);
        assert(ticks >= timeout_ticks && ticks <= timeout_ticks + 1U);
        assert(EncoderCalibration_IsIdle());
        assert(fixture.motor.id_target_a == 0.0f && fixture.motor.iq_target_a == 0.0f);
        assert(fixture.motor.speed_target_rad_s == 0.0f &&
               fixture.motor.speed_ramped_rad_s == 0.0f);
    }
    unstable_motion = false;
    puts("PASS calibration stable window reset and total startup timeout");
}

static void check_braking_cutoff(void)
{
    const EncoderCalibrationPhase phases[] = {CS_OBS_STOP_DECEL, CS_OBS_STOP_CURRENT};
    for (unsigned index = 0U; index < sizeof(phases) / sizeof(phases[0]); ++index)
    {
        CalibrationFixture fixture;
        reset_fixture(&fixture);
        reach_phase(&fixture, phases[index]);
        fixture.observer.electrical_velocity_rad_s = 0.0f;
        MotorTaskResult result = step_fixture(&fixture);
        assert(result.result == MOTOR_TASK_FINALIZE && result.error == MOTOR_FAULT_NONE);
        assert(!EncoderCalibration_IsIdle());
        assert(fixture.motor.id_target_a == 0.0f && fixture.motor.iq_target_a == 0.0f);
        assert(fixture.motor.speed_target_rad_s == 0.0f &&
               fixture.motor.speed_ramped_rad_s == 0.0f);
        assert(
            EncoderCalibration_PublishLut(&fixture.configuration.device.encoder, &fixture.encoder));
        assert(EncoderCalibration_IsIdle());
    }
    simulate_motion = false;
    puts("PASS calibration low-speed braking cutoff in both stop phases");
}

#include "../encoder_calibration_core_test.c"

static void check_atan2_accuracy(void)
{
    /* 标定参考角不能把电角周期近似误差写入编码器补偿表。 */
    const float scales[] = {1.0e-10f, 1.0f, 1.0e10f};
    for (unsigned scale = 0U; scale < sizeof(scales) / sizeof(scales[0]); ++scale)
    {
        for (unsigned sample = 0U; sample < 4096U; ++sample)
        {
            float angle = -_PI + _2PI * (float)sample / 4096.0f;
            float x = scales[scale] * cosf(angle);
            float y = scales[scale] * sinf(angle);
            double error = (double)Math_Atan2(y, x) - atan2((double)y, (double)x);
            assert(fabs(error) < 2.0e-6);
        }
    }
    assert(isnan(Math_Atan2(NAN, 1.0f)));
    assert(isnan(Math_Atan2(1.0f, INFINITY)));
    assert(fabsf(Math_Atan2(0.0f, 0.0f) - _PI / 2.0f) < 2.0e-6f);
    puts("PASS atan2 calibration reference precision");
}

int main(void)
{
    check_atan2_accuracy();
    check_long_sweep_position();
    assert(check_calibration_core() == 0);
    check_finite();
    check_observer();
    check_prepare();
    check_commit();
    check_flow();
    check_cancel();
    check_epoch_failure();
    check_startup_window();
    check_braking_cutoff();
    printf("PASS calibration control: trace=%08x\n", (unsigned)control_hash);
    puts("PASS observer finite/phase equivalence and bounded calibration prepare/cancel/reentry");
    return 0;
}
