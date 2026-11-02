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
        startup->speed_feedback_rad_s = SENSORLESS_ENCODER_CALIB_SPEED_MEC_RAD_S;
        if (startup->speed_pi_output_max > 0.0f)
        {
            motor->speed_ramped_rad_s = SENSORLESS_ENCODER_CALIB_SPEED_MEC_RAD_S;
        }
        observer->electrical_velocity_rad_s = motor->speed_ramped_rad_s;
        if (startup->state == SENSORLESS_STARTUP_CLOSED_LOOP)
        {
            observer->electrical_position_rad += _2PI / 8192.0f;
        }
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

/* 旧调用形状用于前后同一组行为刻画；生产入口只接收显式绑定。 */
static MotorTaskResult step_lut(MotorConfiguration *configuration,
                                FocState *foc,
                                MotorControlState *motor,
                                PiController *speed_pi,
                                EncoderState *encoder,
                                FluxObserverState *observer,
                                SensorlessStartupState *startup,
                                EncoderCalibrationWorkspace *workspace)
{
    const EncoderCalibrationControl control = {
        configuration, foc, motor, speed_pi, encoder, observer, startup};
    return EncoderCalibrationTask_StepLut(&control, workspace);
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
    return EncoderCalibrationTask_StepElectricalZero(&control);
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
    PiController speed = {0};
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
    uint32_t sample_tick = (uint32_t)((SENSORLESS_ENCODER_CALIB_ALIGN_CURRENT_RAMP_TIME_S +
                                       SENSORLESS_ENCODER_CALIB_ALIGN_HOLD_TIME_S -
                                       SENSORLESS_ENCODER_CALIB_ALIGN_SAMPLE_TIME_S) /
                                      FOC_TS_S);
    assert(sample_tick >= ENCODER_OFFSET_LUT_SIZE / ENCODER_CALIBRATION_PREPARE_BINS_PER_TICK);
    EncoderCalibrationTask_Cancel();
    memset(&workspace, 0xA5, sizeof(workspace));
    leave_align = false;
    motor.speed_target_rad_s = 3.0f;
    motor.speed_ramped_rad_s = 2.0f;
    motor.id_target_a = 1.0f;
    motor.iq_target_a = -1.0f;
    MotorTaskResult result =
        step_lut(&configuration, &foc, &motor, &speed, &encoder, &observer, &startup, &workspace);
    assert(result.result == MOTOR_TASK_RUNNING && s_lut.prepare_bin == 0U);
    assert(motor.speed_target_rad_s == 3.0f && motor.speed_ramped_rad_s == 2.0f);
    assert(motor.id_target_a == 1.0f && motor.iq_target_a == -1.0f);
    motor = (MotorControlState){0};
    assert(startup.state == SENSORLESS_STARTUP_IDLE && startup.state_ticks == 0U);
    assert(workspace.bins.samples[0] == 0xA5A5U && current_calls == 0U);
    result =
        step_lut(&configuration, &foc, &motor, &speed, &encoder, &observer, &startup, &workspace);
    assert(result.result == MOTOR_TASK_RUNNING &&
           s_lut.prepare_bin == ENCODER_CALIBRATION_PREPARE_BINS_PER_TICK);
    assert(workspace.error_sum[0] == 0 &&
           workspace.bins.samples[ENCODER_CALIBRATION_PREPARE_BINS_PER_TICK - 1U] == 0U);
    assert(workspace.bins.samples[ENCODER_CALIBRATION_PREPARE_BINS_PER_TICK] ==
           0xA5A5U); /* 首拍不再清6 KiB。 */
    EncoderCalibrationTask_Cancel();
    assert(EncoderCalibrationTask_IsReleased() && s_lut.prepare_bin == 0U);
    memset(&workspace, 0xA5, sizeof(workspace));
    for (unsigned i = 0U; i <= ENCODER_OFFSET_LUT_SIZE / ENCODER_CALIBRATION_PREPARE_BINS_PER_TICK;
         ++i)
    {
        result = step_lut(
            &configuration, &foc, &motor, &speed, &encoder, &observer, &startup, &workspace);
        assert(result.result == MOTOR_TASK_RUNNING);
        assert(s_lut.prepare_bin == i * ENCODER_CALIBRATION_PREPARE_BINS_PER_TICK);
        startup.state_ticks = sample_tick;
    }
    for (unsigned i = 0U; i < ENCODER_OFFSET_LUT_SIZE; ++i)
    {
        assert(workspace.error_sum[i] == 0 && workspace.bins.samples[i] == 0U);
        assert((uint16_t)configuration.device.encoder.linearization_lut_q15[i] == 0x5A5AU);
    }
    leave_align = true;
    result =
        step_lut(&configuration, &foc, &motor, &speed, &encoder, &observer, &startup, &workspace);
    assert(result.result == MOTOR_TASK_RUNNING);
    assert(EncoderCalibrationTask_GetStep() == CS_OBS_WAIT_CLOSED_LOOP);
    EncoderCalibrationTask_Cancel();
    result =
        step_lut(&configuration, &foc, &motor, &speed, &encoder, &observer, &startup, &workspace);
    assert(result.result == MOTOR_TASK_RUNNING && startup.state_ticks == 0U);
    result =
        step_lut(&configuration, &foc, &motor, &speed, &encoder, &observer, &startup, &workspace);
    assert(result.result == MOTOR_TASK_FAILED && result.error == MOTOR_FAULT_ENCODER);
    assert(EncoderCalibrationTask_IsReleased() && s_lut.prepare_bin == 0U);
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
        s_lut.phase = CS_OBS_VERIFY_CW;
        assert(!EncoderCalibrationTask_PublishLut(calibration, &encoder));
        s_lut.phase = CS_OBS_STOP_DECEL;
        FocState foc = {0};
        MotorControlState motor = {0};
        PiController pi = {0};
        SensorlessStartupState startup = {0};
        assert(finish_lut(&(EncoderCalibrationControl){
                              .foc = &foc, .motor = &motor, .speed_pi = &pi, .startup = &startup})
                   .result == MOTOR_TASK_FINALIZE);
        assert(!EncoderCalibrationTask_IsReleased());
        assert(calibration->calib_flag == (ENC_CALIB_ALL | ENC_CALIB_MECHANICAL_ZERO));
        assert(EncoderCalibrationTask_PublishLut(calibration, &encoder));
        assert(EncoderCalibrationTask_IsReleased());
        assert(!EncoderCalibrationTask_PublishLut(calibration, &encoder));
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
        EncoderCalibrationTask_Cancel();
        assert(!EncoderCalibrationTask_PublishLut(calibration, &encoder));
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
    trace_word((uint32_t)EncoderCalibrationTask_GetStep());
    memcpy(&bits, &motor->speed_target_rad_s, sizeof(bits));
    trace_word(bits);
    memcpy(&bits, &motor->id_target_a, sizeof(bits));
    trace_word(bits);
    memcpy(&bits, &motor->iq_target_a, sizeof(bits));
    trace_word(bits);
}

static void check_flow(void)
{
    MotorConfiguration configuration = {0};
    FocState foc = {0};
    MotorControlState motor = {0};
    PiController speed = {0};
    EncoderState encoder = {0};
    FluxObserverState observer = {0};
    SensorlessStartupState startup = {0};
    EncoderCalibrationWorkspace workspace;
    configuration.model.motor_pole_pairs = 1;
    configuration.model.motor_phase_resistance_ohm = 0.1f;
    configuration.model.motor_d_inductance_h = 0.0002f;
    configuration.model.motor_q_inductance_h = 0.0002f;
    configuration.model.motor_flux_wb = 0.015f;
    configuration.application.current_limit_a = 6.0f;
    encoder.has_valid_sample = true;
    simulate_motion = true;
    motion_ticks = 0U;
    EncoderCalibrationTask_Cancel();
    MotorTaskResult result = {MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};
    uint32_t phases = 0U;
    unsigned ticks = 0U;
    while (result.result == MOTOR_TASK_RUNNING && ticks < 200000U)
    {
        encoder.directed_q15 =
            (uint16_t)(uint32_t)(observer.electrical_position_rad * (65536.0f / _2PI));
        result = step_lut(
            &configuration, &foc, &motor, &speed, &encoder, &observer, &startup, &workspace);
        EncoderCalibrationPhase phase = EncoderCalibrationTask_GetStep();
        if (phase >= CS_OBS_ALIGN_ORIGIN && phase <= CS_OBS_STOP_CURRENT)
        {
            phases |= 1U << (phase - CS_OBS_ALIGN_ORIGIN);
        }
        trace_cycle(result, &motor);
        ++ticks;
    }
    assert(result.result == MOTOR_TASK_FINALIZE && ticks < 200000U);
    assert(phases == 0x1FFU && !EncoderCalibrationTask_IsReleased());
    assert(configuration.device.encoder.calib_flag == 0U);
    assert(EncoderCalibrationTask_PublishLut(&configuration.device.encoder, &encoder));
    assert(configuration.device.encoder.calib_flag == ENC_CALIB_LINEARIZED);
    assert(EncoderCalibrationTask_IsReleased());
    for (unsigned i = 0U; i < ENCODER_OFFSET_LUT_SIZE; ++i)
    {
        trace_word((uint16_t)configuration.device.encoder.linearization_lut_q15[i]);
    }
    simulate_motion = false;

    /* 电零位覆盖斜坡、环形均值、完成后的重入及失联故障。 */
    configuration.model.calibration_current_a = 2.0f;
    encoder.linearized_q15 = 65530U;
    ticks = 0U;
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
    assert(EncoderCalibrationTask_IsReleased());
    result =
        step_electrical_zero(&configuration, &configuration.device.encoder, &foc, &motor, &encoder);
    assert(result.result == MOTOR_TASK_RUNNING && !EncoderCalibrationTask_IsReleased());
    encoder.has_valid_sample = false;
    result =
        step_electrical_zero(&configuration, &configuration.device.encoder, &foc, &motor, &encoder);
    assert(result.result == MOTOR_TASK_FAILED && result.error == MOTOR_FAULT_ENCODER);
    assert(EncoderCalibrationTask_IsReleased());
    EncoderCalibrationTask_Cancel();
    printf("PASS calibration flow: trace=%08x\n", (unsigned)trace_hash);
}

static void check_fault_release(void)
{
    /* 在每个运动阶段撤销输入，必须释放借用、清除目标且不能发布半成品。 */
    for (unsigned failure = 0U; failure < 4U; ++failure)
    {
        for (EncoderCalibrationPhase target = CS_OBS_ALIGN_ORIGIN; target <= CS_OBS_STOP_CURRENT;
             target = (EncoderCalibrationPhase)(target + 1))
        {
            if (failure == 2U && (target < CS_OBS_SPEED_STABLE || target > CS_OBS_VERIFY_CW))
            {
                continue;
            }
            if (failure == 3U && target < CS_OBS_STOP_DECEL)
            {
                continue;
            }
            MotorConfiguration configuration = {0};
            configuration.model.motor_pole_pairs = 1;
            configuration.model.motor_phase_resistance_ohm = 0.1f;
            configuration.model.motor_d_inductance_h = 0.0002f;
            configuration.model.motor_q_inductance_h = 0.0002f;
            configuration.model.motor_flux_wb = 0.015f;
            configuration.application.current_limit_a = 6.0f;
            FocState foc = {0};
            MotorControlState motor = {0};
            PiController speed = {0};
            EncoderState encoder = {.has_valid_sample = true};
            FluxObserverState observer = {0};
            SensorlessStartupState startup = {0};
            EncoderCalibrationWorkspace workspace, other_workspace;
            EncoderCalibrationTask_Cancel();
            simulate_motion = true;
            motion_ticks = 0U;
            unsigned ticks = 0U;
            while (EncoderCalibrationTask_GetStep() != target && ticks++ < 200000U)
            {
                encoder.directed_q15 =
                    (uint16_t)(uint32_t)(observer.electrical_position_rad * (65536.0f / _2PI));
                MotorTaskResult result = step_lut(&configuration,
                                                  &foc,
                                                  &motor,
                                                  &speed,
                                                  &encoder,
                                                  &observer,
                                                  &startup,
                                                  &workspace);
                assert(result.result == MOTOR_TASK_RUNNING);
            }
            assert(ticks < 200000U);
            if (failure == 0U)
            {
                encoder.has_valid_sample = false;
            }
            else if (failure == 2U)
            {
                ++observer.position_epoch;
            }
            else if (failure == 3U)
            {
                observer.electrical_velocity_rad_s = 0.0f;
            }
            MotorTaskResult result = step_lut(&configuration,
                                              &foc,
                                              &motor,
                                              &speed,
                                              &encoder,
                                              &observer,
                                              &startup,
                                              failure == 1U ? &other_workspace : &workspace);
            if (failure == 3U)
            {
                assert(result.result == MOTOR_TASK_FINALIZE);
                assert(!EncoderCalibrationTask_IsReleased());
            }
            else
            {
                assert(result.result == MOTOR_TASK_FAILED);
                assert(result.error ==
                       (failure == 2U ? MOTOR_FAULT_SENSORLESS : MOTOR_FAULT_ENCODER));
                assert(EncoderCalibrationTask_IsReleased());
                assert(!EncoderCalibrationTask_PublishLut(&configuration.device.encoder, &encoder));
            }
            assert(motor.id_target_a == 0.0f && motor.iq_target_a == 0.0f);
            assert(motor.speed_target_rad_s == 0.0f && motor.speed_ramped_rad_s == 0.0f);
            assert(configuration.device.encoder.calib_flag == 0U);
            EncoderCalibrationTask_Cancel();
        }
    }
    simulate_motion = false;
    puts("PASS calibration fault release and observer braking cutoff");
}

#include "../encoder_calibration_core_test.c"

int main(void)
{
    assert(check_calibration_core() == 0);
    check_finite();
    check_observer();
    check_prepare();
    check_commit();
    check_flow();
    check_fault_release();
    printf("PASS calibration control: trace=%08x\n", (unsigned)control_hash);
    puts("PASS observer finite/phase equivalence and bounded calibration prepare/cancel/reentry");
    return 0;
}
