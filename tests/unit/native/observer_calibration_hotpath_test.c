/* 仅内存的生产观测器/标定准备路径检查；不访问平台、PWM或Flash。 */
#undef NDEBUG
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "flux_observer.c"
#include "encoder_calibration_task.c"

static bool leave_align;
static unsigned current_calls;
const SensorlessStartupConfig SensorlessStartup_EncoderCalibConfig = {
    .align_current_ramp_time_s = SENSORLESS_ENCODER_CALIB_ALIGN_CURRENT_RAMP_TIME_S,
    .align_hold_time_s = SENSORLESS_ENCODER_CALIB_ALIGN_HOLD_TIME_S,
};

void Clarke_Transform(float a, float b, float c, float *alpha, float *beta)
{
    *alpha = a;
    *beta = (b - c) * 0.5773502692f;
}
void MotorFault_Report(MotorControlState *motor, MotorFaultCode fault)
{
    motor->fault_code = fault;
}
void SensorlessStartup_Reset(SensorlessStartupState *startup)
{
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
    (void)configuration;
    (void)foc;
    (void)motor;
    (void)controller;
    (void)observer;
    assert(config == &SensorlessStartup_EncoderCalibConfig);
    startup->state = leave_align ? SENSORLESS_STARTUP_OPEN_LOOP : SENSORLESS_STARTUP_ALIGN;
    ++startup->state_ticks;
}
void FOC_CurrentController_Reset(FocState *foc)
{
    PiController_Reset(&foc->id_pi);
    PiController_Reset(&foc->iq_pi);
}
void FOC_Current(const MotorConfiguration *configuration,
                 FocState *foc,
                 MotorControlState *motor,
                 float phase,
                 float velocity)
{
    (void)configuration;
    (void)foc;
    (void)motor;
    (void)phase;
    (void)velocity;
    ++current_calls;
}
bool Encoder_IsOnline(const EncoderState *encoder)
{
    return encoder->has_valid_sample;
}
float Encoder_GetMecVelContinuous(const EncoderState *encoder)
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
    return true;
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
    FocState foc = {0};
    MotorConfiguration configuration = {0};
    FluxObserverState observer = {0};
    configuration.model.motor_phase_resistance_ohm = 0.2f;
    configuration.model.motor_d_inductance_h = 0.0002f;
    configuration.model.motor_q_inductance_h = 0.0003f;
    configuration.model.motor_flux_wb = 0.015f;
    foc.bus_voltage_filtered_v = 33.0f;
    FluxObserver_ParamInit(&observer);
    for (unsigned i = 0U; i < 8192U; ++i)
    {
        float phase = (float)i * 0.017f;
        foc.phase_a_current_a = 0.1f * sinf(phase);
        foc.phase_b_current_a = 0.1f * sinf(phase - 2.0943951f);
        foc.phase_c_current_a = 0.1f * sinf(phase + 2.0943951f);
        foc.mod_alpha = 0.01f * cosf(phase);
        foc.mod_beta = 0.01f * sinf(phase);
        FluxObserver_Update(&configuration, &foc, &observer);
        float reference = Math_NormalizeAngle(Math_Atan2(observer.sin, observer.cos));
        assert(memcmp(&reference, &observer.electrical_angle_rad, sizeof(reference)) == 0);
        assert(isfinite(observer.electrical_velocity_rad_s));
    }
    const float invalid[] = {0.0f, -1.0f, INFINITY, NAN};
    for (unsigned i = 0U; i < sizeof(invalid) / sizeof(invalid[0]); ++i)
    {
        uint32_t epoch = observer.position_epoch;
        configuration.model.motor_flux_wb = invalid[i];
        FluxObserver_Update(&configuration, &foc, &observer);
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
                                      Current_Ts);
    assert(sample_tick >= ENCODER_OFFSET_LUT_SIZE / ENCODER_CALIBRATION_PREPARE_BINS_PER_TICK);
    EncoderCalibrationTask_Cancel();
    memset(&workspace, 0xA5, sizeof(workspace));
    leave_align = false;
    MotorTaskResult result = EncoderCalibrationTask_StepLut(&configuration,
                                                            &configuration.device.encoder,
                                                            &foc,
                                                            &motor,
                                                            &speed,
                                                            &encoder,
                                                            &observer,
                                                            &startup,
                                                            &workspace);
    assert(result.result == MOTOR_TASK_RUNNING && s_prepare_bin == 0U);
    assert(startup.state == SENSORLESS_STARTUP_IDLE && startup.state_ticks == 0U);
    assert(workspace.bins.samples[0] == 0xA5A5U && current_calls == 0U);
    result = EncoderCalibrationTask_StepLut(&configuration,
                                            &configuration.device.encoder,
                                            &foc,
                                            &motor,
                                            &speed,
                                            &encoder,
                                            &observer,
                                            &startup,
                                            &workspace);
    assert(result.result == MOTOR_TASK_RUNNING &&
           s_prepare_bin == ENCODER_CALIBRATION_PREPARE_BINS_PER_TICK);
    assert(workspace.error_sum[0] == 0 &&
           workspace.bins.samples[ENCODER_CALIBRATION_PREPARE_BINS_PER_TICK - 1U] == 0U);
    assert(workspace.bins.samples[ENCODER_CALIBRATION_PREPARE_BINS_PER_TICK] ==
           0xA5A5U); /* 首拍不再清6 KiB。 */
    EncoderCalibrationTask_Cancel();
    assert(EncoderCalibrationTask_IsReleased() && s_prepare_bin == 0U);
    memset(&workspace, 0xA5, sizeof(workspace));
    for (unsigned i = 0U; i <= ENCODER_OFFSET_LUT_SIZE / ENCODER_CALIBRATION_PREPARE_BINS_PER_TICK;
         ++i)
    {
        result = EncoderCalibrationTask_StepLut(&configuration,
                                                &configuration.device.encoder,
                                                &foc,
                                                &motor,
                                                &speed,
                                                &encoder,
                                                &observer,
                                                &startup,
                                                &workspace);
        assert(result.result == MOTOR_TASK_RUNNING);
        assert(s_prepare_bin == i * ENCODER_CALIBRATION_PREPARE_BINS_PER_TICK);
        startup.state_ticks = sample_tick;
    }
    for (unsigned i = 0U; i < ENCODER_OFFSET_LUT_SIZE; ++i)
    {
        assert(workspace.error_sum[i] == 0 && workspace.bins.samples[i] == 0U);
        assert((uint16_t)configuration.device.encoder.linearization_lut_q15[i] == 0x5A5AU);
    }
    leave_align = true;
    result = EncoderCalibrationTask_StepLut(&configuration,
                                            &configuration.device.encoder,
                                            &foc,
                                            &motor,
                                            &speed,
                                            &encoder,
                                            &observer,
                                            &startup,
                                            &workspace);
    assert(result.result == MOTOR_TASK_RUNNING);
    assert(EncoderCalibrationTask_GetStep() == CS_OBS_WAIT_CLOSED_LOOP);
    EncoderCalibrationTask_Cancel();
    result = EncoderCalibrationTask_StepLut(&configuration,
                                            &configuration.device.encoder,
                                            &foc,
                                            &motor,
                                            &speed,
                                            &encoder,
                                            &observer,
                                            &startup,
                                            &workspace);
    assert(result.result == MOTOR_TASK_RUNNING && startup.state_ticks == 0U);
    result = EncoderCalibrationTask_StepLut(&configuration,
                                            &configuration.device.encoder,
                                            &foc,
                                            &motor,
                                            &speed,
                                            &encoder,
                                            &observer,
                                            &startup,
                                            &workspace);
    assert(result.result == MOTOR_TASK_FAILED && result.error == MOTOR_FAULT_ENCODER);
    assert(EncoderCalibrationTask_IsReleased() && s_prepare_bin == 0U);
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
        uint16_t expected =
            EncoderCalibrationCore_ApplyLut(workspace.bins.candidate, encoder.directed_q15);
        s_encoder_calib_workspace = &workspace;
        s_calibration_phase = CS_OBS_VERIFY_CW;
        assert(!EncoderCalibrationTask_PublishLut(calibration, &encoder));
        s_lut_verified = true;
        FocState foc = {0};
        MotorControlState motor = {0};
        PiController pi = {0};
        SensorlessStartupState startup = {0};
        assert(encoder_observer_calib_finish(&foc, &motor, &pi, &startup).result ==
               MOTOR_TASK_FINALIZE);
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
        s_encoder_calib_workspace = &workspace;
        s_calibration_phase = CS_LUT_PUBLISH_PENDING;
        EncoderCalibration before_cancel = *calibration;
        EncoderCalibrationTask_Cancel();
        assert(!EncoderCalibrationTask_PublishLut(calibration, &encoder));
        assert(memcmp(calibration, &before_cancel, sizeof(*calibration)) == 0);
    }
}

int main(void)
{
    check_finite();
    check_observer();
    check_prepare();
    check_commit();
    puts("PASS observer finite/phase equivalence and bounded calibration prepare/cancel/reentry");
    return 0;
}
