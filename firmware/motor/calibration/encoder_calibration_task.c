#include "encoder_calibration_task.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "control_config.h"
#include "motor_fault.h"
#include "motor_startup_profile.h"
#include "control_math.h"

/* Mode13/15 任务适配：调度与准入留在本层，采样/成表/验证/角均值归纯计算核心。
 * 任务不自行停相、写模式或保存；本地中止只清理采样引用与控制状态。 */

/* 固定纯计算表示与既有反馈/工作区 ABI 必须一致。 */
typedef char EncoderCalibrationRepresentationMatches
    [(ENCODER_CALIBRATION_Q15_CPR == ENCODER_Q15_CPR &&
      ENCODER_CALIBRATION_Q15_HALF_TURN == ENCODER_Q15_HALF_TURN &&
      ENCODER_CALIBRATION_LUT_SIZE == ENCODER_OFFSET_LUT_SIZE &&
      ENCODER_CALIBRATION_LUT_BITS == ENCODER_OFFSET_LUT_BITS &&
      sizeof(EncoderCalibrationWorkspace) == 6144U)
         ? 1
         : -1];

static EncoderCalibrationWorkspace *s_encoder_calib_workspace = NULL;
static uint16_t s_prepare_bin;

static EncoderCalibrationPhase s_calibration_phase = CS_NULL;
static uint32_t s_zero_loop_count, s_zero_sample_count;
static uint16_t s_zero_sample_anchor;
static int64_t s_zero_unwrapped_sum;

#define ENC_ZERO_ALIGN_TIME (ENCODER_ELEC_ZERO_CURRENT_RAMP_TIME_S + ENCODER_ELEC_ZERO_HOLD_TIME_S)
#define ENC_ZERO_SAMPLE_TIME ENCODER_ELEC_ZERO_HOLD_TIME_S

void EncoderCalibrationTask_Cancel(void)
{
    s_encoder_calib_workspace = NULL;
    s_prepare_bin = 0U;
    s_calibration_phase = CS_NULL;
    s_zero_loop_count = s_zero_sample_count = 0U;
    s_zero_unwrapped_sum = 0;
}
bool EncoderCalibrationTask_IsReleased(void)
{
    return s_encoder_calib_workspace == NULL && s_calibration_phase == CS_NULL &&
           s_zero_loop_count == 0U;
}
EncoderCalibrationPhase EncoderCalibrationTask_GetStep(void)
{
    return s_calibration_phase;
}

static bool encoder_calib_prepare_samples(EncoderCalibrationWorkspace *workspace)
{
    if (workspace == NULL)
    {
        s_encoder_calib_workspace = NULL;
        return false;
    }
    s_encoder_calib_workspace = workspace;
    s_prepare_bin = 0U;
    return true;
}

/* 本地中止：释放采样区并复位控制状态；不触碰功率级与模式（由运行状态机执行）。 */
static void encoder_observer_calib_reset(FocState *foc,
                                         MotorControlState *motor_state,
                                         PiController *speed_pi,
                                         SensorlessStartupState *startup)
{
    s_encoder_calib_workspace = NULL;
    s_prepare_bin = 0U;
    SensorlessStartup_Reset(startup);
    FOC_CurrentController_Reset(foc);
    PiController_Reset(speed_pi);
    motor_state->speed_target_rad_s = 0.0f;
    motor_state->speed_ramped_rad_s = 0.0f;
    motor_state->id_target_a = 0.0f;
    motor_state->iq_target_a = 0.0f;
    s_calibration_phase = CS_NULL;
}

/* 故障上报先于控制状态清理；错误码与原阶段分支保持一致。 */
static MotorTaskResult encoder_observer_calib_fail(FocState *foc,
                                                   MotorControlState *motor_state,
                                                   PiController *speed_pi,
                                                   SensorlessStartupState *startup,
                                                   MotorFaultCode error)
{
    MotorFault_Report(motor_state, error);
    encoder_observer_calib_reset(foc, motor_state, speed_pi, startup);
    return (MotorTaskResult){MOTOR_TASK_FAILED, error};
}

static MotorTaskResult encoder_observer_calib_finish(FocState *foc,
                                                     MotorControlState *motor_state,
                                                     PiController *speed_pi,
                                                     SensorlessStartupState *startup)
{
    /* 保存前结束转矩，避免低速观测器在转子过零后继续施加负 iq。 */
    encoder_observer_calib_reset(foc, motor_state, speed_pi, startup);
    /* 功率级关断与任务衔接由主状态机执行。 */
    return (MotorTaskResult){MOTOR_TASK_COMPLETED, MOTOR_FAULT_NONE};
}

static void encoder_calib_commit_candidate_lut(EncoderCalibration *calibration,
                                               EncoderState *encoder)
{
    uint16_t current_linearized_q15;

    memcpy(calibration->linearization_lut_q15,
           s_encoder_calib_workspace->bins.candidate,
           sizeof(calibration->linearization_lut_q15));
    current_linearized_q15 = EncoderCalibrationCore_ApplyLut(
        s_encoder_calib_workspace->bins.candidate, encoder->directed_q15);
    encoder->linearized_q15 = current_linearized_q15;
    calibration->electrical_zero_q15 = 0U;
    calibration->mechanical_zero_q15 = 0U;
    calibration->calib_flag &= (uint8_t)~(ENC_CALIB_ELECTRICAL_ZERO | ENC_CALIB_MECHANICAL_ZERO);
    calibration->calib_flag |= ENC_CALIB_LINEARIZED;
    /* 慢状态由 2 kHz 慢估计独占：请求重定多圈基准（基准取当前线性化角、零影子归零）。 */
    encoder->rebase_requested = true;
    Encoder_ResetVelocity(encoder);
}

static bool encoder_observer_calib_is_stable(const MotorConfiguration *configuration,
                                             const MotorControlState *motor_state,
                                             FluxObserverState *flux_observer,
                                             SensorlessStartupState *startup,
                                             uint32_t position_epoch)
{
    float target_electrical_speed =
        SENSORLESS_ENCODER_CALIB_SPEED_MEC_RAD_S * (float)configuration->model.motor_pole_pairs;
    float filtered_electrical_speed =
        startup->speed_feedback_rad_s * (float)configuration->model.motor_pole_pairs;

    return startup->state == SENSORLESS_STARTUP_CLOSED_LOOP &&
           Observer_GetPositionEpoch(flux_observer) == position_epoch &&
           Math_Abs(filtered_electrical_speed - target_electrical_speed) <=
               Math_Abs(target_electrical_speed) * SENSORLESS_ENCODER_CALIB_SPEED_ERROR_RATIO &&
           Math_Abs(motor_state->speed_ramped_rad_s - SENSORLESS_ENCODER_CALIB_SPEED_MEC_RAD_S) <=
               SENSORLESS_ENCODER_CALIB_SPEED_MEC_RAD_S *
                   SENSORLESS_ENCODER_CALIB_SPEED_ERROR_RATIO;
}

static bool encoder_observer_calib_is_tracking(FluxObserverState *flux_observer,
                                               SensorlessStartupState *startup,
                                               uint32_t position_epoch)
{
    return startup->state == SENSORLESS_STARTUP_CLOSED_LOOP &&
           Observer_GetPositionEpoch(flux_observer) == position_epoch;
}

static float encoder_observer_calib_get_stop_speed(const MotorConfiguration *configuration)
{
    float minimum_mechanical_speed =
        SensorlessStartup_EncoderCalibConfig.minimum_electrical_velocity_rad_s /
        (float)configuration->model.motor_pole_pairs;
    float stop_speed = minimum_mechanical_speed * SENSORLESS_ENCODER_CALIB_STOP_SPEED_MARGIN;

    return stop_speed < SENSORLESS_ENCODER_CALIB_SPEED_MEC_RAD_S
               ? stop_speed
               : SENSORLESS_ENCODER_CALIB_SPEED_MEC_RAD_S;
}

static bool encoder_observer_calib_can_brake(const MotorConfiguration *configuration,
                                             const EncoderState *encoder,
                                             FluxObserverState *flux_observer,
                                             SensorlessStartupState *startup,
                                             uint32_t position_epoch)
{
    float minimum_speed = SensorlessStartup_EncoderCalibConfig.minimum_electrical_velocity_rad_s;
    float observer_speed = Observer_GetEleVel(flux_observer);
    float observer_phase = Observer_GetElePhase(flux_observer);
    float encoder_speed = Encoder_GetMecVelContinuous(encoder);

    if (!encoder_observer_calib_is_tracking(flux_observer, startup, position_epoch) ||
        !(observer_speed > minimum_speed) ||
        observer_speed > SENSORLESS_OBSERVER_MAX_ELEC_VEL_RAD_S ||
        !(observer_phase >= 0.0f && observer_phase <= _2PI))
    {
        return false;
    }
    /* 发布 LUT 会重启测速；就绪后以编码器速度独立提前截止制动，不依赖电零位。 */
    return !encoder->velocity_ready ||
           (encoder_speed > minimum_speed / (float)configuration->model.motor_pole_pairs &&
            encoder_speed <= SENSORLESS_OBSERVER_MAX_ELEC_VEL_RAD_S /
                                 (float)configuration->model.motor_pole_pairs);
}

/**
 * @brief Mode13 用无感观测器重建线性化 LUT；静态零相位对齐建立本次机械原点。
 */
MotorTaskResult EncoderCalibrationTask_StepLut(const MotorConfiguration *configuration,
                                               EncoderCalibration *calibration,
                                               FocState *foc,
                                               MotorControlState *motor_state,
                                               PiController *speed_pi,
                                               EncoderState *encoder,
                                               FluxObserverState *flux_observer,
                                               SensorlessStartupState *startup,
                                               EncoderCalibrationWorkspace *workspace)
{
    static uint32_t state_ticks;
    static uint32_t stage_ticks;
    static uint32_t observer_position_epoch;
    static uint16_t origin_anchor_q15;
    static int64_t origin_sum_q15;
    static uint32_t origin_sample_count;
    static uint16_t origin_q15;
    static uint16_t previous_directed_q15;
    static float previous_observer_position;
    static float observer_position_origin;
    /* 采样与验证互斥；复用窗口，进入验证前整体清零以隔离两段行程。 */
    static EncoderCalibrationVerifyState verify_state;
    static float stop_start_speed;
    static float stop_current_ref;
    static bool origin_negative_seen;
    static EncoderCalibrationBuildState build_state;
    float required_electrical_theta;
    float observer_position;
    float relative_theta;
    MotorTaskResult outcome = {MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};

    if (s_encoder_calib_workspace != NULL && workspace != s_encoder_calib_workspace)
    {
        return encoder_observer_calib_fail(
            foc, motor_state, speed_pi, startup, MOTOR_FAULT_ENCODER);
    }

    if (!Encoder_IsOnline(encoder))
    {
        return encoder_observer_calib_fail(
            foc, motor_state, speed_pi, startup, MOTOR_FAULT_ENCODER);
    }
    if (configuration->model.motor_pole_pairs <= 0)
    {
        return encoder_observer_calib_fail(
            foc, motor_state, speed_pi, startup, MOTOR_FAULT_POLE_PAIRS);
    }
    if (configuration->model.motor_phase_resistance_ohm <= 0.0f ||
        configuration->model.motor_d_inductance_h <= 0.0f ||
        configuration->model.motor_q_inductance_h <= 0.0f ||
        configuration->model.motor_flux_wb <= 0.0f ||
        configuration->application.current_limit_a <= 0.0f ||
        configuration->application.current_limit_a < SENSORLESS_ENCODER_CALIB_MIN_CURRENT_LIMIT_A)
    {
        return encoder_observer_calib_fail(
            foc, motor_state, speed_pi, startup, MOTOR_FAULT_INVALID_PARAMETERS);
    }

    if (s_calibration_phase == CS_NULL)
    {
        if (!encoder_calib_prepare_samples(workspace))
        {
            return encoder_observer_calib_fail(
                foc, motor_state, speed_pi, startup, MOTOR_FAULT_ENCODER);
        }

        SensorlessStartup_Reset(startup);
        FOC_CurrentController_Reset(foc);
        PiController_Reset(speed_pi);
        state_ticks = 0U;
        stage_ticks = 0U;
        observer_position_epoch = 0U;
        origin_anchor_q15 = 0U;
        origin_sum_q15 = 0;
        origin_sample_count = 0U;
        origin_q15 = 0U;
        previous_directed_q15 = 0U;
        previous_observer_position = 0.0f;
        observer_position_origin = 0.0f;
        verify_state = (EncoderCalibrationVerifyState){0};
        stop_start_speed = SENSORLESS_ENCODER_CALIB_SPEED_MEC_RAD_S;
        stop_current_ref = 0.0f;
        origin_negative_seen = false;
        build_state = (EncoderCalibrationBuildState){0};
        s_calibration_phase = CS_OBS_ALIGN_ORIGIN;
    }

    if ((s_calibration_phase == CS_OBS_STOP_DECEL || s_calibration_phase == CS_OBS_STOP_CURRENT) &&
        !encoder_observer_calib_can_brake(
            configuration, encoder, flux_observer, startup, observer_position_epoch))
    {
        /* 两个停止段均已验证并发布 LUT；低于观测器可用速度时撤销转矩，避免驱动过零。 */
        return encoder_observer_calib_finish(foc, motor_state, speed_pi, startup);
    }

    if (s_calibration_phase == CS_OBS_STOP_CURRENT)
    {
        float current_ratio;

        current_ratio = Math_Constrain(((float)state_ticks + 1.0f) * Current_Ts /
                                           SENSORLESS_ENCODER_CALIB_STOP_CURRENT_RAMP_TIME_S,
                                       0.0f,
                                       1.0f);
        motor_state->id_target_a = 0.0f;
        motor_state->iq_target_a = stop_current_ref * (1.0f - current_ratio);
        FOC_Current(configuration,
                    foc,
                    motor_state,
                    Observer_GetElePhase(flux_observer),
                    Observer_GetEleVel(flux_observer));
    }
    else
    {
        if (s_calibration_phase == CS_OBS_STOP_DECEL)
        {
            startup->speed_pi_output_max = 0.0f;
            float decel_ratio = Math_Constrain((float)state_ticks * Current_Ts /
                                                   SENSORLESS_ENCODER_CALIB_STOP_DECEL_TIME_S,
                                               0.0f,
                                               1.0f);
            float stop_speed = encoder_observer_calib_get_stop_speed(configuration);

            motor_state->speed_target_rad_s =
                stop_start_speed + (stop_speed - stop_start_speed) * decel_ratio;
            motor_state->speed_ramped_rad_s = motor_state->speed_target_rad_s;
        }
        else
        {
            startup->speed_pi_output_max = 1.0f;
            motor_state->speed_target_rad_s = SENSORLESS_ENCODER_CALIB_SPEED_MEC_RAD_S;
        }

        SensorlessStartup_Run(configuration,
                              foc,
                              motor_state,
                              speed_pi,
                              flux_observer,
                              startup,
                              &SensorlessStartup_EncoderCalibConfig);
        if (motor_state->fault_code != MOTOR_FAULT_NONE)
        {
            encoder_observer_calib_reset(foc, motor_state, speed_pi, startup);
            outcome.result = MOTOR_TASK_FAILED;
            outcome.error = motor_state->fault_code;
            return outcome;
        }
    }

    required_electrical_theta = _2PI * (float)configuration->model.motor_pole_pairs;
    observer_position = Observer_GetElePosition(flux_observer);

    /* 找原点、采样、构表、验证共享跟踪准入；稳速窗口和停止段仍保留各自判据。 */
    if (s_calibration_phase >= CS_OBS_FIND_ORIGIN && s_calibration_phase <= CS_OBS_VERIFY_CW &&
        !encoder_observer_calib_is_tracking(flux_observer, startup, observer_position_epoch))
    {
        return encoder_observer_calib_fail(
            foc, motor_state, speed_pi, startup, MOTOR_FAULT_SENSORLESS);
    }

    switch (s_calibration_phase)
    {
    case CS_OBS_ALIGN_ORIGIN:
        /* 原对齐时间内每拍清16 bin；采样尚未开始，不集中清空6 KiB工作区。 */
        for (uint16_t count = 0U; count < 16U && s_prepare_bin < ENCODER_OFFSET_LUT_SIZE; ++count)
        {
            s_encoder_calib_workspace->error_sum[s_prepare_bin] = 0;
            s_encoder_calib_workspace->bins.samples[s_prepare_bin] = 0U;
            ++s_prepare_bin;
        }
        if (startup->state == SENSORLESS_STARTUP_ALIGN &&
            startup->state_ticks >=
                (uint32_t)(((SensorlessStartup_EncoderCalibConfig.align_current_ramp_time_s +
                             SensorlessStartup_EncoderCalibConfig.align_hold_time_s) -
                            SENSORLESS_ENCODER_CALIB_ALIGN_SAMPLE_TIME_S) /
                           Current_Ts))
        {
            uint16_t directed_q15 = encoder->directed_q15;
            if (origin_sample_count == 0U)
            {
                origin_anchor_q15 = directed_q15;
            }
            EncoderCalibrationCore_AddAngleSample(
                directed_q15, origin_anchor_q15, &origin_sum_q15, &origin_sample_count);
        }

        if (startup->state != SENSORLESS_STARTUP_ALIGN)
        {
            if (s_prepare_bin != ENCODER_OFFSET_LUT_SIZE ||
                !EncoderCalibrationCore_MeanQ15(origin_sum_q15, origin_sample_count, &origin_q15))
            {
                return encoder_observer_calib_fail(
                    foc, motor_state, speed_pi, startup, MOTOR_FAULT_ENCODER);
            }

            state_ticks = 0U;
            stage_ticks = 0U;
            s_calibration_phase = CS_OBS_WAIT_CLOSED_LOOP;
        }
        break;

    case CS_OBS_WAIT_CLOSED_LOOP:
        if (startup->state == SENSORLESS_STARTUP_CLOSED_LOOP)
        {
            observer_position_epoch = Observer_GetPositionEpoch(flux_observer);
            state_ticks = 0U;
            stage_ticks = 0U;
            s_calibration_phase = CS_OBS_SPEED_STABLE;
        }
        else if (++stage_ticks >=
                 (uint32_t)(SENSORLESS_ENCODER_CALIB_STARTUP_TIMEOUT_S / Current_Ts))
        {
            return encoder_observer_calib_fail(
                foc, motor_state, speed_pi, startup, MOTOR_FAULT_SENSORLESS);
        }
        break;

    case CS_OBS_SPEED_STABLE:
        if (Observer_GetPositionEpoch(flux_observer) != observer_position_epoch)
        {
            return encoder_observer_calib_fail(
                foc, motor_state, speed_pi, startup, MOTOR_FAULT_SENSORLESS);
        }

        if (encoder_observer_calib_is_stable(
                configuration, motor_state, flux_observer, startup, observer_position_epoch))
        {
            if (++state_ticks >=
                (uint32_t)(SENSORLESS_ENCODER_CALIB_SPEED_STABLE_TIME_S / Current_Ts))
            {
                previous_directed_q15 = encoder->directed_q15;
                previous_observer_position = observer_position;
                origin_negative_seen = false;
                state_ticks = 0U;
                stage_ticks = 0U;
                s_calibration_phase = CS_OBS_FIND_ORIGIN;
            }
        }
        else
        {
            state_ticks = 0U;
        }
        if (++stage_ticks >=
            (uint32_t)(SENSORLESS_ENCODER_CALIB_SPEED_STABLE_TIMEOUT_S / Current_Ts))
        {
            return encoder_observer_calib_fail(
                foc, motor_state, speed_pi, startup, MOTOR_FAULT_SENSORLESS);
        }
        break;

    case CS_OBS_FIND_ORIGIN:
    {
        int16_t previous_relative =
            EncoderCalibrationCore_Q15Difference(previous_directed_q15, origin_q15);
        int16_t current_relative =
            EncoderCalibrationCore_Q15Difference(encoder->directed_q15, origin_q15);

        if (current_relative < -64)
        {
            origin_negative_seen = true;
        }

        if (origin_negative_seen && previous_relative < 0 && current_relative >= 0)
        {
            float crossing_fraction =
                (float)(-previous_relative) / (float)(current_relative - previous_relative);

            observer_position_origin =
                previous_observer_position +
                crossing_fraction * (observer_position - previous_observer_position);
            verify_state.window.previous_observer_position_rad = observer_position;
            verify_state.window.valid_electrical_travel_rad = 0.0f;
            stage_ticks = 0U;
            s_calibration_phase = CS_OBS_SAMPLE_CW;
            break;
        }

        previous_directed_q15 = encoder->directed_q15;
        previous_observer_position = observer_position;
        if (++stage_ticks >=
            (uint32_t)(SENSORLESS_ENCODER_CALIB_FIND_ORIGIN_TIMEOUT_S / Current_Ts))
        {
            return encoder_observer_calib_fail(
                foc, motor_state, speed_pi, startup, MOTOR_FAULT_SENSORLESS);
        }
        break;
    }

    case CS_OBS_SAMPLE_CW:
    {
        relative_theta = observer_position - observer_position_origin;
        EncoderCalibrationSample sample = {
            .observer_position_rad = observer_position,
            .relative_theta_rad = relative_theta,
            .required_electrical_theta_rad = required_electrical_theta,
            .directed_q15 = encoder->directed_q15,
            .stable = encoder_observer_calib_is_stable(
                configuration, motor_state, flux_observer, startup, observer_position_epoch),
        };
        EncoderCalibrationCore_SampleLut(s_encoder_calib_workspace, &verify_state.window, &sample);

        if (verify_state.window.valid_electrical_travel_rad >=
            (float)SENSORLESS_ENCODER_CALIB_MECH_TURNS * required_electrical_theta)
        {
            build_state = (EncoderCalibrationBuildState){0};
            s_calibration_phase = CS_OBS_BUILD_LUT;
            break;
        }
        if (++stage_ticks >= (uint32_t)(SENSORLESS_ENCODER_CALIB_SAMPLE_TIMEOUT_S / Current_Ts))
        {
            return encoder_observer_calib_fail(
                foc, motor_state, speed_pi, startup, MOTOR_FAULT_SENSORLESS);
        }
        break;
    }

    case CS_OBS_BUILD_LUT:
    {
        EncoderCalibrationBuildResult build_result =
            EncoderCalibrationCore_BuildLutStep(s_encoder_calib_workspace,
                                                &build_state,
                                                SENSORLESS_ENCODER_CALIB_LUT_BUILD_BINS_PER_CYCLE,
                                                SENSORLESS_ENCODER_CALIB_MIN_SAMPLES_PER_BIN);
        if (build_result == ENCODER_CALIBRATION_BUILD_INVALID)
        {
            return encoder_observer_calib_fail(
                foc, motor_state, speed_pi, startup, MOTOR_FAULT_ENCODER);
        }
        if (build_result == ENCODER_CALIBRATION_BUILD_COMPLETE)
        {
            verify_state = (EncoderCalibrationVerifyState){0};
            verify_state.window.previous_observer_position_rad = observer_position;
            stage_ticks = 0U;
            s_calibration_phase = CS_OBS_VERIFY_CW;
        }
        break;
    }

    case CS_OBS_VERIFY_CW:
    {
        relative_theta = observer_position - observer_position_origin;
        EncoderCalibrationSample verification_sample = {
            .observer_position_rad = observer_position,
            .relative_theta_rad = relative_theta,
            .required_electrical_theta_rad = required_electrical_theta,
            .directed_q15 = encoder->directed_q15,
            .stable = encoder_observer_calib_is_stable(
                configuration, motor_state, flux_observer, startup, observer_position_epoch),
        };
        EncoderCalibrationCore_VerifyLut(
            &verify_state, s_encoder_calib_workspace->bins.candidate, &verification_sample);

        if (verify_state.window.valid_electrical_travel_rad >=
            (float)SENSORLESS_ENCODER_CALIB_VERIFY_MECH_TURNS * required_electrical_theta)
        {
            if (!EncoderCalibrationCore_ValidateLut(&verify_state,
                                                    SENSORLESS_ENCODER_CALIB_MAX_RMS_RESIDUAL_Q15,
                                                    SENSORLESS_ENCODER_CALIB_MAX_PEAK_RESIDUAL_Q15))
            {
                return encoder_observer_calib_fail(
                    foc, motor_state, speed_pi, startup, MOTOR_FAULT_ENCODER);
            }

            encoder_calib_commit_candidate_lut(calibration, encoder);
            s_encoder_calib_workspace = NULL;
            stop_start_speed = motor_state->speed_ramped_rad_s;
            float stop_speed = encoder_observer_calib_get_stop_speed(configuration);
            if (stop_start_speed < stop_speed)
            {
                stop_start_speed = stop_speed;
            }
            state_ticks = 0U;
            s_calibration_phase = CS_OBS_STOP_DECEL;
            break;
        }
        if (++stage_ticks >= (uint32_t)(SENSORLESS_ENCODER_CALIB_VERIFY_TIMEOUT_S / Current_Ts))
        {
            return encoder_observer_calib_fail(
                foc, motor_state, speed_pi, startup, MOTOR_FAULT_SENSORLESS);
        }
        break;
    }

    case CS_OBS_STOP_DECEL:
    {
        float stop_speed = encoder_observer_calib_get_stop_speed(configuration);
        float observer_mech_vel =
            Observer_GetEleVel(flux_observer) / (float)configuration->model.motor_pole_pairs;
        bool speed_reached =
            Math_Abs(observer_mech_vel) <=
            stop_speed * (1.0f + SENSORLESS_ENCODER_CALIB_STOP_SPEED_TOLERANCE_RATIO);

        state_ticks++;
        if (state_ticks >= (uint32_t)(SENSORLESS_ENCODER_CALIB_STOP_DECEL_TIME_S / Current_Ts) &&
            speed_reached)
        {
            stop_current_ref = Math_Min(motor_state->iq_target_a, 0.0f);
            state_ticks = 0U;
            s_calibration_phase = CS_OBS_STOP_CURRENT;
        }
        else if (state_ticks >=
                 (uint32_t)(SENSORLESS_ENCODER_CALIB_STOP_DECEL_TIMEOUT_S / Current_Ts))
        {
            return encoder_observer_calib_fail(
                foc, motor_state, speed_pi, startup, MOTOR_FAULT_SENSORLESS);
        }
        break;
    }

    case CS_OBS_STOP_CURRENT:
        if (++state_ticks >=
            (uint32_t)(SENSORLESS_ENCODER_CALIB_STOP_CURRENT_RAMP_TIME_S / Current_Ts))
        {
            return encoder_observer_calib_finish(foc, motor_state, speed_pi, startup);
        }
        break;

    default:
        return encoder_observer_calib_fail(
            foc, motor_state, speed_pi, startup, MOTOR_FAULT_ENCODER);
    }
    return outcome;
}

/**
 * @brief Mode15 以对齐后的线性化 Q15 平均角确定电零位。
 */
MotorTaskResult EncoderCalibrationTask_StepElectricalZero(const MotorConfiguration *configuration,
                                                          EncoderCalibration *calibration,
                                                          FocState *foc,
                                                          MotorControlState *motor_state,
                                                          EncoderState *encoder)
{
    MotorTaskResult outcome = {MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};
    float time = (float)s_zero_loop_count * Current_Ts;
    float align_current = configuration->model.calibration_current_a;

    if (align_current < ENCODER_ELEC_ZERO_MIN_ALIGN_CURRENT_A)
    {
        align_current = ENCODER_ELEC_ZERO_MIN_ALIGN_CURRENT_A;
    }

    if (!Encoder_IsOnline(encoder))
    {
        MotorFault_Report(motor_state, MOTOR_FAULT_ENCODER);
        s_zero_loop_count = 0U;
        s_zero_sample_count = 0U;
        s_zero_unwrapped_sum = 0;
        outcome.result = MOTOR_TASK_FAILED;
        outcome.error = MOTOR_FAULT_ENCODER;
        return outcome;
    }
    if ((calibration->calib_flag & ENC_CALIB_LINEARIZED) == 0U)
    {
        MotorFault_Report(motor_state, MOTOR_FAULT_ENCODER_NOT_CALIBRATED);
        outcome.result = MOTOR_TASK_FAILED;
        outcome.error = MOTOR_FAULT_ENCODER_NOT_CALIBRATED;
        return outcome;
    }
    if (align_current <= 0.0f)
    {
        MotorFault_Report(motor_state, MOTOR_FAULT_INVALID_PARAMETERS);
        outcome.result = MOTOR_TASK_FAILED;
        outcome.error = MOTOR_FAULT_INVALID_PARAMETERS;
        return outcome;
    }
    if (configuration->application.current_limit_a > 0.0f &&
        align_current > configuration->application.current_limit_a)
    {
        align_current = configuration->application.current_limit_a;
    }

    if (s_zero_loop_count == 0U)
    {
        s_zero_sample_count = 0U;
        s_zero_sample_anchor = encoder->linearized_q15;
        s_zero_unwrapped_sum = 0;
        calibration->calib_flag &= (uint8_t)~ENC_CALIB_ELECTRICAL_ZERO;
        FOC_CurrentController_Reset(foc);
    }

    motor_state->id_target_a =
        align_current * Math_Constrain(((float)s_zero_loop_count + 1.0f) * Current_Ts /
                                           ENCODER_ELEC_ZERO_CURRENT_RAMP_TIME_S,
                                       0.0f,
                                       1.0f);
    motor_state->iq_target_a = 0.0f;
    FOC_Current(configuration, foc, motor_state, 0.0f, 0.0f);

    if (time >= (ENC_ZERO_ALIGN_TIME - ENC_ZERO_SAMPLE_TIME) &&
        encoder->read_status == ENCODER_READ_OK)
    {
        EncoderCalibrationCore_AddAngleSample(encoder->linearized_q15,
                                              s_zero_sample_anchor,
                                              &s_zero_unwrapped_sum,
                                              &s_zero_sample_count);
    }

    if (time >= ENC_ZERO_ALIGN_TIME)
    {
        bool calibrated = false;
        uint16_t electrical_zero_q15;
        if (EncoderCalibrationCore_MeanQ15(
                s_zero_unwrapped_sum, s_zero_sample_count, &electrical_zero_q15))
        {
            calibrated = Encoder_SetElectricalZeroQ15(calibration, encoder, electrical_zero_q15);
        }

        motor_state->id_target_a = 0.0f;
        motor_state->iq_target_a = 0.0f;
        FOC_CurrentController_Reset(foc);
        s_zero_loop_count = 0U;
        s_zero_sample_count = 0U;
        s_zero_unwrapped_sum = 0;
        /* 停相与任务衔接由主状态机执行；旧实现的"高边短接保持"
         * 不再由模块直接操作功率级。 */

        if (!calibrated)
        {
            MotorFault_Report(motor_state, MOTOR_FAULT_ENCODER);
            outcome.result = MOTOR_TASK_FAILED;
            outcome.error = MOTOR_FAULT_ENCODER;
            return outcome;
        }
        outcome.result = MOTOR_TASK_COMPLETED;

        return outcome;
    }

    s_zero_loop_count++;
    return outcome;
}
