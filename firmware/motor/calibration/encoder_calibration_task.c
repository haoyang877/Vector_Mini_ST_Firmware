#include "encoder_calibration_task.h"

#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "control_config.h"
#include "foc_errhandle.h"
#include "motor_startup_profile.h"
#include "control_math.h"

/* Mode 13/15 编码器标定重建（契约见头文件）。
 * 原 foc_calibration 五任务中的 R_L_Flux/EncoderOffset/CurrentOffset 仍待重建：
 * 其辅助函数（分相电阻/电感/磁链扫描、开环扫角偏移、ADC 零偏）未随本模块恢复。
 * 迁移约定：任务不自行停相或写模式；本地中止只清理采样区与控制状态。 */

static EncoderCalibrationWorkspace *s_encoder_calib_workspace = NULL;

static EncoderCalibrationPhase s_calibration_phase = CS_NULL;
static uint32_t s_zero_loop_count, s_zero_sample_count;
static uint16_t s_zero_sample_anchor;
static int64_t s_zero_unwrapped_sum;

#define ENC_ZERO_ALIGN_TIME (ENCODER_ELEC_ZERO_CURRENT_RAMP_TIME_S + ENCODER_ELEC_ZERO_HOLD_TIME_S)
#define ENC_ZERO_SAMPLE_TIME ENCODER_ELEC_ZERO_HOLD_TIME_S

static void encoder_calib_release_samples(void)
{
    if (s_encoder_calib_workspace != NULL)
    {
        s_encoder_calib_workspace = NULL;
    }
}

void EncoderCalibrationTask_Cancel(void)
{
    encoder_calib_release_samples();
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

static int16_t encoder_calib_q15_difference(uint16_t target_q15, uint16_t source_q15)
{
    int32_t difference = (int32_t)target_q15 - (int32_t)source_q15;

    if (difference > ENCODER_Q15_HALF_TURN)
    {
        difference -= (int32_t)ENCODER_Q15_CPR;
    }
    else if (difference < -ENCODER_Q15_HALF_TURN)
    {
        difference += (int32_t)ENCODER_Q15_CPR;
    }

    return (int16_t)difference;
}

static bool encoder_calib_prepare_samples(EncoderCalibrationWorkspace *workspace)
{
    if (workspace == NULL)
    {
        encoder_calib_release_samples();
        return false;
    }
    s_encoder_calib_workspace = workspace;
    memset(s_encoder_calib_workspace, 0, sizeof(*s_encoder_calib_workspace));
    return true;
}

static uint16_t encoder_calib_apply_candidate_lut(uint16_t directed_q15)
{
    uint16_t lut_index = directed_q15 >> 6;
    uint16_t fraction = directed_q15 & 0x003FU;
    int32_t correction_a = s_encoder_calib_workspace->bins.candidate[lut_index];
    int32_t correction_b = s_encoder_calib_workspace->bins
                               .candidate[(lut_index + 1U) & (ENCODER_OFFSET_LUT_SIZE - 1U)];
    int32_t correction = correction_a + (((correction_b - correction_a) * fraction) >> 6);

    return (uint16_t)((int32_t)directed_q15 - correction);
}

/* 本地中止：释放采样区并复位控制状态；不触碰功率级与模式（由运行状态机执行）。 */
static void encoder_observer_calib_abort(FocState *foc,
                                         MotorControlState *motor_state,
                                         PiController *speed_pi,
                                         SensorlessStartupState *startup)
{
    encoder_calib_release_samples();
    SensorlessStartup_Reset(startup);
    FOC_CurrentController_Reset(foc);
    PiController_Reset(speed_pi);
    motor_state->speed_target_rad_s = 0.0f;
    motor_state->speed_ramped_rad_s = 0.0f;
    motor_state->id_target_a = 0.0f;
    motor_state->iq_target_a = 0.0f;
    s_calibration_phase = CS_NULL;
}

static MotorTaskResult encoder_observer_calib_finish(FocState *foc,
                                                     MotorControlState *motor_state,
                                                     PiController *speed_pi,
                                                     SensorlessStartupState *startup)
{
    MotorTaskResult outcome = {MOTOR_TASK_COMPLETED, MOTOR_FAULT_NONE};

    /* End active torque before saving. A low-speed flux observer must not
     * keep applying negative iq_a after the rotor has crossed zero. */
    SensorlessStartup_Reset(startup);
    FOC_CurrentController_Reset(foc);
    PiController_Reset(speed_pi);
    motor_state->speed_target_rad_s = 0.0f;
    motor_state->speed_ramped_rad_s = 0.0f;
    motor_state->id_target_a = 0.0f;
    motor_state->iq_target_a = 0.0f;
    s_calibration_phase = CS_NULL;
    /* 功率级关断与任务衔接由主状态机执行。 */
    return outcome;
}

static void encoder_calib_commit_candidate_lut(EncoderState *encoder)
{
    uint16_t current_linearized_q15;

    memcpy(encoder->linearization_lut_q15,
           s_encoder_calib_workspace->bins.candidate,
           sizeof(encoder->linearization_lut_q15));
    current_linearized_q15 = encoder_calib_apply_candidate_lut(encoder->directed_q15);
    encoder->linearized_q15 = current_linearized_q15;
    encoder->electrical_zero_q15 = 0U;
    encoder->mechanical_zero_q15 = 0U;
    encoder->calib_flag &= (uint8_t)~(ENC_CALIB_ELECTRICAL_ZERO | ENC_CALIB_MECHANICAL_ZERO);
    encoder->calib_flag |= ENC_CALIB_LINEARIZED;
    /* 慢状态由 2 kHz 慢估计独占：请求重定多圈基准（基准取当前线性化角、零影子归零）。 */
    encoder->rebase_requested = true;
    Encoder_ResetVelocity(encoder);
}

static bool encoder_observer_calib_is_stable(MotorControlState *motor_state,
                                             FluxObserverState *flux_observer,
                                             SensorlessStartupState *startup,
                                             uint32_t position_epoch)
{
    float target_electrical_speed =
        SENSORLESS_ENCODER_CALIB_SPEED_MEC_RAD_S * (float)motor_state->model.motor_pole_pairs;
    float filtered_electrical_speed =
        startup->speed_feedback_rad_s * (float)motor_state->model.motor_pole_pairs;

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

static float encoder_observer_calib_get_stop_speed(const MotorControlState *motor_state)
{
    float minimum_mechanical_speed =
        SensorlessStartup_EncoderCalibConfig.minimum_electrical_velocity_rad_s /
        (float)motor_state->model.motor_pole_pairs;
    float stop_speed = minimum_mechanical_speed * SENSORLESS_ENCODER_CALIB_STOP_SPEED_MARGIN;

    return stop_speed < SENSORLESS_ENCODER_CALIB_SPEED_MEC_RAD_S
               ? stop_speed
               : SENSORLESS_ENCODER_CALIB_SPEED_MEC_RAD_S;
}

static bool encoder_observer_calib_can_brake(const MotorControlState *motor_state,
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
    /* LUT commit restarts the encoder speed estimator. Use it once ready as
     * an independent early cutoff; electrical zero is not needed for speed. */
    return !encoder->velocity_ready ||
           (encoder_speed > minimum_speed / (float)motor_state->model.motor_pole_pairs &&
            encoder_speed <= SENSORLESS_OBSERVER_MAX_ELEC_VEL_RAD_S /
                                 (float)motor_state->model.motor_pole_pairs);
}

/**
 * @brief Mode 13: calibrate the encoder linearization LUT from the sensorless observer.
 *        The static phase=0 alignment provides the mechanical reference origin for this power cycle.
 */
MotorTaskResult EncoderCalibrationTask_StepLut(FocState *foc,
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
    static float sample_previous_observer_position;
    static float sample_valid_electrical_travel;
    static float verify_previous_observer_position;
    static float verify_valid_electrical_travel;
    static float stop_start_speed;
    static float stop_current_ref;
    static bool origin_negative_seen;
    static uint64_t residual_squared_sum;
    static uint32_t residual_sample_count;
    static uint32_t residual_peak_abs_q15;
    static uint16_t candidate_lut_index;
    static uint8_t candidate_lut_stage;
    static int32_t candidate_lut_previous;
    static int32_t candidate_lut_shift;
    static int64_t candidate_lut_sum;
    float required_electrical_theta;
    float observer_position;
    float relative_theta;
    uint16_t reference_q15;
    MotorTaskResult outcome = {MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};

    if (s_encoder_calib_workspace != NULL && workspace != s_encoder_calib_workspace)
    {
        MotorFault_Report(motor_state, MOTOR_FAULT_ENCODER);
        encoder_observer_calib_abort(foc, motor_state, speed_pi, startup);
        return (MotorTaskResult){MOTOR_TASK_FAILED, MOTOR_FAULT_ENCODER};
    }

    if (!Encoder_IsOnline(encoder))
    {
        MotorFault_Report(motor_state, MOTOR_FAULT_ENCODER);
        encoder_observer_calib_abort(foc, motor_state, speed_pi, startup);
        outcome.result = MOTOR_TASK_FAILED;
        outcome.error = MOTOR_FAULT_ENCODER;
        return outcome;
    }
    if (motor_state->model.motor_pole_pairs <= 0)
    {
        MotorFault_Report(motor_state, MOTOR_FAULT_POLE_PAIRS);
        encoder_observer_calib_abort(foc, motor_state, speed_pi, startup);
        outcome.result = MOTOR_TASK_FAILED;
        outcome.error = MOTOR_FAULT_POLE_PAIRS;
        return outcome;
    }
    if (motor_state->model.motor_phase_resistance_ohm <= 0.0f ||
        motor_state->model.motor_d_inductance_h <= 0.0f ||
        motor_state->model.motor_q_inductance_h <= 0.0f ||
        motor_state->model.motor_flux_wb <= 0.0f ||
        motor_state->application.current_limit_a <= 0.0f)
    {
        MotorFault_Report(motor_state, MOTOR_FAULT_INVALID_PARAMETERS);
        encoder_observer_calib_abort(foc, motor_state, speed_pi, startup);
        outcome.result = MOTOR_TASK_FAILED;
        outcome.error = MOTOR_FAULT_INVALID_PARAMETERS;
        return outcome;
    }
    if (motor_state->application.current_limit_a < SENSORLESS_ENCODER_CALIB_MIN_CURRENT_LIMIT_A)
    {
        MotorFault_Report(motor_state, MOTOR_FAULT_INVALID_PARAMETERS);
        encoder_observer_calib_abort(foc, motor_state, speed_pi, startup);
        outcome.result = MOTOR_TASK_FAILED;
        outcome.error = MOTOR_FAULT_INVALID_PARAMETERS;
        return outcome;
    }

    if (s_calibration_phase == CS_NULL)
    {
        if (!encoder_calib_prepare_samples(workspace))
        {
            MotorFault_Report(motor_state, MOTOR_FAULT_ENCODER);
            encoder_observer_calib_abort(foc, motor_state, speed_pi, startup);
            outcome.result = MOTOR_TASK_FAILED;
            outcome.error = MOTOR_FAULT_ENCODER;
            return outcome;
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
        sample_previous_observer_position = 0.0f;
        sample_valid_electrical_travel = 0.0f;
        verify_previous_observer_position = 0.0f;
        verify_valid_electrical_travel = 0.0f;
        stop_start_speed = SENSORLESS_ENCODER_CALIB_SPEED_MEC_RAD_S;
        stop_current_ref = 0.0f;
        origin_negative_seen = false;
        residual_squared_sum = 0U;
        residual_sample_count = 0U;
        residual_peak_abs_q15 = 0U;
        candidate_lut_index = 0U;
        candidate_lut_stage = 0U;
        candidate_lut_previous = 0;
        candidate_lut_shift = 0;
        candidate_lut_sum = 0;
        s_calibration_phase = CS_OBS_ALIGN_ORIGIN;
    }

    if ((s_calibration_phase == CS_OBS_STOP_DECEL || s_calibration_phase == CS_OBS_STOP_CURRENT) &&
        !encoder_observer_calib_can_brake(
            motor_state, encoder, flux_observer, startup, observer_position_epoch))
    {
        /* The LUT is already verified and committed in both stop stages.
         * Below the observer's usable speed, coast instead of driving through zero. */
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
        FOC_Current(foc,
                    motor_state,
                    Observer_GetElePhase(flux_observer),
                    Observer_GetEleVel(flux_observer));
    }
    else
    {
        if (s_calibration_phase == CS_OBS_STOP_DECEL)
        {
            startup->speed_pi_output_max = 0.0f;
        }
        else
        {
            startup->speed_pi_output_max = 1.0f;
        }

        if (s_calibration_phase == CS_OBS_STOP_DECEL)
        {
            float decel_ratio = Math_Constrain((float)state_ticks * Current_Ts /
                                                   SENSORLESS_ENCODER_CALIB_STOP_DECEL_TIME_S,
                                               0.0f,
                                               1.0f);
            float stop_speed = encoder_observer_calib_get_stop_speed(motor_state);

            motor_state->speed_target_rad_s =
                stop_start_speed + (stop_speed - stop_start_speed) * decel_ratio;
            motor_state->speed_ramped_rad_s = motor_state->speed_target_rad_s;
        }
        else
        {
            motor_state->speed_target_rad_s = SENSORLESS_ENCODER_CALIB_SPEED_MEC_RAD_S;
        }

        SensorlessStartup_Run(foc,
                              motor_state,
                              speed_pi,
                              flux_observer,
                              startup,
                              &SensorlessStartup_EncoderCalibConfig);
        if (motor_state->fault_code != MOTOR_FAULT_NONE)
        {
            encoder_observer_calib_abort(foc, motor_state, speed_pi, startup);
            outcome.result = MOTOR_TASK_FAILED;
            outcome.error = motor_state->fault_code;
            return outcome;
        }
    }

    required_electrical_theta = _2PI * (float)motor_state->model.motor_pole_pairs;
    observer_position = Observer_GetElePosition(flux_observer);

    switch (s_calibration_phase)
    {
    case CS_OBS_ALIGN_ORIGIN:
        if (startup->state == SENSORLESS_STARTUP_ALIGN &&
            startup->state_ticks >=
                (uint32_t)(((SensorlessStartup_EncoderCalibConfig.align_current_ramp_time_s +
                             SensorlessStartup_EncoderCalibConfig.align_hold_time_s) -
                            SENSORLESS_ENCODER_CALIB_ALIGN_SAMPLE_TIME_S) /
                           Current_Ts))
        {
            int32_t unwrapped_q15;

            if (origin_sample_count == 0U)
            {
                origin_anchor_q15 = encoder->directed_q15;
            }
            unwrapped_q15 = (int32_t)origin_anchor_q15 +
                            encoder_calib_q15_difference(encoder->directed_q15, origin_anchor_q15);
            origin_sum_q15 += unwrapped_q15;
            origin_sample_count++;
        }

        if (startup->state != SENSORLESS_STARTUP_ALIGN)
        {
            if (origin_sample_count == 0U)
            {
                MotorFault_Report(motor_state, MOTOR_FAULT_ENCODER);
                encoder_observer_calib_abort(foc, motor_state, speed_pi, startup);
                outcome.result = MOTOR_TASK_FAILED;
                outcome.error = MOTOR_FAULT_ENCODER;
                return outcome;
            }

            origin_q15 = (uint16_t)(origin_sum_q15 / (int64_t)origin_sample_count);
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
            MotorFault_Report(motor_state, MOTOR_FAULT_SENSORLESS);
            encoder_observer_calib_abort(foc, motor_state, speed_pi, startup);
            outcome.result = MOTOR_TASK_FAILED;
            outcome.error = MOTOR_FAULT_SENSORLESS;
        }
        break;

    case CS_OBS_SPEED_STABLE:
        if (Observer_GetPositionEpoch(flux_observer) != observer_position_epoch)
        {
            MotorFault_Report(motor_state, MOTOR_FAULT_SENSORLESS);
            encoder_observer_calib_abort(foc, motor_state, speed_pi, startup);
            outcome.result = MOTOR_TASK_FAILED;
            outcome.error = MOTOR_FAULT_SENSORLESS;
            return outcome;
        }

        if (encoder_observer_calib_is_stable(
                motor_state, flux_observer, startup, observer_position_epoch))
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
            MotorFault_Report(motor_state, MOTOR_FAULT_SENSORLESS);
            encoder_observer_calib_abort(foc, motor_state, speed_pi, startup);
            outcome.result = MOTOR_TASK_FAILED;
            outcome.error = MOTOR_FAULT_SENSORLESS;
        }
        break;

    case CS_OBS_FIND_ORIGIN:
    {
        int16_t previous_relative = encoder_calib_q15_difference(previous_directed_q15, origin_q15);
        int16_t current_relative = encoder_calib_q15_difference(encoder->directed_q15, origin_q15);

        if (!encoder_observer_calib_is_tracking(flux_observer, startup, observer_position_epoch))
        {
            MotorFault_Report(motor_state, MOTOR_FAULT_SENSORLESS);
            encoder_observer_calib_abort(foc, motor_state, speed_pi, startup);
            outcome.result = MOTOR_TASK_FAILED;
            outcome.error = MOTOR_FAULT_SENSORLESS;
            return outcome;
        }

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
            sample_previous_observer_position = observer_position;
            sample_valid_electrical_travel = 0.0f;
            stage_ticks = 0U;
            s_calibration_phase = CS_OBS_SAMPLE_CW;
            break;
        }

        previous_directed_q15 = encoder->directed_q15;
        previous_observer_position = observer_position;
        if (++stage_ticks >=
            (uint32_t)(SENSORLESS_ENCODER_CALIB_FIND_ORIGIN_TIMEOUT_S / Current_Ts))
        {
            MotorFault_Report(motor_state, MOTOR_FAULT_SENSORLESS);
            encoder_observer_calib_abort(foc, motor_state, speed_pi, startup);
            outcome.result = MOTOR_TASK_FAILED;
            outcome.error = MOTOR_FAULT_SENSORLESS;
        }
        break;
    }

    case CS_OBS_SAMPLE_CW:
        relative_theta = observer_position - observer_position_origin;
        if (!encoder_observer_calib_is_tracking(flux_observer, startup, observer_position_epoch))
        {
            MotorFault_Report(motor_state, MOTOR_FAULT_SENSORLESS);
            encoder_observer_calib_abort(foc, motor_state, speed_pi, startup);
            outcome.result = MOTOR_TASK_FAILED;
            outcome.error = MOTOR_FAULT_SENSORLESS;
            return outcome;
        }

        if (encoder_observer_calib_is_stable(
                motor_state, flux_observer, startup, observer_position_epoch))
        {
            float observer_delta = observer_position - sample_previous_observer_position;

            if (observer_delta > 0.0f)
            {
                sample_valid_electrical_travel += observer_delta;
            }

            if (relative_theta >= 0.0f)
            {
                uint16_t lut_index = encoder->directed_q15 >> (16U - ENCODER_OFFSET_LUT_BITS);
                int32_t correction_q15;

                reference_q15 = (uint16_t)(relative_theta *
                                           ((float)ENCODER_Q15_CPR / required_electrical_theta));
                correction_q15 = encoder_calib_q15_difference(encoder->directed_q15, reference_q15);
                if (s_encoder_calib_workspace->bins.samples[lut_index] != 0U)
                {
                    int32_t average_q15 =
                        s_encoder_calib_workspace->error_sum[lut_index] /
                        (int32_t)s_encoder_calib_workspace->bins.samples[lut_index];
                    while (correction_q15 - average_q15 > ENCODER_Q15_HALF_TURN)
                    {
                        correction_q15 -= (int32_t)ENCODER_Q15_CPR;
                    }
                    while (correction_q15 - average_q15 < -ENCODER_Q15_HALF_TURN)
                    {
                        correction_q15 += (int32_t)ENCODER_Q15_CPR;
                    }
                }

                if (s_encoder_calib_workspace->bins.samples[lut_index] < UINT16_MAX)
                {
                    s_encoder_calib_workspace->error_sum[lut_index] += correction_q15;
                    s_encoder_calib_workspace->bins.samples[lut_index]++;
                }
            }
        }
        sample_previous_observer_position = observer_position;

        if (sample_valid_electrical_travel >=
            (float)SENSORLESS_ENCODER_CALIB_MECH_TURNS * required_electrical_theta)
        {
            candidate_lut_index = 0U;
            candidate_lut_stage = 0U;
            candidate_lut_previous = 0;
            candidate_lut_shift = 0;
            candidate_lut_sum = 0;
            s_calibration_phase = CS_OBS_BUILD_LUT;
            break;
        }
        if (++stage_ticks >= (uint32_t)(SENSORLESS_ENCODER_CALIB_SAMPLE_TIMEOUT_S / Current_Ts))
        {
            MotorFault_Report(motor_state, MOTOR_FAULT_SENSORLESS);
            encoder_observer_calib_abort(foc, motor_state, speed_pi, startup);
            outcome.result = MOTOR_TASK_FAILED;
            outcome.error = MOTOR_FAULT_SENSORLESS;
        }
        break;

    case CS_OBS_BUILD_LUT:
        if (!encoder_observer_calib_is_tracking(flux_observer, startup, observer_position_epoch))
        {
            MotorFault_Report(motor_state, MOTOR_FAULT_SENSORLESS);
            encoder_observer_calib_abort(foc, motor_state, speed_pi, startup);
            outcome.result = MOTOR_TASK_FAILED;
            outcome.error = MOTOR_FAULT_SENSORLESS;
            return outcome;
        }

        if (candidate_lut_stage == 0U)
        {
            uint16_t bins_processed;

            for (bins_processed = 0U;
                 bins_processed < SENSORLESS_ENCODER_CALIB_LUT_BUILD_BINS_PER_CYCLE &&
                 candidate_lut_index < ENCODER_OFFSET_LUT_SIZE;
                 ++bins_processed, ++candidate_lut_index)
            {
                if (s_encoder_calib_workspace->bins.samples[candidate_lut_index] <
                    SENSORLESS_ENCODER_CALIB_MIN_SAMPLES_PER_BIN)
                {
                    MotorFault_Report(motor_state, MOTOR_FAULT_ENCODER);
                    encoder_observer_calib_abort(foc, motor_state, speed_pi, startup);
                    outcome.result = MOTOR_TASK_FAILED;
                    outcome.error = MOTOR_FAULT_ENCODER;
                    return outcome;
                }
            }

            if (candidate_lut_index >= ENCODER_OFFSET_LUT_SIZE)
            {
                candidate_lut_index = 0U;
                candidate_lut_stage = 1U;
            }
        }
        else if (candidate_lut_stage == 1U)
        {
            uint16_t bins_processed;

            for (bins_processed = 0U;
                 bins_processed < SENSORLESS_ENCODER_CALIB_LUT_BUILD_BINS_PER_CYCLE &&
                 candidate_lut_index < ENCODER_OFFSET_LUT_SIZE;
                 ++bins_processed, ++candidate_lut_index)
            {
                int32_t correction =
                    (int16_t)(s_encoder_calib_workspace->error_sum[candidate_lut_index] /
                              (int32_t)
                                  s_encoder_calib_workspace->bins.samples[candidate_lut_index]);

                if (candidate_lut_index > 0U)
                {
                    while (correction - candidate_lut_previous > ENCODER_Q15_HALF_TURN)
                    {
                        correction -= (int32_t)ENCODER_Q15_CPR;
                    }
                    while (correction - candidate_lut_previous < -ENCODER_Q15_HALF_TURN)
                    {
                        correction += (int32_t)ENCODER_Q15_CPR;
                    }
                }

                s_encoder_calib_workspace->error_sum[candidate_lut_index] = correction;
                candidate_lut_previous = correction;
            }

            if (candidate_lut_index >= ENCODER_OFFSET_LUT_SIZE)
            {
                candidate_lut_index = 0U;
                candidate_lut_sum = 0;
                candidate_lut_stage = 2U;
            }
        }
        else if (candidate_lut_stage == 2U)
        {
            uint16_t bins_processed;

            for (bins_processed = 0U;
                 bins_processed < SENSORLESS_ENCODER_CALIB_LUT_BUILD_BINS_PER_CYCLE &&
                 candidate_lut_index < ENCODER_OFFSET_LUT_SIZE;
                 ++bins_processed, ++candidate_lut_index)
            {
                candidate_lut_sum += s_encoder_calib_workspace->error_sum[candidate_lut_index];
            }

            if (candidate_lut_index >= ENCODER_OFFSET_LUT_SIZE)
            {
                candidate_lut_sum /= (int64_t)ENCODER_OFFSET_LUT_SIZE;
                candidate_lut_shift = 0;
                while (candidate_lut_sum > INT16_MAX)
                {
                    candidate_lut_sum -= (int32_t)ENCODER_Q15_CPR;
                    candidate_lut_shift += (int32_t)ENCODER_Q15_CPR;
                }
                while (candidate_lut_sum < INT16_MIN)
                {
                    candidate_lut_sum += (int32_t)ENCODER_Q15_CPR;
                    candidate_lut_shift -= (int32_t)ENCODER_Q15_CPR;
                }
                candidate_lut_index = 0U;
                candidate_lut_stage = 3U;
            }
        }
        else
        {
            uint16_t bins_processed;

            for (bins_processed = 0U;
                 bins_processed < SENSORLESS_ENCODER_CALIB_LUT_BUILD_BINS_PER_CYCLE &&
                 candidate_lut_index < ENCODER_OFFSET_LUT_SIZE;
                 ++bins_processed, ++candidate_lut_index)
            {
                int32_t correction =
                    s_encoder_calib_workspace->error_sum[candidate_lut_index] - candidate_lut_shift;

                if (correction < INT16_MIN || correction > INT16_MAX)
                {
                    MotorFault_Report(motor_state, MOTOR_FAULT_ENCODER);
                    encoder_observer_calib_abort(foc, motor_state, speed_pi, startup);
                    outcome.result = MOTOR_TASK_FAILED;
                    outcome.error = MOTOR_FAULT_ENCODER;
                    return outcome;
                }

                s_encoder_calib_workspace->bins.candidate[candidate_lut_index] =
                    (int16_t)correction;
            }

            if (candidate_lut_index >= ENCODER_OFFSET_LUT_SIZE)
            {
                verify_previous_observer_position = observer_position;
                verify_valid_electrical_travel = 0.0f;
                residual_squared_sum = 0U;
                residual_sample_count = 0U;
                residual_peak_abs_q15 = 0U;
                stage_ticks = 0U;
                s_calibration_phase = CS_OBS_VERIFY_CW;
            }
        }
        break;

    case CS_OBS_VERIFY_CW:
        relative_theta = observer_position - observer_position_origin;
        if (!encoder_observer_calib_is_tracking(flux_observer, startup, observer_position_epoch))
        {
            MotorFault_Report(motor_state, MOTOR_FAULT_SENSORLESS);
            encoder_observer_calib_abort(foc, motor_state, speed_pi, startup);
            outcome.result = MOTOR_TASK_FAILED;
            outcome.error = MOTOR_FAULT_SENSORLESS;
            return outcome;
        }

        if (encoder_observer_calib_is_stable(
                motor_state, flux_observer, startup, observer_position_epoch))
        {
            float observer_delta = observer_position - verify_previous_observer_position;

            if (observer_delta > 0.0f)
            {
                verify_valid_electrical_travel += observer_delta;
            }

            if (relative_theta >= 0.0f)
            {
                int16_t residual_q15;
                int32_t residual_abs_q15;
                uint16_t linearized_q15;

                reference_q15 = (uint16_t)(relative_theta *
                                           ((float)ENCODER_Q15_CPR / required_electrical_theta));
                linearized_q15 = encoder_calib_apply_candidate_lut(encoder->directed_q15);
                residual_q15 = encoder_calib_q15_difference(linearized_q15, reference_q15);
                residual_abs_q15 = residual_q15 >= 0 ? residual_q15 : -(int32_t)residual_q15;
                residual_squared_sum += (uint64_t)((int64_t)residual_q15 * (int64_t)residual_q15);
                if ((uint32_t)residual_abs_q15 > residual_peak_abs_q15)
                {
                    residual_peak_abs_q15 = (uint32_t)residual_abs_q15;
                }
                residual_sample_count++;
            }
        }
        verify_previous_observer_position = observer_position;

        if (verify_valid_electrical_travel >=
            (float)SENSORLESS_ENCODER_CALIB_VERIFY_MECH_TURNS * required_electrical_theta)
        {
            uint64_t max_rms_squared = (uint64_t)SENSORLESS_ENCODER_CALIB_MAX_RMS_RESIDUAL_Q15 *
                                       (uint64_t)SENSORLESS_ENCODER_CALIB_MAX_RMS_RESIDUAL_Q15;

            if (residual_sample_count == 0U ||
                residual_peak_abs_q15 > SENSORLESS_ENCODER_CALIB_MAX_PEAK_RESIDUAL_Q15 ||
                residual_squared_sum > (uint64_t)residual_sample_count * max_rms_squared)
            {
                MotorFault_Report(motor_state, MOTOR_FAULT_ENCODER);
                encoder_observer_calib_abort(foc, motor_state, speed_pi, startup);
                outcome.result = MOTOR_TASK_FAILED;
                outcome.error = MOTOR_FAULT_ENCODER;
                return outcome;
            }

            encoder_calib_commit_candidate_lut(encoder);
            encoder_calib_release_samples();
            stop_start_speed = motor_state->speed_ramped_rad_s;
            if (stop_start_speed < encoder_observer_calib_get_stop_speed(motor_state))
            {
                stop_start_speed = encoder_observer_calib_get_stop_speed(motor_state);
            }
            state_ticks = 0U;
            s_calibration_phase = CS_OBS_STOP_DECEL;
            break;
        }
        if (++stage_ticks >= (uint32_t)(SENSORLESS_ENCODER_CALIB_VERIFY_TIMEOUT_S / Current_Ts))
        {
            MotorFault_Report(motor_state, MOTOR_FAULT_SENSORLESS);
            encoder_observer_calib_abort(foc, motor_state, speed_pi, startup);
            outcome.result = MOTOR_TASK_FAILED;
            outcome.error = MOTOR_FAULT_SENSORLESS;
        }
        break;

    case CS_OBS_STOP_DECEL:
    {
        float stop_speed = encoder_observer_calib_get_stop_speed(motor_state);
        float observer_mech_vel =
            Observer_GetEleVel(flux_observer) / (float)motor_state->model.motor_pole_pairs;
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
            MotorFault_Report(motor_state, MOTOR_FAULT_SENSORLESS);
            encoder_observer_calib_abort(foc, motor_state, speed_pi, startup);
            outcome.result = MOTOR_TASK_FAILED;
            outcome.error = MOTOR_FAULT_SENSORLESS;
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
        MotorFault_Report(motor_state, MOTOR_FAULT_ENCODER);
        encoder_observer_calib_abort(foc, motor_state, speed_pi, startup);
        outcome.result = MOTOR_TASK_FAILED;
        outcome.error = MOTOR_FAULT_ENCODER;
        break;
    }
    return outcome;
}

/**
 * @brief Calibrate electrical zero using the averaged linearized Q15 angle.
 */
MotorTaskResult EncoderCalibrationTask_StepElectricalZero(FocState *foc,
                                                          MotorControlState *motor_state,
                                                          EncoderState *encoder)
{
    MotorTaskResult outcome = {MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};
    float time = (float)s_zero_loop_count * Current_Ts;
    float align_current = motor_state->model.calibration_current_a;

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
    if ((encoder->calib_flag & ENC_CALIB_LINEARIZED) == 0U)
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
    if (motor_state->application.current_limit_a > 0.0f &&
        align_current > motor_state->application.current_limit_a)
    {
        align_current = motor_state->application.current_limit_a;
    }

    if (s_zero_loop_count == 0U)
    {
        s_zero_sample_count = 0U;
        s_zero_sample_anchor = encoder->linearized_q15;
        s_zero_unwrapped_sum = 0;
        encoder->calib_flag &= (uint8_t)~ENC_CALIB_ELECTRICAL_ZERO;
        FOC_CurrentController_Reset(foc);
    }

    motor_state->id_target_a =
        align_current * Math_Constrain(((float)s_zero_loop_count + 1.0f) * Current_Ts /
                                           ENCODER_ELEC_ZERO_CURRENT_RAMP_TIME_S,
                                       0.0f,
                                       1.0f);
    motor_state->iq_target_a = 0.0f;
    FOC_Current(foc, motor_state, 0.0f, 0.0f);

    if (time >= (ENC_ZERO_ALIGN_TIME - ENC_ZERO_SAMPLE_TIME) &&
        encoder->read_status == ENCODER_READ_OK)
    {
        int32_t unwrapped_q15 = (int32_t)s_zero_sample_anchor +
                                (int16_t)(uint16_t)(encoder->linearized_q15 - s_zero_sample_anchor);
        s_zero_unwrapped_sum += unwrapped_q15;
        s_zero_sample_count++;
    }

    if (time >= ENC_ZERO_ALIGN_TIME)
    {
        bool calibrated = false;
        if (s_zero_sample_count > 0U)
        {
            uint16_t electrical_zero_q15 =
                (uint16_t)(s_zero_unwrapped_sum / (int64_t)s_zero_sample_count);
            calibrated = Encoder_SetElectricalZeroQ15(encoder, electrical_zero_q15);
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
