#include "foc_encoder_calibration.h"

#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "control_config.h"
#include "foc_errhandle.h"
#include "heap.h"
#include "utils.h"

/* Mode 13/15 编码器标定重建（契约见头文件）。
 * 原 foc_calibration 五任务中的 R_L_Flux/EncoderOffset/CurrentOffset 仍待重建：
 * 其辅助函数（分相电阻/电感/磁链扫描、开环扫角偏移、ADC 零偏）未随本模块恢复。
 * 迁移约定：任务不自行停相或写模式；本地中止只清理采样区与控制状态。 */

static int32_t *p_error_sum = NULL;
static uint16_t *calibration_samples = NULL;
static int16_t *candidate_linearization_lut = NULL;

CalibStep_TyepeDef CalibStep = CS_NULL;

#define ENC_ZERO_ALIGN_TIME (ENCODER_ELEC_ZERO_CURRENT_RAMP_TIME_S + ENCODER_ELEC_ZERO_HOLD_TIME_S)
#define ENC_ZERO_SAMPLE_TIME ENCODER_ELEC_ZERO_HOLD_TIME_S

static void Encoder_Calib_ReleaseSamples(void)
{
    if (p_error_sum != NULL)
    {
        HEAP_free(p_error_sum);
        p_error_sum = NULL;
    }
    if (calibration_samples != NULL)
    {
        HEAP_free(calibration_samples);
        calibration_samples = NULL;
    }
    if (candidate_linearization_lut != NULL)
    {
        HEAP_free(candidate_linearization_lut);
        candidate_linearization_lut = NULL;
    }
}

static int16_t Encoder_Calib_Q15Difference(uint16_t target_q15, uint16_t source_q15)
{
    int32_t difference = (int32_t)target_q15 - (int32_t)source_q15;

    if (difference > ENCODER_Q15_HALF_TURN)
        difference -= (int32_t)ENCODER_Q15_CPR;
    else if (difference < -ENCODER_Q15_HALF_TURN)
        difference += (int32_t)ENCODER_Q15_CPR;

    return (int16_t)difference;
}

static bool Encoder_Calib_AllocateSamples(void)
{
    if (p_error_sum == NULL)
        p_error_sum = HEAP_malloc(ENCODER_OFFSET_LUT_SIZE * sizeof(*p_error_sum));
    if (calibration_samples == NULL)
        calibration_samples = HEAP_malloc(ENCODER_OFFSET_LUT_SIZE * sizeof(*calibration_samples));
    if (candidate_linearization_lut == NULL)
        candidate_linearization_lut =
            HEAP_malloc(ENCODER_OFFSET_LUT_SIZE * sizeof(*candidate_linearization_lut));

    if (p_error_sum == NULL || calibration_samples == NULL || candidate_linearization_lut == NULL)
    {
        Encoder_Calib_ReleaseSamples();
        return false;
    }

    memset(p_error_sum, 0, ENCODER_OFFSET_LUT_SIZE * sizeof(*p_error_sum));
    memset(calibration_samples, 0, ENCODER_OFFSET_LUT_SIZE * sizeof(*calibration_samples));
    memset(candidate_linearization_lut,
           0,
           ENCODER_OFFSET_LUT_SIZE * sizeof(*candidate_linearization_lut));
    return true;
}

static uint16_t Encoder_Calib_ApplyCandidateLut(uint16_t directed_q15)
{
    uint16_t lut_index = directed_q15 >> 6;
    uint16_t fraction = directed_q15 & 0x003FU;
    int32_t correction_a = candidate_linearization_lut[lut_index];
    int32_t correction_b =
        candidate_linearization_lut[(lut_index + 1U) & (ENCODER_OFFSET_LUT_SIZE - 1U)];
    int32_t correction = correction_a + (((correction_b - correction_a) * fraction) >> 6);

    return (uint16_t)((int32_t)directed_q15 - correction);
}

/* 本地中止：释放采样区并复位控制状态；不触碰功率级与模式（由运行状态机执行）。 */
static void Encoder_ObserverCalib_Abort(FOC_TypeDef *FOC,
                                        MotorControl_TypeDef *MotorControl,
                                        PI_Controller_TypeDef *SpeedController,
                                        SensorlessStartup_TypeDef *Startup)
{
    Encoder_Calib_ReleaseSamples();
    SensorlessStartup_Reset(Startup);
    FOC_CurrentController_Reset(FOC);
    PI_Controller_Reset(SpeedController);
    MotorControl->speedRef = 0.0f;
    MotorControl->speedShadow = 0.0f;
    MotorControl->idRef = 0.0f;
    MotorControl->iqRef = 0.0f;
    CalibStep = CS_NULL;
}

static MotorWorkOutcome_TypeDef Encoder_ObserverCalib_Finish(FOC_TypeDef *FOC,
                                                             MotorControl_TypeDef *MotorControl,
                                                             PI_Controller_TypeDef *SpeedController,
                                                             SensorlessStartup_TypeDef *Startup)
{
    MotorWorkOutcome_TypeDef outcome = {MOTOR_WORK_SWITCH_MODE, Save_Param, No_Error, true};

    /* End active torque before saving. A low-speed flux observer must not
     * keep applying negative Iq after the rotor has crossed zero. */
    SensorlessStartup_Reset(Startup);
    FOC_CurrentController_Reset(FOC);
    PI_Controller_Reset(SpeedController);
    MotorControl->speedRef = 0.0f;
    MotorControl->speedShadow = 0.0f;
    MotorControl->idRef = 0.0f;
    MotorControl->iqRef = 0.0f;
    CalibStep = CS_NULL;
    /* 功率级关断与模式迁移由运行状态机执行（power_off=true）。 */
    return outcome;
}

static void Encoder_Calib_CommitCandidateLut(Encoder_TypeDef *Encoder)
{
    uint16_t current_linearized_q15;

    memcpy(Encoder->linearization_lut_q15,
           candidate_linearization_lut,
           sizeof(Encoder->linearization_lut_q15));
    current_linearized_q15 = Encoder_Calib_ApplyCandidateLut(Encoder->directed_q15);
    Encoder->linearized_q15 = current_linearized_q15;
    Encoder->previous_linearized_q15 = current_linearized_q15;
    Encoder->shadow_q15 = current_linearized_q15;
    Encoder->mechanical_zero_shadow_q15 = 0;
    Encoder->electrical_zero_q15 = 0U;
    Encoder->mechanical_zero_q15 = 0U;
    Encoder->calib_flag &= (uint8_t)~(ENC_CALIB_ELECTRICAL_ZERO | ENC_CALIB_MECHANICAL_ZERO);
    Encoder->calib_flag |= ENC_CALIB_LINEARIZED;
    Encoder_ResetVelocity(Encoder);
}

static bool Encoder_ObserverCalib_IsStable(MotorControl_TypeDef *MotorControl,
                                           Fluxobserver_TypeDef *Fluxobserver,
                                           SensorlessStartup_TypeDef *Startup,
                                           uint32_t position_epoch)
{
    float target_electrical_speed =
        SENSORLESS_ENCODER_CALIB_SPEED_MEC_RAD_S * (float)MotorControl->motor_pole_pairs;
    float filtered_electrical_speed =
        Startup->speed_feedback * (float)MotorControl->motor_pole_pairs;

    return Startup->state == SENSORLESS_STARTUP_CLOSED_LOOP &&
           Observer_GetPositionEpoch(Fluxobserver) == position_epoch &&
           fast_abs(filtered_electrical_speed - target_electrical_speed) <=
               fast_abs(target_electrical_speed) * SENSORLESS_ENCODER_CALIB_SPEED_ERROR_RATIO &&
           fast_abs(MotorControl->speedShadow - SENSORLESS_ENCODER_CALIB_SPEED_MEC_RAD_S) <=
               SENSORLESS_ENCODER_CALIB_SPEED_MEC_RAD_S *
                   SENSORLESS_ENCODER_CALIB_SPEED_ERROR_RATIO;
}

static bool Encoder_ObserverCalib_IsTracking(Fluxobserver_TypeDef *Fluxobserver,
                                             SensorlessStartup_TypeDef *Startup,
                                             uint32_t position_epoch)
{
    return Startup->state == SENSORLESS_STARTUP_CLOSED_LOOP &&
           Observer_GetPositionEpoch(Fluxobserver) == position_epoch;
}

static float Encoder_ObserverCalib_GetStopSpeed(const MotorControl_TypeDef *MotorControl)
{
    float minimum_mechanical_speed =
        SensorlessStartup_EncoderCalibConfig.minimum_electrical_velocity_rad_s /
        (float)MotorControl->motor_pole_pairs;
    float stop_speed = minimum_mechanical_speed * SENSORLESS_ENCODER_CALIB_STOP_SPEED_MARGIN;

    return stop_speed < SENSORLESS_ENCODER_CALIB_SPEED_MEC_RAD_S
               ? stop_speed
               : SENSORLESS_ENCODER_CALIB_SPEED_MEC_RAD_S;
}

static bool Encoder_ObserverCalib_CanBrake(const MotorControl_TypeDef *MotorControl,
                                           const Encoder_TypeDef *Encoder,
                                           Fluxobserver_TypeDef *Fluxobserver,
                                           SensorlessStartup_TypeDef *Startup,
                                           uint32_t position_epoch)
{
    float minimum_speed = SensorlessStartup_EncoderCalibConfig.minimum_electrical_velocity_rad_s;
    float observer_speed = Observer_GetEleVel(Fluxobserver);
    float observer_phase = Observer_GetElePhase(Fluxobserver);
    float encoder_speed = Encoder_GetMecVelContinuous(Encoder);

    if (!Encoder_ObserverCalib_IsTracking(Fluxobserver, Startup, position_epoch) ||
        !(observer_speed > minimum_speed) ||
        observer_speed > SENSORLESS_OBSERVER_MAX_ELEC_VEL_RAD_S ||
        !(observer_phase >= 0.0f && observer_phase <= _2PI))
        return false;
    /* LUT commit restarts the encoder speed estimator. Use it once ready as
     * an independent early cutoff; electrical zero is not needed for speed. */
    return !Encoder->velocity_ready ||
           (encoder_speed > minimum_speed / (float)MotorControl->motor_pole_pairs &&
            encoder_speed <=
                SENSORLESS_OBSERVER_MAX_ELEC_VEL_RAD_S / (float)MotorControl->motor_pole_pairs);
}

/**
 * @brief Mode 13: calibrate the encoder linearization LUT from the sensorless observer.
 *        The static phase=0 alignment provides the mechanical reference origin for this power cycle.
 */
MotorWorkOutcome_TypeDef Task_Calib_EncoderObserver(FOC_TypeDef *FOC,
                                                    MotorControl_TypeDef *MotorControl,
                                                    PI_Controller_TypeDef *SpeedController,
                                                    Encoder_TypeDef *Encoder,
                                                    Fluxobserver_TypeDef *Fluxobserver,
                                                    SensorlessStartup_TypeDef *Startup)
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
    MotorWorkOutcome_TypeDef outcome = {MOTOR_WORK_RUNNING, Motor_Disable, No_Error, false};

    if (!Encoder_IsOnline(Encoder))
    {
        Set_ErrorNow(Encoder_Error);
        Encoder_ObserverCalib_Abort(FOC, MotorControl, SpeedController, Startup);
        outcome.result = MOTOR_WORK_FAULT;
        outcome.error = Encoder_Error;
        return outcome;
    }
    if (MotorControl->motor_pole_pairs <= 0)
    {
        Set_ErrorNow(PolePairs_Error);
        Encoder_ObserverCalib_Abort(FOC, MotorControl, SpeedController, Startup);
        outcome.result = MOTOR_WORK_FAULT;
        outcome.error = PolePairs_Error;
        return outcome;
    }
    if (MotorControl->motor_phase_resistance <= 0.0f || MotorControl->motor_d_inductance <= 0.0f ||
        MotorControl->motor_q_inductance <= 0.0f || MotorControl->motor_flux <= 0.0f ||
        MotorControl->current_limit <= 0.0f)
    {
        Set_ErrorNow(MotorParam_Error);
        Encoder_ObserverCalib_Abort(FOC, MotorControl, SpeedController, Startup);
        outcome.result = MOTOR_WORK_FAULT;
        outcome.error = MotorParam_Error;
        return outcome;
    }
    if (MotorControl->current_limit < SENSORLESS_ENCODER_CALIB_MIN_CURRENT_LIMIT_A)
    {
        Set_ErrorNow(MotorParam_Error);
        Encoder_ObserverCalib_Abort(FOC, MotorControl, SpeedController, Startup);
        outcome.result = MOTOR_WORK_FAULT;
        outcome.error = MotorParam_Error;
        return outcome;
    }

    if (CalibStep == CS_NULL)
    {
        if (!Encoder_Calib_AllocateSamples())
        {
            Set_ErrorNow(Encoder_Error);
            Encoder_ObserverCalib_Abort(FOC, MotorControl, SpeedController, Startup);
            outcome.result = MOTOR_WORK_FAULT;
            outcome.error = Encoder_Error;
            return outcome;
        }

        SensorlessStartup_Reset(Startup);
        FOC_CurrentController_Reset(FOC);
        PI_Controller_Reset(SpeedController);
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
        CalibStep = CS_OBS_ALIGN_ORIGIN;
    }

    if ((CalibStep == CS_OBS_STOP_DECEL || CalibStep == CS_OBS_STOP_CURRENT) &&
        !Encoder_ObserverCalib_CanBrake(
            MotorControl, Encoder, Fluxobserver, Startup, observer_position_epoch))
    {
        /* The LUT is already verified and committed in both stop stages.
         * Below the observer's usable speed, coast instead of driving through zero. */
        return Encoder_ObserverCalib_Finish(FOC, MotorControl, SpeedController, Startup);
    }

    if (CalibStep == CS_OBS_STOP_CURRENT)
    {
        float current_ratio;

        current_ratio = constrain(((float)state_ticks + 1.0f) * Current_Ts /
                                      SENSORLESS_ENCODER_CALIB_STOP_CURRENT_RAMP_TIME_S,
                                  0.0f,
                                  1.0f);
        MotorControl->idRef = 0.0f;
        MotorControl->iqRef = stop_current_ref * (1.0f - current_ratio);
        FOC_Current(FOC,
                    MotorControl,
                    Observer_GetElePhase(Fluxobserver),
                    Observer_GetEleVel(Fluxobserver));
    }
    else
    {
        if (CalibStep == CS_OBS_STOP_DECEL)
            Startup->speed_pi_output_max = 0.0f;
        else
            Startup->speed_pi_output_max = 1.0f;

        if (CalibStep == CS_OBS_STOP_DECEL)
        {
            float decel_ratio = constrain((float)state_ticks * Current_Ts /
                                              SENSORLESS_ENCODER_CALIB_STOP_DECEL_TIME_S,
                                          0.0f,
                                          1.0f);
            float stop_speed = Encoder_ObserverCalib_GetStopSpeed(MotorControl);

            MotorControl->speedRef =
                stop_start_speed + (stop_speed - stop_start_speed) * decel_ratio;
            MotorControl->speedShadow = MotorControl->speedRef;
        }
        else
        {
            MotorControl->speedRef = SENSORLESS_ENCODER_CALIB_SPEED_MEC_RAD_S;
        }

        SensorlessStartup_Run(FOC,
                              MotorControl,
                              SpeedController,
                              Fluxobserver,
                              Startup,
                              &SensorlessStartup_EncoderCalibConfig);
        if (MotorControl->ErrorNow != No_Error)
        {
            Encoder_ObserverCalib_Abort(FOC, MotorControl, SpeedController, Startup);
            outcome.result = MOTOR_WORK_FAULT;
            outcome.error = MotorControl->ErrorNow;
            return outcome;
        }
    }

    required_electrical_theta = _2PI * (float)MotorControl->motor_pole_pairs;
    observer_position = Observer_GetElePosition(Fluxobserver);

    switch (CalibStep)
    {
    case CS_OBS_ALIGN_ORIGIN:
        if (Startup->state == SENSORLESS_STARTUP_ALIGN &&
            Startup->state_ticks >=
                (uint32_t)(((SensorlessStartup_EncoderCalibConfig.align_current_ramp_time_s +
                             SensorlessStartup_EncoderCalibConfig.align_hold_time_s) -
                            SENSORLESS_ENCODER_CALIB_ALIGN_SAMPLE_TIME_S) /
                           Current_Ts))
        {
            int32_t unwrapped_q15;

            if (origin_sample_count == 0U)
                origin_anchor_q15 = Encoder->directed_q15;
            unwrapped_q15 = (int32_t)origin_anchor_q15 +
                            Encoder_Calib_Q15Difference(Encoder->directed_q15, origin_anchor_q15);
            origin_sum_q15 += unwrapped_q15;
            origin_sample_count++;
        }

        if (Startup->state != SENSORLESS_STARTUP_ALIGN)
        {
            if (origin_sample_count == 0U)
            {
                Set_ErrorNow(Encoder_Error);
                Encoder_ObserverCalib_Abort(FOC, MotorControl, SpeedController, Startup);
                outcome.result = MOTOR_WORK_FAULT;
                outcome.error = Encoder_Error;
                return outcome;
            }

            origin_q15 = (uint16_t)(origin_sum_q15 / (int64_t)origin_sample_count);
            state_ticks = 0U;
            stage_ticks = 0U;
            CalibStep = CS_OBS_WAIT_CLOSED_LOOP;
        }
        break;

    case CS_OBS_WAIT_CLOSED_LOOP:
        if (Startup->state == SENSORLESS_STARTUP_CLOSED_LOOP)
        {
            observer_position_epoch = Observer_GetPositionEpoch(Fluxobserver);
            state_ticks = 0U;
            stage_ticks = 0U;
            CalibStep = CS_OBS_SPEED_STABLE;
        }
        else if (++stage_ticks >=
                 (uint32_t)(SENSORLESS_ENCODER_CALIB_STARTUP_TIMEOUT_S / Current_Ts))
        {
            Set_ErrorNow(Sensorless_Error);
            Encoder_ObserverCalib_Abort(FOC, MotorControl, SpeedController, Startup);
            outcome.result = MOTOR_WORK_FAULT;
            outcome.error = Sensorless_Error;
        }
        break;

    case CS_OBS_SPEED_STABLE:
        if (Observer_GetPositionEpoch(Fluxobserver) != observer_position_epoch)
        {
            Set_ErrorNow(Sensorless_Error);
            Encoder_ObserverCalib_Abort(FOC, MotorControl, SpeedController, Startup);
            outcome.result = MOTOR_WORK_FAULT;
            outcome.error = Sensorless_Error;
            return outcome;
        }

        if (Encoder_ObserverCalib_IsStable(
                MotorControl, Fluxobserver, Startup, observer_position_epoch))
        {
            if (++state_ticks >=
                (uint32_t)(SENSORLESS_ENCODER_CALIB_SPEED_STABLE_TIME_S / Current_Ts))
            {
                previous_directed_q15 = Encoder->directed_q15;
                previous_observer_position = observer_position;
                origin_negative_seen = false;
                state_ticks = 0U;
                stage_ticks = 0U;
                CalibStep = CS_OBS_FIND_ORIGIN;
            }
        }
        else
        {
            state_ticks = 0U;
        }
        if (++stage_ticks >=
            (uint32_t)(SENSORLESS_ENCODER_CALIB_SPEED_STABLE_TIMEOUT_S / Current_Ts))
        {
            Set_ErrorNow(Sensorless_Error);
            Encoder_ObserverCalib_Abort(FOC, MotorControl, SpeedController, Startup);
            outcome.result = MOTOR_WORK_FAULT;
            outcome.error = Sensorless_Error;
        }
        break;

    case CS_OBS_FIND_ORIGIN:
    {
        int16_t previous_relative = Encoder_Calib_Q15Difference(previous_directed_q15, origin_q15);
        int16_t current_relative = Encoder_Calib_Q15Difference(Encoder->directed_q15, origin_q15);

        if (!Encoder_ObserverCalib_IsTracking(Fluxobserver, Startup, observer_position_epoch))
        {
            Set_ErrorNow(Sensorless_Error);
            Encoder_ObserverCalib_Abort(FOC, MotorControl, SpeedController, Startup);
            outcome.result = MOTOR_WORK_FAULT;
            outcome.error = Sensorless_Error;
            return outcome;
        }

        if (current_relative < -64)
            origin_negative_seen = true;

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
            CalibStep = CS_OBS_SAMPLE_CW;
            break;
        }

        previous_directed_q15 = Encoder->directed_q15;
        previous_observer_position = observer_position;
        if (++stage_ticks >=
            (uint32_t)(SENSORLESS_ENCODER_CALIB_FIND_ORIGIN_TIMEOUT_S / Current_Ts))
        {
            Set_ErrorNow(Sensorless_Error);
            Encoder_ObserverCalib_Abort(FOC, MotorControl, SpeedController, Startup);
            outcome.result = MOTOR_WORK_FAULT;
            outcome.error = Sensorless_Error;
        }
        break;
    }

    case CS_OBS_SAMPLE_CW:
        relative_theta = observer_position - observer_position_origin;
        if (!Encoder_ObserverCalib_IsTracking(Fluxobserver, Startup, observer_position_epoch))
        {
            Set_ErrorNow(Sensorless_Error);
            Encoder_ObserverCalib_Abort(FOC, MotorControl, SpeedController, Startup);
            outcome.result = MOTOR_WORK_FAULT;
            outcome.error = Sensorless_Error;
            return outcome;
        }

        if (Encoder_ObserverCalib_IsStable(
                MotorControl, Fluxobserver, Startup, observer_position_epoch))
        {
            float observer_delta = observer_position - sample_previous_observer_position;

            if (observer_delta > 0.0f)
                sample_valid_electrical_travel += observer_delta;

            if (relative_theta >= 0.0f)
            {
                uint16_t lut_index = Encoder->directed_q15 >> (16U - ENCODER_OFFSET_LUT_BITS);
                int32_t correction_q15;

                reference_q15 = (uint16_t)(relative_theta *
                                           ((float)ENCODER_Q15_CPR / required_electrical_theta));
                correction_q15 = Encoder_Calib_Q15Difference(Encoder->directed_q15, reference_q15);
                if (calibration_samples[lut_index] != 0U)
                {
                    int32_t average_q15 =
                        p_error_sum[lut_index] / (int32_t)calibration_samples[lut_index];
                    while (correction_q15 - average_q15 > ENCODER_Q15_HALF_TURN)
                        correction_q15 -= (int32_t)ENCODER_Q15_CPR;
                    while (correction_q15 - average_q15 < -ENCODER_Q15_HALF_TURN)
                        correction_q15 += (int32_t)ENCODER_Q15_CPR;
                }

                if (calibration_samples[lut_index] < UINT16_MAX)
                {
                    p_error_sum[lut_index] += correction_q15;
                    calibration_samples[lut_index]++;
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
            CalibStep = CS_OBS_BUILD_LUT;
            break;
        }
        if (++stage_ticks >= (uint32_t)(SENSORLESS_ENCODER_CALIB_SAMPLE_TIMEOUT_S / Current_Ts))
        {
            Set_ErrorNow(Sensorless_Error);
            Encoder_ObserverCalib_Abort(FOC, MotorControl, SpeedController, Startup);
            outcome.result = MOTOR_WORK_FAULT;
            outcome.error = Sensorless_Error;
        }
        break;

    case CS_OBS_BUILD_LUT:
        if (!Encoder_ObserverCalib_IsTracking(Fluxobserver, Startup, observer_position_epoch))
        {
            Set_ErrorNow(Sensorless_Error);
            Encoder_ObserverCalib_Abort(FOC, MotorControl, SpeedController, Startup);
            outcome.result = MOTOR_WORK_FAULT;
            outcome.error = Sensorless_Error;
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
                if (calibration_samples[candidate_lut_index] <
                    SENSORLESS_ENCODER_CALIB_MIN_SAMPLES_PER_BIN)
                {
                    Set_ErrorNow(Encoder_Error);
                    Encoder_ObserverCalib_Abort(FOC, MotorControl, SpeedController, Startup);
                    outcome.result = MOTOR_WORK_FAULT;
                    outcome.error = Encoder_Error;
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
                int32_t correction = (int16_t)(p_error_sum[candidate_lut_index] /
                                               (int32_t)calibration_samples[candidate_lut_index]);

                if (candidate_lut_index > 0U)
                {
                    while (correction - candidate_lut_previous > ENCODER_Q15_HALF_TURN)
                        correction -= (int32_t)ENCODER_Q15_CPR;
                    while (correction - candidate_lut_previous < -ENCODER_Q15_HALF_TURN)
                        correction += (int32_t)ENCODER_Q15_CPR;
                }

                p_error_sum[candidate_lut_index] = correction;
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
                candidate_lut_sum += p_error_sum[candidate_lut_index];
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
                int32_t correction = p_error_sum[candidate_lut_index] - candidate_lut_shift;

                if (correction < INT16_MIN || correction > INT16_MAX)
                {
                    Set_ErrorNow(Encoder_Error);
                    Encoder_ObserverCalib_Abort(FOC, MotorControl, SpeedController, Startup);
                    outcome.result = MOTOR_WORK_FAULT;
                    outcome.error = Encoder_Error;
                    return outcome;
                }

                candidate_linearization_lut[candidate_lut_index] = (int16_t)correction;
            }

            if (candidate_lut_index >= ENCODER_OFFSET_LUT_SIZE)
            {
                verify_previous_observer_position = observer_position;
                verify_valid_electrical_travel = 0.0f;
                residual_squared_sum = 0U;
                residual_sample_count = 0U;
                residual_peak_abs_q15 = 0U;
                stage_ticks = 0U;
                CalibStep = CS_OBS_VERIFY_CW;
            }
        }
        break;

    case CS_OBS_VERIFY_CW:
        relative_theta = observer_position - observer_position_origin;
        if (!Encoder_ObserverCalib_IsTracking(Fluxobserver, Startup, observer_position_epoch))
        {
            Set_ErrorNow(Sensorless_Error);
            Encoder_ObserverCalib_Abort(FOC, MotorControl, SpeedController, Startup);
            outcome.result = MOTOR_WORK_FAULT;
            outcome.error = Sensorless_Error;
            return outcome;
        }

        if (Encoder_ObserverCalib_IsStable(
                MotorControl, Fluxobserver, Startup, observer_position_epoch))
        {
            float observer_delta = observer_position - verify_previous_observer_position;

            if (observer_delta > 0.0f)
                verify_valid_electrical_travel += observer_delta;

            if (relative_theta >= 0.0f)
            {
                int16_t residual_q15;
                int32_t residual_abs_q15;
                uint16_t linearized_q15;

                reference_q15 = (uint16_t)(relative_theta *
                                           ((float)ENCODER_Q15_CPR / required_electrical_theta));
                linearized_q15 = Encoder_Calib_ApplyCandidateLut(Encoder->directed_q15);
                residual_q15 = Encoder_Calib_Q15Difference(linearized_q15, reference_q15);
                residual_abs_q15 = residual_q15 >= 0 ? residual_q15 : -(int32_t)residual_q15;
                residual_squared_sum += (uint64_t)((int64_t)residual_q15 * (int64_t)residual_q15);
                if ((uint32_t)residual_abs_q15 > residual_peak_abs_q15)
                    residual_peak_abs_q15 = (uint32_t)residual_abs_q15;
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
                Set_ErrorNow(Encoder_Error);
                Encoder_ObserverCalib_Abort(FOC, MotorControl, SpeedController, Startup);
                outcome.result = MOTOR_WORK_FAULT;
                outcome.error = Encoder_Error;
                return outcome;
            }

            Encoder_Calib_CommitCandidateLut(Encoder);
            Encoder_Calib_ReleaseSamples();
            stop_start_speed = MotorControl->speedShadow;
            if (stop_start_speed < Encoder_ObserverCalib_GetStopSpeed(MotorControl))
                stop_start_speed = Encoder_ObserverCalib_GetStopSpeed(MotorControl);
            state_ticks = 0U;
            CalibStep = CS_OBS_STOP_DECEL;
            break;
        }
        if (++stage_ticks >= (uint32_t)(SENSORLESS_ENCODER_CALIB_VERIFY_TIMEOUT_S / Current_Ts))
        {
            Set_ErrorNow(Sensorless_Error);
            Encoder_ObserverCalib_Abort(FOC, MotorControl, SpeedController, Startup);
            outcome.result = MOTOR_WORK_FAULT;
            outcome.error = Sensorless_Error;
        }
        break;

    case CS_OBS_STOP_DECEL:
    {
        float stop_speed = Encoder_ObserverCalib_GetStopSpeed(MotorControl);
        float observer_mech_vel =
            Observer_GetEleVel(Fluxobserver) / (float)MotorControl->motor_pole_pairs;
        bool speed_reached =
            fast_abs(observer_mech_vel) <=
            stop_speed * (1.0f + SENSORLESS_ENCODER_CALIB_STOP_SPEED_TOLERANCE_RATIO);

        state_ticks++;
        if (state_ticks >= (uint32_t)(SENSORLESS_ENCODER_CALIB_STOP_DECEL_TIME_S / Current_Ts) &&
            speed_reached)
        {
            stop_current_ref = fast_min(MotorControl->iqRef, 0.0f);
            state_ticks = 0U;
            CalibStep = CS_OBS_STOP_CURRENT;
        }
        else if (state_ticks >=
                 (uint32_t)(SENSORLESS_ENCODER_CALIB_STOP_DECEL_TIMEOUT_S / Current_Ts))
        {
            Set_ErrorNow(Sensorless_Error);
            Encoder_ObserverCalib_Abort(FOC, MotorControl, SpeedController, Startup);
            outcome.result = MOTOR_WORK_FAULT;
            outcome.error = Sensorless_Error;
        }
        break;
    }

    case CS_OBS_STOP_CURRENT:
        if (++state_ticks >=
            (uint32_t)(SENSORLESS_ENCODER_CALIB_STOP_CURRENT_RAMP_TIME_S / Current_Ts))
        {
            return Encoder_ObserverCalib_Finish(FOC, MotorControl, SpeedController, Startup);
        }
        break;

    default:
        Set_ErrorNow(Encoder_Error);
        Encoder_ObserverCalib_Abort(FOC, MotorControl, SpeedController, Startup);
        outcome.result = MOTOR_WORK_FAULT;
        outcome.error = Encoder_Error;
        break;
    }
    return outcome;
}

/**
 * @brief Calibrate electrical zero using the averaged linearized Q15 angle.
 */
MotorWorkOutcome_TypeDef Task_Calib_EleAngelOffset(FOC_TypeDef *FOC,
                                                   MotorControl_TypeDef *MotorControl,
                                                   Encoder_TypeDef *Encoder)
{
    static uint32_t loop_count;
    static uint32_t sample_count;
    static uint16_t sample_anchor;
    static int64_t unwrapped_sum;
    MotorWorkOutcome_TypeDef outcome = {MOTOR_WORK_RUNNING, Motor_Disable, No_Error, false};
    float time = (float)loop_count * Current_Ts;
    float align_current = MotorControl->calib_current;

    if (align_current < ENCODER_ELEC_ZERO_MIN_ALIGN_CURRENT_A)
        align_current = ENCODER_ELEC_ZERO_MIN_ALIGN_CURRENT_A;

    if (!Encoder_IsOnline(Encoder))
    {
        Set_ErrorNow(Encoder_Error);
        loop_count = 0U;
        sample_count = 0U;
        unwrapped_sum = 0;
        outcome.result = MOTOR_WORK_FAULT;
        outcome.error = Encoder_Error;
        return outcome;
    }
    if ((Encoder->calib_flag & ENC_CALIB_LINEARIZED) == 0U)
    {
        Set_ErrorNow(Encoder_NotCalibrated);
        outcome.result = MOTOR_WORK_FAULT;
        outcome.error = Encoder_NotCalibrated;
        return outcome;
    }
    if (align_current <= 0.0f)
    {
        Set_ErrorNow(MotorParam_Error);
        outcome.result = MOTOR_WORK_FAULT;
        outcome.error = MotorParam_Error;
        return outcome;
    }
    if (MotorControl->current_limit > 0.0f && align_current > MotorControl->current_limit)
        align_current = MotorControl->current_limit;

    if (loop_count == 0U)
    {
        sample_count = 0U;
        sample_anchor = Encoder->linearized_q15;
        unwrapped_sum = 0;
        Encoder->calib_flag &= (uint8_t)~ENC_CALIB_ELECTRICAL_ZERO;
        FOC_CurrentController_Reset(FOC);
    }

    MotorControl->idRef = align_current * constrain(((float)loop_count + 1.0f) * Current_Ts /
                                                        ENCODER_ELEC_ZERO_CURRENT_RAMP_TIME_S,
                                                    0.0f,
                                                    1.0f);
    MotorControl->iqRef = 0.0f;
    FOC_Current(FOC, MotorControl, 0.0f, 0.0f);

    if (time >= (ENC_ZERO_ALIGN_TIME - ENC_ZERO_SAMPLE_TIME) &&
        Encoder->read_status == ENCODER_READ_OK)
    {
        int32_t unwrapped_q15 =
            (int32_t)sample_anchor + (int16_t)(uint16_t)(Encoder->linearized_q15 - sample_anchor);
        unwrapped_sum += unwrapped_q15;
        sample_count++;
    }

    if (time >= ENC_ZERO_ALIGN_TIME)
    {
        bool calibrated = false;
        if (sample_count > 0U)
        {
            uint16_t electrical_zero_q15 = (uint16_t)(unwrapped_sum / (int64_t)sample_count);
            calibrated = Encoder_SetElectricalZeroQ15(Encoder, electrical_zero_q15);
        }

        MotorControl->idRef = 0.0f;
        MotorControl->iqRef = 0.0f;
        FOC_CurrentController_Reset(FOC);
        loop_count = 0U;
        sample_count = 0U;
        unwrapped_sum = 0;
        /* 停相与模式迁移由运行状态机执行（power_off=true）；旧实现的"高边短接保持"
         * 不再由模块直接操作功率级。 */

        if (!calibrated)
        {
            Set_ErrorNow(Encoder_Error);
            outcome.result = MOTOR_WORK_FAULT;
            outcome.error = Encoder_Error;
            return outcome;
        }
        outcome.result = MOTOR_WORK_SWITCH_MODE;
        outcome.next_mode = Save_Param;
        outcome.power_off = true;
        return outcome;
    }

    loop_count++;
    return outcome;
}
