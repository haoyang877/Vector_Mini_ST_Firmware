#include "bsp_system.h"
#include "encoder_calibration.h"

#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "application_config.h"
#include "motor_fault.h"
#include "sensorless_startup.h"
#include "control_math.h"

/* 固定环形 Q15 表示；任务边界静态核对反馈层的同一布局，不引入电机运行对象。 */
#define ENCODER_CALIBRATION_Q15_CPR 65536UL
#define ENCODER_CALIBRATION_Q15_HALF_TURN 32768
#define ENCODER_CALIBRATION_LUT_SIZE 1024U
#define ENCODER_CALIBRATION_LUT_BITS 10U

/** 单帧事实：任务已完成跟踪准入；stable 只控制本帧是否计入采样/验证。 */
typedef struct
{
    float observer_position_rad;
    float relative_theta_rad;
    float required_electrical_theta_rad;
    uint16_t directed_q15;
    bool stable;
} EncoderCalibrationSample;

/** 合格正向电气行程，rad；无效帧也更新前次位置，防止把间隙位移计入。 */
typedef struct
{
    float previous_observer_position_rad;
    float valid_electrical_travel_rad;
} EncoderCalibrationWindow;

/** 四遍计算游标；任务持有，只有 BuildLutStep 推进，开始时整体置零。 */
typedef struct
{
    uint16_t candidate_lut_index;
    uint8_t candidate_lut_stage;
    int32_t candidate_lut_previous;
    int32_t candidate_lut_shift;
    int64_t candidate_lut_sum;
} EncoderCalibrationBuildState;

/** 测量归约状态；采样/验证互斥复用窗口，开始验证时整体清零并重设previous。 */
typedef struct
{
    EncoderCalibrationWindow window;
    uint64_t residual_squared_sum;
    uint32_t residual_sample_count;
    uint32_t residual_peak_abs_q15;
} EncoderCalibrationVerifyState;

/** 只表示计算进度或数据不合格；任务决定故障、阶段转换及候选应用。 */
typedef enum
{
    ENCODER_CALIBRATION_BUILD_RUNNING = 0,
    ENCODER_CALIBRATION_BUILD_COMPLETE,
    ENCODER_CALIBRATION_BUILD_INVALID
} EncoderCalibrationBuildResult;

/* 所有可变状态和表均由任务持有；核心只消费显式采样事实并写计算结果。 */

/* 统一采样/验证的稳定准入与正行程；previous 仍由调用者在帧末写入。 */
static bool begin_sample(EncoderCalibrationWindow *window, const EncoderCalibrationSample *sample)
{
    if (!sample->stable)
    {
        return false;
    }
    float observer_delta = sample->observer_position_rad - window->previous_observer_position_rad;
    if (observer_delta > 0.0f)
    {
        window->valid_electrical_travel_rad += observer_delta;
    }
    return sample->relative_theta_rad >= 0.0f;
}

static int16_t q15_difference(uint16_t target_q15, uint16_t source_q15)
{
    return (int16_t)(uint16_t)(target_q15 - source_q15);
}

static uint16_t apply_lut(const int16_t *candidate, uint16_t directed_q15)
{
    uint16_t lut_index = directed_q15 >> 6;
    uint16_t fraction = directed_q15 & 0x003FU;
    int32_t correction_a = candidate[lut_index];
    int32_t correction_b = candidate[(lut_index + 1U) & (ENCODER_CALIBRATION_LUT_SIZE - 1U)];
    int32_t correction = correction_a + (((correction_b - correction_a) * fraction) >> 6);

    return (uint16_t)((int32_t)directed_q15 - correction);
}

static void
add_angle_sample(uint16_t angle_q15, uint16_t anchor_q15, int64_t *sum_q15, uint32_t *count)
{
    int32_t unwrapped_q15 = (int32_t)anchor_q15 + (int16_t)(uint16_t)(angle_q15 - anchor_q15);
    *sum_q15 += unwrapped_q15;
    (*count)++;
}

static bool mean_q15(int64_t sum_q15, uint32_t count, uint16_t *mean_q15)
{
    if (count == 0U)
    {
        return false;
    }
    *mean_q15 = (uint16_t)(sum_q15 / (int64_t)count);
    return true;
}

static void sample_lut(EncoderCalibrationWorkspace *workspace,
                       EncoderCalibrationWindow *window,
                       const EncoderCalibrationSample *sample)
{
    if (begin_sample(window, sample))
    {
        uint16_t lut_index = sample->directed_q15 >> (16U - ENCODER_CALIBRATION_LUT_BITS);
        uint16_t reference_q15 =
            (uint16_t)(sample->relative_theta_rad * ((float)ENCODER_CALIBRATION_Q15_CPR /
                                                     sample->required_electrical_theta_rad));
        int32_t correction_q15 = q15_difference(sample->directed_q15, reference_q15);
        if (workspace->bins.samples[lut_index] != 0U)
        {
            int32_t average_q15 =
                workspace->error_sum[lut_index] / (int32_t)workspace->bins.samples[lut_index];
            while (correction_q15 - average_q15 > ENCODER_CALIBRATION_Q15_HALF_TURN)
            {
                correction_q15 -= (int32_t)ENCODER_CALIBRATION_Q15_CPR;
            }
            while (correction_q15 - average_q15 < -ENCODER_CALIBRATION_Q15_HALF_TURN)
            {
                correction_q15 += (int32_t)ENCODER_CALIBRATION_Q15_CPR;
            }
        }
        if (workspace->bins.samples[lut_index] < UINT16_MAX)
        {
            workspace->error_sum[lut_index] += correction_q15;
            workspace->bins.samples[lut_index]++;
        }
    }
    window->previous_observer_position_rad = sample->observer_position_rad;
}

static EncoderCalibrationBuildResult build_lut_step(EncoderCalibrationWorkspace *workspace,
                                                    EncoderCalibrationBuildState *state,
                                                    uint16_t bins_per_cycle,
                                                    uint16_t minimum_samples)
{
    /* 四遍只共用游标/每拍上界；每个 bin 的数学与写入顺序保持原样。 */
    /* 保持每拍有界循环，避免编译器将四遍构表展开到快环。 */
#if defined(__clang__)
#pragma clang loop unroll(disable)
#endif
    for (uint16_t bins_processed = 0U; bins_processed < bins_per_cycle &&
                                       state->candidate_lut_index < ENCODER_CALIBRATION_LUT_SIZE;
         ++bins_processed, ++state->candidate_lut_index)
    {
        switch (state->candidate_lut_stage)
        {
        case 0U:
            if (workspace->bins.samples[state->candidate_lut_index] < minimum_samples)
            {
                return ENCODER_CALIBRATION_BUILD_INVALID;
            }
            break;
        case 1U:
        {
            int32_t correction =
                (int16_t)(workspace->error_sum[state->candidate_lut_index] /
                          (int32_t)workspace->bins.samples[state->candidate_lut_index]);
            if (state->candidate_lut_index > 0U)
            {
                while (correction - state->candidate_lut_previous >
                       ENCODER_CALIBRATION_Q15_HALF_TURN)
                {
                    correction -= (int32_t)ENCODER_CALIBRATION_Q15_CPR;
                }
                while (correction - state->candidate_lut_previous <
                       -ENCODER_CALIBRATION_Q15_HALF_TURN)
                {
                    correction += (int32_t)ENCODER_CALIBRATION_Q15_CPR;
                }
            }
            workspace->error_sum[state->candidate_lut_index] = correction;
            state->candidate_lut_previous = correction;
            break;
        }
        case 2U:
            state->candidate_lut_sum += workspace->error_sum[state->candidate_lut_index];
            break;
        default:
        {
            int32_t correction =
                workspace->error_sum[state->candidate_lut_index] - state->candidate_lut_shift;
            if (correction < INT16_MIN || correction > INT16_MAX)
            {
                return ENCODER_CALIBRATION_BUILD_INVALID;
            }
            workspace->bins.candidate[state->candidate_lut_index] = (int16_t)correction;
            break;
        }
        }
    }
    if (state->candidate_lut_index < ENCODER_CALIBRATION_LUT_SIZE)
    {
        return ENCODER_CALIBRATION_BUILD_RUNNING;
    }
    switch (state->candidate_lut_stage)
    {
    case 0U:
        state->candidate_lut_index = 0U;
        state->candidate_lut_stage = 1U;
        break;
    case 1U:
        state->candidate_lut_index = 0U;
        state->candidate_lut_sum = 0;
        state->candidate_lut_stage = 2U;
        break;
    case 2U:
        state->candidate_lut_sum /= (int64_t)ENCODER_CALIBRATION_LUT_SIZE;
        state->candidate_lut_shift = 0;
        while (state->candidate_lut_sum > INT16_MAX)
        {
            state->candidate_lut_sum -= (int32_t)ENCODER_CALIBRATION_Q15_CPR;
            state->candidate_lut_shift += (int32_t)ENCODER_CALIBRATION_Q15_CPR;
        }
        while (state->candidate_lut_sum < INT16_MIN)
        {
            state->candidate_lut_sum += (int32_t)ENCODER_CALIBRATION_Q15_CPR;
            state->candidate_lut_shift -= (int32_t)ENCODER_CALIBRATION_Q15_CPR;
        }
        state->candidate_lut_index = 0U;
        state->candidate_lut_stage = 3U;
        break;
    default:
        return ENCODER_CALIBRATION_BUILD_COMPLETE;
    }
    return ENCODER_CALIBRATION_BUILD_RUNNING;
}

static void verify_lut(EncoderCalibrationVerifyState *state,
                       const int16_t *candidate,
                       const EncoderCalibrationSample *sample)
{
    if (begin_sample(&state->window, sample))
    {
        uint16_t reference_q15 =
            (uint16_t)(sample->relative_theta_rad * ((float)ENCODER_CALIBRATION_Q15_CPR /
                                                     sample->required_electrical_theta_rad));
        uint16_t linearized_q15 = apply_lut(candidate, sample->directed_q15);
        int16_t residual_q15 = q15_difference(linearized_q15, reference_q15);
        int32_t residual_abs_q15 = residual_q15 >= 0 ? residual_q15 : -(int32_t)residual_q15;
        state->residual_squared_sum += (uint64_t)((int64_t)residual_q15 * (int64_t)residual_q15);
        if ((uint32_t)residual_abs_q15 > state->residual_peak_abs_q15)
        {
            state->residual_peak_abs_q15 = (uint32_t)residual_abs_q15;
        }
        state->residual_sample_count++;
    }
    state->window.previous_observer_position_rad = sample->observer_position_rad;
}

static bool validate_lut(const EncoderCalibrationVerifyState *state,
                         uint32_t maximum_rms_q15,
                         uint32_t maximum_peak_q15)
{
    uint64_t max_rms_squared = (uint64_t)maximum_rms_q15 * (uint64_t)maximum_rms_q15;
    return !(
        state->residual_sample_count == 0U || state->residual_peak_abs_q15 > maximum_peak_q15 ||
        state->residual_squared_sum > (uint64_t)state->residual_sample_count * max_rms_squared);
}

/* 编码器标定：纯计算辅助与控制流程同文件私有，不增加模块间接口。
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

#define ENCODER_CALIBRATION_PREPARE_BINS_PER_TICK 4U
#define ENCODER_ELEC_ZERO_CURRENT_RAMP_TIME_S 0.50f
#define ENCODER_ELEC_ZERO_HOLD_TIME_S 1.00f
#define ENC_ZERO_ALIGN_TIME (ENCODER_ELEC_ZERO_CURRENT_RAMP_TIME_S + ENCODER_ELEC_ZERO_HOLD_TIME_S)

/* 会话状态唯一归任务所有；借用的控制对象只在当前调用有效。 */
typedef struct
{
    EncoderCalibrationWorkspace *workspace;
    EncoderCalibrationPhase phase;
    uint16_t prepare_bin;
    uint32_t ticks, stage_ticks, position_epoch;
    uint16_t origin_q15;
    float origin_position_rad;
    /* 对齐、过零、采样/验证、构表和停止互斥；转换时先消费旧值再初始化新成员。 */
    union
    {
        struct
        {
            int64_t sum_q15;
            uint32_t count;
            uint16_t anchor_q15;
        } origin;
        struct
        {
            float previous_position_rad;
            uint16_t previous_q15;
            bool negative_seen;
        } crossing;
        EncoderCalibrationVerifyState verify;
        EncoderCalibrationBuildState build;
        struct
        {
            float speed_rad_s;
            float current_a;
        } stop;
    } stage;
} EncoderLutSession;

typedef struct
{
    uint32_t ticks, sample_count;
    uint16_t anchor_q15;
    int64_t sum_q15;
} EncoderZeroSession;

static EncoderLutSession s_lut;
static EncoderZeroSession s_zero;

void EncoderCalibrationTask_Cancel(void)
{
    s_lut = (EncoderLutSession){0};
    s_zero = (EncoderZeroSession){0};
}

bool EncoderCalibrationTask_IsReleased(void)
{
    return s_lut.workspace == NULL && s_lut.phase == CS_NULL && s_zero.ticks == 0U;
}

EncoderCalibrationPhase EncoderCalibrationTask_GetStep(void)
{
    return s_lut.phase;
}

static void reset_lut_controllers(const EncoderCalibrationControl *drive)
{
    SensorlessStartup_Reset(drive->startup);
    Foc_ResetCurrent(drive->foc);
    PiController_Reset(drive->speed_pi);
}

/* 结束运动命令，不关相、不写参数；输出许可始终由生命周期所有者裁决。 */
static void reset_lut(const EncoderCalibrationControl *drive)
{
    s_lut = (EncoderLutSession){0};
    reset_lut_controllers(drive);
    drive->motor->speed_target_rad_s = 0.0f;
    drive->motor->speed_ramped_rad_s = 0.0f;
    drive->motor->id_target_a = 0.0f;
    drive->motor->iq_target_a = 0.0f;
}

static MotorTaskResult fail_lut(const EncoderCalibrationControl *drive, MotorFaultCode error)
{
    MotorFault_Report(drive->motor, error);
    reset_lut(drive);
    return (MotorTaskResult){MOTOR_TASK_FAILED, error};
}

static MotorTaskResult finish_lut(const EncoderCalibrationControl *drive)
{
    if ((s_lut.phase != CS_OBS_STOP_DECEL && s_lut.phase != CS_OBS_STOP_CURRENT) ||
        s_lut.workspace == NULL)
    {
        return fail_lut(drive, MOTOR_FAULT_ENCODER);
    }
    EncoderCalibrationWorkspace *candidate = s_lut.workspace;
    reset_lut(drive);
    s_lut.workspace = candidate;
    s_lut.phase = CS_LUT_PUBLISH_PENDING;
    return (MotorTaskResult){MOTOR_TASK_FINALIZE, MOTOR_FAULT_NONE};
}

bool EncoderCalibrationTask_PublishLut(EncoderCalibration *calibration, EncoderState *encoder)
{
    if (calibration == NULL || encoder == NULL || s_lut.workspace == NULL ||
        s_lut.phase != CS_LUT_PUBLISH_PENDING)
    {
        return false;
    }
    uint16_t current_linearized_q15;
    /* 后台发布期间保持关相；先撤销LUT资格，快速采样不能读取半张表。 */
    uint8_t published_flags =
        (uint8_t)((calibration->calib_flag &
                   (uint8_t)~(ENC_CALIB_ELECTRICAL_ZERO | ENC_CALIB_MECHANICAL_ZERO)) |
                  ENC_CALIB_LINEARIZED);
    calibration->calib_flag = 0U;
    BspSystem_MemoryBarrier();

    memcpy(calibration->linearization_lut_q15,
           s_lut.workspace->bins.candidate,
           sizeof(calibration->linearization_lut_q15));
    current_linearized_q15 = apply_lut(s_lut.workspace->bins.candidate, encoder->directed_q15);
    encoder->linearized_q15 = current_linearized_q15;
    calibration->electrical_zero_q15 = 0U;
    calibration->mechanical_zero_q15 = 0U;
    BspSystem_MemoryBarrier();
    calibration->calib_flag = published_flags;
    /* 慢状态由 2 kHz 慢估计独占：请求重定多圈基准（基准取当前线性化角、零影子归零）。 */
    encoder->rebase_requested = true;
    Encoder_ResetVelocity(encoder);
    EncoderCalibrationTask_Cancel();
    return true;
}

static bool lut_is_stable(const MotorConfiguration *configuration,
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
           FluxObserver_GetPositionEpoch(flux_observer) == position_epoch &&
           Math_Abs(filtered_electrical_speed - target_electrical_speed) <=
               Math_Abs(target_electrical_speed) * SENSORLESS_ENCODER_CALIB_SPEED_ERROR_RATIO &&
           Math_Abs(motor_state->speed_ramped_rad_s - SENSORLESS_ENCODER_CALIB_SPEED_MEC_RAD_S) <=
               SENSORLESS_ENCODER_CALIB_SPEED_MEC_RAD_S *
                   SENSORLESS_ENCODER_CALIB_SPEED_ERROR_RATIO;
}

static bool lut_is_tracking(FluxObserverState *flux_observer,
                            SensorlessStartupState *startup,
                            uint32_t position_epoch)
{
    return startup->state == SENSORLESS_STARTUP_CLOSED_LOOP &&
           FluxObserver_GetPositionEpoch(flux_observer) == position_epoch;
}

static float lut_stop_speed(const MotorConfiguration *configuration)
{
    float minimum_mechanical_speed =
        SensorlessStartup_EncoderCalibConfig.minimum_electrical_velocity_rad_s /
        (float)configuration->model.motor_pole_pairs;
    float stop_speed = minimum_mechanical_speed * SENSORLESS_ENCODER_CALIB_STOP_SPEED_MARGIN;

    return stop_speed < SENSORLESS_ENCODER_CALIB_SPEED_MEC_RAD_S
               ? stop_speed
               : SENSORLESS_ENCODER_CALIB_SPEED_MEC_RAD_S;
}

static bool lut_can_brake(const MotorConfiguration *configuration,
                          const EncoderState *encoder,
                          FluxObserverState *flux_observer,
                          SensorlessStartupState *startup,
                          uint32_t position_epoch)
{
    float minimum_speed = SensorlessStartup_EncoderCalibConfig.minimum_electrical_velocity_rad_s;
    float observer_speed = FluxObserver_GetElecVel(flux_observer);
    float observer_phase = FluxObserver_GetElecPhase(flux_observer);
    float encoder_speed = Encoder_GetMechVelContinuous(encoder);

    if (!lut_is_tracking(flux_observer, startup, position_epoch) ||
        !(observer_speed > minimum_speed) ||
        observer_speed > SENSORLESS_OBSERVER_MAX_ELEC_VEL_RAD_S ||
        !(observer_phase >= 0.0f && observer_phase <= _2PI))
    {
        return false;
    }
    /* 以编码器速度独立提前截止制动；关相前保持旧LUT，不依赖电零位。 */
    return !encoder->velocity_ready ||
           (encoder_speed > minimum_speed / (float)configuration->model.motor_pole_pairs &&
            encoder_speed <= SENSORLESS_OBSERVER_MAX_ELEC_VEL_RAD_S /
                                 (float)configuration->model.motor_pole_pairs);
}

static MotorTaskResult run_lut_control(const EncoderCalibrationControl *drive)
{
    const MotorConfiguration *configuration = drive->configuration;
    MotorControlState *motor_state = drive->motor;
    const EncoderState *encoder = drive->encoder;
    FluxObserverState *flux_observer = drive->observer;
    SensorlessStartupState *startup = drive->startup;
    FocState *foc = drive->foc;
    PiController *speed_pi = drive->speed_pi;
    if ((s_lut.phase == CS_OBS_STOP_DECEL || s_lut.phase == CS_OBS_STOP_CURRENT) &&
        !lut_can_brake(configuration, encoder, flux_observer, startup, s_lut.position_epoch))
    {
        /* 两个停止段均已验证候选LUT；低于观测器可用速度时撤销转矩，避免驱动过零。 */
        return finish_lut(drive);
    }

    if (s_lut.phase == CS_OBS_STOP_CURRENT)
    {
        float current_ratio;

        current_ratio = Math_Constrain(((float)s_lut.ticks + 1.0f) * FOC_TS_S /
                                           SENSORLESS_ENCODER_CALIB_STOP_CURRENT_RAMP_TIME_S,
                                       0.0f,
                                       1.0f);
        motor_state->id_target_a = 0.0f;
        motor_state->iq_target_a = s_lut.stage.stop.current_a * (1.0f - current_ratio);
        Foc_RunCurrent(configuration,
                       foc,
                       motor_state,
                       FluxObserver_GetElecPhase(flux_observer),
                       FluxObserver_GetElecVel(flux_observer));
    }
    else
    {
        if (s_lut.phase == CS_OBS_STOP_DECEL)
        {
            startup->speed_pi_output_max = 0.0f;
            float decel_ratio = Math_Constrain((float)s_lut.ticks * FOC_TS_S /
                                                   SENSORLESS_ENCODER_CALIB_STOP_DECEL_TIME_S,
                                               0.0f,
                                               1.0f);
            float stop_speed = lut_stop_speed(configuration);

            motor_state->speed_target_rad_s =
                s_lut.stage.stop.speed_rad_s +
                (stop_speed - s_lut.stage.stop.speed_rad_s) * decel_ratio;
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
            reset_lut(drive);
            return (MotorTaskResult){MOTOR_TASK_FAILED, (MotorFaultCode)motor_state->fault_code};
        }
    }

    return (MotorTaskResult){MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};
}

static MotorTaskResult advance_lut(const EncoderCalibrationControl *drive)
{
    const MotorConfiguration *configuration = drive->configuration;
    MotorControlState *motor_state = drive->motor;
    const EncoderState *encoder = drive->encoder;
    FluxObserverState *flux_observer = drive->observer;
    SensorlessStartupState *startup = drive->startup;
    float required_electrical_theta = _2PI * (float)configuration->model.motor_pole_pairs;
    float observer_position = FluxObserver_GetElecPos(flux_observer);

    /* 找原点、采样、构表、验证共享跟踪准入；稳速窗口和停止段仍保留各自判据。 */
    if (s_lut.phase >= CS_OBS_FIND_ORIGIN && s_lut.phase <= CS_OBS_VERIFY_CW &&
        !lut_is_tracking(flux_observer, startup, s_lut.position_epoch))
    {
        return fail_lut(drive, MOTOR_FAULT_SENSORLESS);
    }

    switch (s_lut.phase)
    {
    case CS_OBS_ALIGN_ORIGIN:
        /* 原对齐时间内分拍清表；采样尚未开始，不集中清空6 KiB工作区。 */
        for (uint16_t count = 0U; count < ENCODER_CALIBRATION_PREPARE_BINS_PER_TICK &&
                                  s_lut.prepare_bin < ENCODER_OFFSET_LUT_SIZE;
             ++count)
        {
            s_lut.workspace->error_sum[s_lut.prepare_bin] = 0;
            s_lut.workspace->bins.samples[s_lut.prepare_bin] = 0U;
            ++s_lut.prepare_bin;
        }
        if (startup->state == SENSORLESS_STARTUP_ALIGN &&
            startup->state_ticks >=
                (uint32_t)(((SensorlessStartup_EncoderCalibConfig.align_current_ramp_time_s +
                             SensorlessStartup_EncoderCalibConfig.align_hold_time_s) -
                            SENSORLESS_ENCODER_CALIB_ALIGN_SAMPLE_TIME_S) /
                           FOC_TS_S))
        {
            uint16_t directed_q15 = encoder->directed_q15;
            if (s_lut.stage.origin.count == 0U)
            {
                s_lut.stage.origin.anchor_q15 = directed_q15;
            }
            add_angle_sample(directed_q15,
                             s_lut.stage.origin.anchor_q15,
                             &s_lut.stage.origin.sum_q15,
                             &s_lut.stage.origin.count);
        }

        if (startup->state != SENSORLESS_STARTUP_ALIGN)
        {
            if (s_lut.prepare_bin != ENCODER_OFFSET_LUT_SIZE ||
                !mean_q15(s_lut.stage.origin.sum_q15, s_lut.stage.origin.count, &s_lut.origin_q15))
            {
                return fail_lut(drive, MOTOR_FAULT_ENCODER);
            }

            s_lut.ticks = 0U;
            s_lut.stage_ticks = 0U;
            s_lut.phase = CS_OBS_WAIT_CLOSED_LOOP;
        }
        break;

    case CS_OBS_WAIT_CLOSED_LOOP:
        if (startup->state == SENSORLESS_STARTUP_CLOSED_LOOP)
        {
            s_lut.position_epoch = FluxObserver_GetPositionEpoch(flux_observer);
            s_lut.ticks = 0U;
            s_lut.stage_ticks = 0U;
            s_lut.phase = CS_OBS_SPEED_STABLE;
        }
        else if (++s_lut.stage_ticks >=
                 (uint32_t)(SENSORLESS_ENCODER_CALIB_STARTUP_TIMEOUT_S / FOC_TS_S))
        {
            return fail_lut(drive, MOTOR_FAULT_SENSORLESS);
        }
        break;

    case CS_OBS_SPEED_STABLE:
        if (FluxObserver_GetPositionEpoch(flux_observer) != s_lut.position_epoch)
        {
            return fail_lut(drive, MOTOR_FAULT_SENSORLESS);
        }

        if (lut_is_stable(configuration, motor_state, flux_observer, startup, s_lut.position_epoch))
        {
            if (++s_lut.ticks >=
                (uint32_t)(SENSORLESS_ENCODER_CALIB_SPEED_STABLE_TIME_S / FOC_TS_S))
            {
                s_lut.stage.crossing.previous_q15 = encoder->directed_q15;
                s_lut.stage.crossing.previous_position_rad = observer_position;
                s_lut.stage.crossing.negative_seen = false;
                s_lut.ticks = 0U;
                s_lut.stage_ticks = 0U;
                s_lut.phase = CS_OBS_FIND_ORIGIN;
            }
        }
        else
        {
            s_lut.ticks = 0U;
        }
        if (++s_lut.stage_ticks >=
            (uint32_t)(SENSORLESS_ENCODER_CALIB_SPEED_STABLE_TIMEOUT_S / FOC_TS_S))
        {
            return fail_lut(drive, MOTOR_FAULT_SENSORLESS);
        }
        break;

    case CS_OBS_FIND_ORIGIN:
    {
        int16_t previous_relative =
            q15_difference(s_lut.stage.crossing.previous_q15, s_lut.origin_q15);
        int16_t current_relative = q15_difference(encoder->directed_q15, s_lut.origin_q15);

        if (current_relative < -64)
        {
            s_lut.stage.crossing.negative_seen = true;
        }

        if (s_lut.stage.crossing.negative_seen && previous_relative < 0 && current_relative >= 0)
        {
            float crossing_fraction =
                (float)(-previous_relative) / (float)(current_relative - previous_relative);

            s_lut.origin_position_rad =
                s_lut.stage.crossing.previous_position_rad +
                crossing_fraction *
                    (observer_position - s_lut.stage.crossing.previous_position_rad);
            s_lut.stage.verify.window.previous_observer_position_rad = observer_position;
            s_lut.stage.verify.window.valid_electrical_travel_rad = 0.0f;
            s_lut.stage_ticks = 0U;
            s_lut.phase = CS_OBS_SAMPLE_CW;
            break;
        }

        s_lut.stage.crossing.previous_q15 = encoder->directed_q15;
        s_lut.stage.crossing.previous_position_rad = observer_position;
        if (++s_lut.stage_ticks >=
            (uint32_t)(SENSORLESS_ENCODER_CALIB_FIND_ORIGIN_TIMEOUT_S / FOC_TS_S))
        {
            return fail_lut(drive, MOTOR_FAULT_SENSORLESS);
        }
        break;
    }

    case CS_OBS_SAMPLE_CW:
    case CS_OBS_VERIFY_CW:
    {
        EncoderCalibrationSample sample = {
            .observer_position_rad = observer_position,
            .relative_theta_rad = observer_position - s_lut.origin_position_rad,
            .required_electrical_theta_rad = required_electrical_theta,
            .directed_q15 = encoder->directed_q15,
            .stable = lut_is_stable(
                configuration, motor_state, flux_observer, startup, s_lut.position_epoch),
        };
        if (s_lut.phase == CS_OBS_SAMPLE_CW)
        {
            sample_lut(s_lut.workspace, &s_lut.stage.verify.window, &sample);

            if (s_lut.stage.verify.window.valid_electrical_travel_rad >=
                (float)SENSORLESS_ENCODER_CALIB_MECH_TURNS * required_electrical_theta)
            {
                s_lut.stage.build = (EncoderCalibrationBuildState){0};
                s_lut.phase = CS_OBS_BUILD_LUT;
                break;
            }
            if (++s_lut.stage_ticks >=
                (uint32_t)(SENSORLESS_ENCODER_CALIB_SAMPLE_TIMEOUT_S / FOC_TS_S))
            {
                return fail_lut(drive, MOTOR_FAULT_SENSORLESS);
            }
        }
        else
        {
            verify_lut(&s_lut.stage.verify, s_lut.workspace->bins.candidate, &sample);

            if (s_lut.stage.verify.window.valid_electrical_travel_rad >=
                (float)SENSORLESS_ENCODER_CALIB_VERIFY_MECH_TURNS * required_electrical_theta)
            {
                if (!validate_lut(&s_lut.stage.verify,
                                  SENSORLESS_ENCODER_CALIB_MAX_RMS_RESIDUAL_Q15,
                                  SENSORLESS_ENCODER_CALIB_MAX_PEAK_RESIDUAL_Q15))
                {
                    return fail_lut(drive, MOTOR_FAULT_ENCODER);
                }

                s_lut.stage.stop.speed_rad_s = motor_state->speed_ramped_rad_s;
                float stop_speed = lut_stop_speed(configuration);
                if (s_lut.stage.stop.speed_rad_s < stop_speed)
                {
                    s_lut.stage.stop.speed_rad_s = stop_speed;
                }
                s_lut.ticks = 0U;
                s_lut.phase = CS_OBS_STOP_DECEL;
                break;
            }
            if (++s_lut.stage_ticks >=
                (uint32_t)(SENSORLESS_ENCODER_CALIB_VERIFY_TIMEOUT_S / FOC_TS_S))
            {
                return fail_lut(drive, MOTOR_FAULT_SENSORLESS);
            }
        }
        break;
    }

    case CS_OBS_BUILD_LUT:
    {
        EncoderCalibrationBuildResult build_result =
            build_lut_step(s_lut.workspace,
                           &s_lut.stage.build,
                           SENSORLESS_ENCODER_CALIB_LUT_BUILD_BINS_PER_CYCLE,
                           SENSORLESS_ENCODER_CALIB_MIN_SAMPLES_PER_BIN);
        if (build_result == ENCODER_CALIBRATION_BUILD_INVALID)
        {
            return fail_lut(drive, MOTOR_FAULT_ENCODER);
        }
        if (build_result == ENCODER_CALIBRATION_BUILD_COMPLETE)
        {
            s_lut.stage.verify = (EncoderCalibrationVerifyState){0};
            s_lut.stage.verify.window.previous_observer_position_rad = observer_position;
            s_lut.stage_ticks = 0U;
            s_lut.phase = CS_OBS_VERIFY_CW;
        }
        break;
    }

    case CS_OBS_STOP_DECEL:
    {
        float stop_speed = lut_stop_speed(configuration);
        float observer_mech_vel =
            FluxObserver_GetElecVel(flux_observer) / (float)configuration->model.motor_pole_pairs;
        bool speed_reached =
            Math_Abs(observer_mech_vel) <=
            stop_speed * (1.0f + SENSORLESS_ENCODER_CALIB_STOP_SPEED_TOLERANCE_RATIO);

        s_lut.ticks++;
        if (s_lut.ticks >= (uint32_t)(SENSORLESS_ENCODER_CALIB_STOP_DECEL_TIME_S / FOC_TS_S) &&
            speed_reached)
        {
            s_lut.stage.stop.current_a = Math_Min(motor_state->iq_target_a, 0.0f);
            s_lut.ticks = 0U;
            s_lut.phase = CS_OBS_STOP_CURRENT;
        }
        else if (s_lut.ticks >=
                 (uint32_t)(SENSORLESS_ENCODER_CALIB_STOP_DECEL_TIMEOUT_S / FOC_TS_S))
        {
            return fail_lut(drive, MOTOR_FAULT_SENSORLESS);
        }
        break;
    }

    case CS_OBS_STOP_CURRENT:
        if (++s_lut.ticks >=
            (uint32_t)(SENSORLESS_ENCODER_CALIB_STOP_CURRENT_RAMP_TIME_S / FOC_TS_S))
        {
            return finish_lut(drive);
        }
        break;

    default:
        return fail_lut(drive, MOTOR_FAULT_ENCODER);
    }
    return (MotorTaskResult){MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};
}

MotorTaskResult EncoderCalibrationTask_StepLut(const EncoderCalibrationControl *drive,
                                               EncoderCalibrationWorkspace *workspace)
{
    const MotorConfiguration *configuration = drive->configuration;
    const EncoderState *encoder = drive->encoder;
    if (s_lut.phase == CS_LUT_PUBLISH_PENDING)
    {
        return (MotorTaskResult){MOTOR_TASK_FINALIZE, MOTOR_FAULT_NONE};
    }

    if (s_lut.workspace != NULL && workspace != s_lut.workspace)
    {
        return fail_lut(drive, MOTOR_FAULT_ENCODER);
    }

    if (!Encoder_IsOnline(encoder))
    {
        return fail_lut(drive, MOTOR_FAULT_ENCODER);
    }
    if (configuration->model.motor_pole_pairs <= 0)
    {
        return fail_lut(drive, MOTOR_FAULT_POLE_PAIRS);
    }
    if (configuration->model.motor_phase_resistance_ohm <= 0.0f ||
        configuration->model.motor_d_inductance_h <= 0.0f ||
        configuration->model.motor_q_inductance_h <= 0.0f ||
        configuration->model.motor_flux_wb <= 0.0f ||
        configuration->application.current_limit_a <= 0.0f ||
        configuration->application.current_limit_a < SENSORLESS_ENCODER_CALIB_MIN_CURRENT_LIMIT_A)
    {
        return fail_lut(drive, MOTOR_FAULT_INVALID_PARAMETERS);
    }

    if (s_lut.phase == CS_NULL)
    {
        if (workspace == NULL)
        {
            return fail_lut(drive, MOTOR_FAULT_ENCODER);
        }
        s_lut = (EncoderLutSession){0};
        reset_lut_controllers(drive);
        s_lut.workspace = workspace;
        s_lut.phase = CS_OBS_ALIGN_ORIGIN;
        /* IDLE已预载中性PWM；首拍只建立会话，避免与首拍电流计算叠加超时。
         * 下一拍才开始原对齐斜坡，其采样、保持及超时计数不提前推进。 */
        return (MotorTaskResult){MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};
    }

    MotorTaskResult result = run_lut_control(drive);
    return result.result == MOTOR_TASK_RUNNING ? advance_lut(drive) : result;
}

MotorTaskResult EncoderCalibrationTask_StepElectricalZero(const EncoderCalibrationControl *drive)
{
    const MotorConfiguration *configuration = drive->configuration;
    EncoderCalibration *calibration = &drive->configuration->device.encoder;
    FocState *foc = drive->foc;
    MotorControlState *motor_state = drive->motor;
    EncoderState *encoder = drive->encoder;
    float time = (float)s_zero.ticks * FOC_TS_S;
    float align_current = configuration->model.calibration_current_a;

    if (align_current < ENCODER_ELEC_ZERO_MIN_ALIGN_CURRENT_A)
    {
        align_current = ENCODER_ELEC_ZERO_MIN_ALIGN_CURRENT_A;
    }

    MotorFaultCode error = MOTOR_FAULT_NONE;
    if (!Encoder_IsOnline(encoder))
    {
        s_zero = (EncoderZeroSession){0};
        error = MOTOR_FAULT_ENCODER;
    }
    else if ((calibration->calib_flag & ENC_CALIB_LINEARIZED) == 0U)
    {
        error = MOTOR_FAULT_ENCODER_NOT_CALIBRATED;
    }
    else if (align_current <= 0.0f)
    {
        error = MOTOR_FAULT_INVALID_PARAMETERS;
    }
    if (error != MOTOR_FAULT_NONE)
    {
        MotorFault_Report(motor_state, error);
        return (MotorTaskResult){MOTOR_TASK_FAILED, error};
    }
    if (configuration->application.current_limit_a > 0.0f &&
        align_current > configuration->application.current_limit_a)
    {
        align_current = configuration->application.current_limit_a;
    }

    if (s_zero.ticks == 0U)
    {
        s_zero.sample_count = 0U;
        s_zero.anchor_q15 = encoder->linearized_q15;
        s_zero.sum_q15 = 0;
        calibration->calib_flag &= (uint8_t)~ENC_CALIB_ELECTRICAL_ZERO;
        Foc_ResetCurrent(foc);
    }

    motor_state->id_target_a =
        align_current * Math_Constrain(((float)s_zero.ticks + 1.0f) * FOC_TS_S /
                                           ENCODER_ELEC_ZERO_CURRENT_RAMP_TIME_S,
                                       0.0f,
                                       1.0f);
    motor_state->iq_target_a = 0.0f;
    Foc_RunCurrent(configuration, foc, motor_state, 0.0f, 0.0f);

    if (time >= (ENC_ZERO_ALIGN_TIME - ENCODER_ELEC_ZERO_HOLD_TIME_S) && Encoder_IsOnline(encoder))
    {
        add_angle_sample(
            encoder->linearized_q15, s_zero.anchor_q15, &s_zero.sum_q15, &s_zero.sample_count);
    }

    if (time >= ENC_ZERO_ALIGN_TIME)
    {
        bool calibrated = false;
        uint16_t electrical_zero_q15;
        if (mean_q15(s_zero.sum_q15, s_zero.sample_count, &electrical_zero_q15))
        {
            calibrated = Encoder_SetElectricalZeroQ15(calibration, encoder, electrical_zero_q15);
        }

        motor_state->id_target_a = 0.0f;
        motor_state->iq_target_a = 0.0f;
        Foc_ResetCurrent(foc);
        s_zero = (EncoderZeroSession){0};
        if (!calibrated)
        {
            MotorFault_Report(motor_state, MOTOR_FAULT_ENCODER);
            return (MotorTaskResult){MOTOR_TASK_FAILED, MOTOR_FAULT_ENCODER};
        }
        return (MotorTaskResult){MOTOR_TASK_COMPLETED, MOTOR_FAULT_NONE};
    }

    s_zero.ticks++;
    return (MotorTaskResult){MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};
}
