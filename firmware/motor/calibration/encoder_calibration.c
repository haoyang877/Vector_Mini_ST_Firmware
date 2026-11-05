#include "encoder_calibration.h"

#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "application_config.h"
#include "motor_fault.h"
#include "sensorless_startup.h"
#include "control_math.h"

/** 启动阶段统一管理清表、总超时和连续稳速时间。 */
typedef struct
{
    uint16_t clear_bin;
    uint32_t ticks;
    uint32_t stable_ticks;
} EncoderCalibrationStartupState;

/** 单帧采样输入；stable 只决定是否累计本帧角差与行程。 */
typedef struct
{
    float elec_pos_rad;
    uint16_t directed_q15;
    bool stable;
} EncoderCalibrationSample;

/** 采样参考、单位换算与有效行程由同一阶段持有。 */
typedef struct
{
    float start_elec_rad;
    int32_t motor_travel_q15;
    float q15_per_elec_rad;
    uint16_t start_q15;
    uint16_t previous_q15;
    uint32_t ticks;
    uint32_t position_epoch;
} EncoderCalibrationSamplingState;

/** 两遍构表的游标及环形展开状态。 */
typedef struct
{
    uint16_t bin;
    uint8_t pass;
    int32_t previous_q15;
    int32_t shift_q15;
    int64_t sum_q15;
    uint32_t position_epoch;
} EncoderCalibrationBuildState;

/** 环形角均值以首个样本展开，避免跨零采样落到半圈。 */
typedef struct
{
    int64_t sum_q15;
    uint32_t count;
    uint16_t anchor_q15;
} EncoderCalibrationAngleMean;

/** 停止阶段的速度与电流斜坡起点。 */
typedef struct
{
    uint32_t ticks;
    float speed_rad_s;
    float current_a;
} EncoderCalibrationStopState;

/** 只表示计算进度或数据不合格；任务决定故障、阶段转换及候选应用。 */
typedef enum
{
    ENCODER_CALIBRATION_BUILD_RUNNING = 0,
    ENCODER_CALIBRATION_BUILD_COMPLETE,
    ENCODER_CALIBRATION_BUILD_INVALID
} EncoderCalibrationBuildResult;

/* 任务持有采样与阶段状态；生命周期负责功率许可、关相和持久化调度。 */
#define ENCODER_CALIBRATION_PREPARE_BINS_PER_TICK 4U
#define ENCODER_ELEC_ZERO_CURRENT_RAMP_TIME_S 0.50f
#define ENCODER_ELEC_ZERO_HOLD_TIME_S 1.00f
#define ENC_ZERO_ALIGN_TIME (ENCODER_ELEC_ZERO_CURRENT_RAMP_TIME_S + ENCODER_ELEC_ZERO_HOLD_TIME_S)

/* 会话状态唯一归任务所有；借用的控制对象只在当前调用有效。 */
typedef struct
{
    EncoderCalibrationWorkspace *workspace;
    EncoderCalibrationPhase phase;
    /* 各阶段独占业务状态；转换时先消费旧值，再初始化下一阶段。 */
    union
    {
        EncoderCalibrationStartupState startup;
        EncoderCalibrationSamplingState sampling;
        EncoderCalibrationBuildState build;
        EncoderCalibrationStopState stop;
    } stage;
} EncoderLutSession;

typedef struct
{
    uint32_t ticks;
    EncoderCalibrationAngleMean mean;
} EncoderZeroSession;

static EncoderLutSession s_lut;
static EncoderZeroSession s_zero;

static int16_t delta_q15(uint16_t target_q15, uint16_t source_q15)
{
    return (int16_t)(uint16_t)(target_q15 - source_q15);
}

static uint16_t apply_lut(const int16_t *candidate, uint16_t directed_q15)
{
    uint16_t bin = directed_q15 >> 6;
    uint16_t fraction = directed_q15 & 0x003FU;
    int32_t left_q15 = candidate[bin];
    int32_t right_q15 = candidate[(bin + 1U) & (ENCODER_OFFSET_LUT_SIZE - 1U)];
    int32_t offset_q15 = left_q15 + (((right_q15 - left_q15) * fraction) >> 6);

    return (uint16_t)((int32_t)directed_q15 - offset_q15);
}

static void add_angle(EncoderCalibrationAngleMean *mean, uint16_t angle_q15)
{
    if (mean->count == 0U)
    {
        mean->anchor_q15 = angle_q15;
    }
    mean->sum_q15 += (int32_t)mean->anchor_q15 + delta_q15(angle_q15, mean->anchor_q15);
    mean->count++;
}

/* 保持期至少采样一次后调用。 */
static uint16_t mean_q15(const EncoderCalibrationAngleMean *mean)
{
    return (uint16_t)(mean->sum_q15 / (int64_t)mean->count);
}

static void sample_lut(EncoderCalibrationWorkspace *workspace,
                       EncoderCalibrationSamplingState *sampling,
                       const EncoderCalibrationSample *sample)
{
    float from_start_rad = sample->elec_pos_rad - sampling->start_elec_rad;
    /* 行程按真实电机轴的净位移计数；失稳帧不参与构表，但不能因此额外转圈。 */
    sampling->motor_travel_q15 += delta_q15(sample->directed_q15, sampling->previous_q15);
    sampling->previous_q15 = sample->directed_q15;
    if (!sample->stable || !(from_start_rad >= 0.0f))
    {
        return;
    }

    uint16_t bin = sample->directed_q15 >> (16U - ENCODER_OFFSET_LUT_BITS);
    uint16_t ref_q15 =
        (uint16_t)(sampling->start_q15 + (uint32_t)(from_start_rad * sampling->q15_per_elec_rad));
    int32_t correction_q15 = delta_q15(sample->directed_q15, ref_q15);
    if (workspace->bins.samples[bin] != 0U)
    {
        int32_t average_q15 = workspace->error_sum[bin] / (int32_t)workspace->bins.samples[bin];
        while (correction_q15 - average_q15 > ENCODER_Q15_HALF_TURN)
        {
            correction_q15 -= (int32_t)ENCODER_Q15_CPR;
        }
        while (correction_q15 - average_q15 < -ENCODER_Q15_HALF_TURN)
        {
            correction_q15 += (int32_t)ENCODER_Q15_CPR;
        }
    }
    if (workspace->bins.samples[bin] < UINT16_MAX)
    {
        workspace->error_sum[bin] += correction_q15;
        workspace->bins.samples[bin]++;
    }
}

/* 先逐 bin 求均值并展开，再统一平移写表；samples 与 candidate 按遍复用。 */
static EncoderCalibrationBuildResult build_bins(EncoderCalibrationWorkspace *workspace,
                                                EncoderCalibrationBuildState *build)
{
#if defined(__clang__)
#pragma clang loop unroll(disable)
#endif
    for (uint16_t count = 0U; count < SENSORLESS_ENCODER_CALIB_LUT_BUILD_BINS_PER_CYCLE &&
                              build->bin < ENCODER_OFFSET_LUT_SIZE;
         ++count, ++build->bin)
    {
        if (build->pass == 0U)
        {
            uint16_t samples = workspace->bins.samples[build->bin];
            if (samples == 0U)
            {
                return ENCODER_CALIBRATION_BUILD_INVALID;
            }
            int32_t correction_q15 = (int16_t)(workspace->error_sum[build->bin] / (int32_t)samples);
            if (build->bin > 0U)
            {
                while (correction_q15 - build->previous_q15 > ENCODER_Q15_HALF_TURN)
                {
                    correction_q15 -= (int32_t)ENCODER_Q15_CPR;
                }
                while (correction_q15 - build->previous_q15 < -ENCODER_Q15_HALF_TURN)
                {
                    correction_q15 += (int32_t)ENCODER_Q15_CPR;
                }
            }
            workspace->error_sum[build->bin] = correction_q15;
            build->previous_q15 = correction_q15;
            build->sum_q15 += correction_q15;
        }
        else
        {
            int32_t correction_q15 = workspace->error_sum[build->bin] - build->shift_q15;
            if (correction_q15 < INT16_MIN || correction_q15 > INT16_MAX)
            {
                return ENCODER_CALIBRATION_BUILD_INVALID;
            }
            workspace->bins.candidate[build->bin] = (int16_t)correction_q15;
        }
    }
    if (build->bin < ENCODER_OFFSET_LUT_SIZE)
    {
        return ENCODER_CALIBRATION_BUILD_RUNNING;
    }
    if (build->pass == 1U)
    {
        return ENCODER_CALIBRATION_BUILD_COMPLETE;
    }

    int32_t average_q15 = (int32_t)(build->sum_q15 / ENCODER_OFFSET_LUT_SIZE);
    build->shift_q15 = average_q15 - (int16_t)(uint16_t)average_q15;
    build->bin = 0U;
    build->pass = 1U;
    return ENCODER_CALIBRATION_BUILD_RUNNING;
}

void EncoderCalibration_Cancel(void)
{
    s_lut = (EncoderLutSession){0};
    s_zero = (EncoderZeroSession){0};
}

bool EncoderCalibration_IsIdle(void)
{
    return s_lut.phase == CS_NULL && s_zero.ticks == 0U;
}

EncoderCalibrationPhase EncoderCalibration_GetPhase(void)
{
    return s_lut.phase;
}

/* 结束运动命令，功率输出由生命周期关断。 */
static void reset_drive(const EncoderCalibrationControl *drive)
{
    SensorlessStartup_Reset(drive->startup);
    Foc_ResetCurrent(drive->foc);
    PiController_Reset(drive->speed_pi);
    drive->motor->speed_target_rad_s = 0.0f;
    drive->motor->speed_ramped_rad_s = 0.0f;
    drive->motor->id_target_a = 0.0f;
    drive->motor->iq_target_a = 0.0f;
}

static MotorTaskResult fail_lut(const EncoderCalibrationControl *drive, MotorFaultCode error)
{
    MotorFault_Report(drive->motor, error);
    reset_drive(drive);
    EncoderCalibration_Cancel();
    return (MotorTaskResult){MOTOR_TASK_FAILED, error};
}

static MotorTaskResult finish_lut(const EncoderCalibrationControl *drive)
{
    reset_drive(drive);
    s_lut.phase = CS_LUT_PUBLISH_PENDING;
    return (MotorTaskResult){MOTOR_TASK_FINALIZE, MOTOR_FAULT_NONE};
}

bool EncoderCalibration_PublishLut(EncoderCalibration *calibration, EncoderState *encoder)
{
    if (s_lut.phase != CS_LUT_PUBLISH_PENDING)
    {
        return false;
    }
    /* 调用方持有全中断临界区，反馈在整张表发布后恢复。 */
    memcpy(calibration->linearization_lut_q15,
           s_lut.workspace->bins.candidate,
           sizeof(calibration->linearization_lut_q15));
    encoder->linearized_q15 = apply_lut(s_lut.workspace->bins.candidate, encoder->directed_q15);
    calibration->electrical_zero_q15 = 0U;
    calibration->mechanical_zero_q15 = 0U;
    calibration->calib_flag = ENC_CALIB_LINEARIZED;
    /* 慢状态由 2 kHz 慢估计独占：请求重定多圈基准（基准取当前线性化角、零影子归零）。 */
    encoder->rebase_requested = true;
    Encoder_ResetVelocity(encoder);
    EncoderCalibration_Cancel();
    return true;
}

static const SensorlessStartupConfig *calibration_startup(const MotorConfiguration *config)
{
    return config->preset != NULL ? config->preset->startup : &SensorlessStartup_EncoderCalibConfig;
}

static float calibration_speed(const MotorConfiguration *config)
{
    return config->preset != NULL ? config->preset->calibration_speed_rad_s
                                  : SENSORLESS_ENCODER_CALIB_SPEED_MEC_RAD_S;
}

static bool speed_is_stable(const EncoderCalibrationControl *drive)
{
    float tolerance_rad_s =
        calibration_speed(drive->configuration) * SENSORLESS_ENCODER_CALIB_SPEED_ERROR_RATIO;
    /* 复位后的滤波速度可能暂存旧值，原始速度须已恢复到观测器可用范围。 */
    return FluxObserver_GetElecVel(drive->observer) >=
               calibration_startup(drive->configuration)->minimum_electrical_velocity_rad_s &&
           Math_Abs(drive->startup->speed_feedback_rad_s -
                    calibration_speed(drive->configuration)) <= tolerance_rad_s &&
           Math_Abs(drive->motor->speed_ramped_rad_s - calibration_speed(drive->configuration)) <=
               tolerance_rad_s;
}

static bool tracks_epoch(const EncoderCalibrationControl *drive, uint32_t position_epoch)
{
    return drive->startup->state == SENSORLESS_STARTUP_CLOSED_LOOP &&
           FluxObserver_GetPositionEpoch(drive->observer) == position_epoch;
}

static float get_stop_speed(const MotorConfiguration *config)
{
    float min_mech_rad_s = calibration_startup(config)->minimum_electrical_velocity_rad_s /
                           (float)config->model.motor_pole_pairs;
    return Math_Min(min_mech_rad_s * SENSORLESS_ENCODER_CALIB_STOP_SPEED_MARGIN,
                    calibration_speed(config));
}

static bool can_brake(const EncoderCalibrationControl *drive)
{
    float min_elec_rad_s =
        calibration_startup(drive->configuration)->minimum_electrical_velocity_rad_s;
    float elec_vel_rad_s = FluxObserver_GetElecVel(drive->observer);
    float phase_rad = FluxObserver_GetElecPhase(drive->observer);
    float mech_vel_rad_s = Encoder_GetMechVelContinuous(drive->encoder);
    float pole_pairs = (float)drive->configuration->model.motor_pole_pairs;

    if (drive->startup->state != SENSORLESS_STARTUP_CLOSED_LOOP ||
        !(elec_vel_rad_s > min_elec_rad_s) ||
        elec_vel_rad_s > SENSORLESS_OBSERVER_MAX_ELEC_VEL_RAD_S ||
        !(phase_rad >= 0.0f && phase_rad <= _2PI))
    {
        return false;
    }
    /* 编码器速度独立截止低速制动，卸流阶段也执行同一条件。 */
    return !drive->encoder->velocity_ready ||
           (mech_vel_rad_s > min_elec_rad_s / pole_pairs &&
            mech_vel_rad_s <= SENSORLESS_OBSERVER_MAX_ELEC_VEL_RAD_S / pole_pairs);
}

static MotorTaskResult run_sensorless(const EncoderCalibrationControl *drive)
{
    SensorlessStartup_Run(drive->configuration,
                          drive->foc,
                          drive->motor,
                          drive->speed_pi,
                          drive->observer,
                          drive->startup,
                          calibration_startup(drive->configuration));
    if (drive->motor->fault_code != MOTOR_FAULT_NONE)
    {
        reset_drive(drive);
        EncoderCalibration_Cancel();
        return (MotorTaskResult){MOTOR_TASK_FAILED, (MotorFaultCode)drive->motor->fault_code};
    }
    return (MotorTaskResult){MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};
}

/* 采集业务统一运行固定转速，各阶段只消费自己的状态。 */
static MotorTaskResult step_measure(const EncoderCalibrationControl *drive)
{
    drive->startup->speed_pi_output_max = 1.0f;
    drive->motor->speed_target_rad_s = calibration_speed(drive->configuration);
    MotorTaskResult result = run_sensorless(drive);
    if (result.result != MOTOR_TASK_RUNNING)
    {
        return result;
    }

    switch (s_lut.phase)
    {
    case CS_OBS_STARTUP:
    {
        EncoderCalibrationStartupState *startup = &s_lut.stage.startup;
        for (uint16_t count = 0U; count < ENCODER_CALIBRATION_PREPARE_BINS_PER_TICK &&
                                  startup->clear_bin < ENCODER_OFFSET_LUT_SIZE;
             ++count, ++startup->clear_bin)
        {
            s_lut.workspace->error_sum[startup->clear_bin] = 0;
            s_lut.workspace->bins.samples[startup->clear_bin] = 0U;
        }
        if (startup->clear_bin == ENCODER_OFFSET_LUT_SIZE &&
            drive->startup->state == SENSORLESS_STARTUP_CLOSED_LOOP && speed_is_stable(drive))
        {
            if (++startup->stable_ticks >=
                (uint32_t)(SENSORLESS_ENCODER_CALIB_SPEED_STABLE_TIME_S / FOC_TS_S))
            {
                float elec_pos_rad = FluxObserver_GetElecPos(drive->observer);
                s_lut.stage.sampling = (EncoderCalibrationSamplingState){
                    .start_elec_rad = elec_pos_rad,
                    .q15_per_elec_rad =
                        (float)ENCODER_Q15_CPR /
                        (_2PI * (float)drive->configuration->model.motor_pole_pairs),
                    .start_q15 = drive->encoder->directed_q15,
                    .previous_q15 = drive->encoder->directed_q15,
                    .position_epoch = FluxObserver_GetPositionEpoch(drive->observer),
                };
                s_lut.phase = CS_OBS_SAMPLE_CW;
                break;
            }
        }
        else
        {
            startup->stable_ticks = 0U;
        }
        if (++startup->ticks >= (uint32_t)(SENSORLESS_ENCODER_CALIB_STARTUP_TIMEOUT_S / FOC_TS_S))
        {
            return fail_lut(drive, MOTOR_FAULT_SENSORLESS);
        }
        break;
    }
    case CS_OBS_SAMPLE_CW:
    {
        EncoderCalibrationSamplingState *sampling = &s_lut.stage.sampling;
        if (!tracks_epoch(drive, sampling->position_epoch))
        {
            return fail_lut(drive, MOTOR_FAULT_SENSORLESS);
        }
        EncoderCalibrationSample sample = {
            .elec_pos_rad = FluxObserver_GetElecPos(drive->observer),
            .directed_q15 = drive->encoder->directed_q15,
            .stable = speed_is_stable(drive),
        };
        sample_lut(s_lut.workspace, sampling, &sample);
        if (sampling->motor_travel_q15 < -(int32_t)ENCODER_Q15_CPR)
        {
            return fail_lut(drive, MOTOR_FAULT_ENCODER);
        }
        if (sampling->motor_travel_q15 >=
            (int32_t)(SENSORLESS_ENCODER_CALIB_MECH_TURNS * ENCODER_Q15_CPR))
        {
            /* 整圈处的观测器参考应回到相同电角相位，丢失电周期的参考不能写表。 */
            float expected_rad = (float)SENSORLESS_ENCODER_CALIB_MECH_TURNS *
                                 (_2PI * (float)drive->configuration->model.motor_pole_pairs);
            if (!(Math_Abs(sample.elec_pos_rad - sampling->start_elec_rad - expected_rad) <= _PI))
            {
                return fail_lut(drive, MOTOR_FAULT_SENSORLESS);
            }
            uint32_t position_epoch = sampling->position_epoch;
            s_lut.stage.build = (EncoderCalibrationBuildState){.position_epoch = position_epoch};
            s_lut.phase = CS_OBS_BUILD_LUT;
        }
        else if (++sampling->ticks >=
                 (uint32_t)(SENSORLESS_ENCODER_CALIB_SAMPLE_TIMEOUT_S / FOC_TS_S))
        {
            return fail_lut(drive, MOTOR_FAULT_SENSORLESS);
        }
        break;
    }
    case CS_OBS_BUILD_LUT:
    {
        EncoderCalibrationBuildState *build = &s_lut.stage.build;
        if (!tracks_epoch(drive, build->position_epoch))
        {
            return fail_lut(drive, MOTOR_FAULT_SENSORLESS);
        }
        EncoderCalibrationBuildResult build_result = build_bins(s_lut.workspace, build);
        if (build_result == ENCODER_CALIBRATION_BUILD_INVALID)
        {
            return fail_lut(drive, MOTOR_FAULT_ENCODER);
        }
        if (build_result == ENCODER_CALIBRATION_BUILD_COMPLETE)
        {
            s_lut.stage.stop = (EncoderCalibrationStopState){
                .speed_rad_s = drive->motor->speed_ramped_rad_s,
            };
            float stop_rad_s = get_stop_speed(drive->configuration);
            if (s_lut.stage.stop.speed_rad_s < stop_rad_s)
            {
                s_lut.stage.stop.speed_rad_s = stop_rad_s;
            }
            s_lut.phase = CS_OBS_STOP_DECEL;
        }
        break;
    }
    default:
        return fail_lut(drive, MOTOR_FAULT_ENCODER);
    }
    return (MotorTaskResult){MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};
}

/* 停止业务在同一处完成低速截止、控制输出和阶段推进。 */
static MotorTaskResult step_stop(const EncoderCalibrationControl *drive)
{
    EncoderCalibrationStopState *stop = &s_lut.stage.stop;
    MotorControlState *motor = drive->motor;
    if (!can_brake(drive))
    {
        return finish_lut(drive);
    }

    if (s_lut.phase == CS_OBS_STOP_DECEL)
    {
        float stop_rad_s = get_stop_speed(drive->configuration);
        float ratio = Math_Constrain(
            (float)stop->ticks * FOC_TS_S / SENSORLESS_ENCODER_CALIB_STOP_DECEL_TIME_S, 0.0f, 1.0f);
        drive->startup->speed_pi_output_max = 0.0f;
        motor->speed_target_rad_s = stop->speed_rad_s + (stop_rad_s - stop->speed_rad_s) * ratio;
        motor->speed_ramped_rad_s = motor->speed_target_rad_s;
        MotorTaskResult result = run_sensorless(drive);
        if (result.result != MOTOR_TASK_RUNNING)
        {
            return result;
        }
        float mech_vel_rad_s = FluxObserver_GetElecVel(drive->observer) /
                               (float)drive->configuration->model.motor_pole_pairs;
        ++stop->ticks;
        if (stop->ticks >= (uint32_t)(SENSORLESS_ENCODER_CALIB_STOP_DECEL_TIME_S / FOC_TS_S) &&
            Math_Abs(mech_vel_rad_s) <=
                stop_rad_s * (1.0f + SENSORLESS_ENCODER_CALIB_STOP_SPEED_TOLERANCE_RATIO))
        {
            stop->current_a = Math_Min(motor->iq_target_a, 0.0f);
            stop->ticks = 0U;
            s_lut.phase = CS_OBS_STOP_CURRENT;
        }
        else if (stop->ticks >=
                 (uint32_t)(SENSORLESS_ENCODER_CALIB_STOP_DECEL_TIMEOUT_S / FOC_TS_S))
        {
            return fail_lut(drive, MOTOR_FAULT_SENSORLESS);
        }
    }
    else
    {
        float ratio = Math_Constrain((float)(++stop->ticks) * FOC_TS_S /
                                         SENSORLESS_ENCODER_CALIB_STOP_CURRENT_RAMP_TIME_S,
                                     0.0f,
                                     1.0f);
        motor->id_target_a = 0.0f;
        motor->iq_target_a = stop->current_a * (1.0f - ratio);
        Foc_RunCurrent(drive->configuration,
                       drive->foc,
                       motor,
                       FluxObserver_GetElecPhase(drive->observer),
                       FluxObserver_GetElecVel(drive->observer));
        if (stop->ticks >= (uint32_t)(SENSORLESS_ENCODER_CALIB_STOP_CURRENT_RAMP_TIME_S / FOC_TS_S))
        {
            return finish_lut(drive);
        }
    }
    return (MotorTaskResult){MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};
}

MotorTaskResult EncoderCalibration_StepLut(const EncoderCalibrationControl *drive,
                                           EncoderCalibrationWorkspace *workspace)
{
    switch (s_lut.phase)
    {
    case CS_NULL:
        s_lut = (EncoderLutSession){.workspace = workspace, .phase = CS_OBS_STARTUP};
        SensorlessStartup_Reset(drive->startup);
        /* 首拍仅建立会话，下一拍由无感启动统一复位 FOC 和速度 PI。 */
        return (MotorTaskResult){MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};
    case CS_LUT_PUBLISH_PENDING:
        return (MotorTaskResult){MOTOR_TASK_FINALIZE, MOTOR_FAULT_NONE};
    case CS_OBS_STOP_DECEL:
    case CS_OBS_STOP_CURRENT:
        return step_stop(drive);
    default:
        return step_measure(drive);
    }
}

MotorTaskResult EncoderCalibration_StepElecZero(const EncoderCalibrationControl *drive)
{
    const MotorConfiguration *config = drive->configuration;
    EncoderCalibration *calibration = &drive->configuration->device.encoder;
    FocState *foc = drive->foc;
    MotorControlState *motor = drive->motor;
    EncoderState *encoder = drive->encoder;
    float elapsed_s = (float)s_zero.ticks * FOC_TS_S;
    float align_current_a =
        Math_Min(config->model.calibration_current_a, config->application.current_limit_a);

    if (s_zero.ticks == 0U)
    {
        s_zero.mean = (EncoderCalibrationAngleMean){0};
        calibration->calib_flag &= (uint8_t)~ENC_CALIB_ELECTRICAL_ZERO;
        Foc_ResetCurrent(foc);
    }

    motor->id_target_a = align_current_a * Math_Constrain(((float)s_zero.ticks + 1.0f) * FOC_TS_S /
                                                              ENCODER_ELEC_ZERO_CURRENT_RAMP_TIME_S,
                                                          0.0f,
                                                          1.0f);
    motor->iq_target_a = 0.0f;
    Foc_RunCurrent(config, foc, motor, 0.0f, 0.0f);

    if (elapsed_s >= ENCODER_ELEC_ZERO_CURRENT_RAMP_TIME_S)
    {
        add_angle(&s_zero.mean, encoder->linearized_q15);
    }

    if (elapsed_s >= ENC_ZERO_ALIGN_TIME)
    {
        bool calibrated =
            Encoder_SetElectricalZeroQ15(calibration, encoder, mean_q15(&s_zero.mean));

        motor->id_target_a = 0.0f;
        motor->iq_target_a = 0.0f;
        Foc_ResetCurrent(foc);
        s_zero = (EncoderZeroSession){0};
        if (!calibrated)
        {
            MotorFault_Report(motor, MOTOR_FAULT_ENCODER);
            return (MotorTaskResult){MOTOR_TASK_FAILED, MOTOR_FAULT_ENCODER};
        }
        return (MotorTaskResult){MOTOR_TASK_COMPLETED, MOTOR_FAULT_NONE};
    }

    s_zero.ticks++;
    return (MotorTaskResult){MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};
}
