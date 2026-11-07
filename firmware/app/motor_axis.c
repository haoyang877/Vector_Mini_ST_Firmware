#include "motor_axis.h"
#include "application_config.h"
#include "bsp_encoder.h"
#include "bsp_adc.h"
#include "bsp_pwm.h"
#include "bsp_system.h"
#include "encoder_calibration.h"
#include "motor_sensing.h"
#include "bsp_flash.h"
#include "parameter_record.h"

#include "encoder_feedback.h"
#include "foc.h"
#include "motor_fault.h"
#include "pi_controller.h"
#include "motor_control.h"
#include "motor_lifecycle.h"
#include "flux_observer.h"
#include "sensorless_startup.h"
#include "linear_hall.h"
#include <math.h>

/* 当前产品各有一组板级 PWM 和一个参数区；模拟轴可显式绑定独立端点。 */
static const MotorAxisHardware s_board_hardware = {
    .power = {BspPwm_Enable, BspPwm_Disable, BspPwm_ArePhasesDisabled, BspPwm_SetDuty},
    .read_record = BspFlash_Read,
    .write_record = BspFlash_Write,
};

void MotorAxis_Init(MotorAxis *axis)
{
    if (axis->hardware == NULL)
    {
        axis->hardware = &s_board_hardware;
    }
    axis->foc.set_duty = axis->hardware->power.set_duty;
    axis->calibration.write_record = axis->hardware->write_record;
    axis->foc.bus_voltage_filter_alpha = 0.05f;
    axis->foc.bus_current_filter_alpha = 0.05f;
    /* 固定启动暂存区与停机保存共用；装载失败沿用默认值。 */
    if (axis->hardware->read_record(&axis->record, sizeof(axis->record)) == BSP_OK)
    {
        Param_Download(&axis->record.motor, sizeof(axis->record.motor), &axis->configuration);
    }
    else
    {
        Param_Return_Default(&axis->configuration);
    }
#if BSP_ENABLED_LINEAR_HALL
    LinearHall_Init(&axis->linear_hall);
    if (axis->configuration.configuration_source == MOTOR_CONFIGURATION_STORED &&
        MotorAxisPreset_HasLinearHall(axis->configuration.preset))
    {
        Param_RestoreLinearHall(&axis->record.motor, &axis->linear_hall);
    }
#endif
    float ratio = axis->configuration.preset != NULL
                      ? axis->configuration.preset->electrical->gear_ratio
                      : 1.0f;
    OutputEncoder_Init(&axis->output_encoder, ratio);
    /* 直驱旧记录可直接沿用已有电机机械零位；减速轴须重新标定对应关系。 */
    if (ratio == 1.0f &&
        (axis->configuration.device.encoder.calib_flag & ENC_CALIB_MECHANICAL_ZERO) != 0U)
    {
        axis->output_encoder.params.zero_rad =
            (float)axis->configuration.device.encoder.mechanical_zero_q15 *
            (6.283185307f / 65536.0f);
        axis->output_encoder.params.status |= OUTPUT_ENCODER_ZERO_SET;
    }
    if (axis->configuration.configuration_source == MOTOR_CONFIGURATION_STORED)
    {
        (void)Param_RestoreOutputEncoder(&axis->record, &axis->output_encoder, &axis->linear_hall);
    }
    MotorCurrentOffsets_Init(
        &axis->current_offsets, &axis->configuration, &axis->foc, BSP_ENABLED_BUS_CURRENT != 0);
    axis->current_offsets.phases_disabled = axis->hardware->power.phases_disabled;
    axis->calibration.drive = (EncoderCalibrationControl){&axis->configuration,
                                                          &axis->foc,
                                                          &axis->motor,
                                                          &axis->rotor_encoder,
                                                          &axis->observer,
                                                          &axis->startup};
    axis->calibration.lifecycle = &axis->lifecycle;
    axis->calibration.outer = &axis->outer;
    axis->calibration.hall = &axis->linear_hall;
    axis->calibration.output = &axis->output_encoder;
    axis->calibration.record = &axis->record;
    const MotorLifecycleBindings bindings = {.configuration = &axis->configuration,
                                             .motor = &axis->motor,
                                             .foc = &axis->foc,
                                             .speed_pi = &axis->speed_pi,
                                             .rotor_encoder = &axis->rotor_encoder,
                                             .sensorless_startup = &axis->startup,
                                             .outer = &axis->outer,
                                             .calibration = &axis->calibration.session,
                                             .temperature = &axis->temperature,
                                             .power = &axis->hardware->power};
    MotorLifecycle_Init(&axis->lifecycle, &bindings);
    Encoder_InitFeedback(&axis->configuration.device.encoder,
                         &axis->configuration.model.motor_pole_pairs,
                         &axis->configuration.application.axis_profile,
                         &axis->configuration.application.axis_profile_valid,
                         &axis->rotor_encoder);

    FluxObserver_InitConfigured(&axis->observer, &axis->configuration);
    if (axis->configuration.preset != NULL)
    {
        axis->rotor_encoder.speed_pll_hz = axis->configuration.preset->electrical->speed_pll_hz;
    }
    SensorlessStartup_Reset(&axis->startup);
    Foc_ResetCurrent(&axis->foc);
    PiController_Reset(&axis->speed_pi);
    Foc_ConfigureControllers(&axis->configuration, &axis->foc, &axis->speed_pi);
    Foc_UpdateCurrentLimits(&axis->foc);
    axis->speed_pi.output_min = -1.0f;
    axis->speed_pi.output_max = 1.0f;

    axis->configuration.application.position_error_window_rad = PARAM_APP_POSITION_ERROR_WINDOW_RAD;
    axis->motor.position_velocity_filtered_rad_s = 0.0f;

    MotorOuterLoop_RequestReset(&axis->outer);

    /* 状态机初始化已禁用控制；无轴记录时继续锁存参数故障。 */
    if (!axis->configuration.application.axis_profile_valid)
    {
        MotorFault_Report(&axis->motor, MOTOR_FAULT_INVALID_PARAMETERS);
    }
}

void MotorAxis_SupervisorTick(MotorAxis *axis)
{
#if BSP_ENABLED_MCU_TEMPERATURE
    McuTemperature_Update(&axis->temperature, &axis->foc, &axis->motor);
#endif
    MotorLifecycle_SupervisorTick(&axis->lifecycle);
#if OUTER_LOOP_DIVIDER == SUPERVISOR_DIVIDER
    MotorAxis_OuterTick(axis);
#endif
}

void MotorAxis_OuterTick(MotorAxis *axis)
{
    Encoder_UpdateSlowEstimate(&axis->rotor_encoder);
#if BSP_ENABLED_LINEAR_HALL
    if (MotorAxisPreset_HasLinearHall(axis->configuration.preset))
    {
        /* ADC 快环优先级为 0，BASEPRI 控制临界区不能保护这 8 字节快照。 */
        uint32_t mask = BspSystem_EnterCritical();
        OutputEncoderCalibSample sample = axis->hall_sample;
        BspSystem_ExitCritical(mask);
        float sin_counts = (float)sample.sin_counts;
        float cos_counts = (float)sample.cos_counts;
        if (MotorLifecycle_GetCalibrationType(&axis->lifecycle) ==
                MOTOR_CALIBRATION_ENCODER_LINEARIZATION &&
            EncoderCalibration_GetPhase(&axis->calibration.session) == CS_OBS_SAMPLE_CW)
        {
            LinearHall_CalibrationSample(&axis->calibration.hall_sweep, sin_counts, cos_counts);
            OutputEncoder_CalibSample(&axis->calibration.output_calibration,
                                      (uint16_t)sample.motor_counts,
                                      sample.sin_counts,
                                      sample.cos_counts);
        }
        LinearHall_Update(&axis->linear_hall, sin_counts, cos_counts);
    }
#endif
    /* 等待现有上电采样窗口结束，避免传感器启动初值参与首次辨圈。 */
    if (axis->current_offsets.complete && Encoder_IsOnline(&axis->rotor_encoder) &&
        (axis->configuration.device.encoder.calib_flag & ENC_CALIB_LINEARIZED) != 0U &&
        (axis->output_encoder.params.status & OUTPUT_ENCODER_CAL_OK) != 0U &&
        MotorLifecycle_GetState(&axis->lifecycle) != MOTOR_STATE_CALIBRATION)
    {
        uint16_t motor_q15 = axis->rotor_encoder.linearized_q15;
        if (OutputEncoder_IsReady(&axis->output_encoder))
        {
            OutputEncoder_Update(&axis->output_encoder, motor_q15);
        }
        else if (axis->output_encoder.params.gear_ratio == 1.0f ||
                 (axis->linear_hall.calibration_success &&
                  isfinite(axis->linear_hall.val.sin_cal) &&
                  isfinite(axis->linear_hall.val.cos_cal) &&
                  (axis->linear_hall.val.sin_cal != 0.0f || axis->linear_hall.val.cos_cal != 0.0f)))
        {
            OutputEncoder_Locate(&axis->output_encoder, motor_q15, axis->linear_hall.val.angle_rad);
        }
    }
    MotorOuterWork work;
    MotorLifecycle_BeginOuter(&axis->lifecycle, &work);
    if (work.allowed)
    {
        MotorOuterLoop_SlowTick(
            &axis->outer, &axis->configuration, &work.motor, &work.speed_pi, &axis->rotor_encoder);
    }
    MotorLifecycle_CommitOuter(&axis->lifecycle, &work);
}

void MotorAxis_OuterOverrun(MotorAxis *axis)
{
    MotorFault_Report(&axis->motor, MOTOR_FAULT_CONTROL_OVERRUN);
    axis->hardware->power.disable();
}

void MotorAxis_SampleFast(MotorAxis *axis)
{
    /* 编码器传输与采样和观测器计算重叠。 */
    BspEncoder_Begin();

    MotorVoltage_Update(&axis->foc);
    Foc_UpdateCurrentLimits(&axis->foc);
    /* 自检暂未实现；上电零偏采样在空闲且输出关闭时继续完成。 */
    if (MotorLifecycle_GetState(&axis->lifecycle) == MOTOR_STATE_IDLE)
    {
        (void)MotorCurrentOffsets_Calibrate(
            &axis->current_offsets, &axis->configuration, &axis->motor);
    }
    if (axis->current_offsets.complete)
    {
        MotorCurrent_Update(&axis->foc);
    }

    Clarke_Transform(&axis->foc.phase_current_a, &axis->foc.current_ab_a);

    /* 编码器闭环使用实测角；标定及无感控制仍更新磁链观测器。 */
    if (axis->current_offsets.complete && axis->motor.control_mode != MOTOR_CONTROL_POSITION &&
        axis->motor.control_mode != MOTOR_CONTROL_SPEED)
    {
        float modulation_to_voltage_v = axis->foc.bus_voltage_filtered_v / 1.5f;
        const AlphaBetaVector voltage_ab_v = {axis->foc.mod_ab.alpha * modulation_to_voltage_v,
                                              axis->foc.mod_ab.beta * modulation_to_voltage_v};
        FluxObserver_Update(
            &axis->configuration.model, &axis->foc.current_ab_a, &voltage_ab_v, &axis->observer);
    }

    Encoder_Update(&axis->rotor_encoder);
#if BSP_ENABLED_LINEAR_HALL
    if (MotorAxisPreset_HasLinearHall(axis->configuration.preset))
    {
        axis->hall_sample =
            (OutputEncoderCalibSample){axis->rotor_encoder.directed_q15,
                                       (uint16_t)BspAdc_ReadRawChannel(BSP_ADC_LINEAR_HALL_SIN),
                                       (uint16_t)BspAdc_ReadRawChannel(BSP_ADC_LINEAR_HALL_COS)};
    }
#endif
}

void MotorAxis_ControlFast(MotorAxis *axis)
{
    /* 采样阶段已完成编码器事务，再检查故障并处理本拍启相。 */
    MotorLifecycle_BeginFastTick(&axis->lifecycle);

    /* 捕获本拍代际；标定和正常控制互斥执行。 */
    uint32_t work_epoch = MotorLifecycle_WorkEpoch(&axis->lifecycle);
    MotorTaskResult outcome = {MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};
    if (axis->current_offsets.complete && axis->motor.fault_code == MOTOR_FAULT_NONE)
    {
        MotorState state = MotorLifecycle_GetState(&axis->lifecycle);
        if (state == MOTOR_STATE_CALIBRATION &&
            MotorLifecycle_IsCalibrationRunnable(&axis->lifecycle))
        {
            outcome = MotorCalibration_StepFast(&axis->calibration);
        }
        else if (state == MOTOR_STATE_RUNNING)
        {
            outcome = MotorControl_Step(&axis->configuration,
                                        &axis->foc,
                                        &axis->motor,
                                        &axis->rotor_encoder,
                                        &axis->observer);
        }
        else
        {
            axis->hardware->power.set_duty(0.5f, 0.5f, 0.5f);
        }
    }

    /* 所有分支统一发布，再关断本拍新产生的故障。 */
    MotorLifecycle_PostOutcome(&axis->lifecycle, outcome, work_epoch);
}

void MotorAxis_ProcessPositionPlan(MotorAxis *axis)
{
    MotorPositionPlan *work = &axis->position_work;
    uint32_t mask = BspSystem_EnterControlCritical();
    bool pending = MotorOuterLoop_TakePositionPlan(&axis->outer, work);
    BspSystem_ExitControlCritical(mask);
    if (pending)
    {
        bool valid = MotorOuterLoop_BuildPositionPlan(work);
        mask = BspSystem_EnterControlCritical();
        MotorOuterLoop_PublishPositionPlan(&axis->outer, work, valid);
        BspSystem_ExitControlCritical(mask);
    }
}

uint8_t MotorAxis_NodeId(const MotorAxis *axis)
{
    return axis->configuration.device.node_id;
}
