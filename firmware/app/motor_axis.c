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
#include "motor_state_machine.h"
#include "flux_observer.h"
#include "sensorless_startup.h"
#include "linear_hall.h"
#include <math.h>
#include <string.h>
#include <stddef.h>
#include "crc32.h"

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
    bool output_record_valid = false;
    if (axis->configuration.configuration_source == MOTOR_CONFIGURATION_STORED)
    {
        output_record_valid =
            Param_RestoreOutputEncoder(&axis->record, &axis->output_encoder, &axis->linear_hall);
    }
    axis->parameters.revision = 1U;
    axis->parameters.saved_revision =
        output_record_valid && axis->record.output.magic == PARAM_OUTPUT_ENCODER_V3 ? 1U : 0U;
    axis->parameters.save_status = PARAMETER_OK;
    axis->calibration.parameters = &axis->parameters;
    MotorCurrentOffsets_Init(
        &axis->current_offsets, &axis->configuration, &axis->foc, BSP_ENABLED_BUS_CURRENT != 0);
    axis->current_offsets.phases_disabled = axis->hardware->power.phases_disabled;
    axis->calibration.drive = (EncoderCalibrationControl){&axis->configuration,
                                                          &axis->foc,
                                                          &axis->motor,
                                                          &axis->rotor_encoder,
                                                          &axis->observer,
                                                          &axis->startup};
    axis->calibration.state_machine = &axis->state_machine;
    axis->calibration.outer = &axis->outer;
    axis->calibration.hall = &axis->linear_hall;
    axis->calibration.output = &axis->output_encoder;
    axis->calibration.record = &axis->record;
    const MotorStateMachineBindings bindings = {.configuration = &axis->configuration,
                                                .parameter_revision = &axis->parameters.revision,
                                                .motor = &axis->motor,
                                                .foc = &axis->foc,
                                                .speed_pi = &axis->speed_pi,
                                                .rotor_encoder = &axis->rotor_encoder,
                                                .sensorless_startup = &axis->startup,
                                                .outer = &axis->outer,
                                                .calibration = &axis->calibration.session,
                                                .temperature = &axis->temperature,
                                                .motor_temperature = &axis->motor_temperature,
                                                .output = &axis->output_feedback,
                                                .power = &axis->hardware->power};
    MotorStateMachine_Init(&axis->state_machine, &bindings);
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
#if BSP_ENABLED_MOTOR_TEMPERATURE
    if (axis->state_machine.supervisor_ticks % 20U == 0U)
    {
        float celsius;
        if (BspAdc_ReadMotorTemperature(&celsius))
        {
            axis->motor_temperature.celsius = celsius;
            axis->motor_temperature.sample_ms = BspSystem_TimeMs();
            axis->motor_temperature.valid = isfinite(celsius);
        }
    }
#endif
    MotorStateMachine_SupervisorTick(&axis->state_machine);
#if OUTER_LOOP_DIVIDER == SUPERVISOR_DIVIDER
    MotorAxis_OuterTick(axis);
#endif
}

static void update_outer_loop_encoder_feedback(MotorAxis *axis)
{
    Encoder_UpdateSlowEstimate(&axis->rotor_encoder);
#if BSP_ENABLED_LINEAR_HALL
    if (MotorAxisPreset_HasLinearHall(axis->configuration.preset))
    {
        uint8_t slot = axis->hall_published_slot;
        axis->hall_read_slot = slot;
        BspSystem_MemoryBarrier();
        OutputEncoderCalibSample sample = axis->hall_samples[slot];
        float sin_counts = (float)sample.sin_counts;
        float cos_counts = (float)sample.cos_counts;
        if (MotorStateMachine_GetCalibrationType(&axis->state_machine) ==
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
    if (!axis->current_offsets.complete || !Encoder_IsOnline(&axis->rotor_encoder) ||
        (axis->configuration.device.encoder.calib_flag & ENC_CALIB_LINEARIZED) == 0U ||
        (axis->output_encoder.params.status & OUTPUT_ENCODER_CAL_OK) == 0U ||
        axis->state_machine.state == MOTOR_STATE_CALIBRATION)
    {
        return;
    }
    uint16_t motor_q15 = axis->rotor_encoder.linearized_q15;
    if (OutputEncoder_IsReady(&axis->output_encoder))
    {
        OutputEncoder_Update(&axis->output_encoder, motor_q15);
    }
    else if (axis->output_encoder.params.gear_ratio == 1.0f ||
             (axis->linear_hall.calibration_success && isfinite(axis->linear_hall.val.sin_cal) &&
              isfinite(axis->linear_hall.val.cos_cal) &&
              (axis->linear_hall.val.sin_cal != 0.0f || axis->linear_hall.val.cos_cal != 0.0f)))
    {
        OutputEncoder_Locate(&axis->output_encoder, motor_q15, axis->linear_hall.val.angle_rad);
    }
}

void MotorAxis_OuterTick(MotorAxis *axis)
{
    update_outer_loop_encoder_feedback(axis);
    MotorOuterWork work;
    MotorStateMachine_ReadOuterInputs(&axis->state_machine, &work);
    if (work.allowed)
    {
        MotorOuterLoop_SlowTick(
            &axis->outer, &axis->configuration, &work.motor, &work.speed_pi, &axis->rotor_encoder);
    }
    uint32_t sample_counter = axis->state_machine.fast_sample_ticks;
    if (sample_counter != axis->output_feedback.sample_counter)
    {
        uint16_t flags = 0U;
#if BSP_ENABLED_LINEAR_HALL
        if (MotorAxisPreset_HasLinearHall(axis->configuration.preset))
        {
            flags |= 1U << 4;
            const LinearHallSample *hall = &axis->linear_hall.val;
            if (axis->linear_hall.calibration_success &&
                (axis->linear_hall.lut_status & LINEAR_HALL_LUT_OK) != 0U &&
                isfinite(hall->sin_counts) && isfinite(hall->cos_counts) &&
                hall->sin_counts > 0.0f && hall->sin_counts < 4095.0f && hall->cos_counts > 0.0f &&
                hall->cos_counts < 4095.0f && (hall->sin_cal != 0.0f || hall->cos_cal != 0.0f) &&
                isfinite(hall->angle_rad))
            {
                flags |= 1U;
            }
        }
#endif
        if (axis->output_encoder.ready && Encoder_IsOnline(&axis->rotor_encoder) &&
            (axis->output_encoder.params.status & OUTPUT_ENCODER_CAL_OK) != 0U &&
            (axis->configuration.device.encoder.calib_flag & ENC_CALIB_ALL) == ENC_CALIB_ALL &&
            axis->state_machine.state != MOTOR_STATE_CALIBRATION)
        {
            flags |= 1U << 1;
        }
        if ((axis->output_encoder.params.status & OUTPUT_ENCODER_ZERO_SET) != 0U)
        {
            flags |= 1U << 2;
        }
        if ((axis->output_encoder.params.status & OUTPUT_ENCODER_CAL_OK) != 0U)
        {
            flags |= 1U << 3;
        }
        float angle = fmodf(axis->linear_hall.val.angle_rad - axis->output_encoder.params.zero_rad,
                            6.283185307f);
        if (angle < 0.0f)
        {
            angle += 6.283185307f;
        }
        axis->output_feedback =
            (MotorOutputTelemetry){.hall_position_rad = angle,
                                   .tracked_position_rad = axis->output_encoder.position_rad,
                                   .zero_rad = axis->output_encoder.params.zero_rad,
                                   .gear_ratio = axis->output_encoder.params.gear_ratio,
                                   .sample_ms = BspSystem_TimeMs(),
                                   .sample_counter = sample_counter,
                                   .flags = flags};
    }
    MotorStateMachine_CommitOuterResult(&axis->state_machine, &work);
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
    if (axis->state_machine.state == MOTOR_STATE_IDLE)
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

    if (axis->calibration.lut_request_epoch != 0U)
    {
        MotorCalibration_PublishLutFast(&axis->calibration);
    }
    Encoder_Update(&axis->rotor_encoder);
#if BSP_ENABLED_LINEAR_HALL
    if (MotorAxisPreset_HasLinearHall(axis->configuration.preset))
    {
        uint8_t slot = axis->hall_read_slot ^ 1U;
        axis->hall_samples[slot] =
            (OutputEncoderCalibSample){axis->rotor_encoder.directed_q15,
                                       (uint16_t)BspAdc_ReadRawChannel(BSP_ADC_LINEAR_HALL_SIN),
                                       (uint16_t)BspAdc_ReadRawChannel(BSP_ADC_LINEAR_HALL_COS)};
        BspSystem_MemoryBarrier();
        axis->hall_published_slot = slot;
    }
#endif
}

void MotorAxis_ControlFast(MotorAxis *axis)
{
    /* 采样阶段已完成编码器事务，再检查故障并处理本拍启相。 */
    MotorStateMachine_BeginFastTick(&axis->state_machine);

    /* 捕获本拍代际；标定和正常控制互斥执行。 */
    uint32_t work_epoch = axis->state_machine.epoch;
    MotorTaskResult outcome = {MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};
    if (axis->current_offsets.complete && axis->motor.fault_code == MOTOR_FAULT_NONE)
    {
        if (axis->state_machine.state == MOTOR_STATE_RUNNING)
        {
            outcome = MotorControl_Step(&axis->configuration,
                                        &axis->foc,
                                        &axis->motor,
                                        &axis->rotor_encoder,
                                        &axis->observer);
        }
        else if (MotorStateMachine_IsCalibrationRunnable(&axis->state_machine))
        {
            outcome = MotorCalibration_StepFast(&axis->calibration);
        }
        else
        {
            axis->hardware->power.set_duty(0.5f, 0.5f, 0.5f);
        }
    }

    /* 所有分支统一发布，再关断本拍新产生的故障。 */
    MotorStateMachine_FinishFastTick(&axis->state_machine, outcome, work_epoch);
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

/* 维护仅由前台串行调用。标志使监督跳过正在修改的配置，不屏蔽ADC。 */
static bool begin_parameter_update(MotorAxis *axis)
{
    MotorStateMachine *machine = &axis->state_machine;
    if (machine->state != MOTOR_STATE_IDLE || !MotorStateMachine_IsStopped(machine) ||
        machine->starts_locked || machine->drive.motor->fault_code != MOTOR_FAULT_NONE ||
        machine->calibration_type != MOTOR_CALIBRATION_NONE)
    {
        return false;
    }
    machine->parameters_updating = true;
    BspSystem_MemoryBarrier();
    return true;
}

static void finish_parameter_update(MotorAxis *axis)
{
    Foc_ConfigureControllers(&axis->configuration, &axis->foc, &axis->speed_pi);
    Foc_ResetCurrent(&axis->foc);
    PiController_Reset(&axis->speed_pi);
    MotorOuterLoop_RequestReset(&axis->outer);
    BspSystem_MemoryBarrier();
    axis->state_machine.parameters_updating = false;
}

ParameterStatus MotorAxis_WriteParameter(MotorAxis *axis, const ParameterWriteRequest *request)
{
    if (request->parameter_id == 0x70U)
    {
        return PARAMETER_READ_ONLY;
    }
    ParameterValue current;
    ParameterStatus status = ParameterValue_Read(
        &axis->configuration, axis->parameters.revision, request->parameter_id, &current);
    if (status != PARAMETER_OK)
    {
        return status;
    }
    if ((current.access & PARAMETER_ACCESS_WRITE_IDLE) == 0U)
    {
        return PARAMETER_READ_ONLY;
    }
    if (request->expected_revision != axis->parameters.revision)
    {
        /* 只允许紧邻一次成功提交的同值重试，不推进修订号。 */
        if (request->expected_revision != UINT32_MAX &&
            request->expected_revision + 1U == axis->parameters.revision &&
            request->value_type == current.value_type && request->raw_value == current.raw_value)
        {
            return PARAMETER_OK;
        }
        return PARAMETER_REVISION_CONFLICT;
    }
    if (!begin_parameter_update(axis))
    {
        return PARAMETER_WRONG_STATE;
    }
    if (axis->parameters.revision == UINT32_MAX)
    {
        axis->state_machine.parameters_updating = false;
        return PARAMETER_OUT_OF_RANGE;
    }
    status = ParameterValue_Apply(&axis->configuration, request);
    if (status == PARAMETER_OK &&
        (request->raw_value != current.raw_value || request->value_type != current.value_type))
    {
        ++axis->parameters.revision;
    }
    finish_parameter_update(axis);
    return status;
}

ParameterStatus MotorAxis_SaveParameters(MotorAxis *axis, uint32_t expected_revision)
{
    if (!ParameterValue_ConfigurationValid(&axis->configuration) ||
        !MotorDeviceOffsets_InRange(&axis->configuration.device))
    {
        return PARAMETER_INVALID_ARGUMENT;
    }
    if (expected_revision != axis->parameters.revision)
    {
        return PARAMETER_REVISION_CONFLICT;
    }
    if (!begin_parameter_update(axis))
    {
        return PARAMETER_WRONG_STATE;
    }
    if (axis->parameters.saved_revision == axis->parameters.revision)
    {
        axis->state_machine.parameters_updating = false;
        return PARAMETER_OK;
    }
    Param_Upload(&axis->record.motor, &axis->configuration);
#if BSP_ENABLED_LINEAR_HALL
    if (MotorAxisPreset_HasLinearHall(axis->configuration.preset))
    {
        Param_StoreLinearHall(&axis->record.motor, &axis->linear_hall);
    }
#endif
    Param_StoreOutputEncoder(&axis->record, &axis->output_encoder, &axis->linear_hall);
    uint32_t mask = BspSystem_EnterCritical();
    /* Flash事务复核实际关相；STOP/故障抢占过准备阶段时不写。 */
    ParameterStatus result = PARAMETER_WRONG_STATE;
    if (MotorStateMachine_IsStopped(&axis->state_machine) &&
        axis->motor.fault_code == MOTOR_FAULT_NONE && !axis->state_machine.starts_locked)
    {
        result = axis->hardware->write_record(&axis->record, sizeof(axis->record)) == BSP_OK
                     ? PARAMETER_OK
                     : PARAMETER_PERSIST_FAILED;
        axis->parameters.save_status = result;
        axis->parameters.saved_revision = result == PARAMETER_OK ? axis->parameters.revision : 0U;
    }
    BspSystem_ExitCritical(mask);
    axis->state_machine.parameters_updating = false;
    return result;
}

ParameterStatus
MotorAxis_ReadParameter(MotorAxis *axis, uint8_t id, bool saved, ParameterValue *value)
{
    if (!saved && id == 0x70U)
    {
        if (!MotorStateMachine_IsStopped(&axis->state_machine) ||
            axis->state_machine.calibration_type != MOTOR_CALIBRATION_NONE)
        {
            return PARAMETER_WRONG_STATE;
        }
        Param_Upload(&axis->record.motor, &axis->configuration);
#if BSP_ENABLED_LINEAR_HALL
        if (MotorAxisPreset_HasLinearHall(axis->configuration.preset))
        {
            Param_StoreLinearHall(&axis->record.motor, &axis->linear_hall);
        }
#endif
        Param_StoreOutputEncoder(&axis->record, &axis->output_encoder, &axis->linear_hall);
        *value = (ParameterValue){.parameter_id = id,
                                  .value_type = PARAMETER_VALUE_U32,
                                  .access = PARAMETER_ACCESS_READ,
                                  .revision = axis->parameters.revision,
                                  .raw_value = Param_CoordinateCrc(&axis->record)};
        return PARAMETER_OK;
    }
    if (!saved)
    {
        return ParameterValue_Read(&axis->configuration, axis->parameters.revision, id, value);
    }
    if (!MotorStateMachine_IsStopped(&axis->state_machine) ||
        axis->state_machine.calibration_type != MOTOR_CALIBRATION_NONE)
    {
        return PARAMETER_WRONG_STATE;
    }
    if (axis->hardware->read_record(&axis->record, sizeof(axis->record)) != BSP_OK)
    {
        return PARAMETER_PERSIST_FAILED;
    }
    MotorConfiguration *stored = &axis->calibration.workspace.parameters;
    memset(stored, 0, sizeof(*stored));
    Param_Download(&axis->record.motor, sizeof(axis->record.motor), stored);
    if (stored->configuration_source != MOTOR_CONFIGURATION_STORED)
    {
        return PARAMETER_PERSIST_FAILED;
    }
    if (id == 0x70U)
    {
        if (axis->record.output.magic != PARAM_OUTPUT_ENCODER_V3 ||
            axis->record.output.motor_crc32 != axis->record.motor.record_crc32 ||
            axis->record.output.crc32 !=
                Crc32_Compute(&axis->record.output, offsetof(OutputEncoderRecord, crc32), 0U))
        {
            return PARAMETER_PERSIST_FAILED;
        }
        *value = (ParameterValue){.parameter_id = id,
                                  .value_type = PARAMETER_VALUE_U32,
                                  .access = PARAMETER_ACCESS_READ,
                                  .revision = axis->parameters.saved_revision,
                                  .raw_value = Param_CoordinateCrc(&axis->record)};
        return PARAMETER_OK;
    }
    return ParameterValue_Read(stored, axis->parameters.saved_revision, id, value);
}

ParameterStatus MotorAxis_RestoreControlDefaults(MotorAxis *axis, uint32_t expected_revision)
{
    if (expected_revision != axis->parameters.revision)
    {
        return PARAMETER_REVISION_CONFLICT;
    }
    if (axis->parameters.revision == UINT32_MAX)
    {
        return PARAMETER_OUT_OF_RANGE;
    }
    if (!begin_parameter_update(axis))
    {
        return PARAMETER_WRONG_STATE;
    }
    MotorConfiguration *defaults = &axis->calibration.workspace.parameters;
    memset(defaults, 0, sizeof(*defaults));
    if (axis->configuration.preset != NULL)
    {
        const MotorAxisProfile *profile = &axis->configuration.application.axis_profile;
        if (!Param_CreateAxis(axis->configuration.device.node_id,
                              profile->minimum_position_rad,
                              profile->maximum_position_rad,
                              profile->maximum_speed_rad_s,
                              &axis->configuration.model.thermal,
                              defaults))
        {
            axis->state_machine.parameters_updating = false;
            return PARAMETER_INVALID_ARGUMENT;
        }
    }
    else
    {
        Param_Return_Default(defaults);
    }
    /* 仅控制调参恢复默认。传感器标定、装配几何和模型身份原样保留。 */
    MotorApplicationConfig *app = &axis->configuration.application;
    const MotorApplicationConfig *base = &defaults->application;
    app->speed_limit_rad_s = fminf(base->speed_limit_rad_s, app->speed_ceiling_rad_s);
    app->position_maximum_speed_rad_s =
        fminf(base->position_maximum_speed_rad_s, app->speed_limit_rad_s);
    app->current_limit_a = fminf(base->current_limit_a, axis->configuration.model.thermal.peak_a);
    app->speed_acceleration_rad_s2 = base->speed_acceleration_rad_s2;
    app->speed_deceleration_rad_s2 = base->speed_deceleration_rad_s2;
    app->speed_kp = base->speed_kp;
    app->speed_ki = base->speed_ki;
    app->speed_feedforward = base->speed_feedforward;
    app->speed_feedforward.current_limit_a =
        fminf(app->speed_feedforward.current_limit_a, app->current_limit_a);
    app->position_error_window_rad = base->position_error_window_rad;
    app->position_acceleration_rad_s2 = base->position_acceleration_rad_s2;
    app->position_deceleration_rad_s2 = base->position_deceleration_rad_s2;
    app->cascade_position_kp = base->cascade_position_kp;
    app->cascade_position_kd = base->cascade_position_kd;
    axis->configuration.model.id_kp = defaults->model.id_kp;
    axis->configuration.model.id_ki = defaults->model.id_ki;
    axis->configuration.model.iq_kp = defaults->model.iq_kp;
    axis->configuration.model.iq_ki = defaults->model.iq_ki;
    ++axis->parameters.revision;
    finish_parameter_update(axis);
    return PARAMETER_OK;
}
