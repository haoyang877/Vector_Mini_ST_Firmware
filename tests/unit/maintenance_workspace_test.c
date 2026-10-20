#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "critical_hw.h"
#include "maintenance_workspace.h"
#include "motor_startup_profile.h"
#include "control_config.h"
#include "control_math.h"

/* 只替代控制边界；完整任务源码由脚本插入，计算与阶段分支使用真实实现。 */
static unsigned s_cleanup_order;
static bool s_report_before_cleanup;
const SensorlessStartupConfig SensorlessStartup_EncoderCalibConfig = {
    .align_current_ramp_time_s = SENSORLESS_ENCODER_CALIB_ALIGN_CURRENT_RAMP_TIME_S,
    .align_hold_time_s = SENSORLESS_ENCODER_CALIB_ALIGN_HOLD_TIME_S,
    .minimum_electrical_velocity_rad_s = SENSORLESS_ENCODER_CALIB_MIN_ELEC_VEL_RAD_S,
};

void MotorFault_Report(MotorControlState *motor, MotorFaultCode error)
{
    s_report_before_cleanup = s_cleanup_order == 0U;
    motor->fault_code = error;
}
void SensorlessStartup_Reset(SensorlessStartupState *startup)
{
    ++s_cleanup_order;
    memset(startup, 0, sizeof(*startup));
}
void FOC_CurrentController_Reset(FocState *foc)
{
    ++s_cleanup_order;
    PiController_Reset(&foc->id_pi);
    PiController_Reset(&foc->iq_pi);
}
void SensorlessStartup_Run(FocState *foc,
                           MotorControlState *motor,
                           PiController *pi,
                           FluxObserverState *observer,
                           SensorlessStartupState *startup,
                           const SensorlessStartupConfig *config)
{
    (void)foc;
    (void)motor;
    (void)pi;
    (void)observer;
    (void)config;
    if (startup->state == SENSORLESS_STARTUP_IDLE)
    {
        startup->state = SENSORLESS_STARTUP_ALIGN;
    }
}
void FOC_Current(FocState *foc, MotorControlState *motor, float phase, float velocity)
{
    (void)foc;
    (void)motor;
    (void)phase;
    (void)velocity;
}
float Observer_GetElePhase(FluxObserverState *observer)
{
    return observer->electrical_angle_rad;
}
float Observer_GetEleVel(FluxObserverState *observer)
{
    return observer->electrical_velocity_rad_s;
}
float Observer_GetElePosition(FluxObserverState *observer)
{
    return observer->electrical_position_rad;
}
uint32_t Observer_GetPositionEpoch(FluxObserverState *observer)
{
    return observer->position_epoch;
}
bool Encoder_IsOnline(const EncoderState *encoder)
{
    return encoder->has_valid_sample;
}
float Encoder_GetMecVelContinuous(const EncoderState *encoder)
{
    return encoder->mechanical_velocity_continuous_rad_s;
}
void Encoder_ResetVelocity(EncoderState *encoder)
{
    encoder->velocity_ready = false;
}
bool Encoder_SetElectricalZeroQ15(EncoderState *encoder, uint16_t angle)
{
    encoder->electrical_zero_q15 = angle;
    encoder->calib_flag |= ENC_CALIB_ELECTRICAL_ZERO;
    return true;
}

/* ENCODER_WORKSPACE_REFERENCE：脚本插入真实任务的采样借用和取消函数。 */

/* 离线临界区桩：核对嵌套进入后恢复原屏蔽状态，不接触硬件。 */
static uint32_t s_mask;
static unsigned s_depth;

uint32_t critical_hw_enter(void)
{
    uint32_t previous = s_mask;
    s_mask = 1U;
    ++s_depth;
    return previous;
}

void critical_hw_exit(uint32_t state)
{
    s_mask = state;
    --s_depth;
}

/* 检查所有失败收尾共享的可观察动作；各门禁仍由真实 StepLut 判定。 */
static int check_task_cleanup(void)
{
    for (unsigned scenario = 0U; scenario < 19U; ++scenario)
    {
        FocState foc = {0};
        MotorControlState motor = {0};
        PiController pi = {0};
        EncoderState encoder = {0};
        FluxObserverState observer = {0};
        SensorlessStartupState startup = {0};
        EncoderCalibrationWorkspace *workspace = MaintenanceWorkspace_BorrowEncoder();
        EncoderCalibrationTask_Cancel();
        motor.model.motor_pole_pairs = 21;
        motor.model.motor_phase_resistance_ohm = 1.0f;
        motor.model.motor_d_inductance_h = motor.model.motor_q_inductance_h = 0.001f;
        motor.model.motor_flux_wb = 0.01f;
        motor.application.current_limit_a = 6.0f;
        encoder.has_valid_sample = true;
        EncoderCalibrationTask_StepLut(&foc, &motor, &pi, &encoder, &observer, &startup, workspace);
        /* 先走完对齐前段的64拍准备；不采原点，避免后续反例被未清完门禁遮蔽。 */
        for (unsigned tick = 1U; tick < (ENCODER_OFFSET_LUT_SIZE + 15U) / 16U; ++tick)
        {
            startup.state_ticks = tick;
            MotorTaskResult prepared = EncoderCalibrationTask_StepLut(
                &foc, &motor, &pi, &encoder, &observer, &startup, workspace);
            if (prepared.result != MOTOR_TASK_RUNNING)
            {
                return 36;
            }
        }
        /* 准备控制残值，确保错误码上报、复位及工作区释放不会因原本为零而漏检。 */
        motor.speed_target_rad_s = motor.speed_ramped_rad_s = 15.0f;
        motor.id_target_a = motor.iq_target_a = 2.0f;
        foc.id_pi.integral_output = foc.iq_pi.integral_output = pi.integral_output = 3.0f;
        startup.state = SENSORLESS_STARTUP_CLOSED_LOOP;
        s_cleanup_order = 0U;
        s_report_before_cleanup = false;
        MotorFaultCode error = MOTOR_FAULT_ENCODER;
        switch (scenario)
        {
        case 0U:
            workspace = NULL;
            break;
        case 1U:
            encoder.has_valid_sample = false;
            break;
        case 2U:
            motor.model.motor_pole_pairs = 0;
            error = MOTOR_FAULT_POLE_PAIRS;
            break;
        case 3U:
            motor.model.motor_flux_wb = 0.0f;
            error = MOTOR_FAULT_INVALID_PARAMETERS;
            break;
        case 4U:
            s_calibration_phase = CS_OBS_ALIGN_ORIGIN;
            break;
        case 5U:
            s_calibration_phase = CS_OBS_FIND_ORIGIN;
            observer.position_epoch = 1U;
            error = MOTOR_FAULT_SENSORLESS;
            break;
        case 6U:
            s_calibration_phase = CS_OBS_SAMPLE_CW;
            observer.position_epoch = 1U;
            error = MOTOR_FAULT_SENSORLESS;
            break;
        case 7U:
            s_calibration_phase = CS_OBS_BUILD_LUT;
            break;
        case 8U:
            s_calibration_phase = CS_OBS_VERIFY_CW;
            observer.position_epoch = 1U;
            error = MOTOR_FAULT_SENSORLESS;
            break;
        case 9U:
            s_calibration_phase = CS_OBS_ALIGN_ORIGIN;
            break;
        case 10U:
            s_calibration_phase = CS_OBS_BUILD_LUT;
            observer.position_epoch = 1U;
            error = MOTOR_FAULT_SENSORLESS;
            break;
        case 11U:
            s_calibration_phase = CS_OBS_SPEED_STABLE;
            observer.position_epoch = 1U;
            error = MOTOR_FAULT_SENSORLESS;
            break;
        case 12U:
            motor.application.current_limit_a = 0.0f;
            error = MOTOR_FAULT_INVALID_PARAMETERS;
            break;
        case 13U:
            motor.application.current_limit_a =
                SENSORLESS_ENCODER_CALIB_MIN_CURRENT_LIMIT_A - 0.25f;
            error = MOTOR_FAULT_INVALID_PARAMETERS;
            break;
        case 14U:
        case 15U:
        case 16U:
        case 17U:
            s_calibration_phase = (EncoderCalibrationPhase)(CS_OBS_FIND_ORIGIN + scenario - 14U);
            startup.state = SENSORLESS_STARTUP_OPEN_LOOP;
            error = MOTOR_FAULT_SENSORLESS;
            break;
        default:
            s_calibration_phase = CS_OBS_SPEED_STABLE;
            startup.state = SENSORLESS_STARTUP_OPEN_LOOP;
            break;
        }
        if (scenario == 9U)
        {
            /* 用实际活动阶段检查取消：任务先归还引用，组合层再归还真实工作区。 */
            EncoderCalibrationTask_Cancel();
            if (!EncoderCalibrationTask_IsReleased() || MaintenanceWorkspace_IsReleased())
            {
                return 35;
            }
            MaintenanceWorkspace_ReleaseEncoder();
            continue;
        }
        MotorTaskResult result = EncoderCalibrationTask_StepLut(
            &foc, &motor, &pi, &encoder, &observer, &startup, workspace);
        if (scenario == 18U)
        {
            /* 稳速窗口允许尚未闭环，不能被统一跟踪拒绝误伤。 */
            if (result.result != MOTOR_TASK_RUNNING || motor.fault_code != MOTOR_FAULT_NONE ||
                s_report_before_cleanup || s_cleanup_order != 0U ||
                EncoderCalibrationTask_IsReleased())
            {
                return 34;
            }
            EncoderCalibrationTask_Cancel();
            MaintenanceWorkspace_ReleaseEncoder();
            continue;
        }
        if (result.result != MOTOR_TASK_FAILED || result.error != error ||
            motor.fault_code != error || !s_report_before_cleanup || s_cleanup_order != 2U ||
            !EncoderCalibrationTask_IsReleased() || startup.state != SENSORLESS_STARTUP_IDLE ||
            foc.id_pi.integral_output != 0.0f || foc.iq_pi.integral_output != 0.0f ||
            pi.integral_output != 0.0f || motor.speed_target_rad_s != 0.0f ||
            motor.speed_ramped_rad_s != 0.0f || motor.id_target_a != 0.0f ||
            motor.iq_target_a != 0.0f)
        {
            return 10 + (int)scenario;
        }
        MaintenanceWorkspace_ReleaseEncoder();
        s_calibration_phase = CS_OBS_STOP_DECEL;
        motor.fault_code = MOTOR_FAULT_NONE;
        motor.id_target_a = motor.iq_target_a = 2.0f;
        s_cleanup_order = 0U;
        result =
            EncoderCalibrationTask_StepLut(&foc, &motor, &pi, &encoder, &observer, &startup, NULL);
        /* 离线/参数非法场景仍被入口门禁拒绝；正常参数下低速停止必须完成并撤销转矩。 */
        if (scenario != 1U && scenario != 2U && scenario != 3U && scenario != 12U &&
            scenario != 13U &&
            (result.result != MOTOR_TASK_COMPLETED || result.error != MOTOR_FAULT_NONE ||
             motor.id_target_a != 0.0f || motor.iq_target_a != 0.0f || s_cleanup_order != 2U ||
             !EncoderCalibrationTask_IsReleased()))
        {
            return 20 + (int)scenario;
        }
        EncoderCalibrationTask_Cancel();
        EncoderCalibrationTask_Cancel();
    }
    return 0;
}

/* 对完整基点/当前任务逐拍记录相同输入、输出、阶段和工作区，比较由 Python 执行。 */
static uint64_t s_trace_hash = UINT64_C(14695981039346656037);

static void hash_bytes(const void *value, size_t bytes)
{
    const uint8_t *data = value;
    for (size_t index = 0U; index < bytes; ++index)
    {
        s_trace_hash ^= data[index];
        s_trace_hash *= UINT64_C(1099511628211);
    }
}

static void hash_tick(MotorTaskResult result,
                      const MotorControlState *motor,
                      const EncoderState *encoder,
                      const EncoderCalibrationWorkspace *workspace)
{
    EncoderCalibrationPhase phase = EncoderCalibrationTask_GetStep();
    hash_bytes(&result.result, sizeof(result.result));
    hash_bytes(&result.error, sizeof(result.error));
    hash_bytes(&phase, sizeof(phase));
    hash_bytes(&motor->id_target_a, sizeof(motor->id_target_a));
    hash_bytes(&motor->iq_target_a, sizeof(motor->iq_target_a));
    hash_bytes(&motor->speed_target_rad_s, sizeof(motor->speed_target_rad_s));
    hash_bytes(&motor->speed_ramped_rad_s, sizeof(motor->speed_ramped_rad_s));
    hash_bytes(&encoder->calib_flag, sizeof(encoder->calib_flag));
    hash_bytes(&encoder->electrical_zero_q15, sizeof(encoder->electrical_zero_q15));
    hash_bytes(&encoder->mechanical_zero_q15, sizeof(encoder->mechanical_zero_q15));
    if (workspace != NULL)
    {
        hash_bytes(workspace, sizeof(*workspace));
    }
    hash_bytes(&s_zero_loop_count, sizeof(s_zero_loop_count));
    hash_bytes(&s_zero_sample_count, sizeof(s_zero_sample_count));
    hash_bytes(&s_zero_unwrapped_sum, sizeof(s_zero_unwrapped_sum));
}

static int check_complete_task_trace(void)
{
    FocState foc = {0};
    MotorControlState motor = {0};
    PiController pi = {0};
    EncoderState encoder = {0};
    FluxObserverState observer = {0};
    SensorlessStartupState startup = {0};
    EncoderCalibrationTask_Cancel();
    EncoderCalibrationWorkspace *workspace = MaintenanceWorkspace_BorrowEncoder();
    motor.model.motor_pole_pairs = 21;
    motor.model.motor_phase_resistance_ohm = 1.0f;
    motor.model.motor_d_inductance_h = motor.model.motor_q_inductance_h = 0.001f;
    motor.model.motor_flux_wb = 0.01f;
    motor.application.current_limit_a = 6.0f;
    encoder.has_valid_sample = true;
    encoder.read_status = ENCODER_READ_OK;
    encoder.calib_flag = ENC_CALIB_ALL;
    encoder.electrical_zero_q15 = 3U;
    encoder.mechanical_zero_q15 = 4U;
    for (unsigned bin = 0U; bin < ENCODER_OFFSET_LUT_SIZE; ++bin)
    {
        encoder.linearization_lut_q15[bin] = 42;
    }
    MotorTaskResult result =
        EncoderCalibrationTask_StepLut(&foc, &motor, &pi, &encoder, &observer, &startup, workspace);
    uint32_t align_ticks =
        (uint32_t)((SensorlessStartup_EncoderCalibConfig.align_current_ramp_time_s +
                    SensorlessStartup_EncoderCalibConfig.align_hold_time_s -
                    SENSORLESS_ENCODER_CALIB_ALIGN_SAMPLE_TIME_S) /
                   Current_Ts);
    /* 首拍已清16 bin；其余63拍仍在原对齐采样窗口之前，保留下面原8个原点样本。 */
    for (unsigned tick = 1U; tick < (ENCODER_OFFSET_LUT_SIZE + 15U) / 16U; ++tick)
    {
        if (tick >= align_ticks)
        {
            return 36;
        }
        startup.state_ticks = tick;
        result = EncoderCalibrationTask_StepLut(
            &foc, &motor, &pi, &encoder, &observer, &startup, workspace);
        if (result.result != MOTOR_TASK_RUNNING)
        {
            return 36;
        }
        hash_tick(result, &motor, &encoder, workspace);
    }
    for (unsigned tick = 0U; tick < 8U; ++tick)
    {
        startup.state_ticks = align_ticks + tick;
        result = EncoderCalibrationTask_StepLut(
            &foc, &motor, &pi, &encoder, &observer, &startup, workspace);
        hash_tick(result, &motor, &encoder, workspace);
    }
    startup.state = SENSORLESS_STARTUP_CLOSED_LOOP;
    startup.speed_feedback_rad_s = SENSORLESS_ENCODER_CALIB_SPEED_MEC_RAD_S;
    motor.speed_ramped_rad_s = SENSORLESS_ENCODER_CALIB_SPEED_MEC_RAD_S;
    observer.electrical_velocity_rad_s = SENSORLESS_ENCODER_CALIB_SPEED_MEC_RAD_S * 21.0f;
    float required_theta = _2PI * 21.0f;
    uint32_t samples_per_turn = 20480U / SENSORLESS_ENCODER_CALIB_MECH_TURNS;
    unsigned crossing_ticks = 0U, lut_ticks = 0U;
    for (; lut_ticks < 60000U; ++lut_ticks)
    {
        EncoderCalibrationPhase phase = EncoderCalibrationTask_GetStep();
        startup.speed_feedback_rad_s = SENSORLESS_ENCODER_CALIB_SPEED_MEC_RAD_S;
        if (phase == CS_OBS_FIND_ORIGIN)
        {
            encoder.directed_q15 = crossing_ticks == 0U ? 65408U : 0U;
            observer.electrical_position_rad = crossing_ticks == 0U ? 0.01f : 0.02f;
            ++crossing_ticks;
        }
        else if (phase == CS_OBS_SAMPLE_CW || phase == CS_OBS_VERIFY_CW)
        {
            observer.electrical_position_rad += required_theta / (float)samples_per_turn;
            float relative_theta = observer.electrical_position_rad - 0.02f;
            encoder.directed_q15 =
                (uint16_t)(relative_theta * ((float)ENCODER_Q15_CPR / required_theta));
            if (phase == CS_OBS_SAMPLE_CW && lut_ticks % 17U == 0U)
            {
                startup.speed_feedback_rad_s = 0.0f;
            }
        }
        else if (phase == CS_OBS_STOP_DECEL || phase == CS_OBS_STOP_CURRENT)
        {
            observer.electrical_velocity_rad_s = 0.0f;
        }
        result = EncoderCalibrationTask_StepLut(
            &foc, &motor, &pi, &encoder, &observer, &startup, workspace);
        hash_tick(result, &motor, &encoder, workspace);
        if (result.result != MOTOR_TASK_RUNNING)
        {
            break;
        }
    }
    if (result.result != MOTOR_TASK_COMPLETED || motor.fault_code != MOTOR_FAULT_NONE ||
        encoder.calib_flag != ENC_CALIB_LINEARIZED || !encoder.rebase_requested ||
        encoder.electrical_zero_q15 != 0U || encoder.mechanical_zero_q15 != 0U ||
        !EncoderCalibrationTask_IsReleased() || motor.id_target_a != 0.0f ||
        motor.iq_target_a != 0.0f)
    {
        printf("trace LUT FAIL ticks=%u phase=%u error=%u\n",
               lut_ticks,
               EncoderCalibrationTask_GetStep(),
               result.error);
        return 31;
    }
    for (unsigned bin = 0U; bin < ENCODER_OFFSET_LUT_SIZE; ++bin)
    {
        if (encoder.linearization_lut_q15[bin] != 0)
        {
            return 32;
        }
    }
    MaintenanceWorkspace_ReleaseEncoder();
    encoder.linearized_q15 = 65530U;
    motor.model.calibration_current_a = 2.0f;
    unsigned zero_ticks = 0U;
    for (; zero_ticks < 40000U; ++zero_ticks)
    {
        encoder.linearized_q15 = zero_ticks % 2U == 0U ? 65530U : 10U;
        result = EncoderCalibrationTask_StepElectricalZero(&foc, &motor, &encoder);
        hash_tick(result, &motor, &encoder, NULL);
        if (result.result != MOTOR_TASK_RUNNING)
        {
            break;
        }
    }
    if (result.result != MOTOR_TASK_COMPLETED ||
        (encoder.electrical_zero_q15 != 1U && encoder.electrical_zero_q15 != 2U) ||
        !EncoderCalibrationTask_IsReleased() || motor.id_target_a != 0.0f ||
        motor.iq_target_a != 0.0f)
    {
        return 33;
    }
    printf("trace PASS: lut_ticks=%u zero_ticks=%u zero_q15=%u hash=%016llx\n",
           lut_ticks,
           zero_ticks,
           encoder.electrical_zero_q15,
           (unsigned long long)s_trace_hash);
    return 0;
}

/**
 * @brief 检查实际类型容量、共享地址、互斥、错误归还和内容保持。
 * @return 全部满足契约返回 0，否则返回失败编号。
 */
int main(void)
{
    if (sizeof(EncoderCalibrationWorkspace) != MAINTENANCE_WORKSPACE_CAPACITY_BYTES ||
        sizeof(InterfaceParam) != 2288U || !MaintenanceWorkspace_IsReleased())
    {
        return 1;
    }
    for (unsigned pass = 0U; pass < 4U; ++pass)
    {
        EncoderCalibrationWorkspace *encoder = MaintenanceWorkspace_BorrowEncoder();
        if (encoder == NULL || (uintptr_t)encoder % 8U != 0U ||
            MaintenanceWorkspace_BorrowParameter() != NULL)
        {
            return 2;
        }
        memset(encoder, 0xA5, sizeof(*encoder));
        if (!encoder_calib_prepare_samples(encoder) || EncoderCalibrationTask_IsReleased())
        {
            return 8;
        }
        memset(encoder, 0xA5, sizeof(*encoder));
        MaintenanceWorkspace_ReleaseParameter((InterfaceParam *)encoder);
        if (MaintenanceWorkspace_IsReleased() || MaintenanceWorkspace_BorrowEncoder() != encoder ||
            encoder->bins.samples[ENCODER_OFFSET_LUT_SIZE - 1U] != 0xA5A5U)
        {
            return 3;
        }
        EncoderCalibrationTask_Cancel();
        if (!EncoderCalibrationTask_IsReleased() || MaintenanceWorkspace_IsReleased())
        {
            return 9;
        }
        MaintenanceWorkspace_ReleaseEncoder();
        InterfaceParam *parameter = MaintenanceWorkspace_BorrowParameter();
        if ((void *)parameter != (void *)encoder ||
            MaintenanceWorkspace_BorrowParameter() != NULL ||
            MaintenanceWorkspace_BorrowEncoder() != NULL)
        {
            return 4;
        }
        memset(parameter, 0x5A, sizeof(*parameter));
        MaintenanceWorkspace_ReleaseParameter(NULL);
        MaintenanceWorkspace_ReleaseEncoder();
        if (MaintenanceWorkspace_IsReleased() || parameter->record_crc32 != 0x5A5A5A5AU)
        {
            return 5;
        }
        MaintenanceWorkspace_ReleaseParameter(parameter);
    }
    s_mask = 1U;
    if (!MaintenanceWorkspace_IsReleased() || s_mask != 1U || s_depth != 0U)
    {
        return 7;
    }
    int cleanup = check_task_cleanup();
    if (cleanup != 0)
    {
        return cleanup;
    }
    int trace = check_complete_task_trace();
    if (trace != 0)
    {
        return trace;
    }
    printf("workspace PASS: encoder=%zu parameter=%zu; one aligned shared storage\n",
           sizeof(EncoderCalibrationWorkspace),
           sizeof(InterfaceParam));
    return 0;
}
