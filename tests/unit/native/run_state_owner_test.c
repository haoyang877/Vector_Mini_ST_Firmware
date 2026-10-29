#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "motor_state_machine.h"
#include "motor_maintenance.h"
#include "encoder_calibration_task.h"
#include "motor_fault.h"
#include "motor_command.h"
#include "motor_control.h"
#include "foc_sensing.h"
#include "sensorless_startup.h"
#include "bsp_pwm.h"
#include "bsp_system.h"
#include "bsp_led.h"
#include "bus_voltage_profile.h"
#include "control_math.h"
#include "maintenance_workspace.h"
#include "motor_thermal_control.h"
#include "application_profile_defaults.h"
#include "parameter_values.h"
#define FOC_MAX_MODULATION (0.95f * 0.866025403f)

/* 真实生产所有者和类型；仅替换硬件与 worker，验证命令、资源和身份交接。 */
static MotorConfiguration cfg;
static MotorControlState motor;
static FocState foc;
static PiController pi;
static EncoderState encoder;
static SensorlessStartupState startup;
static bool output_off, stop_fails, start_fails, temperature_fresh, encoder_released;
static unsigned start_count, stop_count, leaf_calls[8];
static bool trace_enabled;
static unsigned trace_event;
static const char *trace_case;
static uint32_t trace_ticket;
static uint8_t trace_source;
static uint16_t trace_start_sequence;
static void trace_capture(const char *event, unsigned result, unsigned detail);
static bool outer_ready;
static bool lut_publish_fail;
static uint32_t critical_depth;
static unsigned validation_interrupt;
static unsigned validation_unmasked_calls, validation_masked_calls;
static unsigned configuration_validation_calls;
static unsigned configuration_masked_calls, configuration_unmasked_calls;
static bool require_locked_validation;
static uint32_t now_ms;
static void tick(void);
static void assert_boot_health(bool expected);
#if STATE_START_ADMISSION_PREVALIDATED
static unsigned start_admission_interrupt;
static void inject_start_admission(unsigned interrupt);
#endif
#if STATE_SUPERVISOR_STAGED
static void test_staged_supervisor(void);
#endif

uint32_t BspSystem_TimeMs(void)
{
    return now_ms;
}
static uint8_t protocol_mode(MotorControlMode mode, MotorMaintenanceKind kind);
static uint8_t protocol_state(MotorState state, uint8_t mode);

/* 本夹具模拟普通 APP 的无 Loader 端口，配置修订号仍有可用空间。 */
bool MotorAxis_ConfigurationRevisionAvailable(void)
{
    return true;
}

uint32_t BspSystem_EnterCritical(void)
{
    return critical_depth++;
}
void BspSystem_ExitCritical(uint32_t mask)
{
    assert(critical_depth == mask + 1U);
    critical_depth = mask;
}

/* 观测真实配置校验的屏蔽状态；在开中断区间模拟一次高优先级事件。 */
bool Test_ConfigurationValid(const MotorConfiguration *configuration)
{
    ++configuration_validation_calls;
    if (critical_depth != 0U)
    {
        ++configuration_masked_calls;
    }
    else
    {
        ++configuration_unmasked_calls;
    }
    bool valid = ParameterValue_ConfigurationValid(configuration);
    if (require_locked_validation)
    {
        assert(critical_depth != 0U);
        require_locked_validation = false;
    }
    if (validation_interrupt != 0U)
    {
        if (critical_depth != 0U)
        {
            ++validation_masked_calls;
        }
        else
        {
            unsigned interrupt = validation_interrupt;
            validation_interrupt = 0U;
            ++validation_unmasked_calls;
            if (interrupt == 1U)
            {
                MotorFault_Report(&motor, MOTOR_FAULT_OVERVOLTAGE);
                MotorStateMachine_FastFaultStop();
            }
            else
            {
                assert(MotorStateMachine_RequestRemoteStop());
                if (interrupt == 3U || interrupt == 4U)
                {
                    tick();
#if STATE_START_SUPERVISOR_PREVALIDATED
                    if (interrupt == 4U)
                    {
                        inject_start_admission(4U);
                    }
#endif
                    cfg.application.current_limit_a = NAN;
                    require_locked_validation = true;
                }
            }
        }
    }
#if STATE_START_ADMISSION_PREVALIDATED
    if (start_admission_interrupt != 0U && critical_depth == 0U)
    {
        unsigned interrupt = start_admission_interrupt;
        start_admission_interrupt = 0U;
        inject_start_admission(interrupt);
    }
#endif
    return valid;
}
bool BspPwm_ArePhasesDisabled(void)
{
    return output_off;
}
BspPwmStatus BspPwm_GetStatus(void)
{
    return (BspPwmStatus){output_off ? BSP_PWM_OFF : BSP_PWM_ON, stop_count};
}
BspResult BspPwm_Enable(void)
{
#if STATE_FAST_POWER_OWNER
    assert(critical_depth == 0U);
#else
    assert(critical_depth != 0U);
#endif
    ++start_count;
    if (!start_fails)
    {
        output_off = false;
    }
    if (trace_enabled)
    {
        printf("TRACE POWER %s %u START %u %u %u\n",
               trace_case,
               trace_event,
               start_count,
               output_off,
               critical_depth);
    }
    return output_off ? BSP_IO_ERROR : BSP_OK;
}
void BspPwm_Disable(void)
{
    ++stop_count;
    if (!stop_fails)
    {
        output_off = true;
    }
    if (trace_enabled)
    {
        printf("TRACE POWER %s %u STOP %u %u %u\n",
               trace_case,
               trace_event,
               stop_count,
               output_off,
               critical_depth);
    }
}
BspResult BspLed_SetCode(BspLedCodeChannel error, uint8_t count)
{
    (void)error;
    (void)count;
    return BSP_OK;
}
bool Encoder_IsOnline(const EncoderState *e)
{
    return e->bad_frame_streak == 0U;
}
uint16_t Encoder_GetBadFrameStreak(const EncoderState *e)
{
    return e->bad_frame_streak;
}
uint8_t Encoder_GetCalibFlag(const EncoderCalibration *calibration)
{
    return calibration->calib_flag;
}
float Encoder_GetMecPos(const EncoderState *e)
{
    return e->mechanical_position_rad;
}
float Encoder_GetMecVel(const EncoderState *e)
{
    return e->mechanical_velocity_rad_s;
}
bool MotorTemperature_IsFresh(void)
{
    return temperature_fresh;
}
float Math_Abs(float value)
{
    return fabsf(value);
}
void FOC_CurrentController_Reset(FocState *f)
{
    f->id_a = f->iq_a = 0.0f;
}
void PiController_Reset(PiController *p)
{
    memset(p, 0, sizeof(*p));
}
void SensorlessStartup_Reset(SensorlessStartupState *s)
{
    memset(s, 0, sizeof(*s));
}
void motor_outer_loop_request_reset(void)
{
}
bool MotorOuterLoop_IsReady(void)
{
    return outer_ready;
}
void EncoderCalibrationTask_Cancel(void)
{
    encoder_released = true;
}
bool EncoderCalibrationTask_IsReleased(void)
{
    return encoder_released;
}
bool EncoderCalibrationTask_PublishLut(EncoderCalibration *calibration, EncoderState *state)
{
    (void)state;
    assert(critical_depth != 0U && output_off && !encoder_released);
    if (lut_publish_fail)
    {
        return false;
    }
    calibration->calib_flag = ENC_CALIB_LINEARIZED;
    encoder_released = true;
    return true;
}
EncoderCalibrationPhase EncoderCalibrationTask_GetStep(void)
{
    return encoder_released ? CS_NULL : CS_OBS_ALIGN_ORIGIN;
}

#if STATE_SUPERVISOR_STAGED
/* 历史owner场景仍调用一拍；新生产所有者按唯一Prepare/Commit配对执行。 */
static void MotorStateMachine_Tick(void)
{
    MotorSupervisorStep step;
    bool allowed = MotorStateMachine_PrepareSupervisor(&step);
    step.calculated = allowed;
    MotorStateMachine_CommitSupervisor(&step);
}
#endif

static void tick(void)
{
#if STATE_FAST_POWER_OWNER
    MotorStateMachine_BeginFastTick();
#endif
    MotorStateMachine_PostOutcome((MotorTaskResult){MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE},
                                  MotorStateMachine_WorkEpoch());
    MotorStateMachine_Tick();
#if STATE_FAST_POWER_OWNER
    /* 模拟下一ADC与后续监督确认；精确分拍契约另在专门用例验证。 */
    MotorStateMachine_BeginFastTick();
    if (MotorStateMachine_GetState() == MOTOR_STATE_STARTING && !output_off)
    {
        MotorStateMachine_PostOutcome((MotorTaskResult){MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE},
                                      MotorStateMachine_WorkEpoch());
        MotorStateMachine_Tick();
    }
#endif
    trace_capture("tick", 0U, 0U);
}

static void reset(void)
{
    bool resume_trace = trace_enabled;
    trace_enabled = false;
    validation_interrupt = 0U;
    require_locked_validation = false;
#if STATE_START_ADMISSION_PREVALIDATED
    start_admission_interrupt = 0U;
#endif
    now_ms = 0U;
    memset(&cfg, 0, sizeof(cfg));
    memset(&motor, 0, sizeof(motor));
    memset(&foc, 0, sizeof(foc));
    memset(&encoder, 0, sizeof(encoder));
    cfg.application.axis_profile_valid = true;
    MotorWindingThermal_Default(&cfg.model.thermal);
    cfg.application.speed_feedforward = (SpeedFeedforwardConfig)PARAM_APP_FEEDFORWARD_INITIALIZER;
    assert(MotorThermalControl_Init(&cfg.model.thermal, now_ms));
    motor.thermal_clear_ready = true;
    motor.thermal_current_limit_a = MotorThermalControl_CurrentLimit(6.0f);
    cfg.model.motor_pole_pairs = 21;
    cfg.configuration_confirmed = true;
    cfg.model.motor_phase_resistance_ohm = 1.0f;
    cfg.model.motor_d_inductance_h = cfg.model.motor_q_inductance_h = 0.001f;
    cfg.model.motor_flux_wb = 0.01f;
    cfg.model.calibration_current_a = 3.0f;
    cfg.application.current_limit_a = 6.0f;
    cfg.application.speed_limit_rad_s = 20.0f;
    cfg.application.speed_acceleration_rad_s2 = cfg.application.speed_deceleration_rad_s2 = 1.0f;
    cfg.application.speed_kp = 0.1f;
    cfg.application.speed_ki = 0.2f;
    cfg.device.phase_a_offset_counts = cfg.device.phase_b_offset_counts =
        cfg.device.phase_c_offset_counts = 2048.0f;
    cfg.device.encoder.calib_flag = ENC_CALIB_ALL;
    foc.bus_voltage_v = foc.bus_voltage_filtered_v = 30.0f;
    foc.temperature_c = 25.0f;
    output_off = outer_ready = temperature_fresh = encoder_released = true;
    lut_publish_fail = false;
    start_fails = false;
    start_count = stop_count = 0U;
    memset(leaf_calls, 0, sizeof(leaf_calls));
    stop_fails = false;
    MotorStateMachine_Init(&cfg, &motor, &foc, &pi, &encoder, &startup);
    for (unsigned i = 0U; i < 4U; ++i)
    {
        tick();
    }
    assert(MotorStateMachine_GetState() == MOTOR_STATE_IDLE);
    trace_ticket = trace_source = trace_start_sequence = 0U;
    trace_enabled = resume_trace;
    trace_capture("reset", 0U, 0U);
}

#if STATE_START_ADMISSION_PREVALIDATED
/* 只在新增开中断校验窗口注入事件；维护仍经真实owner/worker完成，不改私有状态。 */
static void inject_start_admission(unsigned interrupt)
{
    assert(critical_depth == 0U);
    if (interrupt == 1U)
    {
        MotorFault_Report(&motor, MOTOR_FAULT_OVERVOLTAGE);
        MotorStateMachine_FastFaultStop();
    }
    else if (interrupt == 2U)
    {
        assert(MotorStateMachine_RequestRemoteStop());
    }
    else if (interrupt == 3U || interrupt == 4U)
    {
        uint32_t ticket;
        MotorMaintenanceKind kind =
            interrupt == 3U ? MOTOR_MAINTENANCE_SAVE : MOTOR_MAINTENANCE_DEFAULTS;
        assert(MotorStateMachine_RequestMaintenance(kind, &ticket) == MOTOR_CONTROL_ACCEPTED);
        if (kind == MOTOR_MAINTENANCE_DEFAULTS)
        {
            MotorTaskResult outcome =
                MotorMaintenance_Step(&cfg, kind, &foc, &motor, &pi, &encoder, NULL, &startup);
            assert(outcome.result == MOTOR_TASK_COMPLETED);
            MotorStateMachine_PostOutcome(outcome, ticket);
            tick();
            tick();
            assert(MotorStateMachine_GetMaintenanceKind() == MOTOR_MAINTENANCE_SAVE);
        }
        FocBackgroundClaim claim;
        assert(MotorStateMachine_BeginBackground(&claim));
        MotorStateMachine_FinishBackground(&claim, true);
        tick();
        assert(MotorStateMachine_GetState() == MOTOR_STATE_IDLE);
    }
    else if (interrupt == 5U)
    {
        foc.bus_voltage_filtered_v = BUS_VOLTAGE_ENABLE_MIN_V - 1.0f;
    }
    else
    {
        assert(interrupt == 6U);
        uint32_t epoch = MotorStateMachine_WorkEpoch();
        assert(MotorStateMachine_RequestIdentifiedStop(7U, 20U));
        tick();
        assert(MotorStateMachine_GetState() == MOTOR_STATE_IDLE &&
               MotorStateMachine_WorkEpoch() == epoch);
    }
    /* 故障/STOP或跨维护代际时，最终提交必须退回原锁内完整扫描。 */
    require_locked_validation = interrupt <= 4U;
}
#endif

/* 只记录公共值与硬件桩动作；不读私有状态，不改变请求或监督调用次序。 */
static unsigned trace_float(float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return bits;
}

static void trace_capture(const char *event, unsigned result, unsigned detail)
{
    if (!trace_enabled)
    {
        return;
    }
    MotorControlProgress control = {0};
    MotorControlStartProgress start = {0};
    MotorMaintenanceProgress original = {0}, current = {0};
    uint32_t epoch = MotorStateMachine_WorkEpoch();
    MotorStateMachine_ReadControlProgress(&control);
    bool start_found =
        MotorStateMachine_ReadEnableProgress(trace_source, trace_start_sequence, &start);
    bool original_found = MotorStateMachine_ReadMaintenance(trace_ticket, &original);
    bool current_found = MotorStateMachine_ReadMaintenance(epoch, &current);
    printf("TRACE OWNER %s %u %s %u %u time=%u state=%u epoch=%u mode=%u fault=%u "
           "off=%u starts=%u stops=%u refs=%08x,%08x,%08x control=%u,%u,%u,%u "
           "enable=%u,%u,%u,%u,%u old=%u,%u,%u,%u,%u current=%u,%u,%u,%u,%u "
           "released=%u,%u depth=%u\n",
           trace_case,
           trace_event++,
           event,
           result,
           detail,
           now_ms,
           MotorStateMachine_GetState(),
           epoch,
           motor.control_mode,
           motor.fault_code,
           output_off,
           start_count,
           stop_count,
           trace_float(motor.speed_target_rad_s),
           trace_float(motor.id_target_a),
           trace_float(motor.iq_target_a),
           control.state,
           control.result,
           control.source_node,
           control.sequence,
           start_found,
           start.state,
           start.result,
           start.request.source_node,
           start.request.sequence,
           original_found,
           original.ticket,
           original.kind,
           original.result,
           original.effects,
           current_found,
           current.ticket,
           current.kind,
           current.result,
           current.effects,
           EncoderCalibrationTask_IsReleased(),
           MaintenanceWorkspace_IsReleased(),
           critical_depth);
}

static void trace_begin(const char *name)
{
    trace_case = name;
    trace_event = 0U;
    trace_enabled = true;
    trace_capture("begin", 0U, 0U);
}

static MotorControlResult trace_enable(MotorControlMode mode)
{
    MotorControlResult result = MotorStateMachine_RequestEnable(mode);
    trace_capture("enable", result, mode);
    return result;
}
static MotorControlResult trace_identified_enable(const MotorControlStartRequest *request)
{
    MotorControlResult result = MotorStateMachine_RequestIdentifiedEnable(request);
    if (request != NULL)
    {
        trace_source = request->source_node;
        trace_start_sequence = request->sequence;
    }
    trace_capture("identified-enable", result, request ? request->sequence : 0U);
    return result;
}
static MotorControlResult trace_control(const MotorControlCommand *command)
{
    MotorControlResult result = MotorStateMachine_SubmitControl(command);
    trace_capture("control", result, command ? command->sequence : 0U);
    return result;
}
static bool trace_stop(void)
{
    bool result = MotorStateMachine_RequestRemoteStop();
    trace_capture("stop", result, 0U);
    return result;
}
static bool trace_identified_stop(uint8_t source, uint16_t sequence)
{
    bool result = MotorStateMachine_RequestIdentifiedStop(source, sequence);
    trace_capture("identified-stop", result, sequence);
    return result;
}
static MotorControlResult trace_clear(void)
{
    MotorControlResult result = MotorStateMachine_RequestClear();
    trace_capture("clear", result, 0U);
    return result;
}
static MotorControlResult trace_maintenance(MotorMaintenanceKind kind, uint32_t *ticket)
{
    MotorControlResult result = MotorStateMachine_RequestMaintenance(kind, ticket);
    if (result == MOTOR_CONTROL_ACCEPTED && ticket != NULL)
    {
        trace_ticket = *ticket;
    }
    trace_capture("maintenance", result, kind);
    return result;
}
static bool trace_begin_background(FocBackgroundClaim *claim)
{
    bool result = MotorStateMachine_BeginBackground(claim);
    trace_capture("claim", result, result ? claim->epoch : 0U);
    return result;
}
static void trace_finish_background(const FocBackgroundClaim *claim, bool success)
{
    MotorStateMachine_FinishBackground(claim, success);
    trace_capture("finish", success, claim ? claim->epoch : 0U);
}
static void trace_outcome(MotorTaskResult result, uint32_t epoch)
{
    MotorStateMachine_PostOutcome(result, epoch);
    trace_capture("outcome", result.result, epoch);
}
static void trace_direct_tick(void)
{
    MotorStateMachine_Tick();
    trace_capture("direct-tick", 0U, 0U);
}

#define MotorStateMachine_RequestEnable trace_enable
#define MotorStateMachine_RequestIdentifiedEnable trace_identified_enable
#define MotorStateMachine_SubmitControl trace_control
#define MotorStateMachine_RequestRemoteStop trace_stop
#define MotorStateMachine_RequestIdentifiedStop trace_identified_stop
#define MotorStateMachine_RequestClear trace_clear
#define MotorStateMachine_RequestMaintenance trace_maintenance
#define MotorStateMachine_BeginBackground trace_begin_background
#define MotorStateMachine_FinishBackground trace_finish_background
#define MotorStateMachine_PostOutcome trace_outcome
#define MotorStateMachine_Tick trace_direct_tick

/* 以公共维护入口开启会话并返回创建时 ticket；不再经由 SWD 台架邮箱。 */
static uint32_t begin_maintenance(MotorMaintenanceKind kind)
{
    uint32_t ticket = 0U;
    assert(MotorStateMachine_RequestMaintenance(kind, &ticket) == MOTOR_CONTROL_ACCEPTED);
    assert(ticket != 0U);
#if STATE_FAST_POWER_OWNER
    assert(output_off);
    MotorStateMachine_BeginFastTick();
#endif
    return ticket;
}

/* 运行期恢复默认不能更改已冻结的通信地址；真实维护分发仍调用默认值叶接口。 */
static void test_defaults_preserves_device_address(void)
{
    reset();
    cfg.device.node_id = 7U;
    MotorWindingThermalConfig custom = cfg.model.thermal;
    custom.peak_a = 6.0f;
    assert(MotorThermalControl_Configure(&custom, true));
    cfg.model.thermal = custom;
    for (unsigned i = 0U; i < 20U; ++i)
    {
        MotorThermalControl_Sample(3.0f, -1.5f, -1.5f, true, now_ms);
    }
    MotorThermalDiagnostics before, after;
    assert(MotorThermalControl_GetDiagnostics(&before) && before.model.max_heat > 0.0f);
    MotorControlState unchanged = motor;
    MotorConfiguration unchanged_cfg = cfg;
    output_off = false;
    MotorTaskResult refused = MotorMaintenance_Step(
        &cfg, MOTOR_MAINTENANCE_DEFAULTS, &foc, &motor, &pi, &encoder, NULL, &startup);
    assert(refused.result == MOTOR_TASK_FAILED && refused.error == MOTOR_FAULT_WINDING_THERMAL);
    assert(memcmp(&unchanged, &motor, sizeof(motor)) == 0);
    assert(memcmp(&unchanged_cfg, &cfg, sizeof(cfg)) == 0);
    assert(MotorThermalControl_GetDiagnostics(&after));
    assert(memcmp(&before, &after, sizeof(before)) == 0);
    output_off = true;
    uint32_t ticket = begin_maintenance(MOTOR_MAINTENANCE_DEFAULTS);
    uint32_t epoch = MotorStateMachine_WorkEpoch();
    MotorTaskResult outcome = MotorMaintenance_Step(
        &cfg, MOTOR_MAINTENANCE_DEFAULTS, &foc, &motor, &pi, &encoder, NULL, &startup);
    assert(outcome.result == MOTOR_TASK_COMPLETED);
    assert(MotorThermalControl_GetDiagnostics(&after));
    assert(memcmp(before.model.heat, after.model.heat, sizeof(before.model.heat)) == 0);
    assert(after.model.config.peak_a == cfg.model.thermal.peak_a &&
           cfg.model.thermal.peak_a == 7.0f);
    assert(leaf_calls[MOTOR_MAINTENANCE_DEFAULTS] == 1U && cfg.device.node_id == 7U);
    assert(!cfg.configuration_confirmed && cfg.device.encoder.calib_flag == 0U && output_off);
    MotorStateMachine_PostOutcome(outcome, epoch);
    tick();
    MotorMaintenanceProgress progress;
    assert(MotorStateMachine_ReadMaintenance(ticket, &progress));
    assert(progress.result == MOTOR_MAINTENANCE_COMPLETED &&
           progress.effects == MOTOR_MAINTENANCE_EFFECT_UNCOMMITTED);
    tick();
    assert(MotorStateMachine_GetMaintenanceKind() == MOTOR_MAINTENANCE_SAVE);
    uint32_t save_ticket = MotorStateMachine_WorkEpoch();
    FocBackgroundClaim claim;
    assert(MotorStateMachine_BeginBackground(&claim));
    MotorStateMachine_FinishBackground(&claim, true);
    tick();
    assert(MotorStateMachine_ReadMaintenance(save_ticket, &progress) &&
           progress.effects == MOTOR_MAINTENANCE_EFFECT_COMMITTED);
    assert(cfg.device.node_id == 7U && output_off && motor.control_mode == MOTOR_CONTROL_NONE);
    puts("PASS runtime DEFAULTS retain node7/heat, failed hot-config apply changes nothing, "
         "successful defaults synchronize runtime before derived SAVE");
}

static void test_open_loop_owner(void)
{
    const MotorControlMode modes[] = {MOTOR_CONTROL_OPEN_VOLTAGE, MOTOR_CONTROL_OPEN_CURRENT};
    for (unsigned i = 0U; i < 2U; ++i)
    {
        reset();
        cfg.device.encoder.calib_flag = 0U;
        MotorReadiness readiness;
        MotorStateMachine_ReadReadiness(&readiness);
        assert((readiness.startable_modes & (1U << modes[i])) != 0U);
        assert(MotorStateMachine_RequestEnable(modes[i]) == MOTOR_CONTROL_ACCEPTED);
        tick();
        tick();
        assert(MotorStateMachine_GetState() == MOTOR_STATE_RUNNING && !output_off);
        assert(motor.open_loop_frequency_hz == 0 && motor.open_loop_angle_rad == 0 &&
               motor.vd_target_v == 0);
        MotorControlCommand command = {
            .mode = modes[i], .source_node = 2U, .sequence = 51U, .electrical_frequency_hz = 1.0f};
        if (i == 0U)
        {
            command.vd_v = 1.0f;
            command.vq_v = 2.0f;
        }
        else
        {
            command.id_a = 1.0f;
            command.iq_a = 2.0f;
        }
        assert(MotorStateMachine_SubmitControl(&command) == MOTOR_CONTROL_ACCEPTED);
        tick();
        assert(motor.open_loop_frequency_hz == 1.0f && motor.vd_target_v == command.vd_v &&
               motor.vq_target_v == command.vq_v && motor.id_target_a == command.id_a &&
               motor.iq_target_a == command.iq_a);
        command.electrical_frequency_hz = NAN;
        assert(MotorCommand_ValidateTarget(&cfg, &command, &foc, &encoder) ==
               MOTOR_CONTROL_INVALID);
        command.electrical_frequency_hz = MOTOR_OPEN_LOOP_MAX_ELECTRICAL_HZ + 1.0f;
        assert(MotorCommand_ValidateTarget(&cfg, &command, &foc, &encoder) ==
               MOTOR_CONTROL_OUT_OF_RANGE);
        command.electrical_frequency_hz = 1.0f;
        MotorConfiguration invalid_config = cfg;
        invalid_config.application.speed_limit_rad_s = NAN;
        invalid_config.application.axis_profile.magic = 1U;
        invalid_config.application.axis_profile.maximum_speed_rad_s = 20.0f;
        assert(MotorCommand_ValidateTarget(&invalid_config, &command, &foc, &encoder) ==
               MOTOR_CONTROL_OUT_OF_RANGE);
        invalid_config = cfg;
        invalid_config.application.speed_limit_rad_s = 0.01f;
        assert(MotorCommand_ValidateTarget(&invalid_config, &command, &foc, &encoder) ==
               MOTOR_CONTROL_OUT_OF_RANGE);
        MotorControlCommand excessive = command;
        if (i == 0U)
        {
            excessive.vd_v = FOC_MaximumVoltage(foc.bus_voltage_filtered_v);
        }
        else
        {
            excessive.id_a = cfg.application.current_limit_a;
        }
        assert(MotorCommand_ValidateTarget(&cfg, &excessive, &foc, &encoder) ==
               MOTOR_CONTROL_OUT_OF_RANGE);
        if (i == 0U)
        {
            command.id_a = 0.1f;
        }
        else
        {
            command.vd_v = 0.1f;
        }
        assert(MotorCommand_ValidateTarget(&cfg, &command, &foc, &encoder) ==
               MOTOR_CONTROL_INVALID);
        assert(MotorStateMachine_RequestRemoteStop());
        tick();
        assert(output_off && motor.control_mode == MOTOR_CONTROL_NONE &&
               motor.open_loop_frequency_hz == 0 && motor.open_loop_angle_rad == 0 &&
               motor.vd_target_v == 0 && motor.vq_target_v == 0 && motor.id_target_a == 0 &&
               motor.iq_target_a == 0);
    }
    for (unsigned i = 0U; i < 2U; ++i)
    {
        reset();
        cfg.device.encoder.calib_flag = 0U;
        encoder.bad_frame_streak = 1U;
        assert(MotorStateMachine_RequestEnable(modes[i]) == MOTOR_CONTROL_DENIED && output_off);
        reset();
        cfg.configuration_confirmed = false;
        assert(MotorStateMachine_RequestEnable(modes[i]) == MOTOR_CONTROL_DENIED && output_off);
        reset();
        foc.phase_a_current_a = NAN;
        assert(MotorStateMachine_RequestEnable(modes[i]) == MOTOR_CONTROL_DENIED && output_off);
        reset();
        MotorFault_Report(&motor, MOTOR_FAULT_OVERCURRENT);
        assert(MotorStateMachine_RequestEnable(modes[i]) == MOTOR_CONTROL_FAULT && output_off);
    }
    puts("PASS formal open-loop owner: uncalibrated/raw-online start, zero references, target "
         "mapping, bounds and STOP clear");
}

static void test_control(void)
{
    reset();
    trace_begin("test_control");
    assert(MotorStateMachine_RequestEnable(MOTOR_CONTROL_SPEED) == MOTOR_CONTROL_ACCEPTED);
    tick();
    tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_RUNNING && !output_off);
    MotorControlCommand goal = {
        .mode = MOTOR_CONTROL_SPEED, .speed_rad_s = -2.0f, .source_node = 2U, .sequence = 41U};
    MotorControlProgress progress;
    assert(MotorStateMachine_SubmitControl(&goal) == MOTOR_CONTROL_ACCEPTED);
    assert(MotorStateMachine_SubmitControl(&goal) == MOTOR_CONTROL_BUSY);
    MotorStateMachine_ReadControlProgress(&progress);
    assert(progress.state == MOTOR_CONTROL_PROGRESS_PENDING && progress.source_node == 2U &&
           progress.sequence == 41U);
    tick();
    assert(motor.speed_target_rad_s == -2.0f);
    MotorStateMachine_ReadControlProgress(&progress);
    assert(progress.state == MOTOR_CONTROL_PROGRESS_APPLIED && progress.sequence == 41U);
    goal.sequence = 42U;
    goal.speed_rad_s = 3.0f;
    assert(MotorStateMachine_SubmitControl(&goal) == MOTOR_CONTROL_ACCEPTED);
    assert(MotorStateMachine_RequestRemoteStop());
    MotorStateMachine_ReadControlProgress(&progress);
    assert(progress.state == MOTOR_CONTROL_PROGRESS_REJECTED &&
           progress.result == MOTOR_CONTROL_DENIED && progress.sequence == 42U);
    tick();
    assert(output_off && MotorStateMachine_GetState() == MOTOR_STATE_IDLE &&
           motor.speed_target_rad_s == 0.0f);
    assert(cfg.application.current_limit_a == 6.0f && cfg.application.speed_limit_rad_s == 20.0f);
    puts("PASS same-mode target owner, single-slot busy, STOP revocation and unchanged caps");

    reset();
    cfg.device.encoder.calib_flag = ENC_CALIB_LINEARIZED;
    assert(MotorStateMachine_RequestEnable(MOTOR_CONTROL_SPEED) == MOTOR_CONTROL_DENIED);
    assert(output_off && cfg.device.encoder.calib_flag == ENC_CALIB_LINEARIZED);
    puts("PASS partial calibration cannot start closed loop; flags unchanged");

    reset();
    assert(MotorStateMachine_RequestEnable(MOTOR_CONTROL_CURRENT) == MOTOR_CONTROL_ACCEPTED);
    tick();
    tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_RUNNING);
    goal = (MotorControlCommand){
        .mode = MOTOR_CONTROL_CURRENT, .iq_a = 1.0f, .source_node = 2U, .sequence = 43U};
    assert(MotorStateMachine_SubmitControl(&goal) == MOTOR_CONTROL_ACCEPTED);
    MotorStateMachine_PostOutcome((MotorTaskResult){MOTOR_TASK_FAILED, MOTOR_FAULT_ENCODER},
                                  MotorStateMachine_WorkEpoch());
    tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_FAULT &&
           motor.fault_code == MOTOR_FAULT_ENCODER && output_off);
    MotorStateMachine_ReadControlProgress(&progress);
    assert(progress.state == MOTOR_CONTROL_PROGRESS_REJECTED &&
           progress.result == MOTOR_CONTROL_FAULT && progress.sequence == 43U);
    assert(MotorStateMachine_RequestClear() == MOTOR_CONTROL_ACCEPTED);
    for (unsigned i = 0U; i < 4U; ++i)
    {
        tick();
    }
    assert(MotorStateMachine_GetState() == MOTOR_STATE_IDLE &&
           motor.fault_code == MOTOR_FAULT_NONE && motor.control_mode == MOTOR_CONTROL_NONE &&
           motor.speed_target_rad_s == 0.0f);
    puts("PASS worker fault latches error; fresh recovery CLEAR does not restart");
    trace_enabled = false;
}

/* 同源序号和STOP屏障只使用真实所有者入口，断言不通过私有状态伪造启相。 */
static void test_identified_speed_start(void)
{
    MotorControlStartRequest request = {
        .mode = MOTOR_CONTROL_SPEED, .sequence = 101U, .source_node = 2U};
    MotorControlStartProgress progress;

    reset();
    trace_begin("test_identified_speed_start");
    assert(MotorStateMachine_RequestIdentifiedStop(2U, 100U));
    tick();
    request.sequence = 99U;
    assert(MotorStateMachine_RequestIdentifiedEnable(&request) == MOTOR_CONTROL_STALE_SEQUENCE);
    assert(output_off && start_count == 0U);

    request.sequence = 101U;
    assert(MotorStateMachine_RequestIdentifiedEnable(&request) == MOTOR_CONTROL_ACCEPTED);
    assert(MotorStateMachine_ReadEnableProgress(2U, 101U, &progress));
    assert(progress.state == MOTOR_CONTROL_START_PENDING &&
           progress.result == MOTOR_CONTROL_ACCEPTED);
    assert(MotorStateMachine_RequestIdentifiedEnable(&request) == MOTOR_CONTROL_ACCEPTED);
    request.mode = MOTOR_CONTROL_CURRENT;
    assert(MotorStateMachine_RequestIdentifiedEnable(&request) == MOTOR_CONTROL_CONFLICTING_RETRY);
    request.mode = MOTOR_CONTROL_SPEED;
    assert(start_count == 0U);

    request.sequence = 102U;
    assert(MotorStateMachine_RequestIdentifiedEnable(&request) == MOTOR_CONTROL_BUSY);
    assert(MotorStateMachine_RequestIdentifiedEnable(&request) == MOTOR_CONTROL_BUSY);
    assert(MotorStateMachine_ReadEnableProgress(2U, 101U, &progress) &&
           progress.state == MOTOR_CONTROL_START_PENDING);
    /* 最新拒绝记录不能遮蔽仍活动的旧身份；重放不得创建第二次启动。 */
    MotorControlStartRequest active_retry = {
        .mode = MOTOR_CONTROL_SPEED, .sequence = 101U, .source_node = 2U};
    assert(MotorStateMachine_RequestIdentifiedEnable(&active_retry) == MOTOR_CONTROL_ACCEPTED);
    active_retry.mode = MOTOR_CONTROL_CURRENT;
    assert(MotorStateMachine_RequestIdentifiedEnable(&active_retry) ==
           MOTOR_CONTROL_CONFLICTING_RETRY);
    assert(!MotorStateMachine_ReadEnableProgress(3U, 101U, &progress) &&
           progress.state == MOTOR_CONTROL_START_NONE);
    tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_RUNNING && !output_off && start_count == 1U);
    assert(MotorStateMachine_ReadEnableProgress(2U, 101U, &progress) &&
           progress.state == MOTOR_CONTROL_START_STARTED);
    assert(MotorStateMachine_RequestIdentifiedEnable(&request) == MOTOR_CONTROL_BUSY);

    assert(MotorStateMachine_RequestIdentifiedStop(2U, 103U));
    assert(MotorStateMachine_RequestIdentifiedStop(2U, 98U));
    tick();
    assert(output_off && MotorStateMachine_GetState() == MOTOR_STATE_IDLE);
    request.sequence = 102U;
    assert(MotorStateMachine_RequestIdentifiedEnable(&request) == MOTOR_CONTROL_BUSY);
    request.sequence = 103U;
    assert(MotorStateMachine_RequestIdentifiedEnable(&request) == MOTOR_CONTROL_STALE_SEQUENCE);
    request.sequence = 104U;
    assert(MotorStateMachine_RequestIdentifiedEnable(&request) == MOTOR_CONTROL_ACCEPTED);
    assert(MotorStateMachine_RequestIdentifiedStop(2U, 105U));
    assert(MotorStateMachine_ReadEnableProgress(2U, 104U, &progress) &&
           progress.state == MOTOR_CONTROL_START_REJECTED &&
           progress.result == MOTOR_CONTROL_DENIED);
    tick();
    assert(output_off && start_count == 1U);
    assert(MotorStateMachine_RequestIdentifiedEnable(&request) == MOTOR_CONTROL_DENIED);

    request.sequence = 106U;
    assert(MotorStateMachine_RequestIdentifiedEnable(&request) == MOTOR_CONTROL_ACCEPTED);
    motor.fault_code = MOTOR_FAULT_ENCODER;
    MotorStateMachine_FastFaultStop();
    assert(MotorStateMachine_ReadEnableProgress(2U, 106U, &progress) &&
           progress.state == MOTOR_CONTROL_START_REJECTED &&
           progress.result == MOTOR_CONTROL_FAULT);
    assert(MotorStateMachine_RequestIdentifiedEnable(&request) == MOTOR_CONTROL_FAULT);
    assert(output_off && start_count == 1U);

    reset();
    request.sequence = UINT16_MAX;
    assert(MotorStateMachine_RequestIdentifiedStop(2U, request.sequence));
    tick();
    request.sequence = 0U;
    cfg.configuration_confirmed = false;
    assert(MotorStateMachine_RequestIdentifiedEnable(&request) == MOTOR_CONTROL_DENIED);
    cfg.configuration_confirmed = true;
    assert(MotorStateMachine_RequestIdentifiedEnable(&request) == MOTOR_CONTROL_DENIED);
    request.sequence = 1U;
    assert(MotorStateMachine_RequestIdentifiedEnable(&request) == MOTOR_CONTROL_ACCEPTED);
    assert(output_off && start_count == 0U);
    puts("PASS identified speed start: retry, conflict, busy, STOP, fault, wrap, rejected retry");
    trace_enabled = false;
}

/* 启动已受理时，后台不得借用配置并让监督中断误判资源仍被占用。 */

/* 后台发布必须保持原ticket、关相和独占工作区；STOP/故障不能被迟到完成覆盖。 */
static void test_lut_background_publication(void)
{
    for (unsigned scenario = 0U; scenario < 5U; ++scenario)
    {
        reset();
        uint32_t ticket = begin_maintenance(MOTOR_MAINTENANCE_ENCODER_LUT);
        uint32_t epoch = MotorStateMachine_WorkEpoch();
        assert(MaintenanceWorkspace_BorrowEncoder() != NULL);
        encoder_released = false;
        uint8_t previous_flags = cfg.device.encoder.calib_flag;
        MotorStateMachine_PostOutcome((MotorTaskResult){MOTOR_TASK_FINALIZE, MOTOR_FAULT_NONE},
                                      epoch);
        tick();
        assert(output_off && !MotorStateMachine_MaintenanceRunnable());
        assert(cfg.device.encoder.calib_flag == previous_flags);
        assert(!MaintenanceWorkspace_IsReleased());
        if (scenario == 1U)
        {
            assert(MotorStateMachine_RequestRemoteStop());
            tick();
            FocBackgroundClaim cancelled;
            assert(!MotorStateMachine_BeginBackground(&cancelled));
        }
        else
        {
            FocBackgroundClaim claim;
            assert(MotorStateMachine_BeginBackground(&claim));
            assert(claim.kind == FOC_BACKGROUND_LUT_PUBLISH && claim.epoch == epoch);
            if (scenario == 2U)
            {
                assert(MotorStateMachine_RequestRemoteStop());
            }
            if (scenario == 3U)
            {
                MotorFault_Report(&motor, MOTOR_FAULT_OVERVOLTAGE);
                MotorStateMachine_FastFaultStop();
            }
            lut_publish_fail = scenario == 4U;
            uint32_t mask = BspSystem_EnterCritical();
            bool published = MotorStateMachine_BackgroundValid(&claim) &&
                             MotorMaintenance_PublishLut(&cfg, &encoder);
            if (!published)
            {
                MotorMaintenance_Cancel(MOTOR_MAINTENANCE_ENCODER_LUT);
            }
            MotorStateMachine_FinishBackground(&claim, published);
            BspSystem_ExitCritical(mask);
            tick();
            if (scenario == 0U)
            {
                assert(published && cfg.device.encoder.calib_flag == ENC_CALIB_LINEARIZED);
                assert(MotorStateMachine_GetMaintenanceKind() == MOTOR_MAINTENANCE_SAVE);
            }
            else
            {
                assert(!published && cfg.device.encoder.calib_flag == previous_flags);
                assert(MotorStateMachine_GetMaintenanceKind() != MOTOR_MAINTENANCE_SAVE);
            }
            MotorStateMachine_FinishBackground(&claim, true);
        }
        assert(output_off && MaintenanceWorkspace_IsReleased());
        MotorMaintenanceProgress progress;
        assert(MotorStateMachine_ReadMaintenance(ticket, &progress));
        if (scenario == 0U)
        {
            assert(progress.result == MOTOR_MAINTENANCE_COMPLETED);
        }
        else
        {
            assert(progress.result != MOTOR_MAINTENANCE_COMPLETED);
        }
    }
    puts("PASS LUT background publication: OFF, original ticket, workspace, STOP/fault/failure");
}

static void test_calibration_save(void)
{
    MotorMaintenanceProgress progress;
    uint32_t ticket, origin, save;
    reset();
    trace_begin("test_calibration_save");
    cfg.device.encoder.calib_flag = ENC_CALIB_LINEARIZED;
    ticket = begin_maintenance(MOTOR_MAINTENANCE_ELECTRICAL_ZERO);
    assert((MotorStateMachine_GetMaintenanceKind() == MOTOR_MAINTENANCE_ELECTRICAL_ZERO) &&
           !output_off);
    origin = MotorStateMachine_WorkEpoch();
    cfg.device.encoder.calib_flag = ENC_CALIB_ALL;
    MotorStateMachine_PostOutcome((MotorTaskResult){MOTOR_TASK_COMPLETED, MOTOR_FAULT_NONE},
                                  origin);
    tick();
    assert(output_off && MotorStateMachine_ReadMaintenance(ticket, &progress));
    assert(progress.effects == MOTOR_MAINTENANCE_EFFECT_UNCOMMITTED);
    tick();
    assert((MotorStateMachine_GetMaintenanceKind() == MOTOR_MAINTENANCE_SAVE));
    save = MotorStateMachine_WorkEpoch();
    FocBackgroundClaim claim;
    assert(EncoderCalibrationTask_IsReleased());
    assert(MaintenanceWorkspace_BorrowEncoder() != NULL);
    assert(!MotorStateMachine_BeginBackground(&claim));
    assert(!MaintenanceWorkspace_IsReleased());
    MaintenanceWorkspace_ReleaseEncoder();
    assert(MotorStateMachine_BeginBackground(&claim) && claim.kind == FOC_BACKGROUND_SAVE);
    assert(MotorStateMachine_BackgroundValid(&claim));
    MotorStateMachine_FinishBackground(&claim, true);
    tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_IDLE);
    assert(MotorStateMachine_ReadMaintenance(save, &progress));
    assert(progress.result == MOTOR_MAINTENANCE_COMPLETED &&
           progress.effects == MOTOR_MAINTENANCE_EFFECT_COMMITTED);
    assert(cfg.device.encoder.calib_flag == ENC_CALIB_ALL);
    puts("PASS electrical-zero completion distinct from SAVE COMMITTED; derived ticket observable");

    reset();
    ticket = begin_maintenance(MOTOR_MAINTENANCE_ENCODER_LUT);
    assert((MotorStateMachine_GetMaintenanceKind() == MOTOR_MAINTENANCE_ENCODER_LUT));
    origin = MotorStateMachine_WorkEpoch();
    assert(MotorStateMachine_RequestRemoteStop());
    tick();
    MotorStateMachine_PostOutcome((MotorTaskResult){MOTOR_TASK_COMPLETED, MOTOR_FAULT_NONE},
                                  origin);
    tick();
    assert(motor.control_mode == MOTOR_CONTROL_NONE && output_off &&
           MotorStateMachine_GetState() == MOTOR_STATE_IDLE);
    puts(
        "PASS encoder-LUT explicit entry remains; stale completion after STOP cannot request SAVE");
    trace_enabled = false;
}

/* 采样 IRQ 停止而监督继续时，标定必须关相；停机 SAVE 不依赖快采样推进。 */
static void test_maintenance_sampling_loss(void)
{
    const MotorMaintenanceKind powered[] = {MOTOR_MAINTENANCE_ENCODER_LUT,
                                            MOTOR_MAINTENANCE_ELECTRICAL_ZERO};
    for (unsigned i = 0U; i < sizeof(powered) / sizeof(powered[0]); ++i)
    {
        reset();
        (void)begin_maintenance(powered[i]);
        tick();
        assert(MotorStateMachine_GetState() == MOTOR_STATE_MAINTENANCE && !output_off);
        unsigned starts = start_count;
        /* 故意不 PostOutcome：模拟 ADC2 没有新采样，TIM7 监督仍被调用。 */
        MotorStateMachine_Tick();
        assert(MotorStateMachine_GetState() == MOTOR_STATE_FAULT && output_off);
        assert(motor.fault_code != MOTOR_FAULT_NONE && motor.control_mode == MOTOR_CONTROL_NONE);
        tick();
        assert(MotorStateMachine_GetState() == MOTOR_STATE_FAULT && output_off);
        assert(start_count == starts && motor.iq_target_a == 0.0f);
    }
    reset();
    (void)begin_maintenance(MOTOR_MAINTENANCE_SAVE);
    MotorStateMachine_Tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_MAINTENANCE && output_off);
    assert(motor.fault_code == MOTOR_FAULT_NONE);
    puts("PASS maintenance sampling loss stops powered calibration; quiet SAVE remains valid");
}

static void test_stop_and_fault(void)
{
    reset();
    trace_begin("test_stop_and_fault");
    assert(MotorStateMachine_RequestEnable(MOTOR_CONTROL_CURRENT) == MOTOR_CONTROL_ACCEPTED);
    tick();
    tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_RUNNING && !output_off);
    encoder.bad_frame_streak = ENCODER_BAD_FRAME_OFFLINE_COUNT;
    MotorStateMachine_CheckFastFaults();
    MotorStateMachine_FastFaultStop();
    assert(output_off && motor.fault_code == MOTOR_FAULT_ENCODER);
    puts("PASS encoder offline fast check latches error and closes outputs");

    reset();
    assert(MotorStateMachine_RequestEnable(MOTOR_CONTROL_CURRENT) == MOTOR_CONTROL_ACCEPTED);
    tick();
    tick();
    stop_fails = true;
    assert(MotorStateMachine_RequestRemoteStop());
    tick();
    assert(!output_off && MotorStateMachine_GetState() == MOTOR_STATE_STOPPING);
    assert(cfg.application.current_limit_a == 6.0f && cfg.application.speed_limit_rad_s == 20.0f);
    puts("PASS failed output shutdown cannot restore IDLE or relaxed limits");
    trace_enabled = false;
}

static void test_borrow_cancel(void)
{
    FocBackgroundClaim claim, forged;
    MotorMaintenanceProgress progress;
    uint32_t ticket;
    reset();
    trace_begin("test_borrow_cancel");
    ticket = begin_maintenance(MOTOR_MAINTENANCE_SAVE);
    assert(MotorStateMachine_BeginBackground(&claim));
    assert(MotorStateMachine_RequestRemoteStop());
    tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_STOPPING &&
           MotorStateMachine_ReadMaintenance(ticket, &progress) &&
           progress.result == MOTOR_MAINTENANCE_CANCEL_REQUESTED);
    assert(!MotorStateMachine_BeginBackground(&forged));
    forged = claim;
    ++forged.epoch;
    MotorStateMachine_FinishBackground(&forged, true);
    tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_STOPPING);
    MotorStateMachine_FinishBackground(&claim, true);
    tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_IDLE &&
           MotorStateMachine_ReadMaintenance(ticket, &progress) &&
           progress.result == MOTOR_MAINTENANCE_CANCELLED);
    assert(motor.control_mode == MOTOR_CONTROL_NONE &&
           progress.effects != MOTOR_MAINTENANCE_EFFECT_COMMITTED);
    puts("PASS SAVE borrow blocks cancellation ACK; forged/late results cannot claim COMMITTED");
    trace_enabled = false;
}

/* 前台发布与持久化分别确认；整理失败不得派生保存。 */

/* 公共入口验证拒绝优先级；不注入运行所有者的私有状态。 */

/* 电压和反馈拒绝覆盖真实所有者，不再从旧分发器提取准入代码。 */

/* 固定线上编号作为外部参考，防止内部枚举变动改写 CAN 状态。 */
static void test_wire_projection(void)
{
    const uint8_t controls[] = {1U, 2U, 3U, 6U, 0U, 9U, 10U};
    const uint8_t maintenance[] = {7U, 7U, 0U, 7U, 8U, 8U, 0U, 0U};
    for (unsigned i = 0U; i < sizeof(controls); ++i)
    {
        assert(protocol_mode((MotorControlMode)i, MOTOR_MAINTENANCE_NONE) == controls[i]);
    }
    for (unsigned i = 0U; i < sizeof(maintenance); ++i)
    {
        assert(protocol_mode(MOTOR_CONTROL_NONE, (MotorMaintenanceKind)i) == maintenance[i]);
    }
    assert(protocol_mode((MotorControlMode)UINT32_MAX, MOTOR_MAINTENANCE_NONE) == 0U);
    /* v1.1 只收敛初始化反馈：INIT=3、IDLE=5，0 退役，其余线码不改。 */
    const uint8_t states[] = {3U, 5U, 2U, 6U, 5U, 7U, 8U, 9U};
    for (unsigned i = 0U; i < sizeof(states); ++i)
    {
        assert(protocol_state((MotorState)(MOTOR_STATE_INIT + i), 0U) == states[i]);
    }
    assert(protocol_state(MOTOR_STATE_MAINTENANCE, 7U) == 4U);
    assert(protocol_state(MOTOR_STATE_MAINTENANCE, 8U) == 4U);
    assert(protocol_state((MotorState)0, 0U) == 9U);
    puts("PASS fixed CAN status mode numbers preserved across control/task split");
}

static void test_preparation_readiness(void)
{
    for (unsigned flags = 0U; flags <= ENC_CALIB_ALL; ++flags)
    {
        reset();
        cfg.configuration_confirmed = false;
        cfg.device.encoder.calib_flag = (uint8_t)flags;
        MotorStateMachine_Init(&cfg, &motor, &foc, &pi, &encoder, &startup);
        for (unsigned i = 0U; i < 4U; ++i)
        {
            tick();
        }
        MotorReadiness readiness;
        MotorStateMachine_ReadReadiness(&readiness);
        assert(MotorStateMachine_GetState() == MOTOR_STATE_IDLE && output_off && start_count == 0U);
        assert(motor.fault_code == MOTOR_FAULT_NONE && MotorStateMachine_ConfigurationAllowed());
        assert(readiness.self_test_checks == MOTOR_SELF_TEST_ALL);
        assert(readiness.start_blockers & MOTOR_START_CONFIGURATION_REQUIRED);
        assert(readiness.startable_modes == 0U);
        for (unsigned mode = 0U; mode <= MOTOR_CONTROL_VQ; ++mode)
        {
            assert(MotorStateMachine_RequestEnable((MotorControlMode)mode) == MOTOR_CONTROL_DENIED);
        }
        uint32_t ticket;
        assert(MotorStateMachine_RequestMaintenance(MOTOR_MAINTENANCE_ENCODER_LUT, &ticket) ==
               MOTOR_CONTROL_DENIED);
        assert(ticket == 0U && motor.fault_code == MOTOR_FAULT_NONE && output_off);
        cfg.configuration_confirmed = true;
        MotorStateMachine_ReadReadiness(&readiness);
        assert(!(readiness.start_blockers & MOTOR_START_CONFIGURATION_REQUIRED));
        if (flags != ENC_CALIB_ALL)
        {
            assert(readiness.start_blockers & MOTOR_START_CALIBRATION_REQUIRED);
            assert(readiness.startable_modes == 0x60U);
        }
        else
        {
            assert(readiness.start_blockers == 0U && readiness.startable_modes == 0x6FU);
        }
        assert(output_off && MotorStateMachine_GetState() == MOTOR_STATE_IDLE && start_count == 0U);
    }
    reset();
    cfg.device.encoder.calib_flag = 0U;
    uint32_t ticket;
    assert(MotorStateMachine_RequestMaintenance(MOTOR_MAINTENANCE_ELECTRICAL_ZERO, &ticket) ==
           MOTOR_CONTROL_DENIED);
    assert(motor.fault_code == MOTOR_FAULT_NONE && ticket == 0U);
    assert(MotorStateMachine_RequestMaintenance(MOTOR_MAINTENANCE_ENCODER_LUT, &ticket) ==
           MOTOR_CONTROL_ACCEPTED);
    assert(MotorStateMachine_GetState() == MOTOR_STATE_MAINTENANCE);
    assert(MotorStateMachine_RequestRemoteStop());
    tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_IDLE && output_off);
    reset();
    foc.bus_voltage_v = foc.bus_voltage_filtered_v = 0.0f;
    MotorStateMachine_Init(&cfg, &motor, &foc, &pi, &encoder, &startup);
    for (unsigned i = 0U; i < 4U; ++i)
    {
        tick();
    }
    MotorReadiness readiness;
    MotorStateMachine_ReadReadiness(&readiness);
    assert(MotorStateMachine_GetState() == MOTOR_STATE_IDLE &&
           motor.fault_code == MOTOR_FAULT_NONE);
    assert(readiness.start_blockers & MOTOR_START_BUS_UNAVAILABLE);
    assert(MotorStateMachine_RequestMaintenance(MOTOR_MAINTENANCE_SAVE, &ticket) ==
           MOTOR_CONTROL_ACCEPTED);
    assert(output_off);
    puts("PASS unconfigured/partial-calibration IDLE, explicit mode admission and maintenance "
         "prerequisites");
}

/* 启动诊断和实际受理必须一致；多种准备缺口组合不能隐式启相或清错。 */
static void test_readiness_admission_matrix(void)
{
    uint64_t digest = UINT64_C(14695981039346656037);
    for (unsigned gaps = 0U; gaps < 512U; ++gaps)
    {
        for (unsigned mode = MOTOR_CONTROL_CURRENT; mode <= MOTOR_CONTROL_VQ; ++mode)
        {
            reset();
            cfg.configuration_confirmed = (gaps & 1U) == 0U;
            if (gaps & 2U)
            {
                cfg.device.encoder.calib_flag = 0U;
            }
            if (gaps & 4U)
            {
                encoder.bad_frame_streak = ENCODER_BAD_FRAME_OFFLINE_COUNT;
            }
            if (gaps & 8U)
            {
                foc.bus_voltage_v = foc.bus_voltage_filtered_v = 0.0f;
            }
            temperature_fresh = (gaps & 16U) == 0U;
            cfg.application.axis_profile_valid = (gaps & 32U) == 0U;
            motor.uses_sensorless_feedback = (gaps & 64U) != 0U;
            if (gaps & 128U)
            {
                encoder.mechanical_position_rad = NAN;
            }
            if (gaps & 256U)
            {
                MotorFault_Report(&motor, MOTOR_FAULT_OVERCURRENT);
            }
            MotorFaultCode fault = motor.fault_code;
            MotorReadiness readiness;
            MotorStateMachine_ReadReadiness(&readiness);
            bool allowed = (readiness.startable_modes & (1U << mode)) != 0U;
            MotorControlResult result = MotorStateMachine_RequestEnable((MotorControlMode)mode);
            assert((result == MOTOR_CONTROL_ACCEPTED) == allowed);
            assert(output_off && start_count == 0U && motor.fault_code == fault);
            assert(MotorStateMachine_GetState() == MOTOR_STATE_IDLE);
            const uint32_t observations[] = {readiness.start_blockers,
                                             readiness.self_test_checks,
                                             readiness.startable_modes,
                                             readiness.calibration_flags,
                                             (uint32_t)result};
            for (unsigned i = 0U; i < sizeof(observations) / sizeof(observations[0]); ++i)
            {
                for (unsigned byte = 0U; byte < 4U; ++byte)
                {
                    digest = (digest ^ (uint8_t)(observations[i] >> (8U * byte))) *
                             UINT64_C(1099511628211);
                }
            }
        }
    }
    printf("PASS 2048 readiness/admission combinations, digest %016llx\n",
           (unsigned long long)digest);
}

static void test_startup_offset_deadline(void)
{
    reset();
    cfg.device.phase_a_offset_counts = cfg.device.phase_b_offset_counts =
        cfg.device.phase_c_offset_counts = NAN;
    foc.phase_a_current_a = foc.phase_b_current_a = foc.phase_c_current_a = NAN;
    MotorStateMachine_Init(&cfg, &motor, &foc, &pi, &encoder, &startup);
    tick();
    now_ms = 500U;
    tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_INIT && output_off);
    assert(motor.fault_code == MOTOR_FAULT_NONE);
    assert(MotorStateMachine_RequestEnable(MOTOR_CONTROL_CURRENT) == MOTOR_CONTROL_DENIED);
    MotorReadiness readiness;
    MotorStateMachine_ReadReadiness(&readiness);
    assert((readiness.self_test_checks & MOTOR_SELF_TEST_CURRENTS_VALID) == 0U);
    assert(readiness.startable_modes == 0U && start_count == 0U);
    now_ms = 1499U;
    tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_INIT);
    now_ms = 1500U;
    tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_FAULT &&
           motor.fault_code == MOTOR_FAULT_SELF_TEST_TIMEOUT && output_off);

    /* 校正资格恢复后 CLEAR 直接待命，不重新初始化或启相。 */
    cfg.device.phase_a_offset_counts = cfg.device.phase_b_offset_counts =
        cfg.device.phase_c_offset_counts = 2048.0f;
    foc.phase_a_current_a = foc.phase_b_current_a = foc.phase_c_current_a = 0.0f;
    tick();
    assert(MotorStateMachine_RequestClear() == MOTOR_CONTROL_ACCEPTED);
    assert(MotorStateMachine_GetState() == MOTOR_STATE_IDLE && output_off && start_count == 0U);
    temperature_fresh = false;
    now_ms = 1999U;
    tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_IDLE && output_off);
    assert(MotorStateMachine_RequestEnable(MOTOR_CONTROL_CURRENT) == MOTOR_CONTROL_DENIED);
    now_ms = 2000U;
    tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_IDLE &&
           motor.fault_code == MOTOR_FAULT_NONE && output_off);

    reset();
    cfg.device.phase_a_offset_counts = cfg.device.phase_b_offset_counts =
        cfg.device.phase_c_offset_counts = NAN;
    foc.phase_a_current_a = foc.phase_b_current_a = foc.phase_c_current_a = NAN;
    MotorStateMachine_Init(&cfg, &motor, &foc, &pi, &encoder, &startup);
    tick();
    now_ms = 1000U;
    cfg.device.phase_a_offset_counts = cfg.device.phase_b_offset_counts =
        cfg.device.phase_c_offset_counts = 2048.0f;
    foc.phase_a_current_a = foc.phase_b_current_a = foc.phase_c_current_a = 0.0f;
    for (unsigned i = 0U; i < 4U; ++i)
    {
        tick();
    }
    assert(MotorStateMachine_GetState() == MOTOR_STATE_IDLE && output_off && start_count == 0U);
    puts("PASS offset-pending admission and 1500ms boot deadline; healthy CLEAR directly IDLE");
}

static void test_self_test_deadline(void)
{
    reset();
    MotorStateMachine_Init(&cfg, &motor, &foc, &pi, &encoder, &startup);
    tick();
    tick();
    tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_INIT);
    MotorStateMachine_Tick();
    tick();
    tick();
    tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_INIT);
    tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_IDLE && start_count == 0U);
    /* 第四个健康完成拍已到期限，必须先拒绝过期成功。 */
    reset();
    MotorStateMachine_Init(&cfg, &motor, &foc, &pi, &encoder, &startup);
    tick();
    tick();
    tick();
    now_ms = 500U;
    tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_FAULT &&
           motor.fault_code == MOTOR_FAULT_SELF_TEST_TIMEOUT && output_off);
    /* BOOT 资源未释放与必需编码器不健康也不能无限等待。 */
    reset();
    MotorStateMachine_Init(&cfg, &motor, &foc, &pi, &encoder, &startup);
    encoder_released = false;
    tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_INIT);
    now_ms = 500U;
    tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_FAULT &&
           motor.fault_code == MOTOR_FAULT_SELF_TEST_TIMEOUT && output_off);
    reset();
    MotorStateMachine_Init(&cfg, &motor, &foc, &pi, &encoder, &startup);
    encoder.bad_frame_streak = 1U;
    tick();
    now_ms = 500U;
    tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_FAULT &&
           motor.fault_code == MOTOR_FAULT_SELF_TEST_TIMEOUT && output_off);
    reset();
    MotorStateMachine_Init(&cfg, &motor, &foc, &pi, &encoder, &startup);
    now_ms = UINT32_MAX - 100U;
    uint32_t began = now_ms;
    MotorStateMachine_Tick();
    now_ms = began + 499U;
    MotorStateMachine_Tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_INIT && output_off);
    now_ms = began + 500U;
    MotorStateMachine_Tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_FAULT &&
           motor.fault_code == MOTOR_FAULT_SELF_TEST_TIMEOUT && output_off);
    tick();
    assert(MotorStateMachine_RequestClear() == MOTOR_CONTROL_ACCEPTED);
    for (unsigned i = 0U; i < 4U; ++i)
    {
        tick();
    }
    assert(MotorStateMachine_GetState() == MOTOR_STATE_IDLE && output_off);
    reset();
    MotorStateMachine_Init(&cfg, &motor, &foc, &pi, &encoder, &startup);
    output_off = false;
    stop_fails = true;
    tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_FATAL && start_count == 0U);
    assert(MotorStateMachine_RequestClear() != MOTOR_CONTROL_ACCEPTED);
    reset();
    foc.bus_voltage_v = BUS_VOLTAGE_HARD_OVERVOLTAGE_V;
    MotorStateMachine_CheckFastFaults();
    MotorStateMachine_FastFaultStop();
    tick();
    assert(motor.fault_code == MOTOR_FAULT_OVERVOLTAGE &&
           MotorStateMachine_GetState() == MOTOR_STATE_FAULT && output_off);
    puts("PASS continuous self-test window, wrap-safe deadline, explicit recovery and failed "
         "shutdown FATAL");
}

/* INIT／IDLE STOP 保持状态与代际，撤销排队意图；资源占用不产生停止绕行。 */
static void test_quiet_init_and_idle_stop(void)
{
    reset();
    MotorStateMachine_Init(&cfg, &motor, &foc, &pi, &encoder, &startup);
    assert(MotorStateMachine_GetState() == MOTOR_STATE_INIT);
    for (unsigned i = 0U; i < 4U; ++i)
    {
        assert(MotorStateMachine_RequestIdentifiedStop(2U, (uint16_t)(100U + i)));
        tick();
        assert(output_off && start_count == 0U);
        assert(MotorStateMachine_GetState() == (i == 3U ? MOTOR_STATE_IDLE : MOTOR_STATE_INIT));
    }
    uint32_t epoch = MotorStateMachine_WorkEpoch();
    assert(MotorStateMachine_RequestIdentifiedStop(2U, 104U));
    tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_IDLE && output_off);
    assert(MotorStateMachine_WorkEpoch() == epoch);
    MotorControlStartRequest old = {
        .source_node = 2U, .sequence = 104U, .mode = MOTOR_CONTROL_SPEED};
    assert(MotorStateMachine_RequestIdentifiedEnable(&old) == MOTOR_CONTROL_STALE_SEQUENCE);

    reset();
    MotorStateMachine_Init(&cfg, &motor, &foc, &pi, &encoder, &startup);
    temperature_fresh = false;
    tick();
    const uint32_t stop_times_ms[] = {99U, 199U, 499U, 500U};
    for (unsigned i = 0U; i < sizeof(stop_times_ms) / sizeof(stop_times_ms[0]); ++i)
    {
        now_ms = stop_times_ms[i];
        assert(MotorStateMachine_RequestRemoteStop());
        tick();
        assert(MotorStateMachine_GetState() == (i == 3U ? MOTOR_STATE_FAULT : MOTOR_STATE_INIT));
    }
    assert(motor.fault_code == MOTOR_FAULT_SELF_TEST_TIMEOUT && output_off && start_count == 0U);

    reset();
    cfg.device.phase_a_offset_counts = cfg.device.phase_b_offset_counts =
        cfg.device.phase_c_offset_counts = NAN;
    MotorStateMachine_Init(&cfg, &motor, &foc, &pi, &encoder, &startup);
    tick();
    now_ms = 1499U;
    assert(MotorStateMachine_RequestRemoteStop());
    tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_INIT);
    now_ms = 1500U;
    assert(MotorStateMachine_RequestRemoteStop());
    tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_FAULT && output_off);
    assert(motor.fault_code == MOTOR_FAULT_SELF_TEST_TIMEOUT);

    reset();
    MotorStateMachine_Init(&cfg, &motor, &foc, &pi, &encoder, &startup);
    encoder_released = false;
    assert(MotorStateMachine_RequestRemoteStop());
    tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_INIT && output_off);
    tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_INIT);
    encoder_released = true;
    tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_INIT && output_off);
    for (unsigned i = 0U; i < 4U; ++i)
    {
        tick();
    }
    assert(MotorStateMachine_GetState() == MOTOR_STATE_IDLE && start_count == 0U);
    /* INIT 只有一个期限；晚归还不能恢复旧 BOOT 完成时的第二次计时。 */
    reset();
    MotorStateMachine_Init(&cfg, &motor, &foc, &pi, &encoder, &startup);
    encoder_released = false;
    tick();
    now_ms = 499U;
    encoder_released = true;
    tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_INIT);
    now_ms = 500U;
    tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_FAULT && output_off);
    assert(motor.fault_code == MOTOR_FAULT_SELF_TEST_TIMEOUT);
    puts("PASS INIT/IDLE STOP stays in place with unchanged epoch, deadlines and sequence barrier; "
         "INIT waits for resource release");
}

/* 完成后不把最近任务误当作活动会话，但保留旧故障路径的定向清理行为。 */

/* 跨状态/故障/输出与数值边界比较拒绝原因，保护校验职责拆分后的优先级。 */
static void test_target_admission_boundaries(void)
{
    static const float values[] = {NAN,
                                   INFINITY,
                                   -INFINITY,
                                   -100.0f,
                                   -20.001f,
                                   -20.0f,
                                   -6.001f,
                                   -6.0f,
                                   -1.001f,
                                   -1.0f,
                                   0.0f,
                                   1.0f,
                                   1.001f,
                                   6.0f,
                                   6.001f,
                                   20.0f,
                                   20.001f,
                                   100.0f};
    uint64_t digest = UINT64_C(14695981039346656037);
    for (unsigned mode = 0U; mode < 4U; ++mode)
    {
        for (unsigned scenario = 0U; scenario < 16U; ++scenario)
        {
            for (unsigned sample = 0U; sample < 20U; ++sample)
            {
                reset();
                if (scenario != 1U)
                {
                    assert(MotorStateMachine_RequestEnable((MotorControlMode)mode) ==
                           MOTOR_CONTROL_ACCEPTED);
                    tick();
                    tick();
                }
                switch (scenario)
                {
                case 2U:
                    motor.fault_code = MOTOR_FAULT_ENCODER;
                    break;
                case 3U:
                    output_off = true;
                    break;
                case 4U:
                    motor.control_mode = (MotorControlMode)((mode + 1U) % 4U);
                    break;
                case 5U:
                    encoder.bad_frame_streak = 1U;
                    break;
                case 6U:
                    cfg.device.encoder.calib_flag = 0U;
                    break;
                case 7U:
                    motor.uses_sensorless_feedback = true;
                    break;
                case 8U:
                    cfg.application.axis_profile_valid = false;
                    break;
                case 9U:
                    foc.bus_voltage_filtered_v = 25.5f;
                    break;
                case 10U:
                    cfg.application.current_limit_a = NAN;
                    break;
                case 11U:
                    cfg.application.speed_limit_rad_s = NAN;
                    break;
                case 12U:
                    cfg.application.axis_profile.magic = MOTOR_AXIS_PROFILE_NUMERIC_MAGIC;
                    cfg.application.axis_profile.minimum_position_rad = -1.0f;
                    cfg.application.axis_profile.maximum_position_rad = 1.0f;
                    cfg.application.axis_profile.maximum_speed_rad_s = 2.0f;
                    break;
                case 13U:
                    encoder.mechanical_position_rad = NAN;
                    break;
                case 14U:
                    assert(MotorStateMachine_RequestBootStop(UINT64_C(17)) ==
                           MOTOR_CONTROL_ACCEPTED);
                    break;
                default:
                    break;
                }
                MotorControlCommand command = {.mode = (MotorControlMode)mode};
                float value = values[sample < 18U ? sample : 10U];
                switch (mode)
                {
                case MOTOR_CONTROL_CURRENT:
                    command.iq_a = value;
                    break;
                case MOTOR_CONTROL_SPEED:
                    command.speed_rad_s = value;
                    break;
                case MOTOR_CONTROL_POSITION:
                    command.position_rad = value;
                    break;
                case MOTOR_CONTROL_VQ:
                    command.vq_v = value;
                    break;
                }
                if (sample == 18U)
                {
                    /* 非当前模式字段非零是格式错误，不能被故障或越界结果掩盖。 */
                    if (mode == MOTOR_CONTROL_CURRENT)
                    {
                        command.speed_rad_s = 1.0f;
                    }
                    else
                    {
                        command.iq_a = 1.0f;
                    }
                }
                else if (sample == 19U)
                {
                    command.mode = (MotorControlMode)UINT32_MAX;
                }
                MotorControlResult result =
                    MotorStateMachine_SubmitControl(scenario == 15U ? NULL : &command);
                const uint32_t observations[] = {result,
                                                 MotorStateMachine_GetState(),
                                                 motor.fault_code,
                                                 output_off,
                                                 start_count};
                for (unsigned item = 0U; item < 5U; ++item)
                {
                    for (unsigned byte = 0U; byte < 4U; ++byte)
                    {
                        digest = (digest ^ (uint8_t)(observations[item] >> (8U * byte))) *
                                 UINT64_C(1099511628211);
                    }
                }
            }
        }
    }
    assert(digest == UINT64_C(0x9f89d0ff40a00225));
    printf("PASS 1280 target admission boundaries, digest %016llx\n", (unsigned long long)digest);
}

/* 恢复在 FAULT 内核对；直达 IDLE 不绕回初始化，保持新请求才启相。 */
static void test_direct_fault_recovery_and_idle_stop(void)
{
    reset();
    trace_begin("test_direct_fault_recovery_and_idle_stop");
    uint32_t epoch = MotorStateMachine_WorkEpoch();
    assert(MotorStateMachine_RequestEnable(MOTOR_CONTROL_SPEED) == MOTOR_CONTROL_ACCEPTED);
    assert(MotorStateMachine_RequestRemoteStop());
    tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_IDLE && output_off && start_count == 0U);
    assert(MotorStateMachine_WorkEpoch() == epoch);
    encoder_released = false;
    assert(MotorStateMachine_RequestRemoteStop());
    tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_IDLE && output_off);
    assert(MotorStateMachine_WorkEpoch() == epoch);
    encoder_released = true;
    MotorFault_Report(&motor, MOTOR_FAULT_ENCODER);
    tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_FAULT);
    temperature_fresh = false;
    tick();
    assert(MotorStateMachine_RequestClear() == MOTOR_CONTROL_DENIED);
    temperature_fresh = true;
    encoder.bad_frame_streak = 1U;
    tick();
    assert(MotorStateMachine_RequestClear() == MOTOR_CONTROL_DENIED);
    encoder.bad_frame_streak = 0U;
    output_off = false;
    assert(MotorStateMachine_RequestClear() == MOTOR_CONTROL_DENIED);
    output_off = true;
    encoder_released = false;
    assert(MotorStateMachine_RequestClear() == MOTOR_CONTROL_BUSY);
    encoder_released = true;
    tick();
    assert(MotorStateMachine_RequestClear() == MOTOR_CONTROL_ACCEPTED);
    assert(MotorStateMachine_GetState() == MOTOR_STATE_IDLE && output_off && start_count == 0U);
    assert(motor.fault_code == MOTOR_FAULT_NONE && motor.control_mode == MOTOR_CONTROL_NONE);
    assert(MotorStateMachine_RequestEnable(MOTOR_CONTROL_CURRENT) == MOTOR_CONTROL_ACCEPTED);
    tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_RUNNING && start_count == 1U);
    reset();
    MotorStateMachine_Init(&cfg, &motor, &foc, &pi, &encoder, &startup);
    output_off = false;
    stop_fails = true;
    assert(MotorStateMachine_RequestRemoteStop());
    tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_FATAL && start_count == 0U);
    assert(MotorStateMachine_RequestClear() != MOTOR_CONTROL_ACCEPTED);
    assert(MotorStateMachine_RequestRemoteStop());
    tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_FATAL);
    puts("PASS pending start and busy idle STOP stay IDLE; healthy CLEAR directly IDLE; FATAL "
         "stays latched");
    trace_enabled = false;
}

/* 模拟叶算法已经发布结果，验证真实所有者的串联、身份和最终保存边界。 */

/* 首次拒绝不绑定来源，也不能把来源 A 的序号拿来约束来源 B。 */
static void test_rejected_start_does_not_bind_source(void)
{
    for (unsigned pass = 0U; pass < 1U; ++pass)
    {
        for (unsigned faulted = 0U; faulted < 2U; ++faulted)
        {
            reset();
            uint32_t epoch = MotorStateMachine_WorkEpoch();
            if (faulted)
            {
                MotorFault_Report(&motor, MOTOR_FAULT_ENCODER);
                tick();
                epoch = MotorStateMachine_WorkEpoch();
            }
            else
            {
                cfg.configuration_confirmed = false;
            }
            MotorControlStartRequest rejected = {
                .source_node = 2U, .sequence = 100U, .mode = MOTOR_CONTROL_SPEED};
            MotorControlResult expected = faulted ? MOTOR_CONTROL_FAULT : MOTOR_CONTROL_DENIED;
            MotorControlResult actual = MotorStateMachine_RequestIdentifiedEnable(&rejected);
            assert(actual == expected && output_off && start_count == 0U);
            assert(MotorStateMachine_WorkEpoch() == epoch);
            MotorControlStartProgress progress;
            assert(MotorStateMachine_ReadEnableProgress(2U, 100U, &progress));
            assert(progress.state == MOTOR_CONTROL_START_REJECTED && progress.result == expected);
            actual = MotorStateMachine_RequestIdentifiedEnable(&rejected);
            assert(actual == expected && MotorStateMachine_WorkEpoch() == epoch);
            cfg.configuration_confirmed = true;
            if (faulted)
            {
                assert(MotorStateMachine_RequestClear() == MOTOR_CONTROL_ACCEPTED);
            }
            MotorControlStartRequest legal = {
                .source_node = 3U, .sequence = 1U, .mode = MOTOR_CONTROL_SPEED};
            actual = MotorStateMachine_RequestIdentifiedEnable(&legal);
            assert(actual == MOTOR_CONTROL_ACCEPTED);
            tick();
            tick();
            assert(MotorStateMachine_GetState() == MOTOR_STATE_RUNNING && start_count == 1U);
            MotorFault_Report(&motor, MOTOR_FAULT_ENCODER);
            tick();
            legal.sequence = 2U;
            actual = MotorStateMachine_RequestIdentifiedEnable(&legal);
            assert(actual == MOTOR_CONTROL_FAULT);
            assert(MotorStateMachine_RequestClear() == MOTOR_CONTROL_ACCEPTED);
            rejected.sequence = 101U;
            actual = MotorStateMachine_RequestIdentifiedEnable(&rejected);
            assert(actual == MOTOR_CONTROL_DENIED);
            legal.sequence = 3U;
            actual = MotorStateMachine_RequestIdentifiedEnable(&legal);
            assert(actual == MOTOR_CONTROL_ACCEPTED);
        }
    }
    puts("PASS rejected source A keeps identity result; lower-sequence source B can bind; fault "
         "rejection preserves B");
}

/* 热模型用真实三相窗口推进，主状态机须拒绝危险策略并显式清错，不自动重启。 */
static void test_thermal_owner_admission_and_clear(void)
{
    reset();
    cfg.model.thermal.heating_tau_s = 0.1f;
    assert(MotorThermalControl_Configure(&cfg.model.thermal, true));
    for (unsigned window = 0U; window < 100U && MotorThermalControl_CanRun(false); ++window)
    {
        ++now_ms;
        for (unsigned sample = 0U; sample < 20U; ++sample)
        {
            MotorThermalControl_Sample(6.0f, -3.0f, -3.0f, true, now_ms);
        }
    }
    assert(MotorThermalControl_CanRun(true) && !MotorThermalControl_CanRun(false));
    MotorReadiness readiness;
    MotorStateMachine_ReadReadiness(&readiness);
    assert((readiness.startable_modes & (1U << MOTOR_CONTROL_SPEED)) != 0U);
    assert((readiness.startable_modes & (1U << MOTOR_CONTROL_VQ)) == 0U);
    assert(MotorStateMachine_RequestEnable(MOTOR_CONTROL_VQ) == MOTOR_CONTROL_DENIED);
    assert(motor.control_mode == MOTOR_CONTROL_NONE && output_off && start_count == 0U);
    uint32_t ticket;
    assert(MotorStateMachine_RequestMaintenance(MOTOR_MAINTENANCE_ENCODER_LUT, &ticket) ==
           MOTOR_CONTROL_DENIED);
    assert(output_off && motor.fault_code == MOTOR_FAULT_NONE);
    assert(MotorStateMachine_RequestEnable(MOTOR_CONTROL_SPEED) == MOTOR_CONTROL_ACCEPTED);
    tick();
    tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_RUNNING && start_count == 1U);
    for (unsigned window = 0U; window < 100U && MotorThermalControl_CanRun(true); ++window)
    {
        ++now_ms;
        for (unsigned sample = 0U; sample < 20U; ++sample)
        {
            MotorThermalControl_Sample(6.0f, -3.0f, -3.0f, false, now_ms);
        }
    }
    assert(!MotorThermalControl_CanRun(true));
    MotorFault_Report(&motor, MOTOR_FAULT_WINDING_THERMAL);
    tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_FAULT && output_off);
    assert_boot_health(true);
    assert(MotorStateMachine_RequestClear() == MOTOR_CONTROL_DENIED);
    for (unsigned window = 0U; window < 120000U && !MotorThermalControl_CanClearFault(); ++window)
    {
        ++now_ms;
        for (unsigned sample = 0U; sample < 20U; ++sample)
        {
            MotorThermalControl_Sample(0.0f, 0.0f, 0.0f, true, now_ms);
        }
    }
    motor.thermal_clear_ready = MotorThermalControl_CanClearFault();
    assert(motor.thermal_clear_ready);
    tick();
    MotorThermalDiagnostics before, after;
    assert(MotorThermalControl_GetDiagnostics(&before));
    assert(MotorStateMachine_RequestClear() == MOTOR_CONTROL_ACCEPTED);
    assert(MotorThermalControl_GetDiagnostics(&after));
    assert(before.model.max_heat == after.model.max_heat && after.model.max_heat > 0.0f);
    assert(!after.model.fault_latched && motor.fault_code == MOTOR_FAULT_NONE);
    assert(MotorStateMachine_GetState() == MOTOR_STATE_IDLE && output_off && start_count == 1U);
    puts("PASS real thermal owner: hot Vq/LUT denied, speed derating allowed, trip STOP and cooled "
         "CLEAR preserve heat without restart");
}

/* 正常监督轮询必须允许快故障抢占，且不能用旧配置结果跨越状态代际。 */
static void test_preemptible_calibration_validation(void)
{
#if STATE_CALIBRATION_GUARD_PREVALIDATED
    const MotorMaintenanceKind kinds[] = {MOTOR_MAINTENANCE_ENCODER_LUT,
                                          MOTOR_MAINTENANCE_ELECTRICAL_ZERO};
    for (unsigned i = 0U; i < sizeof(kinds) / sizeof(kinds[0]); ++i)
    {
        for (unsigned interrupt = 1U; interrupt <= 3U; ++interrupt)
        {
            reset();
            (void)begin_maintenance(kinds[i]);
            unsigned starts = start_count;
            validation_unmasked_calls = validation_masked_calls = 0U;
            validation_interrupt = interrupt;
            tick();
            assert(validation_interrupt == 0U && validation_unmasked_calls == 1U);
            assert(!require_locked_validation && output_off && critical_depth == 0U);
            assert(start_count == starts);
            if (interrupt == 1U)
            {
                assert(motor.fault_code == MOTOR_FAULT_OVERVOLTAGE);
            }
            tick();
            assert(output_off && start_count == starts);
        }
    }
#endif
}

static void test_preemptible_running_validation(void)
{
#if STATE_VALIDATION_PREEMPTIBLE
    for (unsigned interrupt = 1U; interrupt <= 3U; ++interrupt)
    {
        reset();
        assert(MotorStateMachine_RequestEnable(MOTOR_CONTROL_SPEED) == MOTOR_CONTROL_ACCEPTED);
        tick();
        assert(MotorStateMachine_GetState() == MOTOR_STATE_RUNNING && !output_off);
        validation_unmasked_calls = validation_masked_calls = 0U;
        validation_interrupt = interrupt;
        tick();
        assert(validation_interrupt == 0U && validation_unmasked_calls == 1U);
        assert(!require_locked_validation);
        if (!output_off || motor.iq_target_a != 0.0f)
        {
            fprintf(stderr,
                    "validation interrupt=%u output_off=%u iq=%g state=%u fault=%u\n",
                    interrupt,
                    output_off,
                    (double)motor.iq_target_a,
                    (unsigned)MotorStateMachine_GetState(),
                    (unsigned)motor.fault_code);
        }
        assert(output_off && motor.iq_target_a == 0.0f);
        if (interrupt == 1U)
        {
            assert(motor.fault_code == MOTOR_FAULT_OVERVOLTAGE);
            assert(MotorStateMachine_GetState() == MOTOR_STATE_FAULT);
        }
        else
        {
            tick();
            assert(output_off && MotorStateMachine_GetState() != MOTOR_STATE_RUNNING);
        }
        assert(critical_depth == 0U);
    }
#endif
}

/* 准备完成后的纯配置校验可被抢占，但过期准备和快故障绝不能启相。 */
static void test_preemptible_start_validation(void)
{
#if STATE_VALIDATION_PREEMPTIBLE
    for (unsigned interrupt = 1U; interrupt <= 2U; ++interrupt)
    {
        reset();
        outer_ready = false;
        assert(MotorStateMachine_RequestEnable(MOTOR_CONTROL_SPEED) == MOTOR_CONTROL_ACCEPTED);
        tick();
        assert(MotorStateMachine_GetState() == MOTOR_STATE_STARTING && output_off);
        outer_ready = true;
        validation_unmasked_calls = 0U;
        validation_interrupt = interrupt;
        tick();
        assert(validation_interrupt == 0U && validation_unmasked_calls == 1U);
        assert(output_off && start_count == 0U);
        tick();
        assert(MotorStateMachine_GetState() ==
               (interrupt == 1U ? MOTOR_STATE_FAULT : MOTOR_STATE_IDLE));
    }
#endif
}

/* 两入口只在稳定关相IDLE解锁校验；动态拒绝和跨维护配置不能复用旧结果。 */
static void test_start_admission_prevalidation(void)
{
#if STATE_START_ADMISSION_PREVALIDATED
    trace_enabled = false;
    for (unsigned identified = 0U; identified < 2U; ++identified)
    {
        reset();
        unsigned masked_before = configuration_masked_calls;
        unsigned unmasked_before = configuration_unmasked_calls;
        MotorControlStartRequest request = {
            .mode = MOTOR_CONTROL_SPEED, .source_node = 7U, .sequence = 10U};
        MotorControlResult result = identified ? MotorStateMachine_RequestIdentifiedEnable(&request)
                                               : MotorStateMachine_RequestEnable(request.mode);
        assert(result == MOTOR_CONTROL_ACCEPTED && output_off && start_count == 0U);
        assert(configuration_masked_calls == masked_before &&
               configuration_unmasked_calls == unmasked_before + 1U);
        for (unsigned interrupt = 1U; interrupt <= (identified ? 6U : 5U); ++interrupt)
        {
            reset();
            masked_before = configuration_masked_calls;
            uint32_t epoch = MotorStateMachine_WorkEpoch();
            start_admission_interrupt = interrupt;
            result = identified ? MotorStateMachine_RequestIdentifiedEnable(&request)
                                : MotorStateMachine_RequestEnable(request.mode);
            assert(start_admission_interrupt == 0U && !require_locked_validation &&
                   critical_depth == 0U && output_off && start_count == 0U);
            MotorControlResult expected = interrupt == 1U   ? MOTOR_CONTROL_FAULT
                                          : interrupt == 3U ? MOTOR_CONTROL_ACCEPTED
                                          : interrupt == 6U ? MOTOR_CONTROL_STALE_SEQUENCE
                                                            : MOTOR_CONTROL_DENIED;
            assert(result == expected);
            if (interrupt == 3U || interrupt == 4U)
            {
                assert(MotorStateMachine_WorkEpoch() != epoch);
            }
            if (interrupt == 4U)
            {
                assert(!cfg.configuration_confirmed && cfg.device.encoder.calib_flag == 0U &&
                       leaf_calls[MOTOR_MAINTENANCE_DEFAULTS] == 1U);
            }
            if (interrupt == 5U)
            {
                assert(configuration_masked_calls == masked_before);
            }
            if (identified && interrupt != 6U)
            {
                MotorControlStartProgress progress;
                assert(MotorStateMachine_ReadEnableProgress(7U, 10U, &progress));
                assert(progress.result == expected &&
                       progress.state == (expected == MOTOR_CONTROL_ACCEPTED
                                              ? MOTOR_CONTROL_START_PENDING
                                              : MOTOR_CONTROL_START_REJECTED));
            }
        }
        reset();
        uint32_t ticket;
        assert(MotorStateMachine_RequestMaintenance(MOTOR_MAINTENANCE_DEFAULTS, &ticket) ==
               MOTOR_CONTROL_ACCEPTED);
        unmasked_before = configuration_unmasked_calls;
        masked_before = configuration_masked_calls;
        result = identified ? MotorStateMachine_RequestIdentifiedEnable(&request)
                            : MotorStateMachine_RequestEnable(request.mode);
        assert(result == MOTOR_CONTROL_BUSY && configuration_unmasked_calls == unmasked_before &&
               configuration_masked_calls == masked_before + 1U);
    }
#endif
}

/* ENABLE和STARTING空守卫只解锁配置；动态事件与过期监督载体不能启相或重复提交。 */
static void test_preemptible_start_supervisor_validation(void)
{
#if STATE_START_SUPERVISOR_PREVALIDATED
    trace_enabled = false;
    for (unsigned starting = 0U; starting < 2U; ++starting)
    {
        reset();
        outer_ready = false;
        assert(MotorStateMachine_RequestEnable(MOTOR_CONTROL_SPEED) == MOTOR_CONTROL_ACCEPTED);
        if (starting)
        {
            tick();
        }
        unsigned masked_before = configuration_masked_calls;
        unsigned unmasked_before = configuration_unmasked_calls;
        uint32_t before, after;
        (void)MotorStateMachine_ReadBootHealth(&before);
        tick();
        (void)MotorStateMachine_ReadBootHealth(&after);
        assert(after == before + 1U && configuration_masked_calls == masked_before &&
               configuration_unmasked_calls == unmasked_before + 2U);
        assert(MotorStateMachine_GetState() == MOTOR_STATE_STARTING && output_off);
        for (unsigned interrupt = 1U; interrupt <= 4U; ++interrupt)
        {
            reset();
            outer_ready = false;
            assert(MotorStateMachine_RequestEnable(MOTOR_CONTROL_SPEED) == MOTOR_CONTROL_ACCEPTED);
            if (starting)
            {
                tick();
            }
            outer_ready = true;
            (void)MotorStateMachine_ReadBootHealth(&before);
            masked_before = configuration_masked_calls;
            validation_unmasked_calls = 0U;
            validation_interrupt = interrupt;
            tick();
            (void)MotorStateMachine_ReadBootHealth(&after);
            assert(validation_interrupt == 0U && validation_unmasked_calls == 1U &&
                   !require_locked_validation && configuration_masked_calls > masked_before);
            assert(after == before + (interrupt == 4U ? 4U : 1U));
            assert(output_off && start_count == 0U && critical_depth == 0U);
            assert(MotorStateMachine_GetState() ==
                   (interrupt == 1U ? MOTOR_STATE_FAULT : MOTOR_STATE_IDLE));
            if (interrupt == 4U)
            {
                assert(!cfg.configuration_confirmed && cfg.device.encoder.calib_flag == 0U &&
                       leaf_calls[MOTOR_MAINTENANCE_DEFAULTS] == 1U);
            }
        }
    }
    reset();
    uint32_t ticket;
    assert(MotorStateMachine_RequestMaintenance(MOTOR_MAINTENANCE_DEFAULTS, &ticket) ==
           MOTOR_CONTROL_ACCEPTED);
    unsigned unmasked_before = configuration_unmasked_calls;
    tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_MAINTENANCE &&
           configuration_unmasked_calls == unmasked_before);
#endif
}

/* 固定基线仍做完整扫描；当前健康查询只消费原输入谓词，不改变布尔资格。 */
static void assert_boot_health(bool expected)
{
    unsigned validation_before = configuration_validation_calls;
    uint32_t progress = UINT32_MAX;
    assert(MotorStateMachine_ReadBootHealth(&progress) == expected);
    assert(progress != UINT32_MAX && critical_depth == 0U);
#if STATE_BOOT_HEALTH_INPUTS_ONLY
    assert(configuration_validation_calls == validation_before);
#else
    assert(configuration_validation_calls == validation_before + 1U);
#endif
}

static void test_boot_health_inputs(void)
{
    reset();
    uint32_t progress, next;
    assert_boot_health(true);
    assert(MotorStateMachine_ReadBootHealth(&progress) && progress != 0U);
    assert(MotorStateMachine_ReadBootHealth(&next) && next == progress);
    tick();
    assert(MotorStateMachine_ReadBootHealth(&next) && next != progress);
    MotorStateMachine_Tick();
    assert_boot_health(false);
    tick();
    float *measurements[] = {&foc.bus_voltage_v,
                             &foc.bus_voltage_filtered_v,
                             &foc.phase_a_current_a,
                             &foc.phase_b_current_a,
                             &foc.phase_c_current_a,
                             &foc.temperature_c};
    const float invalid_values[] = {NAN, INFINITY, -INFINITY};
    for (unsigned field = 0U; field < sizeof(measurements) / sizeof(measurements[0]); ++field)
    {
        float saved = *measurements[field];
        for (unsigned value = 0U; value < sizeof(invalid_values) / sizeof(invalid_values[0]);
             ++value)
        {
            *measurements[field] = invalid_values[value];
            assert_boot_health(false);
        }
        *measurements[field] = saved;
    }
    temperature_fresh = false;
    assert_boot_health(false);
    temperature_fresh = true;
    assert_boot_health(true);
    MotorFault_Report(&motor, MOTOR_FAULT_OVERVOLTAGE);
    tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_FAULT);
    cfg.application.current_limit_a = NAN;
    assert(!ParameterValue_ConfigurationValid(&cfg));
    assert_boot_health(true);
    reset();
    MotorStateMachine_Init(&cfg, &motor, &foc, &pi, &encoder, &startup);
    output_off = false;
    stop_fails = true;
    tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_FATAL);
    assert_boot_health(false);
    unsigned validation_before = configuration_validation_calls;
    assert(!MotorStateMachine_ReadBootHealth(NULL));
    assert(configuration_validation_calls == validation_before && critical_depth == 0U);
}

/* 维护身份入口不能用外层临界区重新包住完整配置扫描。 */
static void test_maintenance_admission_prevalidation(void)
{
#if STATE_MAINTENANCE_ADMISSION_PREVALIDATED
    trace_enabled = false;
    for (unsigned identified = 0U; identified < 2U; ++identified)
    {
        for (unsigned interrupt = 0U; interrupt <= 3U; ++interrupt)
        {
            reset();
            uint32_t ticket = 0U;
            MotorMaintenanceRequest request = {
                .kind = MOTOR_MAINTENANCE_ENCODER_LUT, .source_node = 7U, .sequence = 10U};
            unsigned unmasked = configuration_unmasked_calls;
            unsigned masked = configuration_masked_calls;
            validation_interrupt = interrupt < 3U ? interrupt : 0U;
            start_admission_interrupt = interrupt == 3U ? 4U : 0U;
            MotorControlResult result =
                identified ? MotorStateMachine_RequestIdentifiedMaintenance(&request, &ticket)
                           : MotorStateMachine_RequestMaintenance(request.kind, &ticket);
            assert(critical_depth == 0U);
            assert(interrupt == 3U ? configuration_unmasked_calls > unmasked
                                   : configuration_unmasked_calls == unmasked + 1U);
            assert(validation_interrupt == 0U);
            if (interrupt == 0U)
            {
                assert(result == MOTOR_CONTROL_ACCEPTED && ticket != 0U);
#if STATE_FAST_POWER_OWNER
                assert(output_off);
                MotorStateMachine_BeginFastTick();
#endif
                assert(!output_off);
                assert(configuration_masked_calls == masked);
            }
            else
            {
                if (result == MOTOR_CONTROL_ACCEPTED || ticket != 0U || !output_off)
                {
                    fprintf(stderr,
                            "maintenance injection=%u identified=%u result=%u ticket=%u off=%u\n",
                            interrupt,
                            identified,
                            (unsigned)result,
                            ticket,
                            (unsigned)output_off);
                }
                assert(result != MOTOR_CONTROL_ACCEPTED && ticket == 0U && output_off);
                assert(start_count == 0U);
            }
        }
    }
#endif
}

#if STATE_FAST_POWER_OWNER
/* 单拍契约：命令受理绝不启相；STOP、故障和取消可以撤销未消费代际。 */
static void test_fast_power_handoff(void)
{
    for (unsigned cancel = 0U; cancel < 4U; ++cancel)
    {
        reset();
        uint32_t ticket;
        assert(MotorStateMachine_RequestMaintenance(MOTOR_MAINTENANCE_ELECTRICAL_ZERO, &ticket) ==
               MOTOR_CONTROL_ACCEPTED);
        assert(output_off && start_count == 0U && !MotorStateMachine_MaintenanceRunnable());
        if (cancel == 1U)
        {
            assert(MotorStateMachine_RequestRemoteStop());
        }
        else if (cancel == 2U)
        {
            MotorFault_Report(&motor, MOTOR_FAULT_OVERCURRENT);
        }
        else if (cancel == 3U)
        {
            assert(MotorStateMachine_CancelMaintenance(ticket) == MOTOR_CONTROL_ACCEPTED);
        }
        MotorStateMachine_BeginFastTick();
        assert(cancel == 0U ? !output_off && start_count == 1U : output_off && start_count == 0U);
    }
    reset();
    assert(MotorStateMachine_RequestEnable(MOTOR_CONTROL_SPEED) == MOTOR_CONTROL_ACCEPTED);
    MotorStateMachine_PostOutcome((MotorTaskResult){MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE},
                                  MotorStateMachine_WorkEpoch());
    MotorStateMachine_Tick();
    assert(output_off && start_count == 0U && MotorStateMachine_GetState() == MOTOR_STATE_STARTING);
    MotorStateMachine_BeginFastTick();
    assert(!output_off && start_count == 1U &&
           MotorStateMachine_GetState() == MOTOR_STATE_STARTING);
    MotorStateMachine_PostOutcome((MotorTaskResult){MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE},
                                  MotorStateMachine_WorkEpoch());
    MotorStateMachine_Tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_RUNNING && start_count == 1U);
    puts("PASS fast power handoff: acceptance OFF, fast enable, supervisor confirmation, "
         "STOP/fault/cancel");
}
#endif

int main(void)
{
#if STATE_FAST_POWER_OWNER
    test_fast_power_handoff();
#endif
    test_maintenance_admission_prevalidation();
    test_preemptible_calibration_validation();
    test_preemptible_running_validation();
    test_preemptible_start_validation();
    test_start_admission_prevalidation();
    test_preemptible_start_supervisor_validation();
    test_defaults_preserves_device_address();
    test_rejected_start_does_not_bind_source();
    test_direct_fault_recovery_and_idle_stop();
    test_target_admission_boundaries();
    test_preparation_readiness();
    test_readiness_admission_matrix();
    test_self_test_deadline();
    test_quiet_init_and_idle_stop();
    test_startup_offset_deadline();
    test_boot_health_inputs();
    test_wire_projection();
    test_identified_speed_start();
    test_open_loop_owner();
    test_control();
    test_calibration_save();
    test_lut_background_publication();
    test_borrow_cancel();
    test_maintenance_sampling_loss();
    test_stop_and_fault();
    test_thermal_owner_admission_and_clear();
#if STATE_SUPERVISOR_STAGED
    test_staged_supervisor();
#endif
    assert(critical_depth == 0U);
    return 0;
}

/* 调度真实维护分发，只有叶功能替换为桩；任务结果没有全局模式副作用。 */
MotorTaskResult EncoderCalibrationTask_StepLut(const MotorConfiguration *configuration,
                                               EncoderCalibration *calibration,
                                               FocState *f,
                                               MotorControlState *m,
                                               PiController *p,
                                               EncoderState *e,
                                               FluxObserverState *o,
                                               SensorlessStartupState *s,
                                               EncoderCalibrationWorkspace *workspace)
{
    assert(configuration == &cfg && calibration == &cfg.device.encoder);
    (void)workspace;
    (void)f;
    (void)m;
    (void)p;
    (void)e;
    (void)o;
    (void)s;
    ++leaf_calls[MOTOR_MAINTENANCE_ENCODER_LUT];
    return (MotorTaskResult){MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};
}
MotorTaskResult EncoderCalibrationTask_StepElectricalZero(const MotorConfiguration *configuration,
                                                          EncoderCalibration *calibration,
                                                          FocState *f,
                                                          MotorControlState *m,
                                                          EncoderState *e)
{
    assert(configuration == &cfg && calibration == &cfg.device.encoder);
    (void)f;
    (void)m;
    (void)e;
    ++leaf_calls[MOTOR_MAINTENANCE_ELECTRICAL_ZERO];
    return (MotorTaskResult){MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};
}

bool Encoder_SetMechanicalZero(EncoderCalibration *calibration, EncoderState *e)
{
    assert(calibration == &cfg.device.encoder);
    (void)e;
    ++leaf_calls[MOTOR_MAINTENANCE_MECHANICAL_ZERO];
    return true;
}
void MotorOuterLoop_RequestReset(void)
{
}
void Param_Return_Default(MotorConfiguration *configuration)
{
    /* 保留生产默认值接口的本次相关副作用；实际参数记录另由装载夹具验证。 */
    MotorWindingThermal_Default(&configuration->model.thermal);
    configuration->application.speed_feedforward =
        (SpeedFeedforwardConfig)PARAM_APP_FEEDFORWARD_INITIALIZER;
    configuration->device.node_id = 1U;
    configuration->configuration_confirmed = false;
    configuration->device.encoder.calib_flag = 0U;
    ++leaf_calls[MOTOR_MAINTENANCE_DEFAULTS];
}

#if STATE_SUPERVISOR_STAGED
/* 真实Axis调度与真实owner；这里仅替换计算/传感器边界来注入中途事件。 */
static unsigned supervisor_stage, supervisor_outer_calls, supervisor_injection;
static bool supervisor_ready;
static float supervisor_target_seen;
MotorControlMode MotorAxis_SupervisorTick(void);

void Encoder_UpdateSlowEstimate(const MotorConfiguration *configuration, EncoderState *state)
{
    assert(configuration == &cfg && state == &encoder && supervisor_stage == 0U);
    supervisor_stage = 1U;
}
void Temperature_Update(FocState *state, MotorControlState *control)
{
    assert(state == &foc && control == &motor && supervisor_stage == 1U);
    supervisor_stage = 2U;
    if (supervisor_injection == 4U)
    {
        MotorFault_Report(&motor, MOTOR_FAULT_OVERTEMPERATURE);
    }
}
void MotorOuterLoop_SlowTick(const MotorConfiguration *configuration,
                             MotorControlState *candidate,
                             PiController *controller,
                             EncoderState *state)
{
    assert(configuration == &cfg && state == &encoder && supervisor_stage == 2U);
    assert(candidate != &motor && controller != &pi && critical_depth == 0U);
    assert(motor.fault_code == MOTOR_FAULT_NONE);
    assert(MotorStateMachine_GetState() == MOTOR_STATE_STARTING ||
           MotorStateMachine_GetState() == MOTOR_STATE_RUNNING);
    ++supervisor_outer_calls;
    supervisor_stage = 3U;
    supervisor_target_seen = candidate->speed_target_rad_s;
    float published_iq = motor.iq_target_a;
    candidate->id_target_a = 0.0f;
    candidate->iq_target_a = candidate->speed_target_rad_s * 0.1f;
    controller->integral_output = 7.0f;
    outer_ready = supervisor_ready;
    assert(motor.iq_target_a == published_iq);
    if (supervisor_injection == 1U)
    {
        MotorFault_Report(&motor, MOTOR_FAULT_OVERCURRENT);
        MotorStateMachine_FastFaultStop();
    }
    else if (supervisor_injection == 2U)
    {
        assert(MotorStateMachine_RequestRemoteStop());
    }
    else if (supervisor_injection == 3U)
    {
        MotorFault_Report(candidate, MOTOR_FAULT_INVALID_PARAMETERS);
    }
}
static void staged_axis_tick(void)
{
    /* 模拟新鲜快采样窗口；真实热owner跨reset保留历史，不伪造初始化清零。 */
    MotorThermalDiagnostics diagnostics;
    assert(MotorThermalControl_GetDiagnostics(&diagnostics));
    for (unsigned sample = 0U; sample < diagnostics.model.config.samples_per_update; ++sample)
    {
        MotorThermalControl_Sample(0.0f, 0.0f, 0.0f, output_off, now_ms);
    }
    /* 此业务测试助手等待实际启动确认；独立用例核对单拍延迟。 */
    for (unsigned phase = 0U; phase < 2U; ++phase)
    {
        supervisor_stage = 0U;
        MotorStateMachine_PostOutcome((MotorTaskResult){MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE},
                                      MotorStateMachine_WorkEpoch());
        uint32_t before, after;
        (void)MotorStateMachine_ReadBootHealth(&before);
        (void)MotorAxis_SupervisorTick();
        (void)MotorStateMachine_ReadBootHealth(&after);
        assert(after == before + 1U && critical_depth == 0U);
        assert(supervisor_stage == 2U || supervisor_stage == 3U);
#if STATE_FAST_POWER_OWNER
        MotorStateMachine_BeginFastTick();
        if (MotorStateMachine_GetState() != MOTOR_STATE_STARTING || output_off)
        {
            break;
        }
#else
        break;
#endif
    }
}
static void test_staged_supervisor(void)
{
    trace_enabled = false;
    reset();
    supervisor_outer_calls = supervisor_injection = 0U;
    supervisor_ready = outer_ready = false;
    assert(MotorStateMachine_RequestEnable(MOTOR_CONTROL_SPEED) == MOTOR_CONTROL_ACCEPTED);
    staged_axis_tick();
    assert(supervisor_outer_calls == 1U && start_count == 0U && output_off);
    assert(MotorStateMachine_GetState() == MOTOR_STATE_STARTING);
    supervisor_ready = true;
    staged_axis_tick();
    assert(start_count == 1U && !output_off);
    MotorControlCommand command = {.mode = MOTOR_CONTROL_SPEED, .speed_rad_s = 3.5f};
    assert(MotorStateMachine_SubmitControl(&command) == MOTOR_CONTROL_ACCEPTED);
    staged_axis_tick();
    if (supervisor_target_seen != 3.5f || fabsf(motor.iq_target_a - 0.35f) >= 1e-6f)
    {
        fprintf(stderr,
                "staged target=%f iq=%f state=%u fault=%u ready=%u calls=%u\n",
                (double)supervisor_target_seen,
                (double)motor.iq_target_a,
                (unsigned)MotorStateMachine_GetState(),
                (unsigned)motor.fault_code,
                outer_ready,
                supervisor_outer_calls);
    }
    assert(supervisor_target_seen == 3.5f && fabsf(motor.iq_target_a - 0.35f) < 1e-6f);
    assert(pi.integral_output == 7.0f);
#if STATE_PUBLICATION_PREVALIDATED
    /* 配置校验在反馈计算前完成；发布前仍由真实规则复核动态事实。 */
    MotorSupervisorStep publication_step;
    MotorStateMachine_PostOutcome((MotorTaskResult){MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE},
                                  MotorStateMachine_WorkEpoch());
    assert(MotorStateMachine_PrepareSupervisor(&publication_step));
    assert(publication_step.configuration_valid);
    unsigned validations_before_publication = configuration_validation_calls;
    publication_step.calculated = true;
    publication_step.motor.iq_target_a = 0.25f;
    MotorStateMachine_CommitSupervisor(&publication_step);
    assert(configuration_validation_calls == validations_before_publication);
    assert(motor.iq_target_a == 0.25f && !output_off);
#endif
    unsigned calls_before = supervisor_outer_calls;
    assert(MotorStateMachine_RequestRemoteStop());
    staged_axis_tick();
    assert(output_off && motor.iq_target_a == 0.0f && supervisor_outer_calls == calls_before);

    for (unsigned injection = 1U; injection <= 4U; ++injection)
    {
        reset();
        supervisor_injection = 0U;
        supervisor_ready = true;
        assert(MotorStateMachine_RequestEnable(MOTOR_CONTROL_SPEED) == MOTOR_CONTROL_ACCEPTED);
        staged_axis_tick();
        assert(MotorStateMachine_SubmitControl(&command) == MOTOR_CONTROL_ACCEPTED);
        calls_before = supervisor_outer_calls;
        supervisor_injection = injection;
        staged_axis_tick();
        assert(output_off && motor.iq_target_a == 0.0f);
        assert(supervisor_outer_calls == calls_before + (injection == 4U ? 0U : 1U));
        assert(MotorStateMachine_GetState() ==
               (injection == 2U ? MOTOR_STATE_IDLE : MOTOR_STATE_FAULT));
    }
    supervisor_injection = 0U;
    for (MotorControlMode mode = MOTOR_CONTROL_CURRENT; mode <= MOTOR_CONTROL_VQ; ++mode)
    {
        if (mode == MOTOR_CONTROL_SPEED || mode == MOTOR_CONTROL_POSITION)
        {
            continue;
        }
        reset();
        supervisor_ready = false;
        assert(MotorStateMachine_RequestEnable(mode) == MOTOR_CONTROL_ACCEPTED);
        staged_axis_tick();
        assert(MotorStateMachine_GetState() == MOTOR_STATE_RUNNING && !output_off);
        assert(!outer_ready && motor.iq_target_a == 0.0f);
    }

    /* INIT稳定窗口每拍只进一次；重复提交不增加监督epoch。 */
    reset();
    MotorStateMachine_Init(&cfg, &motor, &foc, &pi, &encoder, &startup);
    for (unsigned tick_index = 0U; tick_index < 4U; ++tick_index)
    {
        staged_axis_tick();
        assert(MotorStateMachine_GetState() ==
               (tick_index < 3U ? MOTOR_STATE_INIT : MOTOR_STATE_IDLE));
    }
    MotorSupervisorStep old_step;
    MotorStateMachine_PostOutcome((MotorTaskResult){MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE},
                                  MotorStateMachine_WorkEpoch());
    assert(!MotorStateMachine_PrepareSupervisor(&old_step));
    MotorStateMachine_CommitSupervisor(&old_step);
    uint32_t first, duplicate;
    (void)MotorStateMachine_ReadBootHealth(&first);
    MotorStateMachine_CommitSupervisor(&old_step);
    (void)MotorStateMachine_ReadBootHealth(&duplicate);
    assert(first == duplicate);

    /* 旧计算跨过停机/新启动后，不能覆盖新的控制代际。 */
    assert(MotorStateMachine_RequestEnable(MOTOR_CONTROL_SPEED) == MOTOR_CONTROL_ACCEPTED);
    supervisor_ready = true;
    staged_axis_tick();
    MotorStateMachine_PostOutcome((MotorTaskResult){MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE},
                                  MotorStateMachine_WorkEpoch());
    assert(MotorStateMachine_PrepareSupervisor(&old_step));
    old_step.calculated = true;
    old_step.motor.iq_target_a = 123.0f;
    assert(MotorStateMachine_RequestRemoteStop());
    staged_axis_tick();
    assert(MotorStateMachine_RequestEnable(MOTOR_CONTROL_SPEED) == MOTOR_CONTROL_ACCEPTED);
    staged_axis_tick();
    float new_iq = motor.iq_target_a;
    (void)MotorStateMachine_ReadBootHealth(&first);
    MotorStateMachine_CommitSupervisor(&old_step);
    (void)MotorStateMachine_ReadBootHealth(&duplicate);
    assert(first == duplicate && motor.iq_target_a == new_iq && !output_off);
    /* 同一监督拍也须拒绝不匹配的状态代际，不能只依赖拍序号。 */
    MotorStateMachine_PostOutcome((MotorTaskResult){MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE},
                                  MotorStateMachine_WorkEpoch());
    assert(MotorStateMachine_PrepareSupervisor(&old_step));
    old_step.calculated = true;
    ++old_step.state_epoch;
    old_step.motor.iq_target_a = 123.0f;
    MotorStateMachine_CommitSupervisor(&old_step);
    assert(motor.iq_target_a == new_iq && !output_off);

    reset();
    supervisor_ready = false;
    assert(MotorStateMachine_RequestEnable(MOTOR_CONTROL_POSITION) == MOTOR_CONTROL_ACCEPTED);
    staged_axis_tick();
    assert(output_off && MotorStateMachine_GetState() == MOTOR_STATE_STARTING);
    supervisor_ready = true;
    staged_axis_tick();
    assert(!output_off && MotorStateMachine_GetState() == MOTOR_STATE_RUNNING);
}
#endif

uint32_t BspSystem_EnterControlCritical(void)
{
    return BspSystem_EnterCritical();
}
void BspSystem_ExitControlCritical(uint32_t state)
{
    BspSystem_ExitCritical(state);
}
