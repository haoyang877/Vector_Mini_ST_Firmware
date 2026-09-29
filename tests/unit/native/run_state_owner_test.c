#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "motor_state_machine.h"
#include "motor_maintenance.h"
#include "foc_cogging_calibration.h"
#include "encoder_calibration_task.h"
#include "friction_identification_task.h"
#include "phase_resistance_task.h"
#include "foc_errhandle.h"
#include "motor_execution.h"
#include "foc_sensing.h"
#include "foc_sensorless_run.h"
#include "power_stage_hw.h"
#include "critical_hw.h"
#include "indicator_hw.h"
#include "bus_voltage_profile.h"
#include "utils.h"
#define FOC_MAX_MODULATION (0.95f * 0.866025403f)

/* 真实生产所有者和类型；仅替换硬件与 worker，验证命令、资源和身份交接。 */
static MotorControlState motor;
static FocState foc;
static PiController pi;
static EncoderState encoder;
static SensorlessStartupState startup;
static bool output_off, stop_fails, start_fails, temperature_fresh, encoder_released;
static unsigned start_count, leaf_calls[8];
static bool cogging_active, cogging_pending, cogging_claimed, cogging_cancelled;
static bool outer_ready;
static bool check_borrowed_requests;
static uint32_t critical_depth;
static uint8_t protocol_mode(MotorControlMode mode, MotorMaintenanceKind kind);

/* 本夹具模拟普通 APP 的无 Loader 端口，配置修订号仍有可用空间。 */
bool MotorAxis_ConfigurationRevisionAvailable(void)
{
    return true;
}

uint32_t critical_hw_enter(void)
{
    return critical_depth++;
}
void critical_hw_exit(uint32_t mask)
{
    assert(critical_depth == mask + 1U);
    critical_depth = mask;
}
bool power_stage_hw_phases_disabled(void)
{
    return output_off;
}
void power_stage_hw_start(void)
{
    assert(critical_depth != 0U);
    ++start_count;
    if (!start_fails)
    {
        output_off = false;
    }
}
void power_stage_hw_stop(void)
{
    if (!stop_fails)
    {
        output_off = true;
    }
}
void indicator_hw_set_led(bool error, uint8_t count)
{
    (void)error;
    (void)count;
}
bool Encoder_IsOnline(const EncoderState *e)
{
    return e->bad_frame_streak == 0U;
}
uint16_t Encoder_GetBadFrameStreak(const EncoderState *e)
{
    return e->bad_frame_streak;
}
uint8_t Encoder_GetCalibFlag(const EncoderState *e)
{
    return e->calib_flag;
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
EncoderCalibrationPhase EncoderCalibrationTask_GetStep(void)
{
    return encoder_released ? CS_NULL : CS_OBS_ALIGN_ORIGIN;
}
void PhaseResistanceTask_Cancel(FocState *f, MotorControlState *m)
{
    (void)f;
    (void)m;
}
bool PhaseResistanceTask_IsReleased(void)
{
    return true;
}
void FrictionIdentificationTask_Abort(MotorControlState *m, PiController *p)
{
    (void)m;
    (void)p;
}
bool FrictionIdentificationTask_IsReleased(void)
{
    return true;
}
bool FocCogging_CanStart(const MotorControlState *m, const EncoderState *e)
{
    return m->axis_profile_valid && e->calib_flag == ENC_CALIB_ALL;
}
void FocCogging_Abort(void)
{
    if (cogging_claimed)
    {
        cogging_cancelled = true;
    }
    else
    {
        cogging_active = cogging_pending = false;
    }
}
bool FocCogging_IsReleased(void)
{
    return !cogging_active && !cogging_pending && !cogging_claimed;
}
bool FocCogging_NeedsFinalize(void)
{
    return cogging_pending;
}
bool FocCogging_BeginFinalize(void)
{
    if (!cogging_pending || cogging_claimed)
    {
        return false;
    }
    cogging_pending = false;
    cogging_claimed = true;
    return true;
}
void FocCogging_ReleaseFinalize(bool published)
{
    (void)published;
    cogging_claimed = false;
}
void FocCogging_SaveResult(bool success)
{
    (void)success;
}
void FocCogging_Service(MotorControlState *m, EncoderState *e)
{
    (void)m;
    (void)e;
    if (check_borrowed_requests)
    {
        uint32_t ticket = UINT32_MAX;
        assert(MotorStateMachine_RequestEnable(MOTOR_CONTROL_SPEED) == MOTOR_CONTROL_BUSY);
        assert(MotorStateMachine_RequestClear() == MOTOR_CONTROL_BUSY);
        assert(MotorStateMachine_RequestMaintenance(MOTOR_MAINTENANCE_SAVE, &ticket) ==
               MOTOR_CONTROL_BUSY);
        assert(ticket == 0U);
        check_borrowed_requests = false;
    }
}

static void tick(void)
{
    MotorStateMachine_PostOutcome((MotorTaskResult){MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE},
                                  MotorStateMachine_WorkEpoch());
    MotorStateMachine_Tick();
}

static void reset(void)
{
    memset(&motor, 0, sizeof(motor));
    memset(&foc, 0, sizeof(foc));
    memset(&encoder, 0, sizeof(encoder));
    motor.axis_profile_valid = true;
    motor.motor_pole_pairs = 21;
    motor.current_limit_a = 6.0f;
    motor.speed_limit_rad_s = 20.0f;
    motor.speed_acceleration_rad_s2 = motor.speed_deceleration_rad_s2 = 1.0f;
    motor.speed_kp = 0.1f;
    motor.speed_ki = 0.2f;
    motor.phase_a_offset_counts = motor.phase_b_offset_counts = motor.phase_c_offset_counts =
        2048.0f;
    encoder.calib_flag = ENC_CALIB_ALL;
    foc.bus_voltage_v = foc.bus_voltage_filtered_v = 30.0f;
    foc.temperature_c = 25.0f;
    output_off = outer_ready = temperature_fresh = encoder_released = true;
    start_fails = false;
    start_count = 0U;
    memset(leaf_calls, 0, sizeof(leaf_calls));
    stop_fails = cogging_active = cogging_pending = cogging_claimed = cogging_cancelled = false;
    MotorStateMachine_Init(&motor, &foc, &pi, &encoder, &startup);
    tick();
    assert(MotorStateMachine_GetState() == APP_READY);
}

/* 以公共维护入口开启会话并返回创建时 ticket；不再经由 SWD 台架邮箱。 */
static uint32_t begin_maintenance(MotorMaintenanceKind kind)
{
    uint32_t ticket = 0U;
    assert(MotorStateMachine_RequestMaintenance(kind, &ticket) == MOTOR_CONTROL_ACCEPTED);
    assert(ticket != 0U);
    return ticket;
}

static void test_control(void)
{
    reset();
    assert(MotorStateMachine_RequestEnable(MOTOR_CONTROL_SPEED) == MOTOR_CONTROL_ACCEPTED);
    tick();
    tick();
    assert(MotorStateMachine_GetState() == APP_RUNNING && !output_off);
    MotorControlCommand goal = {.mode = MOTOR_CONTROL_SPEED,
                                .speed_rad_s = -2.0f,
                                .source_node = 2U,
                                .sequence = 41U};
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
    assert(output_off && MotorStateMachine_GetState() == APP_READY &&
           motor.speed_target_rad_s == 0.0f);
    assert(motor.current_limit_a == 6.0f && motor.speed_limit_rad_s == 20.0f);
    puts("PASS same-mode target owner, single-slot busy, STOP revocation and unchanged caps");

    reset();
    encoder.calib_flag = ENC_CALIB_LINEARIZED;
    assert(MotorStateMachine_RequestEnable(MOTOR_CONTROL_SPEED) == MOTOR_CONTROL_DENIED);
    assert(output_off && encoder.calib_flag == ENC_CALIB_LINEARIZED);
    puts("PASS partial calibration cannot start closed loop; flags unchanged");

    reset();
    assert(MotorStateMachine_RequestEnable(MOTOR_CONTROL_CURRENT) == MOTOR_CONTROL_ACCEPTED);
    tick();
    tick();
    assert(MotorStateMachine_GetState() == APP_RUNNING);
    goal = (MotorControlCommand){.mode = MOTOR_CONTROL_CURRENT,
                                .iq_a = 1.0f,
                                .source_node = 2U,
                                .sequence = 43U};
    assert(MotorStateMachine_SubmitControl(&goal) == MOTOR_CONTROL_ACCEPTED);
    MotorStateMachine_PostOutcome((MotorTaskResult){MOTOR_TASK_FAILED, MOTOR_FAULT_ENCODER},
                                  MotorStateMachine_WorkEpoch());
    tick();
    assert(MotorStateMachine_GetState() == APP_FAULT && motor.fault_code == MOTOR_FAULT_ENCODER &&
           output_off);
    MotorStateMachine_ReadControlProgress(&progress);
    assert(progress.state == MOTOR_CONTROL_PROGRESS_REJECTED &&
           progress.result == MOTOR_CONTROL_FAULT && progress.sequence == 43U);
    assert(MotorStateMachine_RequestClear() == MOTOR_CONTROL_ACCEPTED);
    tick();
    assert(MotorStateMachine_GetState() == APP_READY && motor.fault_code == MOTOR_FAULT_NONE &&
           motor.control_mode == MOTOR_CONTROL_NONE && motor.speed_target_rad_s == 0.0f);
    puts("PASS worker fault latches error; fresh recovery CLEAR does not restart");
}

static void test_calibration_save(void)
{
    MotorMaintenanceProgress progress;
    uint32_t ticket, origin, save;
    reset();
    encoder.calib_flag = ENC_CALIB_LINEARIZED;
    ticket = begin_maintenance(MOTOR_MAINTENANCE_ELECTRICAL_ZERO);
    assert(MotorStateMachine_IsMaintenanceActive(MOTOR_MAINTENANCE_ELECTRICAL_ZERO) && !output_off);
    origin = MotorStateMachine_WorkEpoch();
    encoder.calib_flag = ENC_CALIB_ALL;
    MotorStateMachine_PostOutcome((MotorTaskResult){MOTOR_TASK_COMPLETED, MOTOR_FAULT_NONE},
                                  origin);
    tick();
    assert(output_off && MotorStateMachine_ReadMaintenance(ticket, &progress));
    assert(progress.effects == APP_EFFECT_UNCOMMITTED);
    tick();
    assert(MotorStateMachine_IsMaintenanceActive(MOTOR_MAINTENANCE_SAVE));
    save = MotorStateMachine_WorkEpoch();
    FocBackgroundClaim claim;
    assert(MotorStateMachine_BeginBackground(&claim) && claim.kind == FOC_BACKGROUND_SAVE);
    assert(MotorStateMachine_BackgroundValid(&claim));
    MotorStateMachine_FinishBackground(&claim, true);
    tick();
    assert(MotorStateMachine_GetState() == APP_READY);
    assert(MotorStateMachine_ReadMaintenance(save, &progress));
    assert(progress.result == APP_OPERATION_COMPLETED && progress.effects == APP_EFFECT_COMMITTED);
    assert(encoder.calib_flag == ENC_CALIB_ALL);
    puts("PASS electrical-zero completion distinct from SAVE COMMITTED; derived ticket observable");

    reset();
    ticket = begin_maintenance(MOTOR_MAINTENANCE_ENCODER_LUT);
    assert(MotorStateMachine_IsMaintenanceActive(MOTOR_MAINTENANCE_ENCODER_LUT));
    origin = MotorStateMachine_WorkEpoch();
    assert(MotorStateMachine_RequestRemoteStop());
    tick();
    MotorStateMachine_PostOutcome((MotorTaskResult){MOTOR_TASK_COMPLETED, MOTOR_FAULT_NONE},
                                  origin);
    tick();
    assert(motor.control_mode == MOTOR_CONTROL_NONE && output_off &&
           MotorStateMachine_GetState() == APP_READY);
    puts(
        "PASS encoder-LUT explicit entry remains; stale completion after STOP cannot request SAVE");
}

static void test_stop_and_fault(void)
{
    reset();
    assert(MotorStateMachine_RequestEnable(MOTOR_CONTROL_CURRENT) == MOTOR_CONTROL_ACCEPTED);
    tick();
    tick();
    assert(MotorStateMachine_GetState() == APP_RUNNING && !output_off);
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
    assert(!output_off && MotorStateMachine_GetState() == APP_STOPPING);
    assert(motor.current_limit_a == 6.0f && motor.speed_limit_rad_s == 20.0f);
    puts("PASS failed output shutdown cannot restore READY or relaxed limits");
}

static void test_borrow_cancel(void)
{
    FocBackgroundClaim claim, forged;
    MotorMaintenanceProgress progress;
    uint32_t ticket, origin;
    reset();
    ticket = begin_maintenance(MOTOR_MAINTENANCE_SAVE);
    assert(MotorStateMachine_BeginBackground(&claim));
    assert(MotorStateMachine_RequestRemoteStop());
    tick();
    assert(MotorStateMachine_GetState() == APP_STOPPING &&
           MotorStateMachine_ReadMaintenance(ticket, &progress) &&
           progress.result == APP_OPERATION_CANCEL_REQUESTED);
    assert(!MotorStateMachine_BeginBackground(&forged));
    forged = claim;
    ++forged.epoch;
    MotorStateMachine_FinishBackground(&forged, true);
    tick();
    assert(MotorStateMachine_GetState() == APP_STOPPING);
    MotorStateMachine_FinishBackground(&claim, true);
    tick();
    assert(MotorStateMachine_GetState() == APP_READY &&
           MotorStateMachine_ReadMaintenance(ticket, &progress) &&
           progress.result == APP_OPERATION_CANCELLED);
    assert(motor.control_mode == MOTOR_CONTROL_NONE && progress.effects != APP_EFFECT_COMMITTED);
    puts("PASS SAVE borrow blocks cancellation ACK; forged/late results cannot claim COMMITTED");

    reset();
    ticket = begin_maintenance(MOTOR_MAINTENANCE_COGGING);
    cogging_pending = true;
    origin = MotorStateMachine_WorkEpoch();
    MotorStateMachine_PostOutcome((MotorTaskResult){MOTOR_TASK_FINALIZE, MOTOR_FAULT_NONE}, origin);
    tick();
    assert(MotorStateMachine_GetState() == APP_MAINTENANCE && output_off);
    assert(MotorStateMachine_BeginBackground(&claim) && claim.kind == FOC_BACKGROUND_COGGING);
    assert(MotorStateMachine_RequestRemoteStop());
    tick();
    assert(cogging_cancelled && MotorStateMachine_GetState() == APP_STOPPING);
    MotorStateMachine_FinishBackground(&claim, false);
    tick();
    assert(!cogging_claimed && MotorStateMachine_GetState() == APP_READY &&
           motor.control_mode == MOTOR_CONTROL_NONE);
    puts("PASS cogging natural completion stays in maintenance; STOP waits for original foreground "
         "release");
}

/* 前台发布与持久化分别确认；整理失败不得派生保存。 */
static void test_cogging_finalize(void)
{
    for (unsigned success = 0U; success <= 1U; ++success)
    {
        reset();
        uint32_t origin = begin_maintenance(MOTOR_MAINTENANCE_COGGING);
        cogging_pending = true;
        MotorStateMachine_PostOutcome((MotorTaskResult){MOTOR_TASK_FINALIZE, MOTOR_FAULT_NONE},
                                      origin);
        tick();
        assert(output_off && MotorStateMachine_GetState() == APP_MAINTENANCE);
        FocBackgroundClaim claim;
        assert(MotorStateMachine_BeginBackground(&claim) &&
               MotorStateMachine_BackgroundValid(&claim));
        MotorStateMachine_FinishBackground(&claim, success != 0U);
        tick();
        if (success)
        {
            assert(MotorStateMachine_IsMaintenanceActive(MOTOR_MAINTENANCE_SAVE));
            uint32_t save = MotorStateMachine_WorkEpoch();
            assert(save != origin && MotorStateMachine_BeginBackground(&claim));
            MotorStateMachine_FinishBackground(&claim, true);
            tick();
            MotorMaintenanceProgress progress;
            assert(MotorStateMachine_ReadMaintenance(save, &progress));
            assert(progress.result == APP_OPERATION_COMPLETED &&
                   progress.effects == APP_EFFECT_COMMITTED);
            assert(MotorStateMachine_GetState() == APP_READY && output_off);
        }
        else
        {
            assert(MotorStateMachine_GetState() == APP_FAULT &&
                   motor.fault_code == MOTOR_FAULT_COGGING_CALIBRATION);
            assert(!MotorStateMachine_IsMaintenanceActive(MOTOR_MAINTENANCE_SAVE));
        }
    }
    puts("PASS cogging foreground RAM publication derives SAVE; finalize failure faults without "
         "saving");
}

/* 公共入口验证拒绝优先级；不注入运行所有者的私有状态。 */
static void expect_busy_requests(void)
{
    uint32_t ticket = UINT32_MAX;
    MotorControlMode mode = motor.control_mode;
    assert(MotorStateMachine_RequestEnable(MOTOR_CONTROL_SPEED) == MOTOR_CONTROL_BUSY);
    assert(MotorStateMachine_RequestClear() == MOTOR_CONTROL_BUSY);
    assert(MotorStateMachine_RequestMaintenance(MOTOR_MAINTENANCE_SAVE, &ticket) ==
           MOTOR_CONTROL_BUSY);
    assert(ticket == 0U && motor.control_mode == mode);
    assert(!MotorStateMachine_ConfigurationAllowed());

    /* Enable/Maintenance 先报故障；Clear 仍先等待占用释放。 */
    motor.fault_code = MOTOR_FAULT_ENCODER;
    assert(MotorStateMachine_RequestEnable(MOTOR_CONTROL_SPEED) == MOTOR_CONTROL_FAULT);
    assert(MotorStateMachine_RequestMaintenance(MOTOR_MAINTENANCE_SAVE, &ticket) ==
           MOTOR_CONTROL_FAULT);
    assert(MotorStateMachine_RequestClear() == MOTOR_CONTROL_BUSY);
    assert(MotorStateMachine_RequestEnable((MotorControlMode)UINT32_MAX) == MOTOR_CONTROL_INVALID);
    assert(MotorStateMachine_RequestMaintenance((MotorMaintenanceKind)UINT32_MAX, &ticket) ==
           MOTOR_CONTROL_INVALID);
    assert(MotorStateMachine_RequestMaintenance(MOTOR_MAINTENANCE_SAVE, NULL) ==
           MOTOR_CONTROL_INVALID);
    assert(ticket == 0U && motor.control_mode == mode);
}

static void test_request_admission(void)
{
    static const MotorControlMode modes[] = {
        MOTOR_CONTROL_CURRENT, MOTOR_CONTROL_SPEED, MOTOR_CONTROL_POSITION, MOTOR_CONTROL_VQ};
    for (unsigned i = 0U; i < sizeof(modes) / sizeof(modes[0]); ++i)
    {
        reset();
        assert(MotorStateMachine_RequestEnable((MotorControlMode)i) == MOTOR_CONTROL_ACCEPTED);
        assert(motor.control_mode == modes[i] && output_off);
        tick();
        tick();
        assert(MotorStateMachine_GetState() == APP_RUNNING && !output_off);
        MotorControlCommand goal = {.mode = (MotorControlMode)i};
        assert(MotorStateMachine_SubmitControl(&goal) == MOTOR_CONTROL_ACCEPTED);
        expect_busy_requests();
    }
    reset();
    assert(MotorStateMachine_RequestEnable((MotorControlMode)4) == MOTOR_CONTROL_INVALID);
    assert(MotorStateMachine_RequestEnable((MotorControlMode)UINT32_MAX) == MOTOR_CONTROL_INVALID);
    assert(motor.control_mode == MOTOR_CONTROL_NONE && output_off);

    reset();
    assert(MotorStateMachine_RequestBootStop(UINT64_C(0x100000001)) == MOTOR_CONTROL_ACCEPTED);
    expect_busy_requests();

    reset();
    MotorStateMachine_PostOutcome((MotorTaskResult){MOTOR_TASK_STOP_REQUESTED, MOTOR_FAULT_NONE},
                                  MotorStateMachine_WorkEpoch());
    expect_busy_requests();

    reset();
    uint32_t ticket;
    assert(MotorStateMachine_RequestMaintenance(MOTOR_MAINTENANCE_SAVE, &ticket) ==
           MOTOR_CONTROL_ACCEPTED);
    assert(ticket != 0U);
    expect_busy_requests();

    reset();
    FocBackgroundClaim claim;
    assert(MotorStateMachine_RequestMaintenance(MOTOR_MAINTENANCE_SAVE, &ticket) ==
           MOTOR_CONTROL_ACCEPTED);
    assert(MotorStateMachine_BeginBackground(&claim));
    expect_busy_requests();

    reset();
    check_borrowed_requests = true;
    MotorStateMachine_ServiceCompensation();
    assert(!check_borrowed_requests && MotorStateMachine_ConfigurationAllowed());

    reset();
    encoder_released = false;
    assert(MotorStateMachine_RequestEnable(MOTOR_CONTROL_SPEED) == MOTOR_CONTROL_DENIED);
    assert(MotorStateMachine_RequestClear() == MOTOR_CONTROL_BUSY);
    assert(MotorStateMachine_RequestMaintenance(MOTOR_MAINTENANCE_SAVE, &ticket) ==
           MOTOR_CONTROL_BUSY);
    assert(ticket == 0U && output_off);
    puts("PASS four control mappings, busy sources, fault/invalid priority and release guards");
}

static void test_maintenance_identity(void)
{
    reset();
    MotorMaintenanceRequest request = {MOTOR_MAINTENANCE_ELECTRICAL_ZERO, 2U, 1U, 0U};
    uint32_t origin, retry, save;
    assert(MotorStateMachine_RequestIdentifiedMaintenance(&request, &origin) ==
           MOTOR_CONTROL_ACCEPTED);
    assert(MotorStateMachine_RequestIdentifiedMaintenance(&request, &retry) ==
           MOTOR_CONTROL_ACCEPTED);
    assert(origin != 0U && retry == origin);
    request.kind = MOTOR_MAINTENANCE_ENCODER_LUT;
    assert(MotorStateMachine_RequestIdentifiedMaintenance(&request, &retry) ==
           MOTOR_CONTROL_CONFLICTING_RETRY);
    assert(retry == 0U);
    MotorStateMachine_PostOutcome((MotorTaskResult){MOTOR_TASK_COMPLETED, MOTOR_FAULT_NONE},
                                  origin);
    tick();
    tick();
    save = MotorStateMachine_WorkEpoch();
    assert(save != origin && MotorStateMachine_IsMaintenanceActive(MOTOR_MAINTENANCE_SAVE));
    FocBackgroundClaim claim;
    assert(MotorStateMachine_BeginBackground(&claim));
    MotorStateMachine_FinishBackground(&claim, true);
    tick();
    MotorMaintenanceProgress progress;
    assert(MotorStateMachine_ReadMaintenance(origin, &progress));
    assert(progress.kind == MOTOR_MAINTENANCE_ELECTRICAL_ZERO &&
           progress.result == APP_OPERATION_COMPLETED &&
           progress.effects == APP_EFFECT_UNCOMMITTED);
    assert(MotorStateMachine_ReadMaintenance(save, &progress));
    assert(progress.kind == MOTOR_MAINTENANCE_SAVE && progress.effects == APP_EFFECT_COMMITTED);
    puts("PASS identified retry and original calibration terminal survive derived SAVE completion");
}

/* 电压和反馈拒绝覆盖真实所有者，不再从旧分发器提取准入代码。 */
static void test_maintenance_guards(void)
{
    const float values[] = {25.5f, 33.9f, NAN, INFINITY};
    for (unsigned v = 0U; v < sizeof(values) / sizeof(values[0]); ++v)
    {
        for (unsigned kind = 0U; kind < MOTOR_MAINTENANCE_NONE; ++kind)
        {
            if (kind == MOTOR_MAINTENANCE_SAVE || kind == MOTOR_MAINTENANCE_DEFAULTS ||
                kind == MOTOR_MAINTENANCE_MECHANICAL_ZERO)
            {
                continue;
            }
            reset();
            foc.bus_voltage_v = foc.bus_voltage_filtered_v = values[v];
            uint32_t ticket = UINT32_MAX;
            assert(MotorStateMachine_RequestMaintenance((MotorMaintenanceKind)kind, &ticket) !=
                   MOTOR_CONTROL_ACCEPTED);
            assert(ticket == 0U && output_off && start_count == 0U);
            if (isfinite(values[v]))
            {
                assert(motor.fault_code ==
                       (values[v] < 25.6f ? MOTOR_FAULT_UNDERVOLTAGE : MOTOR_FAULT_OVERVOLTAGE));
            }
        }
        for (unsigned mode = 0U; mode <= MOTOR_CONTROL_VQ; ++mode)
        {
            reset();
            foc.bus_voltage_v = foc.bus_voltage_filtered_v = values[v];
            assert(MotorStateMachine_RequestEnable((MotorControlMode)mode) !=
                   MOTOR_CONTROL_ACCEPTED);
            assert(output_off && start_count == 0U);
        }
    }
    reset();
    encoder.calib_flag = 0U;
    uint32_t ticket = UINT32_MAX;
    assert(MotorStateMachine_RequestMaintenance(MOTOR_MAINTENANCE_ELECTRICAL_ZERO, &ticket) ==
           MOTOR_CONTROL_DENIED);
    assert(motor.fault_code == MOTOR_FAULT_ENCODER_NOT_CALIBRATED && output_off && ticket == 0U);
    reset();
    encoder.bad_frame_streak = 1U;
    assert(MotorStateMachine_RequestMaintenance(MOTOR_MAINTENANCE_ENCODER_LUT, &ticket) ==
           MOTOR_CONTROL_DENIED);
    assert(motor.fault_code == MOTOR_FAULT_ENCODER && output_off && ticket == 0U);
    puts(
        "PASS real-owner torque admission rejects unsafe bus and invalid encoder without enabling");
}

static void test_task_results(void)
{
    /* 每个任务由实际 kind 选择；控制模式始终保持 NONE。 */
    for (unsigned kind = 0U; kind < MOTOR_MAINTENANCE_NONE; ++kind)
    {
        reset();
        uint32_t ticket = begin_maintenance((MotorMaintenanceKind)kind);
        assert(motor.control_mode == MOTOR_CONTROL_NONE);
        assert(MotorStateMachine_MaintenanceRunnable());
        MotorTaskResult result = MotorMaintenance_Step(
            (MotorMaintenanceKind)kind, &foc, &motor, &pi, &encoder, NULL, &startup);
        assert(result.error == MOTOR_FAULT_NONE);
        if (kind != MOTOR_MAINTENANCE_SAVE)
        {
            assert(leaf_calls[kind] == 1U);
        }
        assert(MotorStateMachine_WorkEpoch() == ticket);
    }
    const MotorMaintenanceKind ram_only[] = {MOTOR_MAINTENANCE_PHASE_RESISTANCE,
                                             MOTOR_MAINTENANCE_FRICTION};
    for (unsigned i = 0U; i < sizeof(ram_only) / sizeof(ram_only[0]); ++i)
    {
        reset();
        uint32_t ticket = begin_maintenance(ram_only[i]);
        MotorStateMachine_PostOutcome((MotorTaskResult){MOTOR_TASK_COMPLETED, MOTOR_FAULT_NONE},
                                      ticket);
        assert(!MotorStateMachine_MaintenanceRunnable());
        tick();
        tick();
        MotorMaintenanceProgress progress;
        assert(output_off && MotorStateMachine_GetState() == APP_READY);
        assert(MotorStateMachine_ReadMaintenance(ticket, &progress));
        assert(progress.result == APP_OPERATION_COMPLETED &&
               progress.effects == APP_EFFECT_UNCOMMITTED);
        assert(!MotorStateMachine_IsMaintenanceActive(MOTOR_MAINTENANCE_SAVE));
    }
    reset();
    uint32_t ticket = begin_maintenance(MOTOR_MAINTENANCE_ELECTRICAL_ZERO);
    MotorStateMachine_PostOutcome((MotorTaskResult){MOTOR_TASK_COMPLETED, MOTOR_FAULT_NONE},
                                  ticket);
    MotorStateMachine_PostOutcome((MotorTaskResult){MOTOR_TASK_FINALIZE, MOTOR_FAULT_NONE}, ticket);
    MotorStateMachine_PostOutcome((MotorTaskResult){MOTOR_TASK_FAILED, MOTOR_FAULT_ENCODER},
                                  ticket);
    assert(motor.fault_code == MOTOR_FAULT_ENCODER && output_off);
    tick();
    assert(MotorStateMachine_GetState() == APP_FAULT);
    assert(!MotorStateMachine_IsMaintenanceActive(MOTOR_MAINTENANCE_SAVE));
    puts("PASS explicit task dispatch, RAM-only completion and same-epoch fault priority");
}

static void test_power_permission(void)
{
    reset();
    uint32_t ticket = begin_maintenance(MOTOR_MAINTENANCE_COGGING);
    assert(output_off && start_count == 0U);
    MotorStateMachine_PostOutcome((MotorTaskResult){MOTOR_TASK_PREPARED, MOTOR_FAULT_NONE},
                                  ticket - 1U);
    assert(output_off && start_count == 0U);
    MotorStateMachine_PostOutcome((MotorTaskResult){MOTOR_TASK_PREPARED, MOTOR_FAULT_NONE}, ticket);
    assert(!output_off && start_count == 1U);
    MotorStateMachine_PostOutcome((MotorTaskResult){MOTOR_TASK_PREPARED, MOTOR_FAULT_NONE}, ticket);
    assert(start_count == 1U);
    MotorStateMachine_RequestRemoteStop();
    MotorStateMachine_PostOutcome((MotorTaskResult){MOTOR_TASK_PREPARED, MOTOR_FAULT_NONE}, ticket);
    tick();
    assert(output_off && start_count == 1U && MotorStateMachine_GetState() == APP_READY);

    reset();
    ticket = begin_maintenance(MOTOR_MAINTENANCE_COGGING);
    foc.bus_voltage_v = foc.bus_voltage_filtered_v = 24.0f;
    MotorStateMachine_PostOutcome((MotorTaskResult){MOTOR_TASK_PREPARED, MOTOR_FAULT_NONE}, ticket);
    assert(output_off && start_count == 0U && motor.fault_code != MOTOR_FAULT_NONE);
    tick();
    assert(MotorStateMachine_GetState() == APP_FAULT && !MotorStateMachine_MaintenanceRunnable());

    reset();
    ticket = begin_maintenance(MOTOR_MAINTENANCE_COGGING);
    start_fails = true;
    MotorStateMachine_PostOutcome((MotorTaskResult){MOTOR_TASK_PREPARED, MOTOR_FAULT_NONE}, ticket);
    tick();
    assert(output_off && MotorStateMachine_GetState() == APP_FAULT);
    reset();
    start_fails = true;
    assert(MotorStateMachine_RequestMaintenance(MOTOR_MAINTENANCE_PHASE_RESISTANCE, &ticket) !=
           MOTOR_CONTROL_ACCEPTED);
    assert(ticket == 0U && output_off && MotorStateMachine_GetState() == APP_FAULT);
    reset();
    start_fails = true;
    assert(MotorStateMachine_RequestEnable(MOTOR_CONTROL_SPEED) == MOTOR_CONTROL_ACCEPTED);
    tick();
    assert(output_off && MotorStateMachine_GetState() == APP_FAULT);

    reset();
    ticket = begin_maintenance(MOTOR_MAINTENANCE_ELECTRICAL_ZERO);
    stop_fails = true;
    MotorStateMachine_PostOutcome((MotorTaskResult){MOTOR_TASK_COMPLETED, MOTOR_FAULT_NONE},
                                  ticket);
    tick();
    tick();
    assert(!output_off && MotorStateMachine_GetState() == APP_STOPPING);
    assert(!MotorStateMachine_ConfigurationAllowed());
    assert(!MotorStateMachine_IsMaintenanceActive(MOTOR_MAINTENANCE_SAVE));
    assert(MotorStateMachine_OutputExpected());
    puts("PASS PREPARED epoch/STOP gates, duplicate suppression, enable and completion shutdown "
         "failures");
}

/* 固定线上编号作为外部参考，防止内部枚举变动改写 CAN 状态。 */
static void test_wire_projection(void)
{
    const uint8_t controls[] = {1U, 2U, 3U, 6U, 0U};
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
    puts("PASS fixed CAN status mode numbers preserved across control/task split");
}

int main(void)
{
    reset();
    uint32_t progress, next;
    assert(MotorStateMachine_ReadBootHealth(&progress) && progress != 0U);
    assert(MotorStateMachine_ReadBootHealth(&next) && next == progress);
    tick();
    assert(MotorStateMachine_ReadBootHealth(&next) && next != progress);
    MotorStateMachine_Tick();
    assert(!MotorStateMachine_ReadBootHealth(&next));
    assert(!MotorStateMachine_ReadBootHealth(NULL));
    test_wire_projection();
    test_request_admission();
    test_maintenance_guards();
    test_task_results();
    test_power_permission();
    test_maintenance_identity();
    test_control();
    test_calibration_save();
    test_borrow_cancel();
    test_cogging_finalize();
    test_stop_and_fault();
    assert(critical_depth == 0U);
    return 0;
}

/* 调度真实维护分发，只有叶功能替换为桩；任务结果没有全局模式副作用。 */
MotorTaskResult EncoderCalibrationTask_StepLut(FocState *f,
                                               MotorControlState *m,
                                               PiController *p,
                                               EncoderState *e,
                                               FluxObserverState *o,
                                               SensorlessStartupState *s)
{
    (void)f;
    (void)m;
    (void)p;
    (void)e;
    (void)o;
    (void)s;
    ++leaf_calls[MOTOR_MAINTENANCE_ENCODER_LUT];
    return (MotorTaskResult){MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};
}
MotorTaskResult
EncoderCalibrationTask_StepElectricalZero(FocState *f, MotorControlState *m, EncoderState *e)
{
    (void)f;
    (void)m;
    (void)e;
    ++leaf_calls[MOTOR_MAINTENANCE_ELECTRICAL_ZERO];
    return (MotorTaskResult){MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};
}
MotorTaskResult FocCogging_Task(FocState *f, MotorControlState *m, PiController *p, EncoderState *e)
{
    (void)f;
    (void)m;
    (void)p;
    (void)e;
    ++leaf_calls[MOTOR_MAINTENANCE_COGGING];
    return (MotorTaskResult){MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};
}
MotorTaskResult
PhaseResistanceTask_Run(FocState *f, MotorControlState *m, const PhaseResistanceConfig *config)
{
    (void)f;
    (void)m;
    assert(config != NULL && config->test_current_low == 1.0f && config->test_current_high == 2.0f);
    ++leaf_calls[MOTOR_MAINTENANCE_PHASE_RESISTANCE];
    return (MotorTaskResult){MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};
}
MotorTaskResult FrictionIdentificationTask_Task(FocState *f,
                                                MotorControlState *m,
                                                PiController *p,
                                                EncoderState *e,
                                                const FrictionIdentificationTaskConfig *config)
{
    (void)f;
    (void)m;
    (void)p;
    (void)e;
    assert(config != NULL && config->core.speed_point_count == 4U &&
           config->current_ratio_max == 0.90f);
    ++leaf_calls[MOTOR_MAINTENANCE_FRICTION];
    return (MotorTaskResult){MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};
}
void FrictionIdentificationTask_Init(const FrictionIdentificationTaskConfig *config)
{
    assert(config != NULL);
}
bool Encoder_SetMechanicalZero(EncoderState *e)
{
    (void)e;
    ++leaf_calls[MOTOR_MAINTENANCE_MECHANICAL_ZERO];
    return true;
}
void MotorOuterLoop_RequestReset(void)
{
}
void Param_Return_Default(MotorControlState *m, EncoderState *e)
{
    (void)m;
    (void)e;
    ++leaf_calls[MOTOR_MAINTENANCE_DEFAULTS];
}
