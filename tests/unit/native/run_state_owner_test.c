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
#include "control_math.h"
#define FOC_MAX_MODULATION (0.95f * 0.866025403f)

/* 真实生产所有者和类型；仅替换硬件与 worker，验证命令、资源和身份交接。 */
static MotorControlState motor;
static FocState foc;
static PiController pi;
static EncoderState encoder;
static SensorlessStartupState startup;
static bool output_off, stop_fails, start_fails, temperature_fresh, encoder_released;
static unsigned start_count, leaf_calls[8];
static unsigned phase_cancel_count;
static bool cogging_active, cogging_pending, cogging_claimed, cogging_cancelled;
static bool outer_ready;
static bool check_borrowed_requests;
static bool tick_during_compensation;
static unsigned compensation_calls;
static uint32_t critical_depth;
static uint32_t now_ms;
static void tick(void);

uint32_t time_hw_now_ms(void)
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
    ++phase_cancel_count;
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
    return m->application.axis_profile_valid && e->calib_flag == ENC_CALIB_ALL;
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
    ++compensation_calls;
    if (tick_during_compensation)
    {
        tick();
    }
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
    now_ms = 0U;
    memset(&motor, 0, sizeof(motor));
    memset(&foc, 0, sizeof(foc));
    memset(&encoder, 0, sizeof(encoder));
    motor.application.axis_profile_valid = true;
    motor.model.motor_pole_pairs = 21;
    motor.configuration_confirmed = true;
    motor.model.motor_phase_resistance_ohm = 1.0f;
    motor.model.motor_d_inductance_h = motor.model.motor_q_inductance_h = 0.001f;
    motor.model.motor_flux_wb = 0.01f;
    motor.model.calibration_current_a = 3.0f;
    motor.application.current_limit_a = 6.0f;
    motor.application.speed_limit_rad_s = 20.0f;
    motor.application.speed_acceleration_rad_s2 = motor.application.speed_deceleration_rad_s2 =
        1.0f;
    motor.application.speed_kp = 0.1f;
    motor.application.speed_ki = 0.2f;
    motor.device.phase_a_offset_counts = motor.device.phase_b_offset_counts =
        motor.device.phase_c_offset_counts = 2048.0f;
    encoder.calib_flag = ENC_CALIB_ALL;
    foc.bus_voltage_v = foc.bus_voltage_filtered_v = 30.0f;
    foc.temperature_c = 25.0f;
    output_off = outer_ready = temperature_fresh = encoder_released = true;
    start_fails = false;
    start_count = 0U;
    compensation_calls = 0U;
    tick_during_compensation = false;
    memset(leaf_calls, 0, sizeof(leaf_calls));
    stop_fails = cogging_active = cogging_pending = cogging_claimed = cogging_cancelled = false;
    MotorStateMachine_Init(&motor, &foc, &pi, &encoder, &startup);
    for (unsigned i = 0U; i < 4U; ++i)
    {
        tick();
    }
    assert(MotorStateMachine_GetState() == MOTOR_STATE_IDLE);
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
    assert(motor.application.current_limit_a == 6.0f &&
           motor.application.speed_limit_rad_s == 20.0f);
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
}

/* 同源序号和STOP屏障只使用真实所有者入口，断言不通过私有状态伪造启相。 */
static void test_identified_speed_start(void)
{
    MotorControlStartRequest request = {
        .mode = MOTOR_CONTROL_SPEED, .sequence = 101U, .source_node = 2U};
    MotorControlStartProgress progress;

    reset();
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
    motor.configuration_confirmed = false;
    assert(MotorStateMachine_RequestIdentifiedEnable(&request) == MOTOR_CONTROL_DENIED);
    motor.configuration_confirmed = true;
    assert(MotorStateMachine_RequestIdentifiedEnable(&request) == MOTOR_CONTROL_DENIED);
    request.sequence = 1U;
    assert(MotorStateMachine_RequestIdentifiedEnable(&request) == MOTOR_CONTROL_ACCEPTED);
    assert(output_off && start_count == 0U);
    puts("PASS identified speed start: retry, conflict, busy, STOP, fault, wrap, rejected retry");
}

/* 启动已受理时，后台不得借用配置并让监督中断误判资源仍被占用。 */
static void test_start_compensation_race(void)
{
    MotorControlStartRequest request = {
        .mode = MOTOR_CONTROL_SPEED, .sequence = 101U, .source_node = 2U};
    MotorControlStartProgress progress;
    reset();
    assert(MotorStateMachine_RequestIdentifiedEnable(&request) == MOTOR_CONTROL_ACCEPTED);
    tick_during_compensation = true;
    MotorStateMachine_ServiceCompensation();
    tick();
    assert(MotorStateMachine_ReadEnableProgress(2U, 101U, &progress));
    assert(progress.state == MOTOR_CONTROL_START_STARTED);
    assert(MotorStateMachine_GetState() == MOTOR_STATE_RUNNING && start_count == 1U);
    assert(compensation_calls == 0U);
    MotorStateMachine_ServiceCompensation();
    assert(compensation_calls == 1U && MotorStateMachine_GetState() == MOTOR_STATE_RUNNING);
    assert(MotorStateMachine_RequestIdentifiedStop(2U, 102U));
    tick();
    assert(output_off && MotorStateMachine_GetState() == MOTOR_STATE_IDLE);
    puts("PASS accepted start excludes compensation borrow; running service and STOP preserved");
}

static void test_calibration_save(void)
{
    MotorMaintenanceProgress progress;
    uint32_t ticket, origin, save;
    reset();
    encoder.calib_flag = ENC_CALIB_LINEARIZED;
    ticket = begin_maintenance(MOTOR_MAINTENANCE_ELECTRICAL_ZERO);
    assert((MotorStateMachine_GetMaintenanceKind() == MOTOR_MAINTENANCE_ELECTRICAL_ZERO) &&
           !output_off);
    origin = MotorStateMachine_WorkEpoch();
    encoder.calib_flag = ENC_CALIB_ALL;
    MotorStateMachine_PostOutcome((MotorTaskResult){MOTOR_TASK_COMPLETED, MOTOR_FAULT_NONE},
                                  origin);
    tick();
    assert(output_off && MotorStateMachine_ReadMaintenance(ticket, &progress));
    assert(progress.effects == MOTOR_MAINTENANCE_EFFECT_UNCOMMITTED);
    tick();
    assert((MotorStateMachine_GetMaintenanceKind() == MOTOR_MAINTENANCE_SAVE));
    save = MotorStateMachine_WorkEpoch();
    FocBackgroundClaim claim;
    assert(MotorStateMachine_BeginBackground(&claim) && claim.kind == FOC_BACKGROUND_SAVE);
    assert(MotorStateMachine_BackgroundValid(&claim));
    MotorStateMachine_FinishBackground(&claim, true);
    tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_IDLE);
    assert(MotorStateMachine_ReadMaintenance(save, &progress));
    assert(progress.result == MOTOR_MAINTENANCE_COMPLETED &&
           progress.effects == MOTOR_MAINTENANCE_EFFECT_COMMITTED);
    assert(encoder.calib_flag == ENC_CALIB_ALL);
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
}

static void test_stop_and_fault(void)
{
    reset();
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
    assert(motor.application.current_limit_a == 6.0f &&
           motor.application.speed_limit_rad_s == 20.0f);
    puts("PASS failed output shutdown cannot restore IDLE or relaxed limits");
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

    reset();
    ticket = begin_maintenance(MOTOR_MAINTENANCE_COGGING);
    cogging_pending = true;
    origin = MotorStateMachine_WorkEpoch();
    MotorStateMachine_PostOutcome((MotorTaskResult){MOTOR_TASK_FINALIZE, MOTOR_FAULT_NONE}, origin);
    tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_MAINTENANCE && output_off);
    assert(MotorStateMachine_BeginBackground(&claim) && claim.kind == FOC_BACKGROUND_COGGING);
    assert(MotorStateMachine_RequestRemoteStop());
    tick();
    assert(cogging_cancelled && MotorStateMachine_GetState() == MOTOR_STATE_STOPPING);
    MotorStateMachine_FinishBackground(&claim, false);
    tick();
    assert(!cogging_claimed && MotorStateMachine_GetState() == MOTOR_STATE_IDLE &&
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
        assert(output_off && MotorStateMachine_GetState() == MOTOR_STATE_MAINTENANCE);
        FocBackgroundClaim claim;
        assert(MotorStateMachine_BeginBackground(&claim) &&
               MotorStateMachine_BackgroundValid(&claim));
        MotorStateMachine_FinishBackground(&claim, success != 0U);
        tick();
        if (success)
        {
            assert((MotorStateMachine_GetMaintenanceKind() == MOTOR_MAINTENANCE_SAVE));
            uint32_t save = MotorStateMachine_WorkEpoch();
            assert(save != origin && MotorStateMachine_BeginBackground(&claim));
            MotorStateMachine_FinishBackground(&claim, true);
            tick();
            MotorMaintenanceProgress progress;
            assert(MotorStateMachine_ReadMaintenance(save, &progress));
            assert(progress.result == MOTOR_MAINTENANCE_COMPLETED &&
                   progress.effects == MOTOR_MAINTENANCE_EFFECT_COMMITTED);
            assert(MotorStateMachine_GetState() == MOTOR_STATE_IDLE && output_off);
        }
        else
        {
            assert(MotorStateMachine_GetState() == MOTOR_STATE_FAULT &&
                   motor.fault_code == MOTOR_FAULT_COGGING_CALIBRATION);
            assert(!(MotorStateMachine_GetMaintenanceKind() == MOTOR_MAINTENANCE_SAVE));
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
        assert(MotorStateMachine_GetState() == MOTOR_STATE_RUNNING && !output_off);
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
    assert(save != origin && (MotorStateMachine_GetMaintenanceKind() == MOTOR_MAINTENANCE_SAVE));
    FocBackgroundClaim claim;
    assert(MotorStateMachine_BeginBackground(&claim));
    MotorStateMachine_FinishBackground(&claim, true);
    tick();
    MotorMaintenanceProgress progress;
    assert(MotorStateMachine_ReadMaintenance(origin, &progress));
    assert(progress.kind == MOTOR_MAINTENANCE_ELECTRICAL_ZERO &&
           progress.result == MOTOR_MAINTENANCE_COMPLETED &&
           progress.effects == MOTOR_MAINTENANCE_EFFECT_UNCOMMITTED);
    assert(MotorStateMachine_ReadMaintenance(save, &progress));
    assert(progress.kind == MOTOR_MAINTENANCE_SAVE &&
           progress.effects == MOTOR_MAINTENANCE_EFFECT_COMMITTED);
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
                assert(motor.fault_code == MOTOR_FAULT_NONE);
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
    assert(motor.fault_code == MOTOR_FAULT_NONE && output_off && ticket == 0U);
    reset();
    encoder.bad_frame_streak = 1U;
    assert(MotorStateMachine_RequestMaintenance(MOTOR_MAINTENANCE_ENCODER_LUT, &ticket) ==
           MOTOR_CONTROL_DENIED);
    assert(motor.fault_code == MOTOR_FAULT_NONE && output_off && ticket == 0U);
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
        assert(output_off && MotorStateMachine_GetState() == MOTOR_STATE_IDLE);
        assert(MotorStateMachine_ReadMaintenance(ticket, &progress));
        assert(progress.result == MOTOR_MAINTENANCE_COMPLETED &&
               progress.effects == MOTOR_MAINTENANCE_EFFECT_UNCOMMITTED);
        assert(!(MotorStateMachine_GetMaintenanceKind() == MOTOR_MAINTENANCE_SAVE));
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
    assert(MotorStateMachine_GetState() == MOTOR_STATE_FAULT);
    assert(!(MotorStateMachine_GetMaintenanceKind() == MOTOR_MAINTENANCE_SAVE));
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
    assert(output_off && start_count == 1U && MotorStateMachine_GetState() == MOTOR_STATE_IDLE);

    reset();
    ticket = begin_maintenance(MOTOR_MAINTENANCE_COGGING);
    foc.bus_voltage_v = foc.bus_voltage_filtered_v = 24.0f;
    MotorStateMachine_PostOutcome((MotorTaskResult){MOTOR_TASK_PREPARED, MOTOR_FAULT_NONE}, ticket);
    assert(output_off && start_count == 0U && motor.fault_code != MOTOR_FAULT_NONE);
    tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_FAULT &&
           !MotorStateMachine_MaintenanceRunnable());

    reset();
    ticket = begin_maintenance(MOTOR_MAINTENANCE_COGGING);
    start_fails = true;
    MotorStateMachine_PostOutcome((MotorTaskResult){MOTOR_TASK_PREPARED, MOTOR_FAULT_NONE}, ticket);
    tick();
    assert(output_off && MotorStateMachine_GetState() == MOTOR_STATE_FAULT);
    reset();
    start_fails = true;
    assert(MotorStateMachine_RequestMaintenance(MOTOR_MAINTENANCE_PHASE_RESISTANCE, &ticket) !=
           MOTOR_CONTROL_ACCEPTED);
    assert(ticket == 0U && output_off && MotorStateMachine_GetState() == MOTOR_STATE_FAULT);
    reset();
    start_fails = true;
    assert(MotorStateMachine_RequestEnable(MOTOR_CONTROL_SPEED) == MOTOR_CONTROL_ACCEPTED);
    tick();
    assert(output_off && MotorStateMachine_GetState() == MOTOR_STATE_FAULT);

    reset();
    ticket = begin_maintenance(MOTOR_MAINTENANCE_ELECTRICAL_ZERO);
    stop_fails = true;
    MotorStateMachine_PostOutcome((MotorTaskResult){MOTOR_TASK_COMPLETED, MOTOR_FAULT_NONE},
                                  ticket);
    tick();
    tick();
    assert(!output_off && MotorStateMachine_GetState() == MOTOR_STATE_STOPPING);
    assert(!MotorStateMachine_ConfigurationAllowed());
    assert(!(MotorStateMachine_GetMaintenanceKind() == MOTOR_MAINTENANCE_SAVE));
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
        motor.configuration_confirmed = false;
        encoder.calib_flag = (uint8_t)flags;
        MotorStateMachine_Init(&motor, &foc, &pi, &encoder, &startup);
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
        motor.configuration_confirmed = true;
        MotorStateMachine_ReadReadiness(&readiness);
        assert(!(readiness.start_blockers & MOTOR_START_CONFIGURATION_REQUIRED));
        if (flags != ENC_CALIB_ALL)
        {
            assert(readiness.start_blockers & MOTOR_START_CALIBRATION_REQUIRED);
            assert(readiness.startable_modes == 0U);
        }
        else
        {
            assert(readiness.start_blockers == 0U && readiness.startable_modes == 15U);
        }
        assert(output_off && MotorStateMachine_GetState() == MOTOR_STATE_IDLE && start_count == 0U);
    }
    reset();
    encoder.calib_flag = 0U;
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
    MotorStateMachine_Init(&motor, &foc, &pi, &encoder, &startup);
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
            motor.configuration_confirmed = (gaps & 1U) == 0U;
            if (gaps & 2U)
            {
                encoder.calib_flag = 0U;
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
            motor.application.axis_profile_valid = (gaps & 32U) == 0U;
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
    motor.device.phase_a_offset_counts = motor.device.phase_b_offset_counts =
        motor.device.phase_c_offset_counts = NAN;
    foc.phase_a_current_a = foc.phase_b_current_a = foc.phase_c_current_a = NAN;
    MotorStateMachine_Init(&motor, &foc, &pi, &encoder, &startup);
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
    motor.device.phase_a_offset_counts = motor.device.phase_b_offset_counts =
        motor.device.phase_c_offset_counts = 2048.0f;
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
    motor.device.phase_a_offset_counts = motor.device.phase_b_offset_counts =
        motor.device.phase_c_offset_counts = NAN;
    foc.phase_a_current_a = foc.phase_b_current_a = foc.phase_c_current_a = NAN;
    MotorStateMachine_Init(&motor, &foc, &pi, &encoder, &startup);
    tick();
    now_ms = 1000U;
    motor.device.phase_a_offset_counts = motor.device.phase_b_offset_counts =
        motor.device.phase_c_offset_counts = 2048.0f;
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
    MotorStateMachine_Init(&motor, &foc, &pi, &encoder, &startup);
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
    MotorStateMachine_Init(&motor, &foc, &pi, &encoder, &startup);
    tick();
    tick();
    tick();
    now_ms = 500U;
    tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_FAULT &&
           motor.fault_code == MOTOR_FAULT_SELF_TEST_TIMEOUT && output_off);
    /* BOOT 资源未释放与必需编码器不健康也不能无限等待。 */
    reset();
    MotorStateMachine_Init(&motor, &foc, &pi, &encoder, &startup);
    encoder_released = false;
    tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_INIT);
    now_ms = 500U;
    tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_FAULT &&
           motor.fault_code == MOTOR_FAULT_SELF_TEST_TIMEOUT && output_off);
    reset();
    MotorStateMachine_Init(&motor, &foc, &pi, &encoder, &startup);
    encoder.bad_frame_streak = 1U;
    tick();
    now_ms = 500U;
    tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_FAULT &&
           motor.fault_code == MOTOR_FAULT_SELF_TEST_TIMEOUT && output_off);
    reset();
    MotorStateMachine_Init(&motor, &foc, &pi, &encoder, &startup);
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
    MotorStateMachine_Init(&motor, &foc, &pi, &encoder, &startup);
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
    MotorStateMachine_Init(&motor, &foc, &pi, &encoder, &startup);
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
    MotorStateMachine_Init(&motor, &foc, &pi, &encoder, &startup);
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
    motor.device.phase_a_offset_counts = motor.device.phase_b_offset_counts =
        motor.device.phase_c_offset_counts = NAN;
    MotorStateMachine_Init(&motor, &foc, &pi, &encoder, &startup);
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
    MotorStateMachine_Init(&motor, &foc, &pi, &encoder, &startup);
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
    MotorStateMachine_Init(&motor, &foc, &pi, &encoder, &startup);
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
static void test_completed_maintenance_cleanup(void)
{
    reset();
    uint32_t ticket = begin_maintenance(MOTOR_MAINTENANCE_PHASE_RESISTANCE);
    MotorStateMachine_PostOutcome((MotorTaskResult){MOTOR_TASK_COMPLETED, MOTOR_FAULT_NONE},
                                  ticket);
    tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_IDLE);
    assert(MotorStateMachine_GetMaintenanceKind() == MOTOR_MAINTENANCE_NONE);
    assert(MotorStateMachine_ConfigurationAllowed());
    unsigned completed_count = phase_cancel_count;
    MotorFault_Report(&motor, MOTOR_FAULT_OVERCURRENT);
    tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_FAULT && output_off);
    assert(phase_cancel_count > completed_count);
    puts("PASS completed maintenance releases admission and preserves later fault cleanup kind");
}

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
                    encoder.calib_flag = 0U;
                    break;
                case 7U:
                    motor.uses_sensorless_feedback = true;
                    break;
                case 8U:
                    motor.application.axis_profile_valid = false;
                    break;
                case 9U:
                    foc.bus_voltage_filtered_v = 25.5f;
                    break;
                case 10U:
                    motor.application.current_limit_a = NAN;
                    break;
                case 11U:
                    motor.application.speed_limit_rad_s = NAN;
                    break;
                case 12U:
                    motor.application.axis_profile.magic = MOTOR_AXIS_PROFILE_NUMERIC_MAGIC;
                    motor.application.axis_profile.minimum_position_rad = -1.0f;
                    motor.application.axis_profile.maximum_position_rad = 1.0f;
                    motor.application.axis_profile.maximum_speed_rad_s = 2.0f;
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
    MotorStateMachine_Init(&motor, &foc, &pi, &encoder, &startup);
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
}

/* 模拟叶算法已经发布结果，验证真实所有者的串联、身份和最终保存边界。 */
static void test_calibration_plan(void)
{
    reset();
    uint32_t ticket = UINT32_MAX;
    assert(MotorStateMachine_RequestCalibration(0U, &ticket) == MOTOR_CONTROL_INVALID &&
           ticket == 0U);
    assert(MotorStateMachine_RequestCalibration(1U << 8U, &ticket) == MOTOR_CONTROL_INVALID);
    encoder.calib_flag = 0U;
    assert(MotorStateMachine_RequestCalibration(
               MOTOR_CALIBRATION_STEP(MOTOR_MAINTENANCE_ELECTRICAL_ZERO), &ticket) ==
           MOTOR_CONTROL_DENIED);
    encoder.calib_flag = ENC_CALIB_ALL;
    assert(
        MotorStateMachine_RequestCalibration(MOTOR_CALIBRATION_STEP(MOTOR_MAINTENANCE_ENCODER_LUT) |
                                                 MOTOR_CALIBRATION_STEP(MOTOR_MAINTENANCE_COGGING),
                                             &ticket) == MOTOR_CONTROL_DENIED);
    motor.model.motor_flux_wb = NAN;
    assert(MotorStateMachine_RequestCalibration(MOTOR_CALIBRATION_SUPPORTED_STEPS, &ticket) !=
           MOTOR_CONTROL_ACCEPTED);
    reset();
    encoder.calib_flag = 0U;
    assert(MotorStateMachine_RequestCalibration(MOTOR_CALIBRATION_SUPPORTED_STEPS, &ticket) ==
           MOTOR_CONTROL_ACCEPTED);
    const MotorMaintenanceKind order[] = {MOTOR_MAINTENANCE_PHASE_RESISTANCE,
                                          MOTOR_MAINTENANCE_ENCODER_LUT,
                                          MOTOR_MAINTENANCE_ELECTRICAL_ZERO,
                                          MOTOR_MAINTENANCE_MECHANICAL_ZERO,
                                          MOTOR_MAINTENANCE_FRICTION,
                                          MOTOR_MAINTENANCE_COGGING};
    MotorCalibrationProgress flow;
    uint32_t previous = 0U;
    for (unsigned i = 0U; i < sizeof(order) / sizeof(order[0]); ++i)
    {
        assert(MotorStateMachine_ReadCalibration(ticket, &flow));
        assert(flow.step == order[i] && flow.step_ticket != previous);
        assert(flow.result == MOTOR_MAINTENANCE_NO_RESULT &&
               flow.effects != MOTOR_MAINTENANCE_EFFECT_COMMITTED);
        assert(MotorStateMachine_GetFlowState() == MOTOR_FLOW_CALIBRATION);
        assert(!MotorStateMachine_ConfigurationAllowed());
        assert(MotorStateMachine_RequestEnable(MOTOR_CONTROL_CURRENT) == MOTOR_CONTROL_BUSY);
        if (order[i] == MOTOR_MAINTENANCE_ENCODER_LUT)
        {
            encoder.calib_flag = ENC_CALIB_LINEARIZED;
        }
        if (order[i] == MOTOR_MAINTENANCE_ELECTRICAL_ZERO)
        {
            encoder.calib_flag |= ENC_CALIB_ELECTRICAL_ZERO;
        }
        previous = flow.step_ticket;
        MotorStateMachine_PostOutcome((MotorTaskResult){MOTOR_TASK_COMPLETED, MOTOR_FAULT_NONE},
                                      previous);
        tick();
        for (unsigned wait = 0U; wait < 2U; ++wait)
        {
            assert(MotorStateMachine_ReadCalibration(ticket, &flow));
            if (flow.step_ticket != previous)
            {
                break;
            }
            assert(output_off && MotorStateMachine_GetFlowState() == MOTOR_FLOW_CALIBRATION);
            tick();
        }
        MotorMaintenanceProgress child;
        assert(MotorStateMachine_ReadMaintenance(previous, &child));
        assert(child.result == MOTOR_MAINTENANCE_COMPLETED && child.ticket == previous);
        assert(MotorStateMachine_CancelMaintenance(previous) == MOTOR_CONTROL_ACCEPTED);
    }
    assert(MotorStateMachine_ReadCalibration(ticket, &flow));
    assert(flow.step == MOTOR_MAINTENANCE_SAVE && flow.step_ticket != previous);
    assert(flow.completed_steps == MOTOR_CALIBRATION_SUPPORTED_STEPS);
    assert(flow.result == MOTOR_MAINTENANCE_NO_RESULT && output_off);
    FocBackgroundClaim claim;
    assert(MotorStateMachine_BeginBackground(&claim));
    assert(MotorStateMachine_ReadCalibration(ticket, &flow) &&
           flow.result == MOTOR_MAINTENANCE_NO_RESULT);
    MotorStateMachine_FinishBackground(&claim, true);
    tick();
    assert(MotorStateMachine_ReadCalibration(ticket, &flow));
    assert(flow.result == MOTOR_MAINTENANCE_COMPLETED &&
           flow.effects == MOTOR_MAINTENANCE_EFFECT_COMMITTED);
    assert(MotorStateMachine_GetFlowState() == MOTOR_FLOW_IDLE && output_off);
    assert(MotorStateMachine_CancelCalibration(ticket) == MOTOR_CONTROL_ACCEPTED);
    puts("PASS explicit calibration dependency order, independent child tickets and final verified "
         "SAVE");
}

static void test_calibration_plan_cancel_and_failure(void)
{
    for (unsigned scenario = 0U; scenario < 3U; ++scenario)
    {
        reset();
        uint32_t ticket;
        assert(MotorStateMachine_RequestCalibration(
                   MOTOR_CALIBRATION_STEP(MOTOR_MAINTENANCE_MECHANICAL_ZERO), &ticket) ==
               MOTOR_CONTROL_ACCEPTED);
        MotorCalibrationProgress flow;
        assert(MotorStateMachine_ReadCalibration(ticket, &flow));
        MotorStateMachine_PostOutcome((MotorTaskResult){MOTOR_TASK_COMPLETED, MOTOR_FAULT_NONE},
                                      flow.step_ticket);
        tick();
        assert(MotorStateMachine_ReadCalibration(ticket, &flow) &&
               flow.step == MOTOR_MAINTENANCE_SAVE);
        FocBackgroundClaim claim;
        assert(MotorStateMachine_BeginBackground(&claim));
        if (scenario < 2U)
        {
            if (scenario == 0U)
            {
                assert(MotorStateMachine_CancelCalibration(ticket) == MOTOR_CONTROL_ACCEPTED);
            }
            else
            {
                assert(MotorStateMachine_RequestRemoteStop());
            }
            tick();
            assert(MotorStateMachine_ReadCalibration(ticket, &flow) &&
                   flow.result == MOTOR_MAINTENANCE_CANCEL_REQUESTED);
            assert(MotorStateMachine_GetFlowState() == MOTOR_FLOW_CALIBRATION);
        }
        MotorStateMachine_FinishBackground(&claim, scenario != 2U);
        tick();
        assert(MotorStateMachine_ReadCalibration(ticket, &flow));
        assert(flow.effects == MOTOR_MAINTENANCE_EFFECT_UNKNOWN);
        assert(flow.result ==
               (scenario == 2U ? MOTOR_MAINTENANCE_FAILED : MOTOR_MAINTENANCE_CANCELLED));
        assert(output_off);
        if (scenario == 2U)
        {
            assert(MotorStateMachine_GetFlowState() == MOTOR_FLOW_FAULT);
            assert(MotorStateMachine_RequestClear() == MOTOR_CONTROL_ACCEPTED);
        }
        tick();
        assert(MotorStateMachine_GetFlowState() == MOTOR_FLOW_IDLE);
    }
    reset();
    uint32_t ticket;
    assert(MotorStateMachine_RequestCalibration(MOTOR_CALIBRATION_SUPPORTED_STEPS, &ticket) ==
           MOTOR_CONTROL_ACCEPTED);
    MotorStateMachine_PostOutcome((MotorTaskResult){MOTOR_TASK_FAILED, MOTOR_FAULT_ENCODER},
                                  MotorStateMachine_WorkEpoch());
    assert(MotorStateMachine_CancelCalibration(ticket) == MOTOR_CONTROL_ACCEPTED);
    tick();
    MotorCalibrationProgress flow;
    assert(MotorStateMachine_ReadCalibration(ticket, &flow));
    assert(flow.result == MOTOR_MAINTENANCE_FAILED && flow.completed_steps == 0U);
    assert(MotorStateMachine_GetFlowState() == MOTOR_FLOW_FAULT && output_off);
    puts("PASS plan cancellation waits for borrowed SAVE, failed save cannot succeed, fault "
         "preempts cancel");
}

static void test_run_mode_switch(void)
{
    reset();
    assert(MotorStateMachine_RequestModeSwitch(MOTOR_CONTROL_NONE) == MOTOR_CONTROL_INVALID);
    assert(MotorStateMachine_RequestModeSwitch(MOTOR_CONTROL_SPEED) == MOTOR_CONTROL_DENIED);
    assert(MotorStateMachine_RequestEnable(MOTOR_CONTROL_SPEED) == MOTOR_CONTROL_ACCEPTED);
    tick();
    outer_ready = false;
    assert(MotorStateMachine_RequestModeSwitch(MOTOR_CONTROL_POSITION) == MOTOR_CONTROL_ACCEPTED);
    assert(output_off && MotorStateMachine_GetFlowState() == MOTOR_FLOW_RUN);
    assert(MotorStateMachine_RequestModeSwitch(MOTOR_CONTROL_POSITION) == MOTOR_CONTROL_ACCEPTED);
    assert(MotorStateMachine_RequestModeSwitch(MOTOR_CONTROL_CURRENT) == MOTOR_CONTROL_BUSY);
    MotorStateMachine_Tick();
    assert(output_off && start_count == 1U && MotorStateMachine_GetFlowState() == MOTOR_FLOW_RUN);
    tick();
    tick();
    assert(output_off && MotorStateMachine_GetState() == MOTOR_STATE_STARTING && start_count == 1U);
    outer_ready = true;
    tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_RUNNING && !output_off && start_count == 2U);
    assert(motor.control_mode == MOTOR_CONTROL_POSITION &&
           motor.position_target_rad == Encoder_GetMecPos(&encoder));
    assert(MotorStateMachine_RequestModeSwitch(MOTOR_CONTROL_CURRENT) == MOTOR_CONTROL_ACCEPTED);
    assert(MotorStateMachine_RequestRemoteStop());
    tick();
    assert(MotorStateMachine_GetFlowState() == MOTOR_FLOW_IDLE && output_off && start_count == 2U);
    assert(MotorStateMachine_RequestEnable(MOTOR_CONTROL_SPEED) == MOTOR_CONTROL_ACCEPTED);
    tick();
    stop_fails = true;
    assert(MotorStateMachine_RequestModeSwitch(MOTOR_CONTROL_CURRENT) == MOTOR_CONTROL_FAULT);
    assert(MotorStateMachine_GetFlowState() == MOTOR_FLOW_FATAL);
    stop_fails = false;
    tick();
    assert(MotorStateMachine_GetFlowState() == MOTOR_FLOW_FATAL);
    puts("PASS RUN mode switch closes outputs, waits for preparation, STOP revokes and failed stop "
         "is fatal");
}

static void test_identified_run_modes(void)
{
    for (unsigned mode = MOTOR_CONTROL_CURRENT; mode <= MOTOR_CONTROL_VQ; ++mode)
    {
        reset();
        MotorControlStartRequest request = {
            .source_node = 2U, .sequence = 100U, .mode = (MotorControlMode)mode};
        assert(MotorStateMachine_RequestIdentifiedRun(&request, 10U) == MOTOR_CONTROL_ACCEPTED);
        assert(MotorStateMachine_RequestIdentifiedRun(&request, 10U) == MOTOR_CONTROL_ACCEPTED);
        MotorStateMachine_Tick();
        assert(output_off && start_count == 0U &&
               MotorStateMachine_GetFlowState() == MOTOR_FLOW_RUN);
        tick();
        assert(output_off && start_count == 0U);
        tick();
        assert(MotorStateMachine_GetState() == MOTOR_STATE_RUNNING && start_count == 1U);
        MotorControlStartProgress progress;
        assert(MotorStateMachine_ReadEnableProgress(2U, 100U, &progress) &&
               progress.state == MOTOR_CONTROL_START_STARTED);
        request.sequence = 101U;
        request.mode = mode == MOTOR_CONTROL_SPEED ? MOTOR_CONTROL_CURRENT : MOTOR_CONTROL_SPEED;
        assert(MotorStateMachine_RequestIdentifiedRun(&request, 10U) == MOTOR_CONTROL_ACCEPTED);
        tick();
        assert(output_off && start_count == 1U);
        tick();
        assert(MotorStateMachine_GetState() == MOTOR_STATE_RUNNING && start_count == 2U &&
               motor.control_mode == request.mode);
        assert(MotorStateMachine_RequestIdentifiedStop(2U, 102U));
        tick();
        assert(MotorStateMachine_RequestIdentifiedRun(&request, 10U) == MOTOR_CONTROL_DENIED);
        assert(output_off && MotorStateMachine_GetState() == MOTOR_STATE_IDLE);
    }
    reset();
    MotorControlStartRequest request = {
        .source_node = 2U, .sequence = 100U, .mode = MOTOR_CONTROL_SPEED};
    assert(MotorStateMachine_RequestIdentifiedRun(&request, 10U) == MOTOR_CONTROL_ACCEPTED);
    tick();
    tick();
    now_ms = 10U;
    ++request.sequence;
    assert(MotorStateMachine_RequestIdentifiedRun(&request, 10U) == MOTOR_CONTROL_FAULT);
    assert(MotorStateMachine_GetFlowState() == MOTOR_FLOW_FAULT && output_off);
    puts("PASS four identified RUN modes share retry and STOP barriers; new command cannot hide "
         "expired lease");
}

static void test_control_lease(void)
{
    reset();
    MotorControlStartRequest pending = {
        .source_node = 2U, .sequence = 100U, .mode = MOTOR_CONTROL_SPEED};
    assert(MotorStateMachine_RequestIdentifiedRun(&pending, 10U) == MOTOR_CONTROL_ACCEPTED);
    now_ms = 10U;
    MotorStateMachine_Tick();
    assert(output_off && start_count == 0U && MotorStateMachine_GetFlowState() == MOTOR_FLOW_FAULT);
    for (unsigned wrap = 0U; wrap < 2U; ++wrap)
    {
        reset();
        now_ms = wrap ? UINT32_MAX - 5U : 0U;
        MotorControlStartRequest start = {
            .source_node = 2U, .sequence = 100U, .mode = MOTOR_CONTROL_SPEED};
        assert(MotorStateMachine_RequestIdentifiedEnable(&start) == MOTOR_CONTROL_ACCEPTED);
        assert(MotorStateMachine_RefreshControlLease(2U, 100U, 0U) == MOTOR_CONTROL_INVALID);
        assert(MotorStateMachine_RefreshControlLease(3U, 100U, 10U) == MOTOR_CONTROL_DENIED);
        assert(MotorStateMachine_RefreshControlLease(2U, 99U, 10U) == MOTOR_CONTROL_DENIED);
        assert(MotorStateMachine_RefreshControlLease(2U, 100U, 10U) == MOTOR_CONTROL_ACCEPTED);
        tick();
        now_ms += 9U;
        MotorStateMachine_CheckFastFaults();
        assert(motor.fault_code == MOTOR_FAULT_NONE && !output_off);
        ++now_ms;
        assert(MotorStateMachine_RefreshControlLease(2U, 100U, 10U) == MOTOR_CONTROL_DENIED);
        MotorStateMachine_CheckFastFaults();
        MotorStateMachine_FastFaultStop();
        assert(output_off && motor.fault_code == MOTOR_FAULT_CAN_DISCONNECTED);
        tick();
        assert(MotorStateMachine_GetFlowState() == MOTOR_FLOW_FAULT);
        assert(MotorStateMachine_RequestClear() == MOTOR_CONTROL_DENIED);
        assert(MotorStateMachine_RefreshControlLease(2U, 100U, 10U) == MOTOR_CONTROL_ACCEPTED);
        assert(MotorStateMachine_GetFlowState() == MOTOR_FLOW_FAULT && output_off);
        assert(MotorStateMachine_RequestClear() == MOTOR_CONTROL_ACCEPTED);
        tick();
        assert(MotorStateMachine_GetFlowState() == MOTOR_FLOW_IDLE && output_off &&
               start_count == 1U);
        assert(MotorStateMachine_RefreshControlLease(2U, 100U, 10U) == MOTOR_CONTROL_DENIED);
    }
    puts("PASS source-bound control lease, wrap-safe deadline, fresh link plus explicit CLEAR and "
         "no replay");
}

int main(void)
{
    test_calibration_plan();
    test_calibration_plan_cancel_and_failure();
    test_run_mode_switch();
    test_identified_run_modes();
    test_control_lease();
    test_direct_fault_recovery_and_idle_stop();
    test_target_admission_boundaries();
    test_completed_maintenance_cleanup();
    test_preparation_readiness();
    test_readiness_admission_matrix();
    test_self_test_deadline();
    test_quiet_init_and_idle_stop();
    test_startup_offset_deadline();
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
    test_identified_speed_start();
    test_start_compensation_race();
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
                                               SensorlessStartupState *s,
                                               EncoderCalibrationWorkspace *workspace)
{
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
