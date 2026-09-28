#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "foc_run_state.h"
#include "foc_mode_dispatch.h"
#include "foc_cogging_calibration.h"
#include "foc_encoder_calibration.h"
#include "foc_friction_identification.h"
#include "foc_phase_resistance.h"
#include "foc_errhandle.h"
#include "foc_run.h"
#include "foc_sensing.h"
#include "foc_sensorless_run.h"
#include "power_stage_hw.h"
#include "critical_hw.h"
#include "indicator_hw.h"
#include "bus_voltage_profile.h"
#include "boot_hw.h"
#include "utils.h"
#define FOC_MAX_MODULATION (0.95f * 0.866025403f)

/* 真实生产所有者和类型；仅替换硬件与 worker，验证命令、资源和身份交接。 */
static MotorControl_TypeDef motor;
static FOC_TypeDef foc;
static PI_Controller_TypeDef pi;
static Encoder_TypeDef encoder;
static SensorlessStartup_TypeDef startup;
static bool output_off, stop_fails, temperature_fresh, encoder_released;
static bool cogging_active, cogging_pending, cogging_claimed, cogging_cancelled;
static bool outer_ready;
static bool check_borrowed_requests;
static uint32_t critical_depth;

/* 本夹具模拟普通 APP 的无 Loader 端口，配置修订号仍有可用空间。 */
void BootHw_GetInfo(BootHwInfo *info)
{
    *info = (BootHwInfo){0};
}
bool MotorControl_ConfigurationRevisionAvailable(void)
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
    output_off = false;
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
bool Encoder_IsOnline(const Encoder_TypeDef *e)
{
    return e->bad_frame_streak == 0U;
}
uint16_t Encoder_GetBadFrameStreak(const Encoder_TypeDef *e)
{
    return e->bad_frame_streak;
}
uint8_t Encoder_GetCalibFlag(const Encoder_TypeDef *e)
{
    return e->calib_flag;
}
float Encoder_GetMecPos(const Encoder_TypeDef *e)
{
    return e->theta_mech;
}
float Encoder_GetMecVel(const Encoder_TypeDef *e)
{
    return e->vel_mech;
}
bool MotorTemperature_IsFresh(void)
{
    return temperature_fresh;
}
float fast_abs(float value)
{
    return fabsf(value);
}
void FOC_CurrentController_Reset(FOC_TypeDef *f)
{
    f->Id = f->Iq = 0.0f;
}
void PI_Controller_Reset(PI_Controller_TypeDef *p)
{
    memset(p, 0, sizeof(*p));
}
void SensorlessStartup_Reset(SensorlessStartup_TypeDef *s)
{
    memset(s, 0, sizeof(*s));
}
void MotorOuterLoop_RequestReset(void)
{
}
bool MotorOuterLoop_IsReady(void)
{
    return outer_ready;
}
void FocEncoderCalibration_Cancel(void)
{
    encoder_released = true;
}
bool FocEncoderCalibration_IsReleased(void)
{
    return encoder_released;
}
CalibStep_TyepeDef FocEncoderCalibration_GetStep(void)
{
    return encoder_released ? CS_NULL : CS_OBS_ALIGN_ORIGIN;
}
void PhaseResistanceMode_Cancel(FOC_TypeDef *f, MotorControl_TypeDef *m)
{
    (void)f;
    (void)m;
}
bool PhaseResistanceMode_IsReleased(void)
{
    return true;
}
void FocFrictionIdentification_Abort(MotorControl_TypeDef *m, PI_Controller_TypeDef *p)
{
    (void)m;
    (void)p;
}
bool FocFrictionIdentification_IsReleased(void)
{
    return true;
}
bool FocCogging_CanStart(const MotorControl_TypeDef *m, const Encoder_TypeDef *e)
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
void FocCogging_Service(MotorControl_TypeDef *m, Encoder_TypeDef *e)
{
    (void)m;
    (void)e;
    if (check_borrowed_requests)
    {
        uint32_t ticket = UINT32_MAX;
        assert(FocRunState_RequestEnable(MOTOR_CONTROL_SPEED) == MOTOR_CONTROL_BUSY);
        assert(FocRunState_RequestClear() == MOTOR_CONTROL_BUSY);
        assert(FocRunState_RequestMaintenance(MOTOR_MAINTENANCE_SAVE, &ticket) ==
               MOTOR_CONTROL_BUSY);
        assert(ticket == 0U);
        check_borrowed_requests = false;
    }
}

static void tick(void)
{
    FocRunState_PostOutcome(
        (MotorWorkOutcome_TypeDef){MOTOR_WORK_RUNNING, Motor_Disable, No_Error, false},
        FocRunState_WorkEpoch());
    FocRunState_Tick();
}

static void reset(void)
{
    memset(&motor, 0, sizeof(motor));
    memset(&foc, 0, sizeof(foc));
    memset(&encoder, 0, sizeof(encoder));
    motor.axis_profile_valid = true;
    motor.motor_pole_pairs = 21;
    motor.current_limit = 6.0f;
    motor.speed_limit = 20.0f;
    motor.speedAcc = motor.speedDec = 1.0f;
    motor.speed_Kp = 0.1f;
    motor.speed_Ki = 0.2f;
    motor.A_Offset = motor.B_Offset = motor.C_Offset = 2048.0f;
    encoder.calib_flag = ENC_CALIB_ALL;
    foc.Vbus = foc.Vbus_filt = 30.0f;
    foc.temp = 25.0f;
    output_off = outer_ready = temperature_fresh = encoder_released = true;
    stop_fails = cogging_active = cogging_pending = cogging_claimed = cogging_cancelled = false;
    FocRunState_Init(&motor, &foc, &pi, &encoder, &startup);
    tick();
    assert(FocRunState_GetState() == APP_READY);
}

/* 以公共维护入口开启会话并返回创建时 ticket；不再经由 SWD 台架邮箱。 */
static uint32_t begin_maintenance(MotorMaintenanceKind kind)
{
    uint32_t ticket = 0U;
    assert(FocRunState_RequestMaintenance(kind, &ticket) == MOTOR_CONTROL_ACCEPTED);
    assert(ticket != 0U);
    return ticket;
}

static void test_control(void)
{
    reset();
    assert(FocRunState_RequestEnable(MOTOR_CONTROL_SPEED) == MOTOR_CONTROL_ACCEPTED);
    tick();
    tick();
    assert(FocRunState_GetState() == APP_RUNNING && !output_off);
    MotorControlCommand goal = {.mode = MOTOR_CONTROL_SPEED, .speed_rad_s = -2.0f};
    assert(FocRunState_SubmitControl(&goal) == MOTOR_CONTROL_ACCEPTED);
    assert(FocRunState_SubmitControl(&goal) == MOTOR_CONTROL_BUSY);
    tick();
    assert(motor.speedRef == -2.0f);
    assert(FocRunState_RequestRemoteStop());
    tick();
    assert(output_off && FocRunState_GetState() == APP_READY && motor.speedRef == 0.0f);
    assert(motor.current_limit == 6.0f && motor.speed_limit == 20.0f);
    puts("PASS same-mode target owner, single-slot busy, STOP revocation and unchanged caps");

    reset();
    encoder.calib_flag = ENC_CALIB_LINEARIZED;
    assert(FocRunState_RequestEnable(MOTOR_CONTROL_SPEED) == MOTOR_CONTROL_DENIED);
    assert(output_off && encoder.calib_flag == ENC_CALIB_LINEARIZED);
    puts("PASS partial calibration cannot start closed loop; flags unchanged");

    reset();
    assert(FocRunState_RequestEnable(MOTOR_CONTROL_CURRENT) == MOTOR_CONTROL_ACCEPTED);
    tick();
    tick();
    assert(FocRunState_GetState() == APP_RUNNING);
    FocRunState_PostOutcome(
        (MotorWorkOutcome_TypeDef){MOTOR_WORK_FAULT, Motor_Disable, Encoder_Error, false},
        FocRunState_WorkEpoch());
    tick();
    assert(FocRunState_GetState() == APP_FAULT && motor.ErrorNow == Encoder_Error && output_off);
    assert(FocRunState_RequestClear() == MOTOR_CONTROL_ACCEPTED);
    tick();
    assert(FocRunState_GetState() == APP_READY && motor.ErrorNow == No_Error &&
           motor.ModeNow == Motor_Disable && motor.speedRef == 0.0f);
    puts("PASS worker fault latches error; fresh recovery CLEAR does not restart");
}

static void test_calibration_save(void)
{
    MotorMaintenanceProgress progress;
    uint32_t ticket, origin, save;
    reset();
    encoder.calib_flag = ENC_CALIB_LINEARIZED;
    ticket = begin_maintenance(MOTOR_MAINTENANCE_ELECTRICAL_ZERO);
    assert(FocRunState_IsMaintenanceActive(Calib_EleAngelOffset) && !output_off);
    origin = FocRunState_WorkEpoch();
    encoder.calib_flag = ENC_CALIB_ALL;
    FocRunState_PostOutcome(
        (MotorWorkOutcome_TypeDef){MOTOR_WORK_SWITCH_MODE, Save_Param, No_Error, true}, origin);
    tick();
    assert(output_off && FocRunState_ReadMaintenance(ticket, &progress));
    assert(progress.effects == APP_EFFECT_UNCOMMITTED);
    tick();
    assert(FocRunState_IsMaintenanceActive(Save_Param));
    save = FocRunState_WorkEpoch();
    FocBackgroundClaim claim;
    assert(FocRunState_BeginBackground(&claim) && claim.kind == FOC_BACKGROUND_SAVE);
    assert(FocRunState_BackgroundValid(&claim));
    FocRunState_FinishBackground(&claim, true);
    tick();
    assert(FocRunState_GetState() == APP_READY);
    assert(FocRunState_ReadMaintenance(save, &progress));
    assert(progress.result == APP_OPERATION_COMPLETED && progress.effects == APP_EFFECT_COMMITTED);
    assert(encoder.calib_flag == ENC_CALIB_ALL);
    puts("PASS Mode15 completion distinct from SAVE COMMITTED; derived ticket observable");

    reset();
    ticket = begin_maintenance(MOTOR_MAINTENANCE_ENCODER_LUT);
    assert(FocRunState_IsMaintenanceActive(Calib_EncoderObserver));
    origin = FocRunState_WorkEpoch();
    assert(FocRunState_RequestRemoteStop());
    tick();
    FocRunState_PostOutcome(
        (MotorWorkOutcome_TypeDef){MOTOR_WORK_SWITCH_MODE, Save_Param, No_Error, true}, origin);
    tick();
    assert(motor.ModeNow == Motor_Disable && output_off && FocRunState_GetState() == APP_READY);
    puts("PASS Mode13 explicit entry remains; stale completion after STOP cannot request SAVE");
}

static void test_stop_and_fault(void)
{
    reset();
    assert(FocRunState_RequestEnable(MOTOR_CONTROL_CURRENT) == MOTOR_CONTROL_ACCEPTED);
    tick();
    tick();
    assert(FocRunState_GetState() == APP_RUNNING && !output_off);
    encoder.bad_frame_streak = ENCODER_BAD_FRAME_OFFLINE_COUNT;
    FocRunState_CheckFastFaults();
    FocRunState_FastFaultStop();
    assert(output_off && motor.ErrorNow == Encoder_Error);
    puts("PASS encoder offline fast check latches error and closes outputs");

    reset();
    assert(FocRunState_RequestEnable(MOTOR_CONTROL_CURRENT) == MOTOR_CONTROL_ACCEPTED);
    tick();
    tick();
    stop_fails = true;
    assert(FocRunState_RequestRemoteStop());
    tick();
    assert(!output_off && FocRunState_GetState() == APP_STOPPING);
    assert(motor.current_limit == 6.0f && motor.speed_limit == 20.0f);
    puts("PASS failed output shutdown cannot restore READY or relaxed limits");
}

static void test_borrow_cancel(void)
{
    FocBackgroundClaim claim, forged;
    MotorMaintenanceProgress progress;
    uint32_t ticket, origin;
    reset();
    ticket = begin_maintenance(MOTOR_MAINTENANCE_SAVE);
    assert(FocRunState_BeginBackground(&claim));
    assert(FocRunState_RequestRemoteStop());
    tick();
    assert(FocRunState_GetState() == APP_STOPPING &&
           FocRunState_ReadMaintenance(ticket, &progress) &&
           progress.result == APP_OPERATION_CANCEL_REQUESTED);
    assert(!FocRunState_BeginBackground(&forged));
    forged = claim;
    ++forged.epoch;
    FocRunState_FinishBackground(&forged, true);
    tick();
    assert(FocRunState_GetState() == APP_STOPPING);
    FocRunState_FinishBackground(&claim, true);
    tick();
    assert(FocRunState_GetState() == APP_READY && FocRunState_ReadMaintenance(ticket, &progress) &&
           progress.result == APP_OPERATION_CANCELLED);
    assert(motor.ModeNow == Motor_Disable && progress.effects != APP_EFFECT_COMMITTED);
    puts("PASS SAVE borrow blocks cancellation ACK; forged/late results cannot claim COMMITTED");

    reset();
    ticket = begin_maintenance(MOTOR_MAINTENANCE_COGGING);
    cogging_pending = true;
    origin = FocRunState_WorkEpoch();
    FocRunState_PostOutcome(
        (MotorWorkOutcome_TypeDef){MOTOR_WORK_SWITCH_MODE, Motor_Disable, No_Error, true}, origin);
    tick();
    assert(FocRunState_GetState() == APP_MAINTENANCE && output_off);
    assert(FocRunState_BeginBackground(&claim) && claim.kind == FOC_BACKGROUND_COGGING);
    assert(FocRunState_RequestRemoteStop());
    tick();
    assert(cogging_cancelled && FocRunState_GetState() == APP_STOPPING);
    FocRunState_FinishBackground(&claim, false);
    tick();
    assert(!cogging_claimed && FocRunState_GetState() == APP_READY &&
           motor.ModeNow == Motor_Disable);
    puts("PASS cogging natural completion stays in maintenance; STOP waits for original foreground "
         "release");
}

/* 公共入口验证拒绝优先级；不注入运行所有者的私有状态。 */
static void expect_busy_requests(void)
{
    uint32_t ticket = UINT32_MAX;
    ModeNow_TypeDef mode = motor.ModeNow;
    assert(FocRunState_RequestEnable(MOTOR_CONTROL_SPEED) == MOTOR_CONTROL_BUSY);
    assert(FocRunState_RequestClear() == MOTOR_CONTROL_BUSY);
    assert(FocRunState_RequestMaintenance(MOTOR_MAINTENANCE_SAVE, &ticket) == MOTOR_CONTROL_BUSY);
    assert(ticket == 0U && motor.ModeNow == mode);
    assert(!FocRunState_ConfigurationAllowed());

    /* Enable/Maintenance 先报故障；Clear 仍先等待占用释放。 */
    motor.ErrorNow = Encoder_Error;
    assert(FocRunState_RequestEnable(MOTOR_CONTROL_SPEED) == MOTOR_CONTROL_FAULT);
    assert(FocRunState_RequestMaintenance(MOTOR_MAINTENANCE_SAVE, &ticket) == MOTOR_CONTROL_FAULT);
    assert(FocRunState_RequestClear() == MOTOR_CONTROL_BUSY);
    assert(FocRunState_RequestEnable((MotorControlMode)UINT32_MAX) == MOTOR_CONTROL_INVALID);
    assert(FocRunState_RequestMaintenance((MotorMaintenanceKind)UINT32_MAX, &ticket) ==
           MOTOR_CONTROL_INVALID);
    assert(FocRunState_RequestMaintenance(MOTOR_MAINTENANCE_SAVE, NULL) == MOTOR_CONTROL_INVALID);
    assert(ticket == 0U && motor.ModeNow == mode);
}

static void test_request_admission(void)
{
    static const ModeNow_TypeDef modes[] = {Current_Mode, Speed_Mode, Position_Mode, Vq_Mode};
    for (unsigned i = 0U; i < sizeof(modes) / sizeof(modes[0]); ++i)
    {
        reset();
        assert(FocRunState_RequestEnable((MotorControlMode)i) == MOTOR_CONTROL_ACCEPTED);
        assert(motor.ModeNow == modes[i] && output_off);
        tick();
        tick();
        assert(FocRunState_GetState() == APP_RUNNING && !output_off);
        MotorControlCommand goal = {.mode = (MotorControlMode)i};
        assert(FocRunState_SubmitControl(&goal) == MOTOR_CONTROL_ACCEPTED);
        expect_busy_requests();
    }
    reset();
    assert(FocRunState_RequestEnable((MotorControlMode)4) == MOTOR_CONTROL_INVALID);
    assert(FocRunState_RequestEnable((MotorControlMode)UINT32_MAX) == MOTOR_CONTROL_INVALID);
    assert(motor.ModeNow == Motor_Disable && output_off);

    reset();
    assert(FocRunState_RequestBootStop(UINT64_C(0x100000001)) == MOTOR_CONTROL_ACCEPTED);
    expect_busy_requests();

    reset();
    FocRunState_PostOutcome(
        (MotorWorkOutcome_TypeDef){MOTOR_WORK_STOP, Motor_Disable, No_Error, false},
        FocRunState_WorkEpoch());
    expect_busy_requests();

    reset();
    uint32_t ticket;
    assert(FocRunState_RequestMaintenance(MOTOR_MAINTENANCE_SAVE, &ticket) ==
           MOTOR_CONTROL_ACCEPTED);
    assert(ticket != 0U);
    expect_busy_requests();

    reset();
    FocBackgroundClaim claim;
    assert(FocRunState_RequestMaintenance(MOTOR_MAINTENANCE_SAVE, &ticket) ==
           MOTOR_CONTROL_ACCEPTED);
    assert(FocRunState_BeginBackground(&claim));
    expect_busy_requests();

    reset();
    check_borrowed_requests = true;
    FocRunState_ServiceCompensation();
    assert(!check_borrowed_requests && FocRunState_ConfigurationAllowed());

    reset();
    encoder_released = false;
    assert(FocRunState_RequestEnable(MOTOR_CONTROL_SPEED) == MOTOR_CONTROL_DENIED);
    assert(FocRunState_RequestClear() == MOTOR_CONTROL_BUSY);
    assert(FocRunState_RequestMaintenance(MOTOR_MAINTENANCE_SAVE, &ticket) == MOTOR_CONTROL_BUSY);
    assert(ticket == 0U && output_off);
    puts("PASS four control mappings, busy sources, fault/invalid priority and release guards");
}

static void test_maintenance_identity(void)
{
    reset();
    MotorMaintenanceRequest request = {MOTOR_MAINTENANCE_ELECTRICAL_ZERO, 2U, 1U, 0U};
    uint32_t origin, retry, save;
    assert(FocRunState_RequestIdentifiedMaintenance(&request, &origin) == MOTOR_CONTROL_ACCEPTED);
    assert(FocRunState_RequestIdentifiedMaintenance(&request, &retry) == MOTOR_CONTROL_ACCEPTED);
    assert(origin != 0U && retry == origin);
    request.kind = MOTOR_MAINTENANCE_ENCODER_LUT;
    assert(FocRunState_RequestIdentifiedMaintenance(&request, &retry) ==
           MOTOR_CONTROL_CONFLICTING_RETRY);
    assert(retry == 0U);
    FocRunState_PostOutcome(
        (MotorWorkOutcome_TypeDef){MOTOR_WORK_SWITCH_MODE, Save_Param, No_Error, true}, origin);
    tick();
    tick();
    save = FocRunState_WorkEpoch();
    assert(save != origin && FocRunState_IsMaintenanceActive(Save_Param));
    FocBackgroundClaim claim;
    assert(FocRunState_BeginBackground(&claim));
    FocRunState_FinishBackground(&claim, true);
    tick();
    MotorMaintenanceProgress progress;
    assert(FocRunState_ReadMaintenance(origin, &progress));
    assert(progress.kind == MOTOR_MAINTENANCE_ELECTRICAL_ZERO &&
           progress.result == APP_OPERATION_COMPLETED &&
           progress.effects == APP_EFFECT_UNCOMMITTED);
    assert(FocRunState_ReadMaintenance(save, &progress));
    assert(progress.kind == MOTOR_MAINTENANCE_SAVE && progress.effects == APP_EFFECT_COMMITTED);
    puts("PASS identified retry and original calibration terminal survive derived SAVE completion");
}

int main(void)
{
    test_request_admission();
    test_maintenance_identity();
    test_control();
    test_calibration_save();
    test_borrow_cancel();
    test_stop_and_fault();
    assert(critical_depth == 0U);
    return 0;
}
