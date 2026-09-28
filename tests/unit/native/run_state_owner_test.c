#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "foc_run_state.h"
#include "foc_bench_speed.h"
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
static uint32_t critical_depth, seq, requested_duration_ms;

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
    seq = 0U;
    requested_duration_ms = 1000U;
    FocRunState_Init(&motor, &foc, &pi, &encoder, &startup);
    tick();
    assert(FocRunState_GetState() == APP_READY);
    assert(FocBenchSpeed.version == 3U && sizeof(FocBenchSpeed) == 92U);
}

static uint32_t command(uint32_t kind)
{
    seq += 2U;
    FocBenchSpeed.request_seq = seq - 1U;
    FocBenchSpeed.command = kind;
    FocBenchSpeed.duration_ms = requested_duration_ms;
    FocBenchSpeed.iq_limit_a = 1.0f;
    FocBenchSpeed.phase_limit_a = 6.0f;
    FocBenchSpeed.speed_limit_rad_s = 8.0f;
    FocBenchSpeed.target_speed_rad_s = 0.0f;
    FocBenchSpeed.request_seq = seq;
    tick();
    assert(FocBenchSpeed.ack_seq == seq);
    return FocBenchSpeed.ack_result;
}

static void test_control(void)
{
    reset();
    assert(command(FOC_BENCH_SPEED_START) == FOC_BENCH_SPEED_ACCEPTED);
    tick();
    assert(FocRunState_GetState() == APP_RUNNING && !output_off);
    seq += 2U;
    FocBenchSpeed.request_seq = seq - 1U;
    FocBenchSpeed.command = FOC_BENCH_SPEED_UPDATE;
    FocBenchSpeed.target_speed_rad_s = -2.0f;
    FocBenchSpeed.request_seq = seq;
    tick();
    assert(motor.speedRef == -2.0f && FocBenchSpeed.applied_seq == seq);
    MotorControlCommand goal = {.mode = MOTOR_CONTROL_SPEED, .speed_rad_s = 3.0f};
    assert(FocRunState_SubmitControl(&goal) == MOTOR_CONTROL_ACCEPTED);
    assert(FocRunState_SubmitControl(&goal) == MOTOR_CONTROL_BUSY);
    assert(FocRunState_RequestRemoteStop());
    tick();
    assert(output_off && FocRunState_GetState() == APP_READY && motor.speedRef == 0.0f);
    assert(motor.current_limit == 6.0f && motor.speed_limit == 20.0f);
    puts("PASS Watch/CAN same owner, applied sequence, STOP revocation and restored caps");

    reset();
    encoder.calib_flag = ENC_CALIB_LINEARIZED;
    assert(command(FOC_BENCH_SPEED_START) == FOC_BENCH_SPEED_DENIED);
    assert(output_off && encoder.calib_flag == ENC_CALIB_LINEARIZED);
    puts("PASS partial calibration cannot start closed loop; flags unchanged");

    reset();
    assert(command(FOC_BENCH_SPEED_START) == FOC_BENCH_SPEED_ACCEPTED);
    tick();
    FocRunState_Tick();
    assert(FocRunState_GetState() == APP_FAULT && motor.ErrorNow == ControlOverrun_Error &&
           output_off);
    assert(command(FOC_BENCH_CLEAR) == FOC_BENCH_SPEED_ACCEPTED);
    tick();
    assert(FocRunState_GetState() == APP_READY && motor.ModeNow == Motor_Disable &&
           motor.speedRef == 0.0f);
    puts(
        "PASS lifecycle guard fault projects nonzero error; fresh recovery CLEAR does not restart");
}

static void test_calibration_save(void)
{
    reset();
    encoder.calib_flag = ENC_CALIB_LINEARIZED;
    assert(command(FOC_BENCH_ELECTRICAL_ZERO) == FOC_BENCH_SPEED_ACCEPTED);
    assert(FocRunState_IsMaintenanceActive(Calib_EleAngelOffset) && !output_off);
    uint32_t origin = FocRunState_WorkEpoch();
    encoder.calib_flag = ENC_CALIB_ALL;
    FocRunState_PostOutcome(
        (MotorWorkOutcome_TypeDef){MOTOR_WORK_SWITCH_MODE, Save_Param, No_Error, true}, origin);
    tick();
    assert(output_off && FocBenchSpeed.completed_mode == Calib_EleAngelOffset);
    assert(FocBenchSpeed.operation_effects == APP_EFFECT_UNCOMMITTED);
    tick();
    assert(FocRunState_IsMaintenanceActive(Save_Param));
    FocBackgroundClaim claim;
    assert(FocRunState_BeginBackground(&claim) && claim.kind == FOC_BACKGROUND_SAVE);
    assert(FocRunState_BackgroundValid(&claim));
    FocRunState_FinishBackground(&claim, true);
    tick();
    assert(FocRunState_GetState() == APP_READY);
    assert(FocBenchSpeed.operation_result == APP_OPERATION_COMPLETED &&
           FocBenchSpeed.operation_effects == APP_EFFECT_COMMITTED);
    assert(FocBenchSpeed.completed_mode == Calib_EleAngelOffset &&
           FocBenchSpeed.calibration_flags == ENC_CALIB_ALL);
    puts(
        "PASS Mode15 completion distinct from SAVE COMMITTED, original epoch and flags observable");

    reset();
    assert(command(FOC_BENCH_ENCODER_LUT) == FOC_BENCH_SPEED_ACCEPTED);
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

static void test_fast_shutdown(void)
{
    reset();
    requested_duration_ms = 1U;
    assert(command(FOC_BENCH_SPEED_START) == FOC_BENCH_SPEED_ACCEPTED);
    tick();
    for (unsigned i = 0U; i < 20U; ++i)
    {
        FocRunState_CheckFastFaults();
    }
    assert(output_off && motor.ErrorNow == No_Error);
    tick();
    assert(FocRunState_GetState() == APP_READY &&
           FocBenchSpeed.stop_reason == FOC_BENCH_SPEED_TIMEOUT);
    puts("PASS deadline closes outputs without fabricating a guard fault");

    reset();
    assert(command(FOC_BENCH_SPEED_START) == FOC_BENCH_SPEED_ACCEPTED);
    tick();
    foc.Ia = 6.0f;
    FocRunState_CheckFastFaults();
    FocRunState_FastFaultStop();
    assert(output_off && motor.ErrorNow == Over_Current);
    tick();
    assert(FocRunState_GetState() == APP_FAULT &&
           FocBenchSpeed.stop_reason == FOC_BENCH_SPEED_PHASE_CURRENT);
    puts("PASS 6 A sampled phase cutoff immediately closes outputs and latches fault");

    reset();
    assert(command(FOC_BENCH_SPEED_START) == FOC_BENCH_SPEED_ACCEPTED);
    tick();
    stop_fails = true;
    assert(FocRunState_RequestRemoteStop());
    tick();
    assert(!output_off && FocRunState_GetState() == APP_STOPPING);
    assert(FocBenchSpeed.state == FOC_BENCH_SPEED_STOPPING);
    assert(motor.current_limit == 1.0f && motor.speed_limit == 8.0f);
    puts("PASS failed output shutdown cannot restore READY or relaxed limits");
}

static void test_borrow_cancel(void)
{
    FocBackgroundClaim claim, forged;
    reset();
    assert(command(FOC_BENCH_SAVE) == FOC_BENCH_SPEED_ACCEPTED);
    assert(FocRunState_BeginBackground(&claim));
    assert(FocRunState_RequestRemoteStop());
    tick();
    assert(FocRunState_GetState() == APP_STOPPING &&
           FocBenchSpeed.operation_result == APP_OPERATION_CANCEL_REQUESTED);
    assert(!FocRunState_BeginBackground(&forged));
    forged = claim;
    ++forged.epoch;
    FocRunState_FinishBackground(&forged, true);
    tick();
    assert(FocRunState_GetState() == APP_STOPPING);
    FocRunState_FinishBackground(&claim, true);
    tick();
    assert(FocRunState_GetState() == APP_READY &&
           FocBenchSpeed.operation_result == APP_OPERATION_CANCELLED);
    assert(motor.ModeNow == Motor_Disable &&
           FocBenchSpeed.operation_effects != APP_EFFECT_COMMITTED);
    puts("PASS SAVE borrow blocks cancellation ACK; forged/late results cannot claim COMMITTED");

    reset();
    assert(command(FOC_BENCH_COGGING) == FOC_BENCH_SPEED_ACCEPTED);
    cogging_pending = true;
    uint32_t origin = FocRunState_WorkEpoch();
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

int main(void)
{
    test_control();
    test_calibration_save();
    test_borrow_cancel();
    test_fast_shutdown();
    assert(critical_depth == 0U);
    return 0;
}
