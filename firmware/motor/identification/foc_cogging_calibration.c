#include "foc_cogging_calibration.h"
#include <math.h>
#include "foc_calibration.h"
#include "foc_errhandle.h"
#include "hw_conf.h"
#include "utils.h"
#include "bus_voltage_profile.h"

extern MotorControl_TypeDef MotorControl;
extern Encoder_TypeDef OnBoard_Encoder;
CoggingCalibration CoggingCalib;
CoggingFaultSnapshot CoggingFault;
volatile uint32_t CoggingTelemetryState;
volatile CoggingTelemetryFrame CoggingTelemetry;
static bool session_started;
static volatile bool finalize_pending, save_pending;
static uint32_t divider;
static int64_t start_shadow;
static float start_angle, calibration_current;
static float hold_position, previous_position, hold_velocity;
static float reference_velocity, iq_block_sum;
static uint8_t start_reverse;
static uint16_t start_zero;
static int32_t start_pole_pairs;
volatile CoggingCompensationControl CoggingCompensation;
volatile CoggingTorqueGuard TorqueGuard;
volatile uint32_t TorqueTelemetryState;
volatile CoggingTorqueFrame TorqueTelemetry;
static bool compensation_valid;
static uint8_t compensation_reverse;
static uint16_t compensation_zero;
static int32_t compensation_poles;
static uint32_t compensation_crc, torque_tick;

bool FocCogging_SetCompensation(bool enabled)
{
    if (!enabled) {
        CoggingCompensation.request = CoggingCompensation.enabled = 0U;
        return true;
    }
    if ((MotorControl.ModeNow != Motor_Disable && MotorControl.ModeNow != Current_Mode) ||
        MotorControl.ErrorNow != No_Error || MotorControl.isUseSensorless ||
        !FocCogging_TableValid() || CoggingMap.full_scale_a != CURRENT_SENSE_PROFILE_FULL_SCALE_A) {
        compensation_valid = false;
        CoggingCompensation.request = CoggingCompensation.enabled = 0U;
        CoggingCompensation.rejected = 1U;
        return false;
    }
    compensation_reverse = OnBoard_Encoder.reverse;
    compensation_zero = OnBoard_Encoder.electrical_zero_q15;
    compensation_poles = MotorControl.motor_pole_pairs;
    compensation_crc = CoggingMap.crc32;
    compensation_valid = true;
    CoggingCompensation.rejected = 0U;
    CoggingCompensation.request = CoggingCompensation.enabled = 1U;
    return true;
}

float FocCogging_Apply(const MotorControl_TypeDef *m, const Encoder_TypeDef *e)
{
    float limit = isfinite(m->current_limit) ? fmaxf(0.0f, m->current_limit) : 0.0f;
    float command = isfinite(m->iqRef) ? m->iqRef : 0.0f;
    float table = 0.0f;
    const bool eligible = compensation_valid && m->ModeNow == Current_Mode &&
        m->ErrorNow == No_Error && !m->isUseSensorless && Encoder_IsOnline(e) &&
        (e->calib_flag & ENC_CALIB_ALL) == ENC_CALIB_ALL &&
        e->reverse == compensation_reverse && e->electrical_zero_q15 == compensation_zero &&
        m->motor_pole_pairs == compensation_poles && CoggingMap.crc32 == compensation_crc;
    if (TorqueGuard.enabled) limit = fminf(limit, fmaxf(0.0f, TorqueGuard.current_limit_a));
    if (!eligible) {
        CoggingCompensation.blend = 0.0f;
        CoggingCompensation.enabled = 0U;
        /* Keep a first enable request pending until foreground validates it.
         * Invalidate a previously admitted map immediately on identity loss. */
        if (compensation_valid) {
            compensation_valid = false;
            CoggingCompensation.request = 0U;
        }
    } else {
        unsigned index = e->linearized_q15 >> 6;
        float fraction = (float)(e->linearized_q15 & 63U) / 64.0f;
        float a = (float)CoggingMap.iq_q15[index];
        float b = (float)CoggingMap.iq_q15[(index + 1U) & 1023U];
        table = (a + (b-a)*fraction) * (CURRENT_SENSE_PROFILE_FULL_SCALE_A / 32768.0f);
        table = constrain(table, -1.0f, 1.0f);
        CoggingCompensation.blend += constrain((CoggingCompensation.enabled ? 1.0f : 0.0f) -
            CoggingCompensation.blend, -5.0f*Current_Ts, 5.0f*Current_Ts);
    }
    CoggingCompensation.table_a = table;
    command = constrain(command, -limit, limit);
    CoggingCompensation.total_a = constrain(command + table*CoggingCompensation.blend, -limit, limit);
    CoggingCompensation.applied_a = CoggingCompensation.total_a - command;
    return CoggingCompensation.total_a;
}

bool FocCogging_TorqueGuard(MotorControl_TypeDef *m, const Encoder_TypeDef *e)
{
    uint32_t trip = 0U;
    if (!TorqueGuard.enabled) return true;
    if (TorqueGuard.lease_ticks == 0U) trip = 1U;
    else --TorqueGuard.lease_ticks;
    if (!isfinite(TorqueGuard.speed_limit_rad_s) || TorqueGuard.speed_limit_rad_s <= 0.0f ||
        !isfinite(e->vel_mech_continuous) || fabsf(e->vel_mech_continuous) > TorqueGuard.speed_limit_rad_s)
        trip = 2U;
    if (!isfinite(TorqueGuard.current_limit_a) || TorqueGuard.current_limit_a <= 0.0f ||
        !isfinite(m->iqRef) || fabsf(m->iqRef) > TorqueGuard.current_limit_a || m->isUseSensorless)
        trip = 3U;
    if (!trip) return true;
    TorqueGuard.trip = trip;
    Stop_PWM_Generate();
    m->iqRef = m->idRef = 0.0f;
    CoggingCompensation.request = CoggingCompensation.enabled = 0U;
    CoggingCompensation.blend = 0.0f;
    Set_ModeNow(Motor_Disable);
    return false;
}

void FocCogging_TorqueObserve(const FOC_TypeDef *f, const MotorControl_TypeDef *m,
    const Encoder_TypeDef *e)
{
    ++torque_tick;
    if (TorqueTelemetryState != 1U) return;
    TorqueTelemetry.tick = torque_tick;
    TorqueTelemetry.position_rad = (float)e->shadow_q15 * (_2PI / 65536.0f);
    TorqueTelemetry.velocity_rad_s = e->vel_mech_continuous;
    TorqueTelemetry.command_a = m->iqRef;
    TorqueTelemetry.compensation_a = CoggingCompensation.applied_a;
    TorqueTelemetry.total_a = CoggingCompensation.total_a;
    TorqueTelemetry.feedback_a = f->Iq;
    TorqueTelemetry.vbus_v = f->Vbus_filt;
    TorqueTelemetry.blend = CoggingCompensation.blend;
    TorqueTelemetryState = 2U;
}

bool FocCogging_CanStart(const MotorControl_TypeDef *m, const Encoder_TypeDef *e)
{
    /* A full mechanical turn must never be attempted on bounded roll/pitch axes. */
    return m != NULL && e != NULL && m->axis_profile_valid && m->axis_profile.magic == 0U &&
        m->ErrorNow == No_Error && Encoder_IsOnline(e) && e->velocity_ready &&
        (e->calib_flag & ENC_CALIB_ALL) == ENC_CALIB_ALL &&
        isfinite(m->current_limit) && m->current_limit > 0.0f &&
        isfinite(m->calib_current) && m->calib_current > 0.0f &&
        isfinite(m->speed_limit) && m->speed_limit > 0.0f &&
        isfinite(m->speed_Kp) && m->speed_Kp > 0.0f &&
        isfinite(m->speed_Ki) && m->speed_Ki > 0.0f &&
        isfinite(Encoder_GetMecVelContinuous(e)) &&
        fabsf(Encoder_GetMecVelContinuous(e)) < 0.05f && !finalize_pending && !save_pending;
}

static void Stop(FOC_TypeDef *f, MotorControl_TypeDef *m, PI_Controller_TypeDef *pi)
{
    Stop_PWM_Generate();
    m->idRef = m->iqRef = m->speedRef = m->speedShadow = 0.0f;
    PI_Controller_Reset(pi); FOC_CurrentController_Reset(f);
    session_started = false;
    CalibStep = CS_NULL;
    Set_ModeNow(Motor_Disable);
}

void FocCogging_Abort(void)
{
    CoggingCalibration_Abort(&CoggingCalib, COGGING_CANCELLED);
    session_started = false;
    CalibStep = CS_NULL;
}

void FocCogging_Task(FOC_TypeDef *f, MotorControl_TypeDef *m,
    PI_Controller_TypeDef *pi, Encoder_TypeDef *e)
{
    float position, velocity, speed_target;
    if (!session_started) {
        if (!FocCogging_CanStart(m, e) || !isfinite(f->Vbus) || !isfinite(f->Vbus_filt) ||
            f->Vbus >= BUS_VOLTAGE_HARD_OVERVOLTAGE_V || f->Vbus_filt > BUS_VOLTAGE_ENABLE_MAX_V ||
            f->Vbus_filt < BUS_VOLTAGE_ENABLE_MIN_V || !isfinite(f->temp) || f->temp >= 100.0f ||
            !isfinite(f->Iq) || fabsf(f->Iq) > 1.5f ||
            !isfinite(Encoder_GetElePhase(e)) || !isfinite(Encoder_GetEleVel(e))) {
            CoggingCalib.state = COGGING_FAILED; CoggingCalib.reason = COGGING_INVALID_INPUT;
            Set_ErrorNow(CoggingCalibration_Error); Stop(f, m, pi); return;
        }
        start_shadow = e->shadow_q15;
        CoggingFault = (CoggingFaultSnapshot){0};
        start_angle = (float)e->linearized_q15 * (_2PI / 65536.0f);
        hold_position = previous_position = start_angle;
        hold_velocity = 0.0f;
        reference_velocity = iq_block_sum = 0.0f;
        start_reverse = e->reverse; start_zero = e->electrical_zero_q15;
        start_pole_pairs = m->motor_pole_pairs;
        calibration_current = fminf(1.0f, fminf(m->calib_current, m->current_limit));
        (void)CoggingCalibration_Start(&CoggingCalib, start_angle, CURRENT_SENSE_PROFILE_FULL_SCALE_A);
        divider = 0U; PI_Controller_Reset(pi); FOC_CurrentController_Reset(f);
        m->idRef = m->iqRef = m->speedRef = m->speedShadow = 0.0f;
        session_started = true;
        /* This mode owns power admission; the generic mode edge excludes it. */
        FOC_Current(f, m, Encoder_GetElePhase(e), Encoder_GetEleVel(e));
        Start_PWM_Generate();
    }
    position = start_angle + (float)(e->shadow_q15 - start_shadow) * (_2PI / 65536.0f);
    velocity = Encoder_GetMecVelContinuous(e);
    if (!Encoder_IsOnline(e) || (e->calib_flag & ENC_CALIB_ALL) != ENC_CALIB_ALL ||
        e->reverse != start_reverse || e->electrical_zero_q15 != start_zero ||
        m->motor_pole_pairs != start_pole_pairs ||
        !isfinite(Encoder_GetElePhase(e)) || !isfinite(Encoder_GetEleVel(e)) ||
        !isfinite(position) || !isfinite(velocity) || fabsf(velocity) > 1.5f ||
        !isfinite(f->Vbus) || !isfinite(f->Vbus_filt) ||
        f->Vbus >= BUS_VOLTAGE_HARD_OVERVOLTAGE_V ||
        f->Vbus_filt > BUS_VOLTAGE_ENABLE_MAX_V || f->Vbus_filt < BUS_VOLTAGE_ENABLE_MIN_V ||
        !isfinite(f->Iq) || fabsf(f->Iq) > calibration_current + 0.5f ||
        !isfinite(f->temp) || f->temp >= 100.0f ||
        !isfinite(m->current_limit) || m->current_limit < calibration_current ||
        !isfinite(m->speed_limit) || m->speed_limit <= 0.0f ||
        !isfinite(m->speed_Kp) || m->speed_Kp <= 0.0f ||
        !isfinite(m->speed_Ki) || m->speed_Ki <= 0.0f ||
        m->ErrorNow != No_Error) {
        CoggingFault.velocity_rad_s = velocity;
        CoggingFault.iq_a = f->Iq;
        CoggingFault.iq_reference_a = m->iqRef;
        CoggingFault.position_rad = position;
        CoggingFault.target_rad = CoggingCalib.target_rad;
        CoggingFault.vbus_v = f->Vbus_filt;
        CoggingFault.temperature_c = f->temp;
        CoggingFault.encoder_status = e->read_status;
        CoggingFault.previous_fault = m->ErrorNow;
        CoggingCalibration_Abort(&CoggingCalib, COGGING_SAFETY_FAULT);
        if (m->ErrorNow == No_Error) Set_ErrorNow(CoggingCalibration_Error);
        Stop(f, m, pi); return;
    }
    /* Integrate all 20 kHz feedback samples before decimating to 2 kHz. */
    iq_block_sum += f->Iq;
    if (++divider >= SPEED_LOOP_DIVIDER) {
        CoggingTelemetryFrame frame;
        const float iq_average = iq_block_sum / (float)SPEED_LOOP_DIVIDER;
        const bool publish = CoggingTelemetryState == 1U;
        iq_block_sum = 0.0f;
        divider = 0U;
        /* A one-raw-code hysteretic position tracker suppresses differentiating
         * standstill quantization back and forth. Small persistent motion still
         * accumulates and is tracked; the raw position admission is unchanged. */
        const float deadband = _2PI / 32768.0f;
        float delta = position - previous_position;
        delta = delta > deadband ? delta - deadband :
            (delta < -deadband ? delta + deadband : 0.0f);
        previous_position += delta;
        hold_velocity += 0.25f * (delta / Speed_Ts - hold_velocity);
        if (publish) {
            frame.tick = CoggingCalib.total_ticks + 1U;
            frame.state = CoggingCalib.state; frame.index = CoggingCalib.index;
            frame.direction_pass = CoggingCalib.direction_pass;
            frame.points_done = CoggingCalib.points_done;
            frame.target_rad = CoggingCalib.target_rad; frame.reference_rad = hold_position;
            frame.position_rad = position; frame.error_rad = frame.target_rad - position;
            frame.velocity_rad_s = velocity; frame.iq_average_a = iq_average;
            frame.iq_reference_a = m->iqRef; frame.vbus_v = f->Vbus_filt;
            frame.holding_velocity_rad_s = hold_velocity;
        }
        CoggingCalibration_Update(&CoggingCalib, position, velocity, iq_average,
            fabsf(m->iqRef) >= calibration_current * 0.98f);
        if (publish) {
            frame.accepted = CoggingCalib.last_sample_accepted;
            CoggingTelemetry = frame;
            CoggingTelemetryState = 2U;
        }
        if (CoggingCalib.state == COGGING_FAILED) {
            Set_ErrorNow(CoggingCalibration_Error); Stop(f, m, pi); return;
        }
        if (CoggingCalib.state == COGGING_COMPLETE) {
            Stop(f, m, pi); finalize_pending = true; return;
        }
        /* Direct position holding provides stiffness against the local cogging
         * gradient. Keep this tuning independent of the saved running loops.
         * Shift the PI bounds by damping so anti-windup sees the final Iq limit. */
        /* The normal 16-sample speed window delays damping at standstill.
         * Use a local ~90 Hz derivative filter and a slow position ramp. */
        /* Acceleration/deceleration limits reduce endpoint excitation without
         * raising position gains. The reference reaches the exact grid point. */
        const float distance = CoggingCalib.target_rad - hold_position;
        const float braking_speed = sqrtf(4.0f * fabsf(distance)); /* a = 2 rad/s^2 */
        speed_target = copysignf(fminf(braking_speed, fminf(0.1f, m->speed_limit)), distance);
        reference_velocity += constrain(speed_target - reference_velocity, -2.0f * Speed_Ts, 2.0f * Speed_Ts);
        const float step = reference_velocity * Speed_Ts;
        if (distance * step >= 0.0f && fabsf(step) >= fabsf(distance)) {
            hold_position = CoggingCalib.target_rad; reference_velocity = 0.0f;
        } else hold_position += step;
        const float damping = 0.12f * hold_velocity;
        PI_Controller_Configure(pi, 60.0f, 240.0f, Speed_Ts,
            -calibration_current + damping, calibration_current + damping);
        m->speedRef = m->speedShadow = reference_velocity;
        m->idRef = 0.0f;
        m->iqRef = constrain(PI_Controller_Run(pi, hold_position, position) - damping,
            -calibration_current, calibration_current);
    }
    CalibStep = CoggingCalib.direction_pass == 0U ?
        (CoggingCalib.state == COGGING_SAMPLING ? CS_ANTICOGGING_CW_SAMPLE : CS_ANTICOGGING_CW_TEMP) :
        (CoggingCalib.state == COGGING_SAMPLING ? CS_ANTICOGGING_CCW_SAMPLE : CS_ANTICOGGING_CCW_TEMP);
    FOC_Current(f, m, Encoder_GetElePhase(e), Encoder_GetEleVel(e));
}

static uint32_t Signature(void)
{
    return Cogging_EncoderSignature(OnBoard_Encoder.reverse, OnBoard_Encoder.electrical_zero_q15,
        MotorControl.motor_pole_pairs, OnBoard_Encoder.linearization_lut_q15);
}

bool FocCogging_TableValid(void)
{
    return (OnBoard_Encoder.calib_flag & ENC_CALIB_ALL) == ENC_CALIB_ALL &&
        CoggingMap_Valid(&CoggingMap, Signature());
}

void FocCogging_Service(void)
{
    uint32_t primask;
    if (MotorControl.ModeNow != Motor_Disable && MotorControl.ModeNow != Current_Mode) {
        (void)FocCogging_SetCompensation(false);
        CoggingCompensation.blend = 0.0f;
    } else {
        if (MotorControl.ModeNow == Motor_Disable) CoggingCompensation.blend = 0.0f;
        if (CoggingCompensation.request != CoggingCompensation.enabled)
            (void)FocCogging_SetCompensation(CoggingCompensation.request == 1U);
    }
    if (!finalize_pending || MotorControl.ModeNow != Motor_Disable) return;
    /* Claim the non-driving save state before long CRC/copy work. CAN commands
     * cannot start another motor mode while the completed candidate is consumed. */
    primask = __get_PRIMASK();
    __disable_irq();
    if (!finalize_pending || MotorControl.ModeNow != Motor_Disable) {
        __set_PRIMASK(primask); return;
    }
    save_pending = true;
    finalize_pending = false;
    Set_ModeNow(Save_Param);
    __set_PRIMASK(primask);
    if (MotorControl.ErrorNow != No_Error || !CoggingCalibration_Finish(&CoggingCalib, Signature(), &CoggingMap)) {
        CoggingCalib.state = COGGING_FAILED; CoggingCalib.reason = COGGING_INVALID_INPUT;
        save_pending = false;
        Set_ModeNow(Motor_Disable);
        return;
    }
}

CoggingState FocCogging_GetState(void)
{
    return (finalize_pending || save_pending) ? COGGING_SAVING : CoggingCalib.state;
}

void FocCogging_SaveResult(bool success)
{
    if (!save_pending) return;
    save_pending = false;
    if (!success) { CoggingCalib.state = COGGING_FAILED; CoggingCalib.reason = COGGING_SAVE_FAILED; }
}
