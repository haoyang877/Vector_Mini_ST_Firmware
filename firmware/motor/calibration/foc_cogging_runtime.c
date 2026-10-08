#include "foc_cogging_runtime.h"

#include <math.h>
#include <string.h>

#include "control_config.h"
#include "critical_hw.h"
#include "current_sense_profile.h"

/* 运行适配所有者：活动记录、补偿准入/应用、力矩 guard 与同拍遥测。
 * 不读标定会话，不推进采样，不决定功率许可，也不执行 Flash 提交。 */
static CoggingMapRecord s_cogging_map;
/* 前台与快速 ISR 共享该私有控制状态，保留原有 volatile 访问语义。 */
static volatile CoggingCompensationControl s_cogging_compensation;
static volatile uint8_t s_compensation_request;
static volatile CoggingTorqueGuard s_torque_guard;
static volatile uint32_t s_torque_sample_state;
static volatile CoggingTorqueFrame s_torque_sample;
/* 台架保护跳闸只挂起 STOP，真实关相由 APP 消费结果后执行。 */
static volatile bool s_torque_trip_pending;
static uint32_t s_torque_tick;

bool FocCogging_SetCompensation(bool enabled,
                                const MotorControlState *motor_state,
                                const EncoderState *rotor_encoder)
{
    CoggingCompensationAdmitInfo info;
    if (!enabled)
    {
        CoggingCompensation_Disable(&s_cogging_compensation);
        return true;
    }
    info.sensorless_off = !motor_state->uses_sensorless_feedback;
    info.table_valid = FocCogging_TableValid(motor_state, rotor_encoder);
    info.scale_matches = s_cogging_map.full_scale_a == CURRENT_SENSE_PROFILE_FULL_SCALE_A;
    info.identity.reverse = Encoder_GetReverse(rotor_encoder);
    info.identity.electrical_zero_q15 = rotor_encoder->electrical_zero_q15;
    info.identity.pole_pairs = motor_state->model.motor_pole_pairs;
    info.identity.table_crc = s_cogging_map.crc32;
    /* CRC 在开中断状态计算；提交前重读可被 STOP/FAULT 改变的资格。 */
    uint32_t mask = critical_hw_enter();
    info.mode_accepts_request = motor_state->control_mode == MOTOR_CONTROL_NONE ||
                                motor_state->control_mode == MOTOR_CONTROL_CURRENT;
    info.error_clear = motor_state->fault_code == MOTOR_FAULT_NONE;
    bool accepted = CoggingCompensation_Admit(&s_cogging_compensation, &info);
    critical_hw_exit(mask);
    return accepted;
}

float FocCogging_Apply(const MotorControlState *m, const EncoderState *e)
{
    CoggingCompensationTick tick;
    tick.mode_is_current = m->control_mode == MOTOR_CONTROL_CURRENT;
    tick.error_clear = m->fault_code == MOTOR_FAULT_NONE;
    tick.sensorless_off = !m->uses_sensorless_feedback;
    tick.encoder_usable = Encoder_IsOnline(e) && (e->calib_flag & ENC_CALIB_ALL) == ENC_CALIB_ALL;
    tick.command_a = m->iq_target_a;
    tick.limit_a = m->application.current_limit_a;
    tick.tick_s = Current_Ts;
    tick.position_q15 = e->linearized_q15;
    tick.identity.reverse = e->reverse;
    tick.identity.electrical_zero_q15 = e->electrical_zero_q15;
    tick.identity.pole_pairs = m->model.motor_pole_pairs;
    tick.identity.table_crc = s_cogging_map.crc32;
    return CoggingCompensation_Update(&s_cogging_compensation, &s_cogging_map, &tick);
}

bool FocCogging_SetTorqueGuard(uint32_t lease_ticks, float speed_limit_rad_s)
{
    if (lease_ticks != 0U && (!isfinite(speed_limit_rad_s) || speed_limit_rad_s <= 0.0f))
    {
        return false;
    }
    uint32_t mask = critical_hw_enter();
    s_torque_guard.lease_ticks = lease_ticks;
    s_torque_guard.speed_limit_rad_s = lease_ticks != 0U ? speed_limit_rad_s : 0.0f;
    s_torque_guard.enabled = lease_ticks != 0U;
    critical_hw_exit(mask);
    return true;
}

bool FocCogging_ReadTorqueGuard(CoggingTorqueGuard *snapshot)
{
    if (snapshot == NULL)
    {
        return false;
    }
    uint32_t mask = critical_hw_enter();
    *snapshot = s_torque_guard;
    critical_hw_exit(mask);
    return true;
}

bool FocCogging_RequestTorqueSample(void)
{
    uint32_t mask = critical_hw_enter();
    bool accepted = s_torque_sample_state == 0U;
    if (accepted)
    {
        s_torque_sample_state = 1U;
    }
    critical_hw_exit(mask);
    return accepted;
}

bool FocCogging_ReadTorqueSample(CoggingTorqueFrame *sample)
{
    if (sample == NULL)
    {
        return false;
    }
    uint32_t mask = critical_hw_enter();
    bool ready = s_torque_sample_state == 2U;
    if (ready)
    {
        *sample = s_torque_sample;
        s_torque_sample_state = 0U;
    }
    critical_hw_exit(mask);
    return ready;
}

bool FocCogging_TorqueGuard(MotorControlState *m, const EncoderState *e)
{
    uint32_t trip = 0U;
    if (!s_torque_guard.enabled)
    {
        return true;
    }
    /* 租约：主机必须持续续租，进程失联即到期停机。 */
    if (s_torque_guard.lease_ticks == 0U)
    {
        trip = 1U;
    }
    else
    {
        --s_torque_guard.lease_ticks;
    }
    /* 测速上限：覆盖主机存活但脚本下发错误指令的情形。 */
    if (!isfinite(s_torque_guard.speed_limit_rad_s) || s_torque_guard.speed_limit_rad_s <= 0.0f ||
        !isfinite(e->mechanical_velocity_continuous_rad_s) ||
        fabsf(e->mechanical_velocity_continuous_rad_s) > s_torque_guard.speed_limit_rad_s)
    {
        trip = 2U;
    }
    if (trip == 0U)
    {
        return true;
    }
    s_torque_guard.trip = trip;
    s_torque_trip_pending = true;
    m->iq_target_a = m->id_target_a = 0.0f;
    CoggingCompensation_Disable(&s_cogging_compensation);
    CoggingCompensation_ZeroBlend(&s_cogging_compensation);
    return false;
}

bool FocCogging_TakeTorqueTrip(void)
{
    bool pending = s_torque_trip_pending;
    s_torque_trip_pending = false;
    return pending;
}

void FocCogging_TorqueObserve(const FocState *f, const MotorControlState *m, const EncoderState *e)
{
    ++s_torque_tick;
    if (s_torque_sample_state != 1U)
    {
        return;
    }
    s_torque_sample.tick = s_torque_tick;
    s_torque_sample.position_rad = Encoder_GetMecPos(e);
    s_torque_sample.velocity_rad_s = e->mechanical_velocity_continuous_rad_s;
    s_torque_sample.command_a = m->iq_target_a;
    s_torque_sample.compensation_a = s_cogging_compensation.applied_a;
    s_torque_sample.total_a = s_cogging_compensation.total_a;
    s_torque_sample.feedback_a = f->iq_a;
    s_torque_sample.vbus_v = f->bus_voltage_filtered_v;
    s_torque_sample.blend = s_cogging_compensation.blend;
    s_torque_sample_state = 2U;
}

bool FocCogging_TableValid(const MotorControlState *motor_state, const EncoderState *rotor_encoder)
{
    return (Encoder_GetCalibFlag(rotor_encoder) & ENC_CALIB_ALL) == ENC_CALIB_ALL &&
           CoggingMap_Valid(&s_cogging_map,
                            Cogging_EncoderSignature(Encoder_GetReverse(rotor_encoder),
                                                     rotor_encoder->electrical_zero_q15,
                                                     motor_state->model.motor_pole_pairs,
                                                     rotor_encoder->linearization_lut_q15));
}

void FocCogging_RequestCompensation(bool enabled)
{
    uint32_t mask = critical_hw_enter();
    s_compensation_request = enabled ? 1U : 2U;
    critical_hw_exit(mask);
}

void FocCogging_Service(MotorControlState *motor_state, EncoderState *rotor_encoder)
{
    uint32_t mask = critical_hw_enter();
    if (s_compensation_request != 0U)
    {
        s_cogging_compensation.request = s_compensation_request == 1U ? 1U : 0U;
        s_compensation_request = 0U;
    }
    critical_hw_exit(mask);
    if (motor_state->control_mode != MOTOR_CONTROL_NONE &&
        motor_state->control_mode != MOTOR_CONTROL_CURRENT)
    {
        /* 离开允许模式立即关断补偿；停机预开启由 request 位保持。 */
        CoggingCompensation_Disable(&s_cogging_compensation);
        CoggingCompensation_ZeroBlend(&s_cogging_compensation);
    }
    else
    {
        if (motor_state->control_mode == MOTOR_CONTROL_NONE)
        {
            CoggingCompensation_ZeroBlend(&s_cogging_compensation);
        }
        if (s_cogging_compensation.request != s_cogging_compensation.enabled)
        {
            (void)FocCogging_SetCompensation(
                s_cogging_compensation.request == 1U, motor_state, rotor_encoder);
        }
    }
}

void FocCogging_ResetMap(void)
{
    memset(&s_cogging_map, 0, sizeof(s_cogging_map));
}

void FocCogging_ImportMap(const CoggingMapRecord *record)
{
    s_cogging_map = *record;
}

void FocCogging_ExportMap(CoggingMapRecord *record,
                          const MotorControlState *motor,
                          const EncoderState *encoder)
{
    memset(record, 0, sizeof(*record));
    if (FocCogging_TableValid(motor, encoder))
    {
        *record = s_cogging_map;
    }
}
