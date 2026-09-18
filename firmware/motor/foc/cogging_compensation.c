#include "cogging_compensation.h"

#include <math.h>
#include <stddef.h>

#include "utils.h"

/* 已准入身份快照：Admit 成功时写入，Update 在 20 kHz 路径只做字段比较。 */
static bool compensation_admitted;
static CoggingCompensationIdentity admitted_identity;

volatile CoggingCompensationControl CoggingCompensation;

bool CoggingCompensation_Admit(volatile CoggingCompensationControl *control,
                               const CoggingCompensationAdmitInfo *info)
{
    if (control == NULL || info == NULL)
    {
        return false;
    }
    if (!info->mode_accepts_request || !info->error_clear || !info->sensorless_off ||
        !info->table_valid || !info->scale_matches)
    {
        compensation_admitted = false;
        control->request = 0U;
        control->enabled = 0U;
        control->rejected = 1U;
        return false;
    }
    /* 快照身份后即可在 20 kHz 路径用廉价的字段比较替代整表 CRC 复核。 */
    admitted_identity = info->identity;
    compensation_admitted = true;
    control->rejected = 0U;
    control->request = 1U;
    control->enabled = 1U;
    return true;
}

void CoggingCompensation_Disable(volatile CoggingCompensationControl *control)
{
    if (control == NULL)
    {
        return;
    }
    control->request = 0U;
    control->enabled = 0U;
}

void CoggingCompensation_ZeroBlend(volatile CoggingCompensationControl *control)
{
    if (control != NULL)
    {
        control->blend = 0.0f;
    }
}

float CoggingCompensation_Update(volatile CoggingCompensationControl *control,
                                 const CoggingMapRecord *record,
                                 const CoggingCompensationTick *tick)
{
    float limit;
    float command;
    float table = 0.0f;
    bool eligible;
    if (control == NULL || record == NULL || tick == NULL)
    {
        return 0.0f;
    }
    /* 指令所有权仍属用户：本函数只返回叠加后的总指令，不写 MotorControl.iqRef。 */
    limit = isfinite(tick->limit_a) ? fmaxf(0.0f, tick->limit_a) : 0.0f;
    command = isfinite(tick->command_a) ? tick->command_a : 0.0f;
    /* 身份结构含填充字节，逐字段比较；CRC 用准入快照，不重算整表。 */
    eligible = compensation_admitted && tick->mode_is_current && tick->error_clear &&
               tick->sensorless_off && tick->encoder_usable &&
               tick->identity.reverse == admitted_identity.reverse &&
               tick->identity.electrical_zero_q15 == admitted_identity.electrical_zero_q15 &&
               tick->identity.pole_pairs == admitted_identity.pole_pairs &&
               tick->identity.table_crc == admitted_identity.table_crc;
    if (!eligible)
    {
        /* 身份丢失或模式不匹配时立即退出，不保留渐变；资格恢复需重新准入。 */
        control->blend = 0.0f;
        control->enabled = 0U;
        if (compensation_admitted)
        {
            compensation_admitted = false;
            control->request = 0U;
        }
    }
    else
    {
        const float step = COGGING_COMPENSATION_BLEND_RATE_PER_S * tick->tick_s;
        table = constrain(CoggingMap_LookupA(record, tick->position_q15),
                          -COGGING_COMPENSATION_LIMIT_A,
                          COGGING_COMPENSATION_LIMIT_A);
        control->blend += constrain((control->enabled ? 1.0f : 0.0f) - control->blend, -step, step);
    }
    control->table_a = table;
    command = constrain(command, -limit, limit);
    control->total_a = constrain(command + table * control->blend, -limit, limit);
    control->applied_a = control->total_a - command;
    return control->total_a;
}
