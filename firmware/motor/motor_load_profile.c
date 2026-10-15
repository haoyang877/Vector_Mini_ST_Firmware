/** @file
 * @brief 同一电机的阻尼装配整定及独立存储记录，纯计算，无硬件访问。
 */
#include "motor_load_profile.h"

#include <stddef.h>

typedef char MotorLoadRecordSize[(sizeof(MotorLoadRecord) == 16U) ? 1 : -1];

/* 两配置同时存在；轮档锁定速度用约5 ms滤波，避免开环速度纹波反复清锁定计时。 */
static const MotorCalibrationProfile wheel_profile = {{0.50f,
                                                       0.30f,
                                                       2.0f,
                                                       0.15f,
                                                       0.50f,
                                                       2.0f,
                                                       0.5f,
                                                       0.0f,
                                                       250.0f,
                                                       315.0f,
                                                       2.0f,
                                                       0.20f,
                                                       0.01f,
                                                       0.25f,
                                                       0.10f,
                                                       3.0f,
                                                       0.50f,
                                                       0.20f},
                                                      15.0f,
                                                      5.0f,
                                                      8.0f,
                                                      0.50f,
                                                      0.0f,
                                                      10U};

/* 阻尼档开环 Iq 渐升至 5.75 A，矢量幅值约 5.77 A。
 * 调用方须提供 6 A 限流；预定位和电零位对齐保持 5 A。 */
static const MotorCalibrationProfile roll_profile = {{0.80f,
                                                      0.50f,
                                                      5.0f,
                                                      4.97f,
                                                      5.75f,
                                                      1.0f,
                                                      0.5f,
                                                      6.0f,
                                                      250.0f,
                                                      315.0f,
                                                      6.0f,
                                                      0.20f,
                                                      0.01f,
                                                      0.25f,
                                                      0.10f,
                                                      5.0f,
                                                      0.50f,
                                                      0.20f},
                                                     15.0f,
                                                     10.0f,
                                                     16.0f,
                                                     0.10f,
                                                     5.0f,
                                                     5U};

bool MotorLoadProfile_IsValid(uint32_t flags)
{
    return flags == 0U || flags == MOTOR_LOAD_DAMPING_RING ||
           flags == (MOTOR_LOAD_DAMPING_RING | MOTOR_LOAD_FRICTION_FEEDFORWARD);
}

const MotorCalibrationProfile *MotorLoadProfile_Calibration(uint32_t flags)
{
    if (!MotorLoadProfile_IsValid(flags))
    {
        return NULL;
    }
    return (flags & MOTOR_LOAD_DAMPING_RING) != 0U ? &roll_profile : &wheel_profile;
}

bool MotorLoadProfile_FeedforwardEnabled(uint32_t flags)
{
    return flags == (MOTOR_LOAD_DAMPING_RING | MOTOR_LOAD_FRICTION_FEEDFORWARD);
}

bool MotorLoadProfile_PassiveHoldEnabled(uint32_t flags)
{
    return flags == MOTOR_LOAD_DAMPING_RING ||
           flags == (MOTOR_LOAD_DAMPING_RING | MOTOR_LOAD_FRICTION_FEEDFORWARD);
}

static uint32_t record_crc(const MotorLoadRecord *record)
{
    const uint32_t words[3] = {record->magic, record->version, record->flags};
    uint32_t crc = 0xFFFFFFFFU;
    unsigned word, byte, bit;
    for (word = 0U; word < 3U; ++word)
    {
        for (byte = 0U; byte < 4U; ++byte)
        {
            crc ^= (words[word] >> (8U * byte)) & 0xFFU;
            for (bit = 0U; bit < 8U; ++bit)
            {
                crc = (crc >> 1) ^ (0xEDB88320U & (0U - (crc & 1U)));
            }
        }
    }
    return ~crc;
}

bool MotorLoadRecord_Create(uint32_t flags, MotorLoadRecord *output)
{
    MotorLoadRecord record;
    if (output == NULL || !MotorLoadProfile_IsValid(flags))
    {
        return false;
    }
    record.magic = MOTOR_LOAD_RECORD_MAGIC;
    record.version = MOTOR_LOAD_RECORD_VERSION;
    record.flags = flags;
    record.crc32 = record_crc(&record);
    *output = record;
    return true;
}

bool MotorLoadRecord_Load(const MotorLoadRecord *record, uint32_t *flags)
{
    if (record == NULL || flags == NULL)
    {
        return false;
    }
    if ((record->magic == 0U && record->version == 0U && record->flags == 0U &&
         record->crc32 == 0U) ||
        (record->magic == UINT32_MAX && record->version == UINT32_MAX &&
         record->flags == UINT32_MAX && record->crc32 == UINT32_MAX))
    {
        *flags = 0U;
        return true;
    }
    if (record->magic != MOTOR_LOAD_RECORD_MAGIC || record->version != MOTOR_LOAD_RECORD_VERSION ||
        !MotorLoadProfile_IsValid(record->flags) || record->crc32 != record_crc(record))
    {
        return false;
    }
    *flags = record->flags;
    return true;
}
