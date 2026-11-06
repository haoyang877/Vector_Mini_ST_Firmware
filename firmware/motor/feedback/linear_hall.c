#include "linear_hall.h"

#include <math.h>
#include "control_math.h"

void LinearHall_CalibrationBegin(LinearHallCalibration *cal)
{
    *cal = (LinearHallCalibration){
        .sin_min = INFINITY, .sin_max = -INFINITY, .cos_min = INFINITY, .cos_max = -INFINITY};
}

void LinearHall_CalibrationSample(LinearHallCalibration *cal, float sin_counts, float cos_counts)
{
    cal->sin_min = fminf(cal->sin_min, sin_counts);
    cal->sin_max = fmaxf(cal->sin_max, sin_counts);
    cal->cos_min = fminf(cal->cos_min, cos_counts);
    cal->cos_max = fmaxf(cal->cos_max, cos_counts);
}

bool LinearHall_CalibrationFinish(LinearHallCalibration *cal, LinearHall *hall)
{
    float sin_amp = (cal->sin_max - cal->sin_min) * 0.5f;
    float cos_amp = (cal->cos_max - cal->cos_min) * 0.5f;
    uint8_t faults = LINEAR_HALL_CAL_FAULT_NONE;
    if (cal->sin_min > cal->sin_max)
    {
        /* 两路成对接收，未收到样本时没有可判断的极值和幅值。 */
        faults = LINEAR_HALL_CAL_FAULT_NO_SAMPLES;
    }
    else
    {
        if (cal->sin_min < HALL_CAL_MIN_COUNTS)
        {
            faults |= LINEAR_HALL_CAL_FAULT_SIN_LOW;
        }
        if (cal->sin_max > HALL_CAL_MAX_COUNTS)
        {
            faults |= LINEAR_HALL_CAL_FAULT_SIN_HIGH;
        }
        if (cal->cos_min < HALL_CAL_MIN_COUNTS)
        {
            faults |= LINEAR_HALL_CAL_FAULT_COS_LOW;
        }
        if (cal->cos_max > HALL_CAL_MAX_COUNTS)
        {
            faults |= LINEAR_HALL_CAL_FAULT_COS_HIGH;
        }
        if (sin_amp < HALL_CAL_MIN_AMP_COUNTS)
        {
            faults |= LINEAR_HALL_CAL_FAULT_SIN_AMP_LOW;
        }
        if (cos_amp < HALL_CAL_MIN_AMP_COUNTS)
        {
            faults |= LINEAR_HALL_CAL_FAULT_COS_AMP_LOW;
        }
    }
    hall->calibration_faults = faults;
    if (faults != LINEAR_HALL_CAL_FAULT_NONE)
    {
        hall->calibration_success = false;
        return false;
    }
    /* 扫圈中任一路触及超限区或幅值不足，都保留两路原系数。 */
    hall->sin_off_counts = (cal->sin_max + cal->sin_min) * 0.5f;
    hall->cos_off_counts = (cal->cos_max + cal->cos_min) * 0.5f;
    hall->sin_amp = sin_amp;
    hall->cos_amp = cos_amp;
    hall->calibration_success = true;
    return true;
}

void LinearHall_Init(LinearHall *hall)
{
    *hall = (LinearHall){.sin_off_counts = 1930.5f,
                         .cos_off_counts = 1906.0f,
                         .sin_amp = 1829.5f,
                         .cos_amp = 1787.0f,
                         .calibration_success = false};
}

void LinearHall_Update(LinearHall *hall, float sin_counts, float cos_counts)
{
    LinearHallSample *sample = &hall->val;
    sample->sin_counts = sin_counts;
    sample->cos_counts = cos_counts;
    sample->sin_cal = (sin_counts - hall->sin_off_counts) / hall->sin_amp;
    sample->cos_cal = (cos_counts - hall->cos_off_counts) / hall->cos_amp;

    /* 缺样或两个通道同时过中心时没有可用方向，保留上一拍角度。 */
    if (!isfinite(sample->sin_cal) || !isfinite(sample->cos_cal) ||
        (sample->sin_cal == 0.0f && sample->cos_cal == 0.0f))
    {
        return;
    }
    sample->sin_cal = fminf(1.0f, fmaxf(-1.0f, sample->sin_cal));
    sample->cos_cal = fminf(1.0f, fmaxf(-1.0f, sample->cos_cal));
    float angle = -Math_Atan2(sample->sin_cal, sample->cos_cal); //反向
    if (angle < 0.0f)
    {
        angle += _2PI;
    }
    if (angle >= _2PI)
    {
        angle = 0.0f;
    }
    sample->angle_rad = angle;
}

float LinearHall_getAngle(const LinearHall *hall)
{
    return hall->val.angle_rad;
}
