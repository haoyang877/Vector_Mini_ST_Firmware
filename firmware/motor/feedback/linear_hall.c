#include "linear_hall.h"

#include <math.h>
#include <string.h>
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
    /* 新幅值或失败结果不能搭配旧误差表。零表仍可输出原始角。 */
    memset(hall->lut_q15, 0, sizeof(hall->lut_q15));
    hall->lut_status = 0U;
    hall->filter_initialized = false;
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

static bool update_angle(LinearHall *hall, float sin_counts, float cos_counts)
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
        return false;
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
    sample->angle_raw_rad = angle;
    float coordinate = angle * ((float)LINEAR_HALL_LUT_SIZE / _2PI);
    unsigned index = (unsigned)coordinate;
    float fraction = coordinate - (float)index;
    index &= LINEAR_HALL_LUT_SIZE - 1U;
    float a = (float)hall->lut_q15[index];
    float b = (float)hall->lut_q15[(index + 1U) & (LINEAR_HALL_LUT_SIZE - 1U)];
    angle += (a + fraction * (b - a)) * (_2PI / 65536.0f);
    if (angle < 0.0f)
    {
        angle += _2PI;
    }
    if (angle >= _2PI)
    {
        angle -= _2PI;
    }
    sample->angle_rad = angle;
    return true;
}

void LinearHall_Update(LinearHall *hall, float sin_counts, float cos_counts)
{
    (void)update_angle(hall, sin_counts, cos_counts);
}

static float wrap_delta(float angle)
{
    if (angle > _PI)
    {
        angle -= _2PI;
    }
    if (angle < -_PI)
    {
        angle += _2PI;
    }
    return angle;
}

static float wrap_angle(float angle)
{
    if (angle < 0.0f)
    {
        angle += _2PI;
    }
    if (angle >= _2PI)
    {
        angle -= _2PI;
    }
    return angle;
}

void LinearHall_UpdateFiltered(LinearHall *hall, float sin_counts, float cos_counts)
{
    if (!update_angle(hall, sin_counts, cos_counts))
    {
        return;
    }
    float angle = hall->val.angle_rad;
    if (!hall->filter_initialized)
    {
        hall->filter_previous_rad[0] = hall->filter_previous_rad[1] = angle;
        hall->filter_stage_rad[0] = hall->filter_stage_rad[1] = angle;
        hall->filter_initialized = true;
        return;
    }
    float previous = hall->filter_previous_rad[0];
    float current_delta = wrap_delta(angle - previous);
    float older_delta = wrap_delta(hall->filter_previous_rad[1] - previous);
    float middle = current_delta > 0.0f ? fmaxf(0.0f, fminf(current_delta, older_delta))
                                        : fminf(0.0f, fmaxf(current_delta, older_delta));
    hall->filter_previous_rad[1] = previous;
    hall->filter_previous_rad[0] = angle;
    angle = wrap_angle(previous + middle);
    /* 中值去孤立突跳；两级相同极点平滑连续纹波，2*y1-y2补偿匀速滞后。
     * 4 kHz下alpha=1/16，单级极点约41 Hz；状态保持单圈值以免累计精度损失。 */
    float first = wrap_angle(hall->filter_stage_rad[0] +
                             0.0625f * wrap_delta(angle - hall->filter_stage_rad[0]));
    float second = wrap_angle(hall->filter_stage_rad[1] +
                              0.0625f * wrap_delta(first - hall->filter_stage_rad[1]));
    hall->filter_stage_rad[0] = first;
    hall->filter_stage_rad[1] = second;
    hall->val.angle_rad = wrap_angle(first + wrap_delta(first - second));
}

float LinearHall_getAngle(const LinearHall *hall)
{
    return hall->val.angle_rad;
}
