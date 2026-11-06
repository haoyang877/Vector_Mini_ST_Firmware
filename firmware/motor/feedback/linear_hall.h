#ifndef LINEAR_HALL_H
#define LINEAR_HALL_H

#include <stdbool.h>
#include <stdint.h>

#define HALL_CAL_MIN_COUNTS 10.0f
#define HALL_CAL_MAX_COUNTS (4096.0f - HALL_CAL_MIN_COUNTS)
#define HALL_CAL_MIN_AMP_COUNTS 800.0f

/**
 * @brief 校验单通道存储系数能否还原为合格的 ADC 极值。
 * @param offset 中点，单位 count。
 * @param amp 半峰峰值，单位 count。
 * @return 幅值和两端极值均满足标定阈值时返回 true。
 */
static inline bool LinearHall_CoefficientsInRange(float offset, float amp)
{
    return amp >= HALL_CAL_MIN_AMP_COUNTS && offset - amp >= HALL_CAL_MIN_COUNTS &&
           offset + amp <= HALL_CAL_MAX_COUNTS;
}

/* 最近一次 Hall 标定的故障位，同次标定的多个原因可组合。 */
typedef enum
{
    LINEAR_HALL_CAL_FAULT_NONE = 0,
    LINEAR_HALL_CAL_FAULT_NO_SAMPLES = 1U << 0,
    LINEAR_HALL_CAL_FAULT_SIN_LOW = 1U << 1,
    LINEAR_HALL_CAL_FAULT_SIN_HIGH = 1U << 2,
    LINEAR_HALL_CAL_FAULT_COS_LOW = 1U << 3,
    LINEAR_HALL_CAL_FAULT_COS_HIGH = 1U << 4,
    LINEAR_HALL_CAL_FAULT_SIN_AMP_LOW = 1U << 5,
    LINEAR_HALL_CAL_FAULT_COS_AMP_LOW = 1U << 6
} LinearHallCalibrationFault;

/* 外环同拍的原码、归一化信号及输出轴单圈角（rad）。 */
typedef struct
{
    float sin_counts, cos_counts;
    float sin_cal, cos_cal;
    float angle_rad;
} LinearHallSample;

/* 每轴一个实例；两个偏置单位为 ADC count，两路幅值单位同为 ADC count。 */
typedef struct
{
    float sin_off_counts, cos_off_counts;
    float sin_amp, cos_amp;
    bool calibration_success;   /* 最近一次标定结果；没有已存结果时为 false。 */
    uint8_t calibration_faults; /* 随系数恢复的故障位；成功或没有已存结果时为零。 */
    LinearHallSample val;
} LinearHall;

/* 本次标定的两路 ADC 原码极值。 */
typedef struct
{
    float sin_min, sin_max;
    float cos_min, cos_max;
} LinearHallCalibration;

/**
 * @brief 清空上次扫圈并开始记录极值。
 * @param cal 调用方独占的标定工作区；同一组标定调用须串行。
 */
void LinearHall_CalibrationBegin(LinearHallCalibration *cal);

/**
 * @brief 逐拍记录两路 ADC 原码极值，保留超限原码供结束时检查。
 * @param cal 已开始的标定工作区。
 * @param sin_counts 本拍 sin 原始 ADC 值，单位 count。
 * @param cos_counts 同拍 cos 原始 ADC 值，单位 count。
 * @note 调用方在 Begin 之后、标定采样阶段传入两路有限 ADC 原码，并覆盖至少一整圈。
 */
void LinearHall_CalibrationSample(LinearHallCalibration *cal, float sin_counts, float cos_counts);

/**
 * @brief 结束本次标定；用两路极值分别计算偏置和幅值，写入当前轴 RAM。
 * @param cal 当前扫圈工作区。
 * @param hall 待更新的轴对象；与角度更新在同一上下文串行调用。
 * @note 调用方在 Begin 之后结束采样，再调用一次；更新 calibration_success 和 calibration_faults。
 * @return 两路原码均在 [10, 4086] count 内且各自半峰峰值至少 800 count 时返回 true；失败保留原系数。
 */
bool LinearHall_CalibrationFinish(LinearHallCalibration *cal, LinearHall *hall);

/**
 * @brief 使用当前实测偏置和幅值初始化两路霍尔。
 * @param hall 当前轴独占的霍尔对象。
 */
void LinearHall_Init(LinearHall *hall);

/**
 * @brief 独立去偏并除以各通道幅值，更新 atan2 单圈角。
 * @param hall 当前轴的霍尔对象；本拍结果写入 val。
 * @param sin_counts 本拍同步读取的 sin ADC 原码，缺样传 NAN。
 * @param cos_counts 同拍 cos ADC 原码，缺样传 NAN。
 * @note 外环单写；缺样或归一化后两路同时为零时保持上一角度，初值为零。
 */
void LinearHall_Update(LinearHall *hall, float sin_counts, float cos_counts);

/**
 * @brief 获取当前霍尔传感器的角度。
 * @param hall 当前轴的霍尔对象。
 * @return 当前角度（rad）。
 */
float LinearHall_getAngle(const LinearHall *hall);

#endif
