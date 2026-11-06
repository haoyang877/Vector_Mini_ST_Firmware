#ifndef OUTPUT_ENCODER_H
#define OUTPUT_ENCODER_H

#include <stdbool.h>
#include <stdint.h>
#include "encoder_config.h"
#include "linear_hall.h"

#define OUTPUT_ENCODER_CAL_OK 0x80U
#define OUTPUT_ENCODER_ZERO_SET 0x100U
#define OUTPUT_ENCODER_CAL_SAMPLES 256U

typedef enum
{
    OUTPUT_ENCODER_CAL_NONE = 0,
    OUTPUT_ENCODER_CAL_NO_SAMPLES = 1,
    OUTPUT_ENCODER_CAL_HALL = 2,
    OUTPUT_ENCODER_CAL_RELATION = 3
} OutputEncoderCalibFault;

/* 输出方向以 Hall 正向为基准；直驱默认沿电机编码器正向。status 保存结果和机械零位资格。 */
typedef struct
{
    float gear_ratio, phase_rad, zero_rad;
    int32_t direction;
    uint32_t status;
} OutputEncoderParams;

typedef struct
{
    OutputEncoderParams params;
    float position_rad, initial_position_rad;
    int64_t motor_delta_q15;
    uint16_t previous_motor_q15;
    bool ready;
} OutputEncoder;

/* 只保存首个输出圈的稀疏原码，结束后用新 Hall 系数及新 LUT 计算对应关系。 */
typedef struct
{
    int32_t motor_counts;
    uint16_t sin_counts, cos_counts;
} OutputEncoderCalibSample;

typedef struct
{
    OutputEncoderCalibSample samples[OUTPUT_ENCODER_CAL_SAMPLES];
    int32_t motor_counts, first_counts, step_counts;
    uint16_t previous_q15, count;
} OutputEncoderCalib;

/**
 * @brief 初始化输出轴反馈，减速比为 1 时使用直驱编码器。
 * @param encoder 独占输出状态。
 * @param gear_ratio 正整数减速比；非整数传动不支持单圈绝对辨圈。
 */
void OutputEncoder_Init(OutputEncoder *encoder, float gear_ratio);

/**
 * @brief 由同拍角度建立输出轴单圈绝对初始位置。
 * @param encoder 已加载参数且尚未定位的对象。
 * @param motor_q15 线性化后的电机单圈计数，65536 counts/转。
 * @param hall_rad Hall 单圈角，rad；直驱忽略。
 * @note 调用方保证传感器就绪、标定成功及减速比匹配；位置归入 [0, 2π)。
 */
void OutputEncoder_Locate(OutputEncoder *encoder, uint16_t motor_q15, float hall_rad);

/**
 * @brief 累加电机单圈增量，更新输出轴连续位置。
 * @param encoder 已定位的对象。
 * @param motor_q15 本拍电机单圈计数，65536 counts/转。
 * @note 单写外环调用；相邻采样实际位移必须小于半个电机圈。
 */
void OutputEncoder_Update(OutputEncoder *encoder, uint16_t motor_q15);

/**
 * @brief 读取输出轴连续位置。
 * @param encoder 已定位的对象。
 * @return 相对输出机械零位的位置，rad，运行中不按圈回绕。
 */
float OutputEncoder_GetPos(const OutputEncoder *encoder);

/**
 * @brief 查询本次上电绝对定位是否完成。
 * @param encoder 输出状态。
 * @return 已定位返回 true。
 */
bool OutputEncoder_IsReady(const OutputEncoder *encoder);

/**
 * @brief 将当前位置设置为输出轴机械零位。
 * @param encoder 已定位且位于工装零点的对象。
 * @note 调用方负责停机与持久化；不修改电机换相零位。
 */
void OutputEncoder_SetZero(OutputEncoder *encoder);

/**
 * @brief 清空配对样本，按输出轴 1.01 圈设置采样间隔。
 * @param cal 独占工作区。
 * @param gear_ratio 当前轴的正整数减速比。
 */
void OutputEncoder_CalibBegin(OutputEncoderCalib *cal, float gear_ratio);

/**
 * @brief 按实际电机位移记录 Hall 与电机编码器配对原码。
 * @param cal 已开始的工作区。
 * @param motor_q15 方向校正后的电机原码，尚未使用 LUT。
 * @param sin_counts 同拍 Hall sin 原码。
 * @param cos_counts 同拍 Hall cos 原码。
 * @note 4 kHz 单写，仅在观测器采样阶段调用，不做拟合。
 */
void OutputEncoder_CalibSample(OutputEncoderCalib *cal,
                               uint16_t motor_q15,
                               uint16_t sin_counts,
                               uint16_t cos_counts);

/**
 * @brief 用最终 Hall 系数和编码器 LUT 计算方向、相位并检查辨圈余量。
 * @param encoder 待提交的输出对象；成功清除旧机械零位，失败保留系数并记录原因。
 * @param cal 已停止采集的工作区。
 * @param hall 本次完成幅值、偏置标定的 Hall 对象。
 * @param motor 本次新发布的电机编码器 LUT。
 * @return 配对样本覆盖整圈且关系一致时返回 true。
 * @note 只在可抢占的停机前台调用，调用方复核会话后提交结果。
 */
bool OutputEncoder_CalibEnd(OutputEncoder *encoder,
                            const OutputEncoderCalib *cal,
                            const LinearHall *hall,
                            const EncoderCalibration *motor);

#endif
