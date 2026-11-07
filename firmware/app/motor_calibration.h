#ifndef MOTOR_CALIBRATION_H
#define MOTOR_CALIBRATION_H
#include "application_config.h"
#include "motor_lifecycle.h"
#include "output_encoder.h"
#include "bsp_flash.h"
#include "parameter_record.h"

/** @brief 单轴标定的候选与工作区；绑定的活动对象在整个会话内有效。 */
typedef struct
{
    EncoderCalibrationControl drive;
    MotorLifecycle *life;
    MotorOuterLoop *outer;
    LinearHall *hall;
    OutputEncoder *output;
    MotorAxisRecord *record;
    BspResult (*write_record)(const void *data, size_t length);
    EncoderCalibrationSession session;
    EncoderCalibrationWorkspace workspace;
#if BSP_ENABLED_LINEAR_HALL
    LinearHallCalibration hall_sweep;
    OutputEncoderCalib output_calibration;
    uint32_t coast_epoch, coast_start_ms, stable_since_ms;
#endif
} MotorCalibration;

/**
 * @brief 推进当前轴的一次快拍标定。
 * @param cal 当前轴独占的标定会话。
 * @return 本拍任务结果，由调用方按原代际提交。
 * @note 仅 ADC 快拍调用。
 */
MotorTaskResult MotorCalibration_StepFast(MotorCalibration *cal);

/**
 * @brief 在停机前台发布原会话的标定结果并保存参数。
 * @param cal 当前轴的候选和本机记录。
 * @param progress 长计算期间的健康服务。
 * @param progress_context 健康服务所属的应用实例。
 * @note 在最终提交前复核原代际及数字关相，Flash 回读失败锁存故障。
 */
void MotorCalibration_ProcessPublication(MotorCalibration *cal,
                                         OutputEncoderCalibrationProgress progress,
                                         void *progress_context);
#endif
