#ifndef MOTOR_CALIBRATION_H
#define MOTOR_CALIBRATION_H
#include "application_config.h"
#include "motor_state_machine.h"
#include "output_encoder.h"
#include "bsp_flash.h"
#include "parameter_record.h"

/** @brief 单轴标定的候选与工作区；绑定的活动对象在整个会话内有效。 */
typedef struct
{
    EncoderCalibrationControl drive;
    MotorStateMachine *state_machine;
    MotorOuterLoop *outer;
    LinearHall *hall;
    OutputEncoder *output;
    MotorAxisRecord *record;
    BspResult (*write_record)(const void *data, size_t length);
    /* 前台登记待发布 LUT 的会话，ADC 在下一拍安装并确认。零值无请求。 */
    volatile uint32_t lut_request_epoch, lut_published_epoch;
    EncoderCalibrationSession session;
    EncoderCalibrationWorkspace workspace;
#if BSP_ENABLED_LINEAR_HALL
    LinearHallCalibration hall_sweep;
    OutputEncoderCalib output_calibration;
    uint32_t coast_epoch, coast_start_ms, stable_since_ms;
#endif
} MotorCalibration;

/**
 * @brief 在 ADC 拍内安装前台请求的 LUT，Hall 轴由前台先确认停稳。
 * @param calibration 本轴的标定工作区。
 * @note 邮箱仅由本模块的 LUT 流程写入；安装前复核会话、故障、STOP 和关相。
 *       在读取本拍编码器之前调用，只提交固定长度的已建表结果，Flash 由前台处理。
 */
void MotorCalibration_PublishLutFast(MotorCalibration *calibration);

/**
 * @brief 推进当前轴的一次快拍标定。
 * @param calibration 当前轴独占的标定会话。
 * @return 本拍任务结果，由调用方按原代际提交。
 * @note 仅 ADC 快拍调用。
 */
MotorTaskResult MotorCalibration_StepFast(MotorCalibration *calibration);

/**
 * @brief 在停机前台发布原会话的标定结果并保存参数。
 * @param calibration 当前轴的候选和本机记录。
 * @note 在最终提交前复核原代际及数字关相，Flash 回读失败锁存故障。
 */
void MotorCalibration_ProcessPublication(MotorCalibration *calibration);
#endif
