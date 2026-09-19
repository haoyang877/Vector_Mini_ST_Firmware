#ifndef FOC_ENCODER_CALIBRATION_H
#define FOC_ENCODER_CALIBRATION_H

#include "data_type.h"
#include "angle_feedback.h"
#include "foc_algorithm.h"
#include "foc_pid.h"
#include "foc_sensorless.h"
#include "foc_sensorless_run.h"
#include "motor_work.h"

/* Mode 13/15 编码器标定任务（按裁决在原 foc_calibration 删除后重建）：
 * - Task_Calib_EncoderObserver：无感观测器闭环下重建编码器 1024 点线性化 LUT；
 * - Task_Calib_EleAngelOffset：d 轴对齐并写入电角度零位（需先完成 LUT 标定）。
 * 原五任务中的 R_L_Flux/EncoderOffset/CurrentOffset 仍待重建，不在本模块。
 * 迁移约定：任务不自行停相或写模式；完成/失败经 MotorWorkOutcome 上报，
 * 功率级与模式迁移由运行状态机执行；本地中止只清理控制状态。 */

/* 标定阶段机步骤：保留历史编号供 Keil Watch 与诊断按数值对照；
 * 未重建任务的步骤（ADC 偏置/R/L/磁链/编码器偏移等）不会出现。 */
typedef enum
{
    CS_NULL = 0,
    CS_ADC_OFFSET_START,
    CS_ADC_OFFSET_LOOP,
    CS_ADC_OFFSET_END,
    CS_MOTOR_R_START,
    CS_MOTOR_RA_LOOP,
    CS_MOTOR_RB_LOOP,
    CS_MOTOR_RC_LOOP,
    CS_MOTOR_R_END,
    CS_MOTOR_L_START,
    CS_MOTOR_LD_LOOP,
    CS_MOTOR_LQ_LOOP,
    CS_MOTOR_L_END,
    CS_MOTOR_FLUX_START,
    CS_MOTOR_FLUX_LOOP,
    CS_MOTOR_FLUX_END,
    CS_ANTICOGGING_START,
    CS_ANTICOGGING_CW_TEMP,
    CS_ANTICOGGING_CW_SAMPLE,
    CS_ANTICOGGING_CCW_TEMP,
    CS_ANTICOGGING_CCW_SAMPLE,
    CS_ANTICOGGING_END,
    CS_OBS_ALIGN_ORIGIN,
    CS_OBS_WAIT_CLOSED_LOOP,
    CS_OBS_SPEED_STABLE,
    CS_OBS_FIND_ORIGIN,
    CS_OBS_SAMPLE_CW,
    CS_OBS_BUILD_LUT,
    CS_OBS_VERIFY_CW,
    CS_OBS_STOP_DECEL,
    CS_OBS_STOP_CURRENT,
    CS_ENC_OFFSET_ALIGN,
    CS_ENC_OFFSET_ALIGN_LOOP,
    CS_ENC_OFFSET_RAMP_CW,
    CS_ENC_OFFSET_SAMPLE_CW,
    CS_ENC_OFFSET_END
} CalibStep_TyepeDef;

/* 当前标定阶段；前台与诊断只读，标定任务为唯一写者。 */
extern CalibStep_TyepeDef CalibStep;

/**
 * @brief 观测器 LUT 标定周期任务（Mode 13）：无感闭环下重建 1024 点线性化 LUT。
 * @param FOC 电流控制与观测量。
 * @param MotorControl 电机与控制状态；任务只写指令类字段。
 * @param SpeedController 速度环控制器状态（无感启动复用）。
 * @param Encoder 编码器状态；验证通过后提交 LUT 并清除电/机械零位标志。
 * @param Fluxobserver 观测器状态；机械参考由静态对齐与过零插值建立。
 * @param Startup 无感启动状态机状态。
 * @return 采集中返回 MOTOR_WORK_RUNNING；标定完成返回
 *         MOTOR_WORK_SWITCH_MODE(Save_Param) 并携带 power_off；
 *         任一失败返回 MOTOR_WORK_FAULT（错误码已置位）。
 * @note 仅由 20 kHz 快速中断调用；本地中止只清理采样区与控制状态，
 *       停相与模式回退由运行状态机按结果协议执行。
 */
MotorWorkOutcome_TypeDef Task_Calib_EncoderObserver(FOC_TypeDef *FOC,
                                                    MotorControl_TypeDef *MotorControl,
                                                    PI_Controller_TypeDef *SpeedController,
                                                    Encoder_TypeDef *Encoder,
                                                    Fluxobserver_TypeDef *Fluxobserver,
                                                    SensorlessStartup_TypeDef *Startup);

/**
 * @brief 电角度零位标定周期任务（Mode 15）：d 轴对齐平均线性化角并写电角度零位。
 * @param FOC 电流控制与观测量。
 * @param MotorControl 电机与控制状态；idRef 由本任务驱动。
 * @param Encoder 编码器状态；要求已置 ENC_CALIB_LINEARIZED，结果写电零位。
 * @return 对齐中返回 MOTOR_WORK_RUNNING；成功返回
 *         MOTOR_WORK_SWITCH_MODE(Save_Param)（随后由保存流程提交）；
 *         任一失败返回 MOTOR_WORK_FAULT（错误码已置位）。
 * @note 仅由 20 kHz 快速中断调用；完成/失败经结果协议上报，
 *       停相与模式回退由运行状态机执行（完成后请求 power_off）。
 */
MotorWorkOutcome_TypeDef Task_Calib_EleAngelOffset(FOC_TypeDef *FOC,
                                                   MotorControl_TypeDef *MotorControl,
                                                   Encoder_TypeDef *Encoder);

#endif
