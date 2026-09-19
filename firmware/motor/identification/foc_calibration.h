#ifndef __FOC_CALIBRATION_H__
#define __FOC_CALIBRATION_H__

#include "main.h"
#include "foc_algorithm.h"
#include "data_type.h"
#include "motor_work.h"
#include "encoder.h"
#include "foc_sensorless.h"

#define OFFSET_LUT_NUM ENCODER_OFFSET_LUT_SIZE
#define MAX_MOTOR_POLE_PAIRS 20U
#define COGGING_MAP_NUM 1024U

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

/* Read by owner-ISR RTT telemetry; calibration remains the only writer. */
extern CalibStep_TyepeDef CalibStep;

/**
 * @brief R/L/磁链辨识周期任务；内部阶段机推进，完成后自行请求保存参数。
 * @param FOC 电流控制与观测量。
 * @param MotorControl 电机与控制状态。
 * @note 仅由 20 kHz 快速中断调用；阶段 2 迁移中，任务内部仍自行推进模式与功率级。
 */
void Task_Calib_R_L_Flux(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl);
/**
 * @brief 编码器偏移标定周期任务；内部阶段机推进。
 * @param FOC 电流控制与观测量。
 * @param MotorControl 电机与控制状态。
 * @param Encoder 编码器状态；标定过程会更新其偏移与标志。
 * @param Fluxobserver 观测器状态；对齐阶段使用。
 * @note 仅由 20 kHz 快速中断调用；阶段 2 迁移中，任务内部仍自行推进模式与功率级。
 */
void Task_Calib_EncoderOffset(FOC_TypeDef *FOC,
                              MotorControl_TypeDef *MotorControl,
                              Encoder_TypeDef *Encoder,
                              Fluxobserver_TypeDef *Fluxobserver);
/**
 * @brief 无感观测器标定周期任务；内部阶段机推进。
 * @param FOC 电流控制与观测量。
 * @param MotorControl 电机与控制状态。
 * @param SpeedController 速度环控制器状态。
 * @param Encoder 编码器状态。
 * @param Fluxobserver 观测器状态；标定过程会写入其参数。
 * @param Startup 无感启动状态机状态。
 * @return 采集中返回 MOTOR_WORK_RUNNING；标定完成时返回
 *         MOTOR_WORK_SWITCH_MODE(Save_Param)，并携带 power_off 请求关断功率级。
 * @note 仅由 20 kHz 快速中断调用；完成路径不再自写模式或功率级，
 *       其余内部阶段仍由阶段机自身推进。
 */
MotorWorkOutcome_TypeDef Task_Calib_EncoderObserver(FOC_TypeDef *FOC,
                                                    MotorControl_TypeDef *MotorControl,
                                                    PI_Controller_TypeDef *SpeedController,
                                                    Encoder_TypeDef *Encoder,
                                                    Fluxobserver_TypeDef *Fluxobserver,
                                                    SensorlessStartup_TypeDef *Startup);
/**
 * @brief 电角度零位标定周期任务；内部阶段机推进。
 * @param FOC 电流控制与观测量。
 * @param MotorControl 电机与控制状态。
 * @param Encoder 编码器状态；标定结果写入电角度零位。
 * @note 仅由 20 kHz 快速中断调用；阶段 2 迁移中，任务内部仍自行推进模式与功率级。
 */
void Task_Calib_EleAngelOffset(FOC_TypeDef *FOC,
                               MotorControl_TypeDef *MotorControl,
                               Encoder_TypeDef *Encoder);
/**
 * @brief 上电电流零偏标定周期任务；达到样本数后返回停机请求。
 * @param FOC 电流控制与观测量。
 * @param MotorControl 电机与控制状态；标定结果写入三相偏置。
 * @return 采集中返回 MOTOR_WORK_RUNNING；完成一次标定后返回 MOTOR_WORK_STOP。
 * @note 仅由 20 kHz 快速中断调用；本任务不写模式或功率级，转换由运行状态机执行。
 */
MotorWorkOutcome_TypeDef Task_Calib_CurrentOffset(FOC_TypeDef *FOC,
                                                  MotorControl_TypeDef *MotorControl);

#endif
