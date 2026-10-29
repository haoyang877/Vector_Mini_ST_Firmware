#ifndef ENCODER_CALIBRATION_TASK_H
#define ENCODER_CALIBRATION_TASK_H

#include "encoder_calibration_core.h"
#include "motor_types.h"
#include "angle_feedback.h"
#include "foc_algorithm.h"
#include "foc_pid.h"
#include "flux_observer.h"
#include "sensorless_startup.h"
#include "motor_task_result.h"

/* Mode13/15 编码器标定任务适配，纯计算契约见 encoder_calibration_core.h：
 * - EncoderCalibrationTask_StepLut：无感观测器闭环下重建编码器 1024 点线性化 LUT；
 * - EncoderCalibrationTask_StepElectricalZero：d 轴对齐并写入电角度零位（需先完成 LUT 标定）。
 * 任务不自行停相、写模式或保存；完成/失败经 MotorTaskResult 上报，
 * 功率级与模式迁移由运行状态机执行；本地中止只清理控制状态。 */

/* 仅保留实际LUT/电零位阶段；显式编号保持原Watch与诊断数值。 */
typedef enum
{
    CS_NULL = 0,
    CS_OBS_ALIGN_ORIGIN = 22,
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
    CS_ENC_OFFSET_END,
    CS_LUT_PUBLISH_PENDING
} EncoderCalibrationPhase;

/* 当前标定阶段；前台与诊断只读，标定任务为唯一写者。 */
/**
 * @brief 读取标定阶段值。
 * @return 当前观测器标定阶段，不返回可写状态。
 */
EncoderCalibrationPhase EncoderCalibrationTask_GetStep(void);

/**
 * @brief 退出观测器编码器标定并释放其工作区。
 * @note 只清理本模块的内存与阶段变量，不停相、不改模式、不写参数；可重复调用。
 */
void EncoderCalibrationTask_Cancel(void);
/**
 * @brief 确认两个标定任务及工作区已释放。
 * @return 无采样工作区、无活动阶段时返回 true。
 * @note 仅只读查询，取消由 owner 显式调用。
 */
bool EncoderCalibrationTask_IsReleased(void);

/**
 * @brief 在关相后台发布已验证的候选LUT并释放标定引用。
 * @param calibration 独占的设备标定对象。
 * @param encoder 独占的角度运行状态。
 * @return 候选有效且发布成功返回true，否则保持原标定返回false。
 * @note 所有者须在同一临界区复核会话、STOP、故障和关相；不在快环调用。
 */
bool EncoderCalibrationTask_PublishLut(EncoderCalibration *calibration, EncoderState *encoder);

/**
 * @brief 观测器 LUT 标定周期任务：无感闭环下重建 1024 点线性化 LUT。
 * @param configuration 活动配置只读视图，读取电机模型与应用限值。
 * @param calibration 保留既有调用形状；本快环入口不再写入正式LUT。
 * @param foc 电流控制与观测量。
 * @param motor_state 电机与控制状态；任务只写指令类字段。
 * @param speed_pi 速度环控制器状态（无感启动复用）。
 * @param encoder 编码器运行状态，关相前仍使用原LUT反馈。
 * @param flux_observer 观测器状态；机械参考由静态对齐与过零插值建立。
 * @param startup 无感启动状态机状态。
 * @param workspace 调用方独占采样区，活动任务期间地址须不变；NULL 表示不可借用。
 * @return 采集中返回 MOTOR_TASK_RUNNING；标定完成返回
 *         MOTOR_TASK_FINALIZE，所有者关相后在后台发布，再独立衔接保存；
 *         任一失败返回 MOTOR_TASK_FAILED（错误码已置位）。
 * @note 仅由 20 kHz 快速中断调用；本地中止只清理采样区与控制状态，
 *       停相与模式回退由运行状态机按结果协议执行。FINALIZE保留候选借用，
 *       发布或取消后调用方确认 IsReleased 才可复用空间；不分配或释放内存。
 */
MotorTaskResult EncoderCalibrationTask_StepLut(const MotorConfiguration *configuration,
                                               EncoderCalibration *calibration,
                                               FocState *foc,
                                               MotorControlState *motor_state,
                                               PiController *speed_pi,
                                               EncoderState *encoder,
                                               FluxObserverState *flux_observer,
                                               SensorlessStartupState *startup,
                                               EncoderCalibrationWorkspace *workspace);

/**
 * @brief 电角度零位标定周期任务：d 轴对齐平均线性化角并写电角度零位。
 * @param configuration 活动配置只读视图，读取对齐电流与应用限值。
 * @param calibration configuration->device.encoder 的可写视图，要求已线性化并接收电零位。
 * @param foc 电流控制与观测量。
 * @param motor_state 电机与控制状态；id_target_a 由本任务驱动。
 * @param encoder 编码器运行状态，提供在线状态与本拍线性化角。
 * @return 对齐中返回 MOTOR_TASK_RUNNING；成功返回
 *         MOTOR_TASK_COMPLETED（随后由所有者受理独立保存）；
 *         任一失败返回 MOTOR_TASK_FAILED（错误码已置位）。
 * @note 仅由 20 kHz 快速中断调用；完成/失败经结果协议上报，
 *       停相、任务衔接及保存由主状态机执行。
 */
MotorTaskResult EncoderCalibrationTask_StepElectricalZero(const MotorConfiguration *configuration,
                                                          EncoderCalibration *calibration,
                                                          FocState *foc,
                                                          MotorControlState *motor_state,
                                                          EncoderState *encoder);

#endif
