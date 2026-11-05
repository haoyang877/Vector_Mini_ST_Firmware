#ifndef ENCODER_CALIBRATION_H
#define ENCODER_CALIBRATION_H

#include "motor_types.h"
#include "encoder_feedback.h"
#include "foc.h"
#include "flux_observer.h"

/** 调用方独占的采样区；计数与候选按构表阶段复用，布局保持 6144 B。 */
typedef struct
{
    int32_t error_sum[ENCODER_OFFSET_LUT_SIZE];
    union
    {
        uint16_t samples[ENCODER_OFFSET_LUT_SIZE];
        int16_t candidate[ENCODER_OFFSET_LUT_SIZE];
    } bins;
} EncoderCalibrationWorkspace;

/** @brief 当前调用借用的单轴控制对象；由调用方显式组装，不持有独立状态。 */
typedef struct
{
    MotorConfiguration *configuration;
    FocState *foc;
    MotorControlState *motor;
    EncoderState *encoder;
    FluxObserverState *observer;
    SensorlessStartupState *startup;
} EncoderCalibrationControl;

/* 编码器标定任务：
 * - EncoderCalibration_StepLut：无感观测器闭环下重建编码器 1024 点线性化 LUT；
 * - EncoderCalibration_StepElecZero：d 轴对齐并写入电角度零位（需先完成 LUT 标定）。
 * 任务为单轴非重入实现，不自行停相、写模式或保存；完成/失败经 MotorTaskResult 上报，
 * 功率级与模式迁移由运行状态机执行；本地中止只清理控制状态。 */

/* 按业务阶段推进；现有阶段保留诊断编号，电零位独立计时。 */
typedef enum
{
    CS_NULL = 0,
    CS_OBS_STARTUP = 22,
    CS_OBS_SAMPLE_CW = 26,
    CS_OBS_BUILD_LUT = 27,
    CS_OBS_STOP_DECEL = 29,
    CS_OBS_STOP_CURRENT = 30,
    CS_LUT_PUBLISH_PENDING = 36
} EncoderCalibrationPhase;

/* 当前标定阶段；前台与诊断只读，标定任务为唯一写者。 */
/**
 * @brief 读取标定阶段值。
 * @return 当前观测器标定阶段，不返回可写状态。
 */
EncoderCalibrationPhase EncoderCalibration_GetPhase(void);

/**
 * @brief 取消LUT及电零位会话并释放工作区借用。
 * @note 只清理本模块的内存与阶段变量，不停相、不改模式、不写参数；可重复调用。
 */
void EncoderCalibration_Cancel(void);
/**
 * @brief 确认两个标定任务及工作区已释放。
 * @return LUT 和电零位任务均处于空闲时返回 true。
 * @note 仅只读查询，取消由 owner 显式调用。
 */
bool EncoderCalibration_IsIdle(void);

/**
 * @brief 在关相后台发布已构建的候选LUT并释放标定引用。
 * @param calibration 独占的设备标定对象。
 * @param encoder 独占的角度运行状态。
 * @return 候选有效且发布成功返回true，否则保持原标定返回false。
 * @note 两个对象必须有效。所有者须持有全中断临界区并复核会话、STOP、故障和关相，
 *       发布完成后才能恢复中断；在前台调用。
 */
bool EncoderCalibration_PublishLut(EncoderCalibration *calibration, EncoderState *encoder);

/**
 * @brief 观测器 LUT 标定周期任务：无感闭环下重建 1024 点线性化 LUT。
 * @param drive 单轴控制绑定；所有对象在当前调用内有效，任务不保存绑定地址。
 * @param workspace 调用方独占且非空的采样区，活动任务期间地址须不变。
 * @return 采集中返回 MOTOR_TASK_RUNNING；标定完成返回
 *         MOTOR_TASK_FINALIZE，所有者关相后在后台发布，持久化由所有者决定；
 *         任一失败返回 MOTOR_TASK_FAILED（错误码已置位）。
 * @note 仅由 20 kHz 快速中断调用；本地中止只清理采样区与控制状态，
 *       停相与模式回退由运行状态机按结果协议执行。FINALIZE保留候选借用，
 *       发布或取消后调用方确认 IsIdle 才可复用空间；不分配或释放内存。
 *       调用方保证配置有效且编码器已就绪；无感启动负责运行参数检查。稳速后以当前角为相对参考，
 *       构表完成后减速并卸流，发布后须重新标定电零位。
 */
MotorTaskResult EncoderCalibration_StepLut(const EncoderCalibrationControl *drive,
                                           EncoderCalibrationWorkspace *workspace);

/**
 * @brief 电角度零位标定周期任务：d 轴对齐平均线性化角并写电角度零位。
 * @param drive 单轴绑定，仅访问配置、FOC、电机和编码器；其余成员可为NULL。
 * @return 对齐中返回 MOTOR_TASK_RUNNING；成功返回
 *         MOTOR_TASK_COMPLETED；是否保存由所有者决定；
 *         任一失败返回 MOTOR_TASK_FAILED（错误码已置位）。
 * @note 仅由 20 kHz 快速中断调用；完成/失败经结果协议上报，
 *       停相和任务衔接由主状态机执行，本任务只更新RAM标定。
 *       调用方保证编码器就绪、LUT 有效，标定电流与电流限值均为有效正值。
 */
MotorTaskResult EncoderCalibration_StepElecZero(const EncoderCalibrationControl *drive);

#endif
