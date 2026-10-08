#ifndef FOC_COGGING_CALIBRATION_H
#define FOC_COGGING_CALIBRATION_H
#include "motor_types.h"
#include "angle_feedback.h"
#include "foc_algorithm.h"
#include "foc_pid.h"
#include "cogging_calibration.h"
#include "motor_task_result.h"
/* 保留原运行入口的头文件兼容；任务通过记录复制边界发布，不持有活动表。 */
#include "foc_cogging_runtime.h"

/**
 * @brief 检查标定启动前置条件：无边界轴、无故障、编码器在线且已标定、运动量在限。
 * @param motor 电机控制状态，只读。
 * @param encoder 编码器状态，只读。
 * @return 允许进入标定模式返回 true；否则返回 false。
 * @note 前台调用；不启动 PWM、不修改任何状态。
 */
bool FocCogging_CanStart(const MotorControlState *motor, const EncoderState *encoder);

/**
 * @brief 标定模式主任务：会话启动、安全闸门、2 kHz 抽点、位置保持与外环整形。
 * @param foc foc 状态。
 * @param motor 电机控制状态。
 * @param pi 标定专用位置 PI（复用速度环控制器对象）。
 * @param encoder 编码器状态。
 * @return 零矢量准备完成为 PREPARED，采集结束为 FINALIZE，失败为 FAILED，其余 RUNNING。
 * @note 20 kHz 调用；不启相、不停相、不选择下一任务；所有者按 epoch/guards
 *       授予功率许可，采集结束后关相并认领前台整理。失败只清理功能本地状态。
 */
MotorTaskResult
FocCogging_Task(FocState *foc, MotorControlState *motor, PiController *pi, EncoderState *encoder);

/**
 * @brief 中止采样或标记前台认领取消；不操作 PWM、不切换模式。
 * @return 无返回值。
 * @note 所有者先关相并清理控制目标；已认领时须等待 ReleaseFinalize 才释放会话。
 */
void FocCogging_Abort(void);

/**
 * @brief 读取采样状态，成表/保存挂起时报告 COGGING_SAVING。
 * @return 当前标定状态。
 * @note COMPLETE 只代表采样完成，不能证明 Flash 已提交；前台与 CAN 查询只读。
 */
CoggingState FocCogging_GetState(void);

/**
 * @brief 通知标定保存结果；失败时把状态标记为 COGGING_SAVE_FAILED。
 * @param success 参数存储实际提交成功且 APP 认领仍有效时为 true。
 * @return 无返回值。
 * @note 仅前台调用；仅在保存流程挂起时生效。
 */
void FocCogging_SaveResult(bool success);

/**
 * @brief 查询自然完成后的待整理表。
 * @return 需要前台认领时返回 true。
 */
bool FocCogging_NeedsFinalize(void);
/**
 * @brief 确认标定与前台会话已归还。
 * @return 无活动标定、待整理或前台借用时返回 true。
 */
bool FocCogging_IsReleased(void);
/**
 * @brief 认领已结束的齿槽采样会话。
 * @return 认领成功返回 true。
 * @note APP 维护 owner 在短临界区调用；必须由 ReleaseFinalize 归还。
 */
bool FocCogging_BeginFinalize(void);
/**
 * @brief 在调用方候选空间构建完整表。
 * @param record 前台独占候选空间。
 * @param motor 认领期间稳定的配置。
 * @param encoder 认领期间稳定的安装校准。
 * @return 表构建成功返回 true。
 * @note 仅前台调用，CRC 保持中断开启；取消不改写正在读取的样本。
 */
bool FocCogging_BuildCandidate(CoggingMapRecord *record,
                               const MotorControlState *motor,
                               const EncoderState *encoder);
/**
 * @brief 发布已构建的完整候选表。
 * @param record 完整候选，仅复制值。
 * @return 会话未取消且仍由前台认领时返回 true。
 * @note APP 在同一临界区复核原始身份、无故障与关相后调用。
 */
bool FocCogging_PublishCandidate(const CoggingMapRecord *record);
/**
 * @brief 归还前台会话并记录发布结果。
 * @param published 候选是否已提交到 RAM，不表示写入 Flash。
 * @note 候选已释放后在 owner 临界区调用；取消不能撤回已发布 RAM。
 */
void FocCogging_ReleaseFinalize(bool published);
#endif
