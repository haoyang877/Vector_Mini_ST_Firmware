#ifndef FOC_RUN_STATE_H
#define FOC_RUN_STATE_H

#include <stdbool.h>

#include "motor_work.h"
#include "foc_algorithm.h"
#include "foc_pid.h"
#include "angle_feedback.h"
#include "foc_sensorless.h"
#include "app_lifecycle.h"
#include "motor_control_command.h"

/**
 * @param motor 组合层终身电机对象。
 * @param foc 组合层终身 FOC 对象。
 * @param pi 组合层终身速度 PI。
 * @param encoder 组合层终身编码器对象。
 * @param startup 组合层终身无感启动对象。
 * @brief 初始化运行所有者：核心进入 BOOT 态，清空命令和采样进展。
 * @note 由启动组合层在中断使能前调用一次；不访问硬件。
 */
void FocRunState_Init(MotorControl_TypeDef *motor,
                      FOC_TypeDef *foc,
                      PI_Controller_TypeDef *pi,
                      Encoder_TypeDef *encoder,
                      SensorlessStartup_TypeDef *startup);
/**
 * @brief 快速环故障及显式台架截止检查：核对反馈、轴配置、期限和采样限制。
 * @note 仅由 20 kHz 快速中断调用；随后必须调用 FastFaultStop 完成故障关断。
 *       台架期限到达可直接关相；采样电流截止只在显式会话内生效。不清错、不写 Flash。
 */
void FocRunState_CheckFastFaults(void);
/**
 * @brief 故障紧急关断快车道：故障锁存且功率级开启时立即关断硬件。
 * @note 仅由 20 kHz 快速中断调用；本板无 TIM1 BKIN 与驱动器故障引脚，软件是
 *       唯一关断路径，本入口把"故障 → 功率级关断"保持在快速环时延内。
 */
void FocRunState_FastFaultStop(void);
/**
 * @brief 上报本周期 worker 结果：20 kHz 快环写入，2 kHz 状态机消费。
 * @param epoch worker 执行前捕获的原始身份。
 * @param outcome 本周期模式 worker 的结果；MOTOR_WORK_RUNNING 不入箱。
 * @note 仅由 ADC2 JEOS 的 20 kHz 快速中断末尾调用；每拍登记感测处理进展，
 *       非 RUNNING 结果与消费端通过短临界区交换，只保留最新一项。
 */
void FocRunState_PostOutcome(MotorWorkOutcome_TypeDef outcome, uint32_t epoch);
/**
 * @brief 执行一次运行状态机：消费 worker 结果，完成故障指示、功率级启停迁移与提交。
 * @note 仅由 2 kHz 监督中断调用；顺序为 worker 结果应用 → 故障反应 →
 *       维护会话服务（SAVE 结果上报/派生完成/取消确认）→ 停机/使能迁移 →
 *       变化检测与影子提交（ModeLast/ErrorLast 与前台镜像）。
 */
void FocRunState_Tick(void);

/**
 * @brief 读取运行所有者的生命周期状态。
 * @return 当前内部状态，数值不能直接用作线上枚举。
 * @note 只读；与其他电机测量关联采样时调用方须置于同一短临界区。
 */
AppLifecycleState FocRunState_GetState(void);

/**
 * @brief 查询指定维护模式是否已由生命周期所有者受理。
 * @param mode 维护模式；只有与当前维护会话匹配时才获准执行。
 * @return 会话已经进入 MAINTENANCE 且模式匹配返回 true。
 * @note 快环只读查询，用于阻止自管功率的标定在维护准入前启相。
 */
bool FocRunState_IsMaintenanceActive(ModeNow_TypeDef mode);

/**
 * @brief 由后台提出远程停机请求，下一 2 kHz 监督拍执行正常关相路径。
 * @return 请求已登记返回 true；不表示功率输出已经关闭。
 * @note 与监督中断通过短临界区交接；故障快车道仍可优先关断。
 */
bool FocRunState_RequestRemoteStop(void);

/**
 * @brief 向运行所有者提交同模式目标，CAN 与 Watch 共用受理和应用路径。
 * @param command SI 单位值命令；成功时复制，不保留指针。
 * @return 已受理、参数错误、状态拒绝、忙、越限或故障；不报告物理已执行。
 * @note 前台或 2 kHz Watch 消费调用；单槽邮箱不覆盖未应用目标。
 *       下一监督拍复核；只允许已 RUNNING 的相同模式，
 *       不隐式使能、不切换模式、不清故障；STOP 会取消尚未应用的目标。
 */
MotorControlResult FocRunState_SubmitControl(const MotorControlCommand *command);

typedef enum
{
    FOC_BACKGROUND_NONE,
    FOC_BACKGROUND_SAVE,
    FOC_BACKGROUND_COGGING
} FocBackgroundKind;
typedef struct
{
    uint32_t epoch;
    FocBackgroundKind kind;
} FocBackgroundClaim;
/**
 * @brief 认领当前维护会话的前台工作。
 * @param claim 接收原始操作 epoch 和任务类型，调用方保留到完成。
 * @return 已关相且认领成功返回 true。
 * @note 前台调用；认领未归还时不得开始其他任务。
 */
bool FocRunState_BeginBackground(FocBackgroundClaim *claim);
/**
 * @brief 复核认领仍属于当前维护操作。
 * @param claim 原始认领值，不得以当前 epoch 重建。
 * @return 身份匹配、未取消、无故障且关相返回 true。
 * @note 最终写入或发布前必须在同一临界区复核。
 */
bool FocRunState_BackgroundValid(const FocBackgroundClaim *claim);
/**
 * @brief 归还前台工作并提交原始会话结果。
 * @param claim 原始认领，必须精确匹配已借出的资源。
 * @param success SAVE 表示回读已验证；COGGING 表示已发布 RAM。
 * @note 缓冲已释放后调用；迟到结果只归还资源，不复活目标。
 */
void FocRunState_FinishBackground(const FocBackgroundClaim *claim, bool success);
/**
 * @brief 读取 worker 开始执行时的会话身份。
 * @return 当前操作 epoch，无维护操作时为生命周期 epoch。
 * @note 在调用 worker 前读取，并原样传给 PostOutcome。
 */
uint32_t FocRunState_WorkEpoch(void);
/**
 * @brief 在稳定配置借用下处理前台补偿请求。
 * @note 仅前台调用；CRC 开中断，期间拒绝新维护和 READY 启动；STOP 可立即关相。
 */
void FocRunState_ServiceCompensation(void);
#endif
