#ifndef MOTOR_STATE_MACHINE_H
#define MOTOR_STATE_MACHINE_H

#include <stdbool.h>

#include "motor_task_result.h"
#include "foc_algorithm.h"
#include "foc_pid.h"
#include "angle_feedback.h"
#include "foc_sensorless.h"
#include "app_lifecycle.h"
#include "motor_control_command.h"
#include "motor_lifecycle_request.h"

/**
 * @param motor 组合层终身电机对象。
 * @param foc 组合层终身 foc 对象。
 * @param pi 组合层终身速度 PI。
 * @param encoder 组合层终身编码器对象。
 * @param startup 组合层终身无感启动对象。
 * @brief 初始化运行所有者：核心进入 BOOT 态，清空命令和采样进展。
 * @note 由启动组合层在中断使能前调用一次；不访问硬件。
 */
void MotorStateMachine_Init(MotorControlState *motor,
                            FocState *foc,
                            PiController *pi,
                            EncoderState *encoder,
                            SensorlessStartupState *startup);
/**
 * @brief 快速环故障检查：核对编码器反馈与轴配置。
 * @note 仅由 20 kHz 快速中断调用；随后必须调用 FastFaultStop 完成故障关断。不清错、不写 Flash。
 */
void MotorStateMachine_CheckFastFaults(void);
/**
 * @brief 故障紧急关断快车道：故障锁存且功率级开启时立即关断硬件。
 * @note 仅由 20 kHz 快速中断调用；本板无 TIM1 BKIN 与驱动器故障引脚，软件是
 *       唯一关断路径，本入口把"故障 → 功率级关断"保持在快速环时延内。
 */
void MotorStateMachine_FastFaultStop(void);
/**
 * @brief 上报本周期 worker 结果：20 kHz 快环写入，2 kHz 状态机消费。
 * @param epoch worker 执行前捕获的原始身份。
 * @param outcome 本周期功能结果；MOTOR_TASK_RUNNING 不入箱。
 * @note 仅由 ADC2 JEOS 的 20 kHz 快速中断末尾调用；每拍登记感测处理进展，
 *       非 RUNNING 结果与消费端通过短临界区交换，保留首个终态，失败优先。
 */
void MotorStateMachine_PostOutcome(MotorTaskResult outcome, uint32_t epoch);
/**
 * @brief 执行一次运行状态机：消费 worker 结果，完成故障指示、功率级启停迁移与提交。
 * @note 仅由 2 kHz 监督中断调用；顺序为 worker 结果应用 → 故障反应 →
 *       维护会话服务（SAVE 结果上报/派生完成/取消确认）→ 停机/使能迁移 →
 *       目标消费与故障镜像。
 */
void MotorStateMachine_Tick(void);

/**
 * @brief 读取运行所有者的生命周期状态。
 * @return 当前内部状态，数值不能直接用作线上枚举。
 * @note 只读；与其他电机测量关联采样时调用方须置于同一短临界区。
 */
AppLifecycleState MotorStateMachine_GetState(void);

/**
 * @brief 查询指定维护模式是否已由生命周期所有者受理。
 * @param kind 维护任务；只有与当前维护会话匹配时才获准执行。
 * @return 会话已经进入 MAINTENANCE 且模式匹配返回 true。
 * @note 快环只读查询，不等于快环可执行或功率许可。
 */
bool MotorStateMachine_IsMaintenanceActive(MotorMaintenanceKind kind);

/**
 * @brief 由后台提出远程停机请求，下一 2 kHz 监督拍执行正常关相路径。
 * @return 请求已登记返回 true；不表示功率输出已经关闭。
 * @note 与监督中断通过短临界区交接；故障快车道仍可优先关断。
 */
bool MotorStateMachine_RequestRemoteStop(void);

/**
 * @brief 向运行所有者提交同模式目标，前台共用受理和应用路径。
 * @param command SI 单位值命令；成功时复制，不保留指针。
 * @return 已受理、参数错误、状态拒绝、忙、越限或故障；不报告物理已执行。
 * @note 前台消费调用；单槽邮箱不覆盖未应用目标。
 *       下一监督拍复核；只允许已 RUNNING 的相同模式，
 *       不隐式使能、不切换模式、不清故障；STOP 会取消尚未应用的目标。
 */
MotorControlResult MotorStateMachine_SubmitControl(const MotorControlCommand *command);
/**
 * @brief 请求以零目标或当前位置启动明确控制模式。
 * @param mode 业务模式，不接受线上或历史编号。
 * @return 受理、参数错误、忙、故障或守卫拒绝；受理不代表已启相。
 * @note 前台消费调用，有界非阻塞；功率迁移仍在监督调度提交。
 */
MotorControlResult MotorStateMachine_RequestEnable(MotorControlMode mode);
/**
 * @brief 请求原有维护任务并分配创建时身份。
 * @param kind 明确业务任务。
 * @param ticket 接收原始身份，失败清零。
 * @return 受理、错误、忙、故障或守卫拒绝。
 * @note 不保留调用方指针；不执行 Flash 或长耗时工作。
 */
MotorControlResult MotorStateMachine_RequestMaintenance(MotorMaintenanceKind kind,
                                                        uint32_t *ticket);
/**
 * @brief 按来源和原始请求序号幂等受理维护，复用同一维护所有者。
 * @param request 只读值身份，保留字节必须为零；返回后不保留指针。
 * @param ticket 成功接收原始任务身份，失败清零。
 * @return 受理、非法/冲突/陈旧身份、忙、故障或守卫拒绝。
 * @note 同来源序号和任务重试返回原 ticket，不重新执行；同序号不同任务拒绝。
 *       同来源序号按 16 位半范围窗口递进，复位后清空；前台调用，不执行长任务。
 */
MotorControlResult
MotorStateMachine_RequestIdentifiedMaintenance(const MotorMaintenanceRequest *request,
                                               uint32_t *ticket);
/**
 * @brief 精确取消原 ticket 对应的维护，不影响其他任务。
 * @param ticket 受理时返回的非零身份。
 * @return 已登记取消或终态重复取消返回受理；陈旧身份非法，其他状态拒绝。
 * @note 受理不代表资源已经归还，须查询同 ticket 的 CANCELLED 终态；不恢复旧目标。
 */
MotorControlResult MotorStateMachine_CancelMaintenance(uint32_t ticket);
/**
 * @brief 核对恢复资格并同步清除当前可恢复锁存故障。
 * @return 受理表示锁存已清或 READY 本来无故障；忙或拒绝保持锁存，FATAL 不可清。
 * @note 临界区内完成 CLEAR 提交；故障清除后进入 SELF_TEST，不表示自检完成或可启动。
 *       只清错，不恢复模式或旧目标。
 */
MotorControlResult MotorStateMachine_RequestClear(void);
/**
 * @brief 读取原始维护身份的进展。
 * @param ticket 请求受理时返回的身份。
 * @param progress 接收一致结果值，失败清零。
 * @return 当前、最近维护或最近带来源请求的终态身份匹配返回 true；陈旧身份返回 false。
 * @note 前台查询；完成不代表持久化，须同时检查 effects。
 */
bool MotorStateMachine_ReadMaintenance(uint32_t ticket, MotorMaintenanceProgress *progress);
/**
 * @brief 查询停机参数应用是否具备所有者与资源资格。
 * @return READY、关相、无命令/维护/配置借用和故障时返回 true。
 * @note 读资格与实际应用须处于同一调用方临界区，不能作为延后写入许可证。
 */
bool MotorStateMachine_ConfigurationAllowed(void);

/**
 * @brief 为唯一升级交接票据保留停机所有权并撤销当前目标。
 * @param ticket 前台交接所有者提供的非零票据，复位前不得复用其他身份。
 * @return 同票据重复或首次登记返回受理，其他票据忙，FATAL拒绝。
 * @note 不表示已关相；保留后拒绝启动、维护和配置写入，失败也不自动释放或恢复运动。
 */
MotorControlResult MotorStateMachine_RequestBootStop(uint64_t ticket);
/**
 * @brief 核对升级票据仍拥有停机且所有实际资源均已归还。
 * @param ticket 原始非零票据。
 * @return READY或FAULT、数字关相、无控制/维护/借用/后台时返回true。
 * @note 必须在最终邮箱写入与复位的同一短临界区复核；不表示保存业务已完成。
 */
bool MotorStateMachine_BootHandoffReady(uint64_t ticket);
/**
 * @brief 读取监督拍进展与基础采样健康，不要求主机在线或电机已使能。
 * @param progress 输出已完成监督拍计数；调用方必须比较新进展。
 * @return 采样新鲜且未进入FATAL返回true；空指针返回false。
 * @note 前台短临界区只读；通信健康由组合层补充，不授权运动或Flash。
 */
bool MotorStateMachine_ReadBootHealth(uint32_t *progress);

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
bool MotorStateMachine_BeginBackground(FocBackgroundClaim *claim);
/**
 * @brief 复核认领仍属于当前维护操作。
 * @param claim 原始认领值，不得以当前 epoch 重建。
 * @return 身份匹配、未取消、无故障且关相返回 true。
 * @note 最终写入或发布前必须在同一临界区复核。
 */
bool MotorStateMachine_BackgroundValid(const FocBackgroundClaim *claim);
/**
 * @brief 归还前台工作并提交原始会话结果。
 * @param claim 原始认领，必须精确匹配已借出的资源。
 * @param success SAVE 表示回读已验证；COGGING 表示已发布 RAM。
 * @note 缓冲已释放后调用；迟到结果只归还资源，不复活目标。
 */
void MotorStateMachine_FinishBackground(const FocBackgroundClaim *claim, bool success);
/**
 * @brief 读取 worker 开始执行时的会话身份。
 * @return 当前操作 epoch，无维护操作时为生命周期 epoch。
 * @note 在调用 worker 前读取，并原样传给 PostOutcome。
 */
uint32_t MotorStateMachine_WorkEpoch(void);
/**
 * @brief 在稳定配置借用下处理前台补偿请求。
 * @note 仅前台调用；CRC 开中断，期间拒绝新维护和 READY 启动；STOP 可立即关相。
 */
void MotorStateMachine_ServiceCompensation(void);
/**
 * @brief 读取当前实际维护类型，不投影到控制模式。
 * @return MAINTENANCE 时返回任务类型，其他状态返回 NONE。
 */
MotorMaintenanceKind MotorStateMachine_GetMaintenanceKind(void);
/**
 * @brief 检查快速功能是否仍具有当前任务执行资格。
 * @return 当前任务未终结、未取消、无待消费结果且无故障返回 true。
 */
bool MotorStateMachine_MaintenanceRunnable(void);
/**
 * @brief 读取当前是否需要运行电压保护确认计数。
 * @return 实际输出未关、控制已选择或转矩维护执行中返回 true，不代表已获启相许可。
 */
bool MotorStateMachine_OutputExpected(void);
#endif
