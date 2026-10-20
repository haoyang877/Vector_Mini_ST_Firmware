#ifndef MOTOR_PARAMETER_TRANSACTION_H
#define MOTOR_PARAMETER_TRANSACTION_H

#include "foc_algorithm.h"
#include "motor_state_machine.h"
#include "parameter_value_service.h"

/* 单轴参数事务所有者：仅持有修订、请求缓存和保存票据；显式借用对象，不保留对象指针。
 * 既有非 static 符号供只读 Watch，生产调用者只能通过下列接口推进状态。 */

/**
 * @brief 重置参数事务并从唯一工作区装载现有参数记录。
 * @param configuration Axis 提供的唯一活动配置，含设备编码器标定。
 * @note 仅启动时在中断使能前调用；失败采用既有默认值，不写 Flash。
 */
void MotorParameterTransaction_Init(MotorConfiguration *configuration);
/**
 * @brief 登记一次已完成的活动配置变化，修订在 UINT32_MAX 饱和。
 * @note 上电校正完成或持有参数临界区时调用，不修改控制对象。
 */
void MotorParameterTransaction_AdvanceRevision(void);
/**
 * @brief 对同一维护完成身份仅登记一次活动修订变化。
 * @param work_epoch 快拍执行前捕获的维护身份，不得改用当前身份。
 * @note 仅快拍确认准入、无故障且结果为 COMPLETED/FINALIZE 后调用。
 */
void MotorParameterTransaction_AdvanceMaintenanceRevision(uint32_t work_epoch);
/**
 * @brief 读取活动标量并按来源与序号重放最近原请求。
 * @param configuration Axis 提供的唯一活动配置，只读借用。
 * @param request 原参数 ID 和来源/序号，不保留指针。
 * @param value 接收原整数位模式、权限及活动修订，失败清零。
 * @return 原参数读取、忙或身份冲突结果。
 * @note 前台调用；内部保持原短临界区，不执行存储。
 */
ParameterStatus MotorParameterTransaction_Read(const MotorConfiguration *configuration,
                                               const ParameterReadRequest *request,
                                               ParameterValue *value);
/**
 * @brief 原子应用合法停机标量并缓存完整原请求，成功写只增加一次修订。
 * @param configuration 唯一活动配置对象，错误拒绝不应用新值。
 * @param runtime 唯一运行状态，仅检查当前故障，不保存或修改配置。
 * @param foc 唯一电流控制器对象，仅成功应用时按原边界复位。
 * @param speed_pi 唯一速度 PI 对象，仅成功应用时按原边界复位。
 * @param request 原始整数请求与 expected_revision，不保留指针。
 * @param applied 接收新修订与应用值，失败清零。
 * @return 原类型、范围、故障、状态、身份或修订校验结果。
 * @note 前台调用；原临界区内完成资格、应用、复位及回读，不自动保存。
 */
ParameterStatus MotorParameterTransaction_Write(MotorConfiguration *configuration,
                                                MotorControlState *runtime,
                                                FocState *foc,
                                                PiController *speed_pi,
                                                const ParameterWriteRequest *request,
                                                ParameterValue *applied);
/**
 * @brief 将当前活动版本的保存请求交给唯一主状态机并缓存原票据。
 * @param request 来源、序号及 expected_revision，不保留指针。
 * @param ticket 接收原始身份，拒绝清零，重放返回原身份。
 * @return ACCEPTED 仅表示受理，OK 必须来自真实后台提交。
 * @note 前台有界；旧取消主动收敛，不在请求入口擦写 Flash。
 */
ParameterStatus MotorParameterTransaction_RequestSave(const ParameterSaveRequest *request,
                                                      uint32_t *ticket);
/**
 * @brief 查询原保存票据的持久结果，不创建新任务。
 * @param ticket 受理时返回的原身份。
 * @param persisted_revision 成功返回实际提交修订，其它结果清零。
 * @return 进行中、成功、失败、取消或陈旧票据。
 * @note 前台调用，使用原临界区收敛终态。
 */
ParameterStatus MotorParameterTransaction_PollSave(uint32_t ticket, uint32_t *persisted_revision);
/**
 * @brief 查询是否已无待完成保存，并收敛原票据取消结果。
 * @return 没有 ACCEPTED 保存返回 true，不代表维护工作区已归还。
 * @note 内部短临界区；主状态机仍负责真实释放资格。
 */
bool MotorParameterTransaction_SaveIdle(void);
/**
 * @brief 查询是否还能分配新的活动修订。
 * @return 已初始化且修订未饱和返回 true。
 * @note 维护准入时调用方保持原短临界区，不暴露修订地址。
 */
bool MotorParameterTransaction_RevisionAvailable(void);
/**
 * @brief 借用参数记录并在原 claim 仍有效时执行存储和回读验证。
 * @param configuration 稳定的唯一活动配置，含编码器标定，不保留指针。
 * @param claim Axis 从主状态机取得的原 SAVE 认领。
 * @return 真正写入并回读成功返回 true；借用失败或认领失效返回 false。
 * @note 仅停机前台；参数导出开中断，复核与存储使用原临界区，返回前归还记录。
 */
bool MotorParameterTransaction_SaveRecord(const MotorConfiguration *configuration,
                                          const FocBackgroundClaim *claim);
/**
 * @brief 接收后台完成事实，只更新匹配原票据的保存终态。
 * @param ticket_epoch 原 SAVE 认领身份。
 * @param current FinishBackground 前同一临界区复核的资格事实。
 * @param success 实际存储及回读结果，不能用 ACK 或受理代替。
 * @note Axis 已归还记录并调用 FinishBackground；调用方继续持有原临界区。
 */
void MotorParameterTransaction_FinishSave(uint32_t ticket_epoch, bool current, bool success);

#endif
