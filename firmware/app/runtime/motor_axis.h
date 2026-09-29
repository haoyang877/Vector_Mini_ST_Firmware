#ifndef MOTOR_AXIS_H
#define MOTOR_AXIS_H
#include <stdbool.h>
#include "motor_types.h"
#include "motor_status.h"
#include "motor_state_types.h"
#include "motor_lifecycle_request.h"
#include "motor_control_command.h"
#include "parameter_value_service.h"
/**
 * @brief 装载参数并初始化唯一电机执行对象。
 * @note 仅启动组合层在中断启用前调用一次，不自动使能。
 */
void MotorAxis_Init(void);
/**
 * @brief 读取参数所有者持有的设备节点地址值。
 * @return 当前设备配置地址；初始化前为零，不返回内部对象。
 * @note 启动组合层在 Init 后使用；不修改正在运行的通信端点。
 */
uint8_t MotorAxis_NodeId(void);
/**
 * @brief 执行快速采样、控制和结果发布。
 * @note 20 kHz ADC2 JEOS 上下文，执行有界。
 */
void MotorAxis_FastTick(void);
/**
 * @brief 执行慢估计、外环和生命周期监督。
 * @return 本次提交后的模式，仅用于指示器。
 * @note 2 kHz TIM7 上下文。
 */
MotorControlMode MotorAxis_SupervisorTick(void);
/**
 * @brief 采集一致的只读电机测量值。
 * @param sample 调用方快照空间，不保留指针。
 * @param valid_bits 接收既有测量有效位。
 * @param maintenance_kind 接收同一临界区内的维护任务类型，空闲为 NONE。
 * @param control_progress 接收同一临界区内的控制请求进展。
 * @return 关联的生命周期状态。
 * @note 前台调用；内部短临界区，不暴露控制对象地址。
 */
MotorState MotorAxis_ReadStatus(MotorStatus *sample,
                                uint8_t *valid_bits,
                                MotorMaintenanceKind *maintenance_kind,
                                MotorControlProgress *control_progress);
/**
 * @brief 执行受理的停机前台维护。
 * @note 主循环调用；表整理和 Flash 操作禁止在 ISR 执行。
 */
void MotorAxis_ServiceBackground(void);
/**
 * @brief 读取单个一致的活动参数值与修订号。
 * @param request 参数 ID 与来源/序号；返回后不保留指针。
 * @param value 接收类型、权限、活动 revision 和原始整数值；失败清零。
 * @return 成功、忙、无来源或编码错误。
 * @note 短临界区，不创建完整参数副本，不返回内部地址。
 */
ParameterStatus MotorAxis_ReadParameter(const ParameterReadRequest *request, ParameterValue *value);
/**
 * @brief 原子应用受理的停机单标量，并重放最近相同身份请求。
 * @param request 原始请求值，返回后不保留指针。
 * @param applied 接收原始应用值与新 revision；失败清零。
 * @return 已应用、忙、状态、类型、范围或 revision 错误。
 * @note 同身份不同内容拒绝；写成功只增加一次 revision，不自动保存。
 *       入口查询旧保存终态；取消尚未确认归还时仍拒绝写入。
 */
ParameterStatus MotorAxis_WriteParameter(const ParameterWriteRequest *request,
                                         ParameterValue *applied);
/**
 * @brief 认领指定活动版本的持久保存任务。
 * @param request 请求身份与 expected_revision，返回后不保留指针。
 * @param ticket 接收创建时身份，失败清零；相同请求返回原身份。
 * @return ACCEPTED 仅表示任务受理；真实提交完成后才为 OK。
 * @note 前台有界调用，不在入口擦写 Flash，不允许运行或资源借用中保存。
 *       新请求主动查询旧保存终态，不要求主机继续轮询旧任务。
 */
ParameterStatus MotorAxis_RequestSave(const ParameterSaveRequest *request, uint32_t *ticket);
/**
 * @brief 查询原始保存任务的持久结果。
 * @param ticket 受理时返回的原身份。
 * @param persisted_revision 成功时接收已经写入并回读验证的活动版本；失败清零。
 * @return ACCEPTED 进行中、OK 已提交、保存失败、取消或陈旧身份。
 * @note 不重复创建任务，不以当前 epoch 冒充原身份，不宣称 ACK 等于持久成功。
 */
ParameterStatus MotorAxis_PollSave(uint32_t ticket, uint32_t *persisted_revision);
/**
 * @brief 收敛已取消保存的业务结果并查询是否仍有未完成保存。
 * @return 没有进行中的保存返回true；真实前台资源是否归还还须由运行所有者确认。
 * @note 前台有界调用，只查询原ticket，不写Flash、不恢复目标。
 */
bool MotorAxis_SaveIdle(void);
/**
 * @brief 查询维护是否仍能分配新的活动配置修订。
 * @return 已初始化且修订未耗尽返回true；否则禁止会修改配置的维护。
 * @note 维护准入与本查询须在同一短临界区；允许读取或保存当前版本，不返回对象地址。
 */
bool MotorAxis_ConfigurationRevisionAvailable(void);
#endif
