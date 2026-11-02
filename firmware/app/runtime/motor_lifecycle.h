#ifndef MOTOR_LIFECYCLE_H
#define MOTOR_LIFECYCLE_H
#include "motor_types.h"
#include "foc.h"
#include "pi_controller.h"
#include "encoder_feedback.h"
#include "sensorless_startup.h"
#include "motor_state_types.h"

/**
 * @brief 初始化运行所有者：进入空闲态，清空命令和采样进展；自检暂不实现。
 * @param configuration 组合层终身配置只读视图；状态机不拥有配置写入口。
 * @param motor 组合层终身电机运行对象。
 * @param foc 组合层终身 foc 对象。
 * @param pi 组合层终身速度 PI。
 * @param encoder 组合层终身编码器对象。
 * @param startup 组合层终身无感启动对象。
 * @note 由启动组合层在中断使能前调用一次；不访问硬件。
 */
void MotorLifecycle_Init(const MotorConfiguration *configuration,
                         MotorControlState *motor,
                         FocState *foc,
                         PiController *pi,
                         EncoderState *encoder,
                         SensorlessStartupState *startup);

/**
 * @brief 快环先检查并关闭故障输出，再消费已准备的启相请求；STOP和故障优先。
 * @note 仅快速采样上下文调用一次；不扫描配置、不重置控制器、不屏蔽中断。
 *   前台/监督最后发布原代际，实际启相前重核动态保护；受理不再同步启相。
 */
void MotorLifecycle_BeginFastTick(void);

/**
 * @brief 上报本周期 worker 结果并关闭本拍故障输出，由2 kHz状态机消费结果。
 * @param epoch worker 执行前捕获的原始身份。
 * @param outcome 本周期控制或标定结果；标定完成和候选发布请求由监督/前台接续。
 * @note 仅由20 kHz快速采样上下文末尾调用；每拍登记感测处理进展，
 *       原代故障立即锁存并关相，停止交给监督清理，标定完成仅发布有界标志。
 */
void MotorLifecycle_PostOutcome(MotorTaskResult outcome, uint32_t epoch);

/**
 * @brief 读取运行所有者的生命周期状态。
 * @return 当前内部状态，数值不能直接用作线上枚举。
 * @note 只读；与其他电机测量关联采样时调用方须置于同一短临界区。
 */
MotorState MotorLifecycle_GetState(void);

/**
 * @brief 由后台提出远程停机请求，下一 2 kHz 监督拍执行正常关相路径。
 * @return 请求已登记返回 true；不表示功率输出已经关闭。
 * @note 撤销待启相请求；运行在关相后回IDLE，故障保持锁存。
 *       与监督中断通过短临界区交接。
 */
bool MotorLifecycle_RequestRemoteStop(void);

/**
 * @brief 请求以零目标或当前位置启动明确控制模式。
 * @param mode 业务模式，不接受线上或历史编号。
 * @return 受理、参数错误、忙、故障或守卫拒绝；受理不代表已启相。
 * @note 前台消费调用，有界非阻塞；功率迁移仍在监督调度提交。
 *       受理只登记模式和启动意图，准备期间保持IDLE，ADC实际启相后进入RUNNING。
 */
MotorControlResult MotorLifecycle_RequestEnable(MotorControlMode mode);

/**
 * @brief 在原模式运行时校验并直接提交易失目标，不启动或切换模式。
 * @param target 调用期间有效的目标值；单位见MotorTarget，未使用字段须为零。
 * @return ACCEPTED表示已写入目标；故障、停机、模式不符或越限时保持原目标。
 * @note 前台调用，短临界区与快慢环互斥；不保存Flash，不等待机械响应。
 */
MotorControlResult MotorLifecycle_SetTarget(const MotorTarget *target);

/**
 * @brief 从空闲状态请求一项明确标定，受理后进入CALIBRATION。
 * @param calibration_type 编码器线性化、电气零位或机械零位；NONE和非法值拒绝。
 * @return 已受理、参数错误、忙、状态拒绝或故障；受理不代表标定已经完成。
 * @note 前台调用，不执行算法；有功率标定由下一ADC快拍复核后启相，不自动保存Flash。
 */
MotorControlResult MotorLifecycle_RequestCalibration(MotorCalibrationType calibration_type);

/**
 * @brief 读取当前标定类型，完成或取消后返回NONE。
 * @return 当前标定类型。
 */
MotorCalibrationType MotorLifecycle_GetCalibrationType(void);

/**
 * @brief 判断本快拍是否可执行当前标定算法。
 * @return 当前会话有效、无停止/故障且输出符合该标定要求时返回true。
 * @note 仅由ADC快环调用；完成或等待LUT发布时返回false。
 */
bool MotorLifecycle_IsCalibrationRunnable(void);

/**
 * @brief 读取待发布的编码器线性化候选及原会话代际。
 * @param calibration_epoch 返回待发布会话的代际，不可使用后续代际替代。
 * @return 当前候选仍有效且已关相时返回true。
 * @note 前台在同一全中断临界区内读取、发布候选并调用CompleteCalibrationPublication。
 */
bool MotorLifecycle_ReadPendingCalibrationPublication(uint32_t *calibration_epoch);

/**
 * @brief 提交原会话的LUT发布结果，成功后由监督层触发COMPLETE。
 * @param calibration_epoch ReadPendingCalibrationPublication返回的原代际。
 * @param publication_succeeded 候选是否已经成功发布到活动配置。
 * @note 与候选发布位于同一全中断临界区；过期结果忽略，失败锁存编码器故障。
 */
void MotorLifecycle_CompleteCalibrationPublication(uint32_t calibration_epoch,
                                                   bool publication_succeeded);

/**
 * @brief 核对恢复资格并同步清除当前可恢复锁存故障。
 * @return 受理表示锁存已清或 IDLE 本来无故障；忙或拒绝保持锁存，不可恢复故障保持锁存。
 * @note 临界区内确认故障源恢复、基础健康及收尾完成；CLEAR 成功直接 IDLE，仍需新启动请求。
 *       只清错，不恢复模式或旧目标。
 */
MotorControlResult MotorLifecycle_RequestClear(void);

/**
 * @brief 读取监督拍进展与基础采样健康，不要求主机在线或电机已使能。
 * @param progress 输出已完成监督拍计数；调用方必须比较新进展。
 * @return 采样新鲜且未进入不可清除故障返回true；空指针返回false。
 * @note 前台短临界区只读；通信健康由组合层补充，不授权运动或Flash。
 */
bool MotorLifecycle_ReadBootHealth(uint32_t *progress);

/**
 * @brief 读取 worker 开始执行时的会话身份。
 * @return 当前运行代际；停止或取消后立即失效原worker代际。
 * @note 在调用 worker 前读取，并原样传给 PostOutcome。
 */
uint32_t MotorLifecycle_WorkEpoch(void);

/**
 * @brief 完成一次状态监督、外环计算与安全提交。
 * @note 仅2kHz监督调用；用栈上副本计算，STOP、故障或代际改变时丢弃结果。
 */
void MotorLifecycle_SupervisorTick(void);
/**
 * @brief 永久禁止本次上电再启动，并登记停机请求。
 * @return 已锁定返回true；不可恢复故障返回false。
 * @note 前台升级交接使用，复位前不可撤销；不代表输出已经关闭。
 */
bool MotorLifecycle_LockStop(void);
/**
 * @brief 查询是否已停止且数字输出实际关闭。
 * @return 无待启动/停止且处于空闲或故障、模式失能和关相时返回true。
 * @note 只读；升级复位前须由调用方在同一最终临界区复核。
 */
bool MotorLifecycle_IsStopped(void);
/**
 * @brief 一次性读取主状态、控制模式、故障和现有测量。
 * @return 同一临界区的值快照；无效测量为NAN，measurements_valid表示采样新鲜。
 * @note 前台调用；内部短暂屏蔽中断，不编码或发送数据。
 */
MotorStatus MotorLifecycle_ReadStatus(void);
#endif
