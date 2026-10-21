#ifndef MOTOR_STATE_MACHINE_H
#define MOTOR_STATE_MACHINE_H
#include "motor_readiness.h"

#include <stdbool.h>

#include "motor_task_result.h"
#include "foc_algorithm.h"
#include "foc_pid.h"
#include "angle_feedback.h"
#include "flux_observer.h"
#include "motor_state_types.h"
#include "motor_control_command.h"
#include "motor_lifecycle_request.h"

/**
 * @brief 初始化运行所有者：核心进入 INIT 态，清空命令和采样进展。
 * @param configuration 组合层终身配置只读视图；状态机不拥有配置写入口。
 * @param motor 组合层终身电机运行对象。
 * @param foc 组合层终身 foc 对象。
 * @param pi 组合层终身速度 PI。
 * @param encoder 组合层终身编码器对象。
 * @param startup 组合层终身无感启动对象。
 * @note 由启动组合层在中断使能前调用一次；不访问硬件。
 */
void MotorStateMachine_Init(const MotorConfiguration *configuration,
                            MotorControlState *motor,
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
/** @brief 单次监督的栈上计算载体；不是活动运行对象，禁止保留到下一拍。 */
typedef struct
{
    uint32_t supervisor_epoch;
    uint32_t state_epoch;
    MotorControlState motor;
    PiController speed_pi;
    bool control_allowed;
    bool calculated;
} MotorSupervisorStep;

/**
 * @brief 消费一次监督输入、推进状态并准备获准控制的本拍副本。
 * @param step 调用方栈上载体，必须非空；随后同拍交给CommitSupervisor，不得复用。
 * @return 本拍允许执行既有控制准备或外环计算时返回true。
 * @note 仅由2kHz监督调用一次；STOP/故障优先，新目标在计算前应用。
 *       自检、结果消费及保存衔接只在此推进；不得重复调用来代替最终复核。
 */
bool MotorStateMachine_PrepareSupervisor(MotorSupervisorStep *step);

/**
 * @brief 复核本拍身份和保护事实，发布有效计算结果并决定是否启相。
 * @param step 本拍PrepareSupervisor得到的载体；计算结束后设置calculated，再提交一次。
 * @note 仅由同一2kHz监督在计算后调用；过期或重复提交无效。
 *       SPEED/POSITION须本拍计算且原ready有效；此处不重复推进自检或消费任务结果。
 */
void MotorStateMachine_CommitSupervisor(const MotorSupervisorStep *step);

/**
 * @brief 读取运行所有者的生命周期状态。
 * @return 当前内部状态，数值不能直接用作线上枚举。
 * @note 只读；与其他电机测量关联采样时调用方须置于同一短临界区。
 */
MotorState MotorStateMachine_GetState(void);

/**
 * @brief 由后台提出远程停机请求，下一 2 kHz 监督拍执行正常关相路径。
 * @return 请求已登记返回 true；不表示功率输出已经关闭。
 * @note INIT／IDLE 保持原状态并撤销排队意图；只有 STARTING／RUNNING／MAINTENANCE
 *       进入 STOPPING，停止完成直接 IDLE。与监督中断通过短临界区交接。
 */
bool MotorStateMachine_RequestRemoteStop(void);

/**
 * @brief 以线上来源和序号执行远程 STOP，并阻止同源旧启动请求迟到后启相。
 * @param source_node 已校验的远程来源；0 和 255 不推进序号屏障。
 * @param sequence 原始 16 位序号；旧 STOP 仍执行关断，不回退屏障。
 * @return 已登记停机返回 true，不表示功率输出已关闭。
 * @note 与启动受理在同一短临界区排序；STOP 不能因序号陈旧而拒绝。
 */
bool MotorStateMachine_RequestIdentifiedStop(uint8_t source_node, uint16_t sequence);

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
 * @brief 复制最近带来源控制请求的实际进展。
 * @param progress 调用方值空间，不保留指针；空指针不执行。
 * @note 前台只读，内部短临界区；需要关联测量时与测量采集置于同一临界区。
 *       来源 0 的本地请求不覆盖远程进展；APPLIED 不表示机械到位。
 */
void MotorStateMachine_ReadControlProgress(MotorControlProgress *progress);
/**
 * @brief 请求以零目标或当前位置启动明确控制模式。
 * @param mode 业务模式，不接受线上或历史编号。
 * @return 受理、参数错误、忙、故障或守卫拒绝；受理不代表已启相。
 * @note 前台消费调用，有界非阻塞；功率迁移仍在监督调度提交。
 *       受理只登记模式和启动意图，主状态暂留 IDLE；监督拍再进入 STARTING。
 */
MotorControlResult MotorStateMachine_RequestEnable(MotorControlMode mode);

/**
 * @brief 幂等受理带身份的四模式零目标／当前位置启动，复用唯一运行所有者的原启相路径。
 * @param request 原始来源、序号和 CURRENT／SPEED／POSITION／VQ 模式；函数返回后不保留指针。
 * @return 受理或明确拒绝原因；受理不表示启相，结果由 ReadEnableProgress 查询。
 * @note 同来源序号重试不再执行；新拒绝也消耗序号，STOP 后旧序号不能启动。
 *       仅 IDLE 启动；切模式须先 STOP 回 IDLE；拒绝不提交来源绑定。
 *       序号窗口仅在当前 APP 运行期内有效，跨复位防重放尚需线路契约。
 */
MotorControlResult
MotorStateMachine_RequestIdentifiedEnable(const MotorControlStartRequest *request);

/**
 * @brief 读取一条带身份启动的进展，包括最近请求及仍保留的活动请求。
 * @param source_node 原始来源。
 * @param sequence 原始序号。
 * @param progress 接收值结果；未找到时清零，不保留指针。
 * @return 身份仍在有界记录内时返回 true。
 * @note STARTED 是历史结果，当前是否仍运行须再读取状态；不代表物理运动。
 */
bool MotorStateMachine_ReadEnableProgress(uint8_t source_node,
                                          uint16_t sequence,
                                          MotorControlStartProgress *progress);
/**
 * @brief 请求原有维护任务并分配创建时身份。
 * @param kind 明确业务任务。
 * @param ticket 接收原始身份，失败清零。
 * @return 受理、错误、忙、故障或守卫拒绝。
 * @note 不保留调用方指针；同步进入 MAINTENANCE，出力标定可在受理内启相。
 *       不执行 Flash 或长耗时工作；SAVE 的实际写入由前台认领后执行。
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
 * @return 受理表示锁存已清或 IDLE 本来无故障；忙或拒绝保持锁存，FATAL 不可清。
 * @note 临界区内确认故障源恢复、基础健康及收尾完成；CLEAR 成功直接 IDLE，仍需新启动请求。
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
 * @return IDLE、关相、无命令/维护/配置借用和故障时返回 true。
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
 * @return IDLE或FAULT、数字关相、无控制/维护/借用/后台时返回true。
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

/**
 * @brief 复制基础自检事实及各模式当前启动资格。
 * @param readiness 调用方值空间；空指针不执行。
 * @note 前台短临界区只读；资格随采样、配置和状态变化，不能作为延迟使能许可。
 */
void MotorStateMachine_ReadReadiness(MotorReadiness *readiness);

typedef enum
{
    FOC_BACKGROUND_NONE,
    FOC_BACKGROUND_SAVE,
    FOC_BACKGROUND_COGGING /* 退役编号，不再可执行。 */
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
 * @param success 仅 SAVE，true 表示回读已验证。
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
