#ifndef MOTOR_LIFECYCLE_H
#define MOTOR_LIFECYCLE_H
#include "motor_types.h"
#include "foc.h"
#include "pi_controller.h"
#include "encoder_feedback.h"
#include "sensorless_startup.h"

#include "motor_fault.h"
#include "motor_control.h"
#include "encoder_calibration.h"
#include "motor_sensing.h"

typedef enum
{
    MOTOR_STATE_IDLE,
    MOTOR_STATE_SELF_TEST,
    MOTOR_STATE_CALIBRATION,
    MOTOR_STATE_RUNNING,
    MOTOR_STATE_FAULT
} MotorState;

/* 每次只处理一个已由运行层校验的事件；无硬件、配置或任务依赖。 */
typedef enum
{
    MOTOR_STATE_EVENT_RUN,
    MOTOR_STATE_EVENT_CALIBRATE,
    MOTOR_STATE_EVENT_COMPLETE,
    MOTOR_STATE_EVENT_STOP,
    MOTOR_STATE_EVENT_FAULT,
    MOTOR_STATE_EVENT_CLEAR
} MotorStateEvent;

/** @brief 请求受理结果；受理不代表已经启相或标定完成。 */
typedef enum
{
    MOTOR_CONTROL_ACCEPTED = 0,
    MOTOR_CONTROL_INVALID,
    MOTOR_CONTROL_DENIED,
    MOTOR_CONTROL_BUSY,
    MOTOR_CONTROL_OUT_OF_RANGE,
    MOTOR_CONTROL_FAULT,
    MOTOR_CONTROL_PERSIST_FAILED
} MotorControlResult;

/** @brief 标定类型；带 Hall 的联合标定完成后自动保存到 Flash。 */
typedef enum
{
    MOTOR_CALIBRATION_NONE,
    MOTOR_CALIBRATION_ENCODER_LINEARIZATION,
    MOTOR_CALIBRATION_ELECTRICAL_ZERO,
    MOTOR_CALIBRATION_MECHANICAL_ZERO,
    MOTOR_CALIBRATION_JOINT_ELECTRICAL_ZERO /* 联合任务的电零位对齐，不接受外部请求。 */
} MotorCalibrationType;

/** @brief 关相后的标定交接，依次由快拍、前台和监督拍推进。 */
typedef enum
{
    MOTOR_CALIBRATION_NO_HANDOFF,
    MOTOR_CALIBRATION_AWAITING_PUBLICATION,
    MOTOR_CALIBRATION_AWAITING_COMPLETION
} MotorCalibrationHandoff;

/** @brief 一次性读取的状态值；模式和故障使用电机内部编号，不是线路编号。 */
typedef struct
{
    MotorState state;
    uint16_t mode, fault;
    bool measurements_valid;
    float position_reference_rad, position_rad;
    float speed_reference_rad_s, speed_rad_s;
    float bus_voltage_v, bus_current_a;
    float iq_reference_a, iq_a, temperature_c, vq_v, vd_v;
} MotorStatus;

/** @brief 同模式易失目标；位置rad、速度rad/s、电流A、电压V、电角频率Hz。 */
typedef struct
{
    uint16_t mode;
    float position_rad, speed_rad_s, iq_a, vq_v, frequency_hz, id_a, vd_v;
} MotorTarget;

/** @brief 一组物理功率输出；每个函数操作同一驱动器。板级或夹具在启动时绑定。 */
typedef struct
{
    void (*enable)(void);
    void (*disable)(void);
    bool (*phases_disabled)(void);
    void (*set_duty)(float phase_a, float phase_b, float phase_c);
} MotorPowerOutput;

/** @brief 生命周期借用的轴对象；其寿命覆盖本实例。 */
typedef struct
{
    const MotorConfiguration *configuration;
    MotorControlState *motor;
    FocState *foc;
    PiController *speed_pi;
    EncoderState *rotor_encoder;
    SensorlessStartupState *sensorless_startup;
    MotorOuterLoop *outer;
    EncoderCalibrationSession *calibration;
    volatile McuTemperatureTelemetry *temperature;
    const MotorPowerOutput *power;
} MotorLifecycleBindings;

/** @brief 一根轴的功率许可、故障、请求和会话编号。 */
typedef struct
{
    MotorState state;
    MotorLifecycleBindings drive;
    MotorFaultRecovery recovery;
    MotorCalibrationType calibration_type;
    uint32_t epoch;
    volatile uint32_t supervisor_ticks;
    volatile uint32_t power_enable_epoch, fast_sample_ticks;
    uint32_t observed_fast_sample_ticks;
    volatile bool stop_pending;
    volatile MotorCalibrationHandoff calibration_handoff;
    bool enable_pending, fault_locked, fast_samples_fresh, starts_locked;
} MotorLifecycle;

/**
 * @brief 建立单轴空闲生命周期，在采样开放前调用一次。
 * @param bindings 轴的控制对象及实际功率输出，所有绑定在运行期间保持有效。
 * @param life 调用方持有的实例状态。
 */
void MotorLifecycle_Init(MotorLifecycle *life, const MotorLifecycleBindings *bindings);

/**
 * @brief 快环先检查并关闭故障输出，再消费已准备的启相请求；STOP和故障优先。
 * @note 仅快速采样上下文调用一次；不扫描配置、不重置控制器、不屏蔽中断。
 *   前台/监督最后发布原代际，实际启相前重核动态保护；受理不再同步启相。
 * @param life 调用方持有的实例状态。
 */
void MotorLifecycle_BeginFastTick(MotorLifecycle *life);

/**
 * @brief 上报本周期 worker 结果并关闭本拍故障输出，由2 kHz状态机消费结果。
 * @param epoch worker 执行前捕获的原始身份。
 * @param outcome 本周期控制或标定结果；标定完成和候选发布请求由监督/前台接续。
 * @note 仅由20 kHz快速采样上下文末尾调用；每拍登记感测处理进展，
 *       原代故障立即锁存并关相，停止交给监督清理，标定完成仅发布有界标志。
 * @param life 调用方持有的实例状态。
 */
void MotorLifecycle_PostOutcome(MotorLifecycle *life, MotorTaskResult outcome, uint32_t epoch);

/**
 * @brief 读取运行所有者的生命周期状态。
 * @return 当前内部状态，数值不能直接用作线上枚举。
 * @note 只读；与其他电机测量关联采样时调用方须置于同一短临界区。
 * @param life 调用方持有的实例状态。
 */
MotorState MotorLifecycle_GetState(const MotorLifecycle *life);

/**
 * @brief 由后台提出远程停机请求，下一监督拍执行正常关相路径。
 * @note 撤销待启相请求；运行在关相后回IDLE，故障保持锁存。
 *       与监督中断通过短临界区交接。
 * @param life 调用方持有的实例状态。
 */
void MotorLifecycle_RequestStop(MotorLifecycle *life);

/**
 * @brief 请求以零目标或当前位置启动明确控制模式。
 * @param mode 业务模式，不接受线上或历史编号。
 * @return 受理、参数错误、忙、故障或守卫拒绝；受理不代表已启相。
 * @note 前台消费调用，有界非阻塞；功率迁移仍在监督调度提交。
 *       受理只登记模式和启动意图，准备期间保持IDLE，ADC实际启相后进入RUNNING。
 * @param life 调用方持有的实例状态。
 */
MotorControlResult MotorLifecycle_RequestEnable(MotorLifecycle *life, MotorControlMode mode);

/**
 * @brief 在原模式运行时校验并直接提交易失目标，不启动或切换模式。
 * @param target 调用期间有效的目标值；单位见MotorTarget，未使用字段须为零。
 * @return ACCEPTED表示已写入目标；故障、停机、模式不符或越限时保持原目标。
 * @note 前台调用，短临界区与快慢环互斥；不保存Flash，不等待机械响应。
 * @param life 调用方持有的实例状态。
 */
MotorControlResult MotorLifecycle_SetTarget(MotorLifecycle *life, const MotorTarget *target);

/**
 * @brief 从空闲状态请求一项明确标定，受理后进入CALIBRATION。
 * @param calibration_type 编码器线性化、电气零位或机械零位；NONE和非法值拒绝。
 * @return 已受理、参数错误、忙、状态拒绝或故障；受理不代表标定已经完成。
 * @note 前台只受理请求；快拍复核后启相，关相前台在原会话仍有效时保存。
 * @param life 调用方持有的实例状态。
 */
MotorControlResult MotorLifecycle_RequestCalibration(MotorLifecycle *life,
                                                     MotorCalibrationType calibration_type);

/**
 * @brief 读取当前标定类型，完成或取消后返回NONE。
 * @return 当前标定类型。
 * @param life 调用方持有的实例状态。
 */
MotorCalibrationType MotorLifecycle_GetCalibrationType(const MotorLifecycle *life);

/**
 * @brief 判断本快拍是否可执行当前标定算法。
 * @return 当前会话有效、无停止/故障且输出符合该标定要求时返回true。
 * @note 仅由ADC快环调用；完成或等待LUT发布时返回false。
 * @param life 调用方持有的实例状态。
 */
bool MotorLifecycle_IsCalibrationRunnable(const MotorLifecycle *life);

/**
 * @brief 读取待发布或保存的角度标定会话代际。
 * @param calibration_epoch 非空输出指针，返回待发布会话的代际，不可使用后续代际替代。
 * @return 当前候选仍有效且已关相时返回true。
 * @note 前台发布、写 Flash 和完成通知前均复核原代际。
 * @param life 调用方持有的实例状态。
 */
bool MotorLifecycle_ReadPendingCalibrationPublication(const MotorLifecycle *life,
                                                      uint32_t *calibration_epoch);

/**
 * @brief 关相发布编码器结果后，在同一联合标定会话内转入电零位对齐。
 * @note 前台持全中断临界区调用；调用方已确认本会话待发布、支持 Hall 且 LUT 发布成功。
 *       下一快拍仍检查启相条件，STOP 或故障不能重新启相。
 * @param life 调用方持有的实例状态。
 */
void MotorLifecycle_ContinueElectricalZero(MotorLifecycle *life);

/**
 * @brief 提交原会话的发布结果，成功后由监督层触发COMPLETE。
 * @param calibration_epoch ReadPendingCalibrationPublication返回的原代际。
 * @param publication_succeeded 候选是否已经成功发布到活动配置。
 * @note 进入全中断临界区后复核原会话；过期结果忽略，候选发布失败锁存编码器故障。
 * @param life 调用方持有的实例状态。
 */
void MotorLifecycle_CompleteCalibrationPublication(MotorLifecycle *life,
                                                   uint32_t calibration_epoch,
                                                   bool publication_succeeded);

/**
 * @brief 核对恢复资格并同步清除当前可恢复锁存故障。
 * @return 受理表示锁存已清或 IDLE 本来无故障；忙或拒绝保持锁存，不可恢复故障保持锁存。
 * @note 临界区内确认故障源恢复、基础健康及收尾完成；CLEAR 成功直接 IDLE，仍需新启动请求。
 *       只清错，不恢复模式或旧目标。
 * @param life 调用方持有的实例状态。
 */
MotorControlResult MotorLifecycle_RequestClear(MotorLifecycle *life);

/**
 * @brief 读取监督拍进展与基础采样健康，不要求主机在线或电机已使能。
 * @param progress 输出已完成监督拍计数；调用方必须比较新进展。
 * @return 采样新鲜且未进入不可清除故障返回true；空指针返回false。
 * @note 前台短临界区只读；通信健康由组合层补充，不授权运动或Flash。
 * @param life 调用方持有的实例状态。
 */
bool MotorLifecycle_ReadBootHealth(MotorLifecycle *life, uint32_t *progress);

/**
 * @brief 读取 worker 开始执行时的会话身份。
 * @return 当前运行代际；停止或取消后立即失效原worker代际。
 * @note 在调用 worker 前读取，并原样传给 PostOutcome。
 * @param life 调用方持有的实例状态。
 */
uint32_t MotorLifecycle_WorkEpoch(const MotorLifecycle *life);

/**
 * @brief 完成一次 2 kHz 状态监督和故障恢复检查。
 * @note 仅由监督上下文调用。
 * @param life 调用方持有的实例状态。
 */
void MotorLifecycle_SupervisorTick(MotorLifecycle *life);

/** @brief 外环一次调用的栈上副本。owner 和 epoch 标识可提交的轴与会话。 */
typedef struct
{
    const MotorLifecycle *owner;
    uint32_t epoch;
    MotorControlState motor;
    PiController speed_pi;
    bool allowed;
} MotorOuterWork;

/**
 * @brief 在控制临界区取得本拍外环输入。
 * @param life 所属轴的生命周期。
 * @param work 调用者持有的副本，计算和提交期间有效。
 */
void MotorLifecycle_BeginOuter(MotorLifecycle *life, MotorOuterWork *work);

/**
 * @brief 处理停止和故障，再以原会话身份提交外环结果。
 * @param life 所属轴的生命周期。
 * @param work 来自 BeginOuter 的本拍结果。
 * @note 外环上下文调用，同一实例每拍仅有一个计算者。
 */
void MotorLifecycle_CommitOuter(MotorLifecycle *life, const MotorOuterWork *work);
/**
 * @brief 永久禁止本次上电再启动，并登记停机请求。
 * @return 已锁定返回true；不可恢复故障返回false。
 * @note 前台升级交接使用，复位前不可撤销；不代表输出已经关闭。
 * @param life 调用方持有的实例状态。
 */
bool MotorLifecycle_LockStop(MotorLifecycle *life);
/**
 * @brief 从该轴绑定的输出端读取三相是否实际关闭。
 * @param life 已初始化的生命周期。
 * @return 所有相通道关闭时为 true。
 */
bool MotorLifecycle_ArePhasesDisabled(const MotorLifecycle *life);

/**
 * @brief 查询是否已停止且数字输出实际关闭。
 * @return 无待启动/停止且处于空闲或故障、模式失能和关相时返回true。
 * @note 只读；升级复位前须由调用方在同一最终临界区复核。
 * @param life 调用方持有的实例状态。
 */
bool MotorLifecycle_IsStopped(const MotorLifecycle *life);
/**
 * @brief 一次性读取主状态、控制模式、故障和现有测量。
 * @return 同一临界区的值快照；无效测量为NAN，measurements_valid表示采样新鲜。
 * @note 前台调用；内部短暂屏蔽中断，不编码或发送数据。
 * @param life 调用方持有的实例状态。
 */
MotorStatus MotorLifecycle_ReadStatus(const MotorLifecycle *life);
/**
 * @brief 计算五态的下一状态，非法状态或事件统一进入故障。
 * @param state 当前状态；上电由运行层置为IDLE，自检仅预留。
 * @param event 单个事件；运行层负责健康校验、关相和资源释放。
 * @return 下一状态；清错回空闲，自检占位保持原状态。
 * @note 纯函数；RUN事件仅在实际启相确认后提交，完成/停止前必须关闭输出。
 */
MotorState MotorLifecycle_NextState(MotorState state, MotorStateEvent event);
#endif
