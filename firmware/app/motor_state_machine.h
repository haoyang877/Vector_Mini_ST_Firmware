#ifndef MOTOR_STATE_MACHINE_H
#define MOTOR_STATE_MACHINE_H

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

/**
 * @brief 按单个事件计算下一主状态。
 * @param state 调用方持有的当前状态，每根轴独立。
 * @param event 已满足执行条件的事件，非法事件进入故障。
 * @return 迁移后的状态。没有对应迁移时保持原状态，非法状态进入故障。
 */
MotorState MotorStateMachine_NextState(MotorState state, MotorStateEvent event);

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
    MOTOR_CALIBRATION_COMBINED_ELECTRICAL_ZERO /* 联合标定的电零位阶段，不接受外部请求。 */
} MotorCalibrationType;

/** @brief 状态机保存的真实标定终态；成功只在实际保存后的COMPLETE发布。 */
typedef enum
{
    MOTOR_CALIBRATION_RESULT_NONE,
    MOTOR_CALIBRATION_RESULT_RUNNING,
    MOTOR_CALIBRATION_RESULT_COMPLETED,
    MOTOR_CALIBRATION_RESULT_CANCELLED,
    MOTOR_CALIBRATION_RESULT_FAILED
} MotorCalibrationResult;

/** @brief 关相后的标定交接，依次由快拍、前台和监督拍推进。 */
typedef enum
{
    MOTOR_CALIBRATION_NO_HANDOFF,
    MOTOR_CALIBRATION_AWAITING_PUBLICATION,
    MOTOR_CALIBRATION_AWAITING_COMPLETION
} MotorCalibrationHandoff;

/** @brief 监督单写的电机NTC，时间戳按毫秒回绕；无首样本时valid为false。 */
typedef struct
{
    float celsius;
    uint32_t sample_ms;
    bool valid;
} MotorTemperatureTelemetry;

/** @brief 外环单写的输出反馈；计数来自实际完成的ADC快拍。 */
typedef struct
{
    float hall_position_rad, tracked_position_rad, zero_rad, gear_ratio;
    uint32_t sample_ms, sample_counter;
    uint16_t flags;
} MotorOutputTelemetry;

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
    float motor_temperature_c;
    bool motor_temperature_valid;
    MotorOutputTelemetry output;
    uint32_t output_age_ms;
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

/** @brief 状态机借用的轴对象；其寿命覆盖本实例。 */
typedef struct
{
    const MotorConfiguration *configuration;
    const uint32_t *parameter_revision;
    MotorControlState *motor;
    FocState *foc;
    PiController *speed_pi;
    EncoderState *rotor_encoder;
    SensorlessStartupState *sensorless_startup;
    MotorOuterLoop *outer;
    EncoderCalibrationSession *calibration;
    volatile McuTemperatureTelemetry *temperature;
    const MotorTemperatureTelemetry *motor_temperature;
    const MotorOutputTelemetry *output;
    const MotorPowerOutput *power;
} MotorStateMachineBindings;

typedef struct MotorStateMachine MotorStateMachine;

/** @brief 外环一次调用的栈上副本。owner 和 epoch 标识可提交的轴与会话。 */
typedef struct
{
    const MotorStateMachine *owner;
    uint32_t epoch;
    MotorControlState motor;
    PiController speed_pi;
    bool allowed;
} MotorOuterWork;

/** @brief 一根轴的功率许可、故障、请求和会话编号。 */
struct MotorStateMachine
{
    MotorState state; /* 本轴主状态，外部只读。 */
    MotorStateMachineBindings drive;
    MotorFaultRecovery recovery;
    volatile MotorCalibrationType calibration_type;
    volatile uint32_t calibration_result_epoch, calibration_saved_epoch;
    volatile MotorCalibrationResult calibration_result;
    volatile MotorFaultCode calibration_failure;
    uint32_t epoch; /* 当前会话号，外部只读。 */
    volatile uint32_t supervisor_ticks, outer_ticks;
    volatile uint32_t phase_enable_epoch; /* 本会话待 ADC 复核的启相许可，零表示无许可。 */
    volatile uint32_t fast_sample_ticks;
    uint32_t observed_fast_sample_ticks;
    volatile bool stop_pending;
    volatile MotorCalibrationHandoff calibration_handoff;
    volatile uint32_t motion_renewed_ms; /* 前台唯一写者，监督读取。 */
    volatile bool motion_lease_active;   /* 仅协议启动启用，结束会话撤销。 */
    volatile bool parameters_updating;   /* 关相前台正在提交配置，禁止启相和配置扫描。 */
    bool start_pending;                  /* 普通启动已受理，尚未确认三相输出开启。 */
    bool fault_locked, fast_samples_fresh, starts_locked;
};

/**
 * @brief 初始化本轴状态为 IDLE，在采样开放前调用一次。
 * @param bindings 轴的控制对象及实际功率输出，所有绑定在运行期间保持有效。
 * @param machine 调用方持有的实例状态。
 */
void MotorStateMachine_Init(MotorStateMachine *machine, const MotorStateMachineBindings *bindings);

/**
 * @brief 快环先检查并关闭故障输出，再消费已准备的启相请求；STOP和故障优先。
 * @note 仅快速采样上下文调用一次；不扫描配置、不重置控制器、不屏蔽中断。
 *   前台/监督最后发布原代际，实际启相前重核动态保护；受理不再同步启相。
 * @param machine 调用方持有的实例状态。
 */
void MotorStateMachine_BeginFastTick(MotorStateMachine *machine);

/**
 * @brief 快拍结束时处理控制或标定结果，本拍发生故障时立即关相。
 * @param epoch 执行控制或标定前读取的会话号，用来识别过期结果。
 * @param outcome 本周期控制或标定结果；标定完成和候选发布请求由监督/前台接续。
 * @note 仅由20 kHz快速采样上下文末尾调用；每拍登记感测处理进展，
 *       原代故障立即锁存并关相，停止交给监督清理，标定完成仅发布有界标志。
 * @param machine 调用方持有的实例状态。
 */
void MotorStateMachine_FinishFastTick(MotorStateMachine *machine,
                                      MotorTaskResult outcome,
                                      uint32_t epoch);

/**
 * @brief 登记停机请求并立即撤销本轴的启动和启相许可。
 * @note ADC 遇到待停请求先关相。监督拍或外环处理停机，确认关相后回空闲，已有故障优先。
 * @param machine 调用方持有的实例状态。
 */
void MotorStateMachine_RequestStop(MotorStateMachine *machine);

/**
 * @brief 请求以零目标或当前位置启动明确控制模式。
 * @param mode 业务模式，不接受线上或历史编号。
 * @return 受理、参数错误、忙、故障或守卫拒绝；受理不代表已启相。
 * @note 前台调用。启动受理后保持 IDLE，外环发布启相意图，ADC 确认三相开启后进入 RUNNING。
 * @param machine 调用方持有的实例状态。
 */
MotorControlResult MotorStateMachine_RequestStart(MotorStateMachine *machine,
                                                  MotorControlMode mode);

/**
 * @brief 在原模式运行时校验并写入控制目标，不启动或切换模式。
 * @param target 调用期间有效的目标值；单位见MotorTarget，未使用字段须为零。
 * @return ACCEPTED 表示目标已受理；拒绝不覆盖上次目标。
 * @note 校验通过后直接写本轴。ADC 可在各字段写入之间抢占。
 * @param machine 调用方持有的实例状态。
 */
MotorControlResult MotorStateMachine_SetTarget(MotorStateMachine *machine,
                                               const MotorTarget *target);

/**
 * @brief 从空闲状态请求一项明确标定，受理后进入CALIBRATION。
 * @param calibration_type 编码器线性化、电气零位或机械零位；NONE和非法值拒绝。
 * @return 已受理、参数错误、忙、状态拒绝或故障；受理不代表标定已经完成。
 * @note 前台只受理请求；快拍复核后启相，关相前台在原会话仍有效时保存。
 * @param machine 调用方持有的实例状态。
 */
MotorControlResult MotorStateMachine_RequestCalibration(MotorStateMachine *machine,
                                                        MotorCalibrationType calibration_type);

/**
 * @brief 读取当前标定类型，完成或取消后返回NONE。
 * @return 当前标定类型。
 * @param machine 调用方持有的实例状态。
 */
MotorCalibrationType MotorStateMachine_GetCalibrationType(const MotorStateMachine *machine);

/**
 * @brief 判断本快拍是否可执行当前标定算法。
 * @return 当前会话有效、无停止/故障且输出符合该标定要求时返回true。
 * @note 仅由ADC快环调用；完成或等待LUT发布时返回false。
 * @param machine 调用方持有的实例状态。
 */
bool MotorStateMachine_IsCalibrationRunnable(const MotorStateMachine *machine);

/**
 * @brief 读取待发布或保存的角度标定会话代际。
 * @param calibration_epoch 非空输出指针，返回待发布会话的代际，不可使用后续代际替代。
 * @return 当前候选仍有效且已关相时返回true。
 * @note 前台发布、写 Flash 和完成通知前均复核原代际。
 * @param machine 调用方持有的实例状态。
 */
bool MotorStateMachine_ReadPendingCalibrationPublication(const MotorStateMachine *machine,
                                                         uint32_t *calibration_epoch);

/**
 * @brief 关相发布编码器结果后，在同一联合标定会话内转入电零位对齐。
 * @note 由 ADC 安装 LUT 后在同一快拍调用，已确认会话、实际关相和 Hall 能力。
 *       下一快拍仍检查启相条件，STOP 或故障不能重新启相。
 * @param machine 调用方持有的实例状态。
 */
void MotorStateMachine_ContinueElectricalZero(MotorStateMachine *machine);

/**
 * @brief 提交原会话的发布结果，成功后由监督层触发COMPLETE。
 * @param calibration_epoch ReadPendingCalibrationPublication返回的原代际。
 * @param publication_succeeded 候选是否已经成功发布到活动配置。
 * @note ADC 或前台复核原会话；Flash 提交时由调用方持事务保护。过期结果忽略。
 * @param machine 调用方持有的实例状态。
 */
void MotorStateMachine_CompleteCalibrationPublication(MotorStateMachine *machine,
                                                      uint32_t calibration_epoch,
                                                      bool publication_succeeded);

/**
 * @brief 核对恢复资格并同步清除当前可恢复锁存故障。
 * @return 受理表示锁存已清或 IDLE 本来无故障；忙或拒绝保持锁存，不可恢复故障保持锁存。
 * @note 临界区内确认故障源恢复、基础健康及收尾完成；CLEAR 成功直接 IDLE，仍需新启动请求。
 *       只清错，不恢复模式或旧目标。
 * @param machine 调用方持有的实例状态。
 */
MotorControlResult MotorStateMachine_RequestFaultClear(MotorStateMachine *machine);

/**
 * @brief 读取监督拍进展与基础采样健康，不要求主机在线或电机已使能。
 * @param progress 输出已完成监督拍计数；调用方必须比较新进展。
 * @return 采样新鲜且未进入不可清除故障返回true；空指针返回false。
 * @note 前台短临界区只读；通信健康由组合层补充，不授权运动或Flash。
 * @param machine 调用方持有的实例状态。
 */
bool MotorStateMachine_ReadBootHealth(MotorStateMachine *machine, uint32_t *progress);

/**
 * @brief 完成一次 2 kHz 状态监督和故障恢复检查。
 * @note 仅由监督上下文调用。
 * @param machine 调用方持有的实例状态。
 */
void MotorStateMachine_SupervisorTick(MotorStateMachine *machine);

/**
 * @brief 在控制临界区取得本拍外环输入。
 * @param machine 所属轴的状态机。
 * @param work 调用者持有的副本，计算和提交期间有效。
 */
void MotorStateMachine_ReadOuterInputs(MotorStateMachine *machine, MotorOuterWork *work);

/**
 * @brief 处理停止和故障，再以原会话身份提交外环结果。
 * @param machine 所属轴的状态机。
 * @param work 来自 MotorStateMachine_ReadOuterInputs 的本拍结果。
 * @note 每轴一个外环计算者。有效结果在此直接写入控制对象，ADC 只复核启相许可。
 */
void MotorStateMachine_CommitOuterResult(MotorStateMachine *machine, const MotorOuterWork *work);
/**
 * @brief 永久禁止本次上电再启动，并登记停机请求。
 * @return 已锁定返回true；不可恢复故障返回false。
 * @note 前台升级交接使用，复位前不可撤销；不代表输出已经关闭。
 * @param machine 调用方持有的实例状态。
 */
bool MotorStateMachine_LockStartsAndRequestStop(MotorStateMachine *machine);
/**
 * @brief 从该轴绑定的输出端读取三相是否实际关闭。
 * @param machine 已初始化的状态机。
 * @return 所有相通道关闭时为 true。
 */
bool MotorStateMachine_ArePhasesDisabled(const MotorStateMachine *machine);

/**
 * @brief 查询是否已停止且数字输出实际关闭。
 * @return 无待启动/停止且处于空闲或故障、模式失能和关相时返回true。
 * @note 只读；升级交接在启动锁保持期间复核停机，底层在复位前再次确认关相和时限。
 * @param machine 调用方持有的实例状态。
 */
bool MotorStateMachine_IsStopped(const MotorStateMachine *machine);
/**
 * @brief 在前台读取同一时刻的状态机、反馈与目标。
 * @param machine 本轴实例。
 * @param status 成功时写入一致快照，连续两次被抢占时保持原值。
 * @return 本次取得完整快照时返回 true。
 * @note 仅串行前台调用，不屏蔽 ADC、外环或监督中断。
 */
bool MotorStateMachine_ReadStatus(const MotorStateMachine *machine, MotorStatus *status);

#endif
