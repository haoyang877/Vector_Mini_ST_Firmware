#ifndef MOTOR_STATE_INTERNAL_H
#define MOTOR_STATE_INTERNAL_H

#include <stdbool.h>
#include <stdint.h>

#include "motor_state_types.h"
#include "motor_control_mode.h"
#include "motor_lifecycle_request.h"

/* 仅内部类型：取值既不是 CAN Mode/Error，也不是持久化 ABI 编号。
 * 生产路径由 APP 唯一调用方串行化事件、执行动作并发布
 * 真实硬件/资源 guards。返回 DISABLE_POWER 只是请求，不是门极已关断的证据。 */
typedef enum
{
    MOTOR_MAINTENANCE_CLASS_NONE,
    MOTOR_MAINTENANCE_CLASS_CALIBRATION,
    MOTOR_MAINTENANCE_CLASS_SAVE,
    MOTOR_MAINTENANCE_CLASS_DEFAULTS,
    MOTOR_MAINTENANCE_CLASS_ZERO,
    MOTOR_MAINTENANCE_CLASS_INVALID
} MotorMaintenanceClass;
typedef enum
{
    MOTOR_STATE_ACCEPTED,
    MOTOR_STATE_REJECT_ARGUMENT,
    MOTOR_STATE_REJECT_EVENT,
    MOTOR_STATE_REJECT_STATE,
    MOTOR_STATE_REJECT_BUSY,
    MOTOR_STATE_REJECT_STALE_EPOCH,
    MOTOR_STATE_REJECT_MODE,
    MOTOR_STATE_REJECT_OPERATION,
    MOTOR_STATE_REJECT_GUARD,
    MOTOR_STATE_REJECT_FAULT,
    MOTOR_STATE_REJECT_INTERRUPTED,
    MOTOR_STATE_REJECT_FATAL
} MotorStateRejection;

enum
{
    MOTOR_STATE_EVENT_BOOT_DONE = 1U,
    MOTOR_STATE_EVENT_SELF_TEST_DONE = 2U,
    MOTOR_STATE_EVENT_ENABLE = 4U,
    MOTOR_STATE_EVENT_START_DONE = 8U,
    MOTOR_STATE_EVENT_STOP = 16U,
    MOTOR_STATE_EVENT_FAULT = 32U,
    MOTOR_STATE_EVENT_FATAL = 64U,
    MOTOR_STATE_EVENT_CLEAR = 128U,
    MOTOR_STATE_EVENT_BEGIN_OPERATION = 256U,
    MOTOR_STATE_EVENT_OPERATION_DONE = 512U,
    MOTOR_STATE_EVENT_OPERATION_FAILED = 1024U,
    MOTOR_STATE_EVENT_LINK_RECOVERED = 2048U,
    MOTOR_STATE_EVENT_CANCEL = 4096U,
    MOTOR_STATE_EVENT_CANCEL_DONE = 8192U
};
enum
{
    MOTOR_STATE_ACTION_DISABLE_POWER = 1U,
    MOTOR_STATE_ACTION_REVOKE_REQUESTS = 2U,
    MOTOR_STATE_ACTION_CANCEL_CONTROL = 4U,
    MOTOR_STATE_ACTION_CANCEL_OPERATION = 8U,
    MOTOR_STATE_ACTION_SELF_TEST = 16U,
    MOTOR_STATE_ACTION_START_CONTROL = 32U,
    MOTOR_STATE_ACTION_BEGIN_OPERATION = 64U,
    MOTOR_STATE_ACTION_FAULT_CLEARED = 128U
};
/* 来源适配器可以使用低 29 位存放内部故障源。 */
#define MOTOR_STATE_FAULT_GUARD UINT32_C(0x20000000)
#define MOTOR_STATE_FAULT_OPERATION UINT32_C(0x40000000)
#define MOTOR_STATE_FAULT_INTERNAL UINT32_C(0x80000000)

typedef struct
{
    uint32_t events, faults, completion_epoch, request_epoch;
    MotorControlMode control_mode;
    MotorMaintenanceKind maintenance_kind;
    MotorMaintenanceEffect effects;
} MotorStateEvent;
typedef struct
{
    uint32_t active_faults;
    bool phases_disabled, control_idle, operation_idle, maintenance_released;
    /* inputs_valid 含快速采样进展、测量有限性与温度新鲜度，不含启动电压窗口。
     * configuration_valid 是本次读取的数值校验结果，不是持久有效缓存。
     * self_test_ready 含稳定窗口/既有完成资格以及当前基础健康。 */
    bool inputs_valid, configuration_valid, self_test_ready;
    /* 由适配器根据当前角色、模式/操作依赖、参数有效性与健康计算。
     * 缺少恢复证据时为 false。READY 允许缺少某项标定：start_permitted
     * 只拒绝受影响模式；基础参数错误则自检失败。 */
    bool start_permitted, operation_permitted, startup_confirmed;
    /* 适配器还需自行校验来源与队列时效，不能只信任 request_epoch。 */
    bool request_fresh;
} MotorStateFacts;
/* 唯一所有者上下文中的规则字段；不含模式镜像、分类镜像或返回快照。 */
typedef struct
{
    MotorState state;
    /* 身份保留给既有定向清理；是否仍为当前会话与具体种类是不同事实。 */
    MotorMaintenanceKind maintenance_kind;
    bool maintenance_active;
    MotorMaintenanceResult operation_result;
    MotorMaintenanceEffect operation_effects;
    uint32_t epoch, first_fault, latched_faults;
    bool tested;
} MotorStateRulesContext;

typedef struct
{
    uint32_t actions;
    MotorStateRejection rejection;
} MotorStateDecision;

/**
 * @brief 从具体维护任务推导内部安全类别，不保存第二份任务状态。
 * @param kind 具体任务或 NONE；非法值返回非法类别。
 * @return 校准、保存、默认、零位、无任务或非法类别。
 * @note 模块内部纯函数，无硬件访问；类别不是协议编号。
 */
MotorMaintenanceClass MotorStateRules_Classify(MotorMaintenanceKind kind);

/**
 * @brief 初始化生命周期实例为 BOOT 态，epoch 置 1。
 * @param context 唯一所有者持有的规则字段；为 NULL 时不做任何事。
 * @note 只做赋值，不触发动作、不访问硬件。
 */
void MotorStateRules_Init(MotorStateRulesContext *context);
/**
 * @brief 执行一次生命周期状态迁移，返回动作请求与拒绝原因。
 * @param context 唯一所有者持有的规则字段；调用方串行化访问。
 * @param event 本拍事件集与载荷（events/faults/epoch/control_mode/maintenance_kind/effects）。
 * @param facts 调用方提供的当前健康、许可与资源事实。
 * @return 迁移结果：动作位与拒绝原因；不复制上下文，动作不是硬件证据。
 * @note 优先级 FATAL > FAULT > STOP > CANCEL > 完成通知 > 新请求；完成与新请求同拍
 *       报 INTERRUPTED。LINK_RECOVERED 永不清除锁存；空事件轮询 guards 与停止确认。
 *       当前操作失败按故障处理，不被同拍 STOP 吞掉；陈旧完成不能污染新会话。
 *       已处 FAULT 可单独确认 CANCEL_DONE，继续锁存全部活动故障；不得伪造 guards 清零。
 *       请求需 request_epoch 且 request_fresh；完成需阶段 epoch。停止/取消不回滚 Flash，
 *       也不承诺效果未提交。被取消的操作必须用新取消 epoch 的 CANCEL_DONE 确认，
 *       且匹配操作、资源静默与实际效果；UNKNOWN 表示提交结果仍需外部查询。
 *       存在未确认取消时不得通过 CLEAR 或回到 READY。SAVE 完成必须 COMMITTED，
 *       UNKNOWN/UNCOMMITTED 不得宣称成功。有效上下文缺少事件/guards 会锁入 FATAL
 *       （API 误用）。动作执行方必须在启相短临界区原子复核紧急禁止与 epoch，
 *       本纯核心无法代替该硬件门。READY/SELF_TEST 且无故障时重复 CLEAR 是幂等 no-op。
 */
MotorStateDecision MotorStateRules_Step(MotorStateRulesContext *context,
                                        const MotorStateEvent *event,
                                        const MotorStateFacts *facts);

#endif
