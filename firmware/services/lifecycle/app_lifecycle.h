#ifndef YG_APP_LIFECYCLE_H
#define YG_APP_LIFECYCLE_H

#include <stdbool.h>
#include <stdint.h>

/* 仅内部类型：取值既不是 CAN Mode/Error，也不是持久化 ABI 编号。
 * 本核心不接入生产路径；每个上下文由唯一调用方串行化事件、执行动作并发布
 * 真实硬件/资源 guards。返回 DISABLE_POWER 只是请求，不是门极已关断的证据。 */
typedef enum
{
    APP_BOOT,
    APP_SELF_TEST,
    APP_READY,
    APP_STARTING,
    APP_RUNNING,
    APP_MAINTENANCE,
    APP_STOPPING,
    APP_FAULT,
    APP_FATAL
} AppLifecycleState;
typedef enum
{
    APP_CONTROL_NONE,
    APP_CONTROL_CURRENT,
    APP_CONTROL_SPEED,
    APP_CONTROL_POSITION,
    APP_CONTROL_IMPEDANCE,
    APP_CONTROL_SENSORLESS,
    APP_CONTROL_OPEN_VOLTAGE,
    APP_CONTROL_VQ
} AppControlMode;
typedef enum
{
    APP_OPERATION_NONE,
    APP_OPERATION_CALIBRATION,
    APP_OPERATION_SAVE,
    APP_OPERATION_DEFAULTS,
    APP_OPERATION_ZERO
} AppOperation;
typedef enum
{
    APP_OPERATION_NO_RESULT,
    APP_OPERATION_COMPLETED,
    APP_OPERATION_CANCEL_REQUESTED,
    APP_OPERATION_CANCELLED,
    APP_OPERATION_FAILED
} AppOperationResult;
typedef enum
{
    APP_EFFECT_UNKNOWN,
    APP_EFFECT_UNCOMMITTED,
    APP_EFFECT_COMMITTED
} AppOperationEffect;
typedef enum
{
    APP_ACCEPTED,
    APP_REJECT_ARGUMENT,
    APP_REJECT_EVENT,
    APP_REJECT_STATE,
    APP_REJECT_BUSY,
    APP_REJECT_STALE_EPOCH,
    APP_REJECT_MODE,
    APP_REJECT_OPERATION,
    APP_REJECT_GUARD,
    APP_REJECT_FAULT,
    APP_REJECT_INTERRUPTED,
    APP_REJECT_FATAL
} AppLifecycleRejection;

enum
{
    APP_EVENT_BOOT_DONE = 1U,
    APP_EVENT_SELF_TEST_DONE = 2U,
    APP_EVENT_ENABLE = 4U,
    APP_EVENT_START_DONE = 8U,
    APP_EVENT_STOP = 16U,
    APP_EVENT_FAULT = 32U,
    APP_EVENT_FATAL = 64U,
    APP_EVENT_CLEAR = 128U,
    APP_EVENT_BEGIN_OPERATION = 256U,
    APP_EVENT_OPERATION_DONE = 512U,
    APP_EVENT_OPERATION_FAILED = 1024U,
    APP_EVENT_LINK_RECOVERED = 2048U,
    APP_EVENT_CANCEL = 4096U,
    APP_EVENT_CANCEL_DONE = 8192U
};
enum
{
    APP_ACTION_DISABLE_POWER = 1U,
    APP_ACTION_REVOKE_REQUESTS = 2U,
    APP_ACTION_CANCEL_CONTROL = 4U,
    APP_ACTION_CANCEL_OPERATION = 8U,
    APP_ACTION_SELF_TEST = 16U,
    APP_ACTION_START_CONTROL = 32U,
    APP_ACTION_BEGIN_OPERATION = 64U,
    APP_ACTION_FAULT_CLEARED = 128U
};
/* 来源适配器可以使用低 29 位存放内部故障源。 */
#define APP_FAULT_GUARD UINT32_C(0x20000000)
#define APP_FAULT_OPERATION UINT32_C(0x40000000)
#define APP_FAULT_INTERNAL UINT32_C(0x80000000)

typedef struct
{
    uint32_t events, faults, completion_epoch, request_epoch;
    AppControlMode control_mode;
    AppOperation operation;
    AppOperationEffect effects;
} AppLifecycleEvent;
typedef struct
{
    uint32_t active_faults;
    bool phases_disabled, control_idle, operation_idle, maintenance_released;
    bool samples_fresh, fault_sources_clear, self_test_passed;
    /* 由适配器根据当前角色、模式/操作依赖、参数有效性与健康计算。
     * 缺少恢复证据时为 false。READY 允许缺少某项标定：start_permitted
     * 只拒绝受影响模式；基础参数错误则自检失败。 */
    bool start_permitted, operation_permitted, startup_confirmed;
    /* 适配器还需自行校验来源与队列时效，不能只信任 request_epoch。 */
    bool request_fresh;
} AppLifecycleGuards;
typedef struct
{
    AppLifecycleState state;
    AppControlMode control_mode;
    AppOperation operation, last_operation;
    AppOperationResult operation_result;
    AppOperationEffect operation_effects;
    uint32_t epoch, first_fault, latched_faults;
} AppLifecycleSnapshot;
typedef struct
{
    /* 存储公开以便分配；只有 Init/Step 可以修改其内容。 */
    AppLifecycleSnapshot snapshot;
    bool tested;
} AppLifecycle;
typedef struct
{
    AppLifecycleSnapshot snapshot;
    uint32_t actions;
    AppLifecycleRejection rejection;
} AppLifecycleResult;

/**
 * @brief 初始化生命周期实例为 BOOT 态，epoch 置 1。
 * @param app 调用方持有的生命周期实例；为 NULL 时不做任何事。
 * @note 只做赋值，不触发动作、不访问硬件。
 */
void AppLifecycle_Init(AppLifecycle *app);
/**
 * @brief 读取只读状态快照。
 * @param app 生命周期实例。
 * @param snapshot 输出快照；成功时写入当前状态副本。
 * @return 参数有效返回 true；任一参数为空返回 false 且不写输出。
 * @note 快照不是硬件回读；任何上下文只读调用。
 */
bool AppLifecycle_Read(const AppLifecycle *app, AppLifecycleSnapshot *snapshot);
/**
 * @brief 执行一次生命周期状态迁移，返回动作请求与拒绝原因。
 * @param app 生命周期实例；由唯一调用方串行调用。
 * @param event 本拍事件集与载荷（events/faults/epoch/control_mode/operation/effects）。
 * @param guards 调用方提供的当前健康与资源 guards。
 * @return 迁移结果：新快照、动作位与拒绝原因；快照不是硬件证据。
 * @note 优先级 FATAL > FAULT > STOP > CANCEL > 完成通知 > 新请求；完成与新请求同拍
 *       报 INTERRUPTED。LINK_RECOVERED 永不清除锁存；空事件轮询 guards 与停止确认。
 *       当前操作失败按故障处理，不被同拍 STOP 吞掉；陈旧完成不能污染新会话。
 *       请求需 request_epoch 且 request_fresh；完成需阶段 epoch。停止/取消不回滚 Flash，
 *       也不承诺效果未提交。被取消的操作必须用新取消 epoch 的 CANCEL_DONE 确认，
 *       且匹配操作、资源静默与实际效果；UNKNOWN 表示提交结果仍需外部查询。
 *       存在未确认取消时不得通过 CLEAR 或回到 READY。SAVE 完成必须 COMMITTED，
 *       UNKNOWN/UNCOMMITTED 不得宣称成功。有效上下文缺少事件/guards 会锁入 FATAL
 *       （API 误用）。动作执行方必须在启相短临界区原子复核紧急禁止与 epoch，
 *       本纯核心无法代替该硬件门。READY/SELF_TEST 且无故障时重复 CLEAR 是幂等 no-op。
 */
AppLifecycleResult AppLifecycle_Step(AppLifecycle *app,
                                     const AppLifecycleEvent *event,
                                     const AppLifecycleGuards *guards);

#endif
