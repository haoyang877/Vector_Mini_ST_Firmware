#ifndef APP_LOADER_H
#define APP_LOADER_H
#include "motor_lifecycle.h"
#include "yg_protocol.h"

/** @brief 一次上电内的 Loader 停机票据与健康确认状态。 */
typedef struct
{
    MotorLifecycle *lifecycle;
    ProtocolEndpoint *endpoint;
    const bool *communication_ready;
    const uint8_t *node_id; /**< 交接时读取所属轴的当前节点号。 */
    void (*publish_identity)(void *context);
    void *identity_context;
    BootEntryRequest request; /**< boot_epoch 非零表示本次上电已经登记过事务，终态仍保留。 */
    BootEntryResult result;
    uint32_t entry_started_ms;
    uint32_t health_checked_ms;
    uint32_t observed_supervisor_ticks;
    bool supported;
    uint8_t healthy_intervals;
} AppLoader;

/**
 * @brief 在协议启动前绑定本实例的交接服务。
 * @param loader 已绑定本机协议和生命周期的实例。
 */
void AppLoader_Init(AppLoader *loader);

/**
 * @brief 推进前台交接超时及健康确认窗口。
 * @param loader 本实例的事务。
 * @param now_ms 本次处理的单调毫秒时钟。
 */
void AppLoader_Process(AppLoader *loader, uint32_t now_ms);
#endif
