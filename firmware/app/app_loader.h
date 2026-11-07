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
    uint8_t (*read_node_id)(void *context);
    void (*publish_identity)(void *context);
    void *identity_context;
    BootEntryRequest request;
    BootEntryResult result;
    uint32_t entry_started_ms;
    uint32_t health_checked_ms;
    uint32_t observed_supervisor_ticks;
    /* requested表示事务已登记，包括拒绝/失败终态；终态不能因同一请求重试而恢复。 */
    bool requested;
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
