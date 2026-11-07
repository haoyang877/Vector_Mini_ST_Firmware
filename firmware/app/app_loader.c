#include "app_loader.h"
#include "boot_hw.h"
#include "bsp_system.h"

#define BOOT_ENTRY_TIMEOUT_MS 1000U
#define BOOT_HEALTH_INTERVAL_MS 100U
#define BOOT_REQUIRED_HEALTHY_INTERVALS 11U

#if defined(YG_APP_LOADER_IMAGE) && YG_APP_LOADER_IMAGE
/* 56负责停机交接；57仅在持续健康且关相时持久确认当前试运行镜像。 */
/**
 * @brief 处理镜像确认请求，复核连续健康记录、当前健康状态、通信及数字关相。
 * @param context 初始化时绑定的 Loader 实例。
 * @param version 主机核对的当前镜像版本。
 * @param crc32 主机核对的当前镜像CRC32。
 * @return 全部条件满足且底层确认成功时返回true，否则返回false。
 * @note 前台回调；禁止与进入Loader事务并行。无论成功与否都刷新身份页。
 */
static bool confirm_image(void *context, uint32_t version, uint32_t crc32)
{
    AppLoader *loader = context;
    uint32_t supervisor_ticks;
    bool confirmed = loader->supported && !loader->requested &&
                     loader->healthy_intervals >= BOOT_REQUIRED_HEALTHY_INTERVALS &&
                     MotorLifecycle_ReadBootHealth(loader->lifecycle, &supervisor_ticks) &&
                     *loader->communication_ready && Protocol_IsReady(loader->endpoint) &&
                     MotorLifecycle_ArePhasesDisabled(loader->lifecycle) &&
                     BootHw_Confirm(version, crc32);
    loader->publish_identity(loader->identity_context);
    return confirmed;
}

/**
 * @brief 比较两次Loader请求是否属于同一事务，供重发查询复用已登记的结果。
 * @param left 第一个有效请求，非NULL。
 * @param right 第二个有效请求，非NULL。
 * @return 随机事务标识、来源节点和序号全部一致时返回true。
 * @note 纯比较；boot_epoch是主机生成的事务标识，不是设备持久启动次数。
 */
static bool same_request(const BootEntryRequest *left, const BootEntryRequest *right)
{
    return left->boot_epoch == right->boot_epoch && left->source_node == right->source_node &&
           left->sequence == right->sequence;
}

/**
 * @brief 根据交接截止时间和实际停机状态推进已登记的Loader请求。
 * @param now_ms 当前单调毫秒计数，允许32位自然回绕。
 * @note 前台调用；仅推进PENDING/READY，超时优先判定，拒绝或失败终态不再恢复。
 *       READY仅表示可回复交接票据，不执行复位。
 */
static void poll_entry(AppLoader *loader, uint32_t now_ms)
{
    if (!loader->requested ||
        (loader->result != BOOT_ENTRY_PENDING && loader->result != BOOT_ENTRY_READY))
    {
        return;
    }
    if ((uint32_t)(now_ms - loader->entry_started_ms) >= BOOT_ENTRY_TIMEOUT_MS)
    {
        loader->result = BOOT_ENTRY_FAILED;
    }
    else if (MotorLifecycle_IsStopped(loader->lifecycle))
    {
        loader->result = BOOT_ENTRY_READY;
    }
    else
    {
        loader->result = BOOT_ENTRY_PENDING;
    }
}

/**
 * @brief 登记或查询进入Loader事务，并返回当前结果及可用的交接票据。
 * @param context 初始化时绑定的 Loader 实例。
 * @param request 借用的请求；NULL、零事务标识或无效来源返回UNAVAILABLE。
 * @param reply 调用方持有的结果；NULL时不处理请求。
 * @note 前台回调；首次有效请求复制身份并请求锁定停机，相同请求可查询，其他事务被拒绝。
 *       停机请求受理不代表已关相，票据必须等poll_entry确认READY后才给出。
 */
static void handle_entry(void *context, const BootEntryRequest *request, BootEntryReply *reply)
{
    AppLoader *loader = context;
    uint32_t now_ms;
    if (reply == NULL)
    {
        return;
    }
    *reply = (BootEntryReply){.result = BOOT_ENTRY_UNAVAILABLE};
    if (!loader->supported || request == NULL || request->boot_epoch == 0U ||
        request->source_node == UINT8_MAX)
    {
        return;
    }
    if (loader->requested && !same_request(request, &loader->request))
    {
        reply->result = BOOT_ENTRY_DENIED;
        return;
    }
    now_ms = BspSystem_TimeMs();
    if (!loader->requested)
    {
        loader->request = *request;
        loader->entry_started_ms = now_ms;
        loader->requested = true;
        loader->result =
            MotorLifecycle_LockStop(loader->lifecycle) ? BOOT_ENTRY_PENDING : BOOT_ENTRY_DENIED;
    }
    poll_entry(loader, now_ms);
    reply->result = loader->result;
    reply->ack_ticket = loader->result == BOOT_ENTRY_READY ? loader->request.boot_epoch : 0U;
}

/**
 * @brief 在交接回复物理发送完成后，复核票据、时限和停机状态并尝试复位进Loader。
 * @param context 初始化时绑定的 Loader 实例。
 * @param ticket 通信层返回的64位交接票据，必须匹配当前READY事务。
 * @param success 对应回复帧是否物理发送成功；仅入队不能视为成功。
 * @note 前台回调；最终复核和票据失效化在同一临界区内完成。
 *       发送或复位失败均不恢复票据，防止重复消费旧发送结果。
 */
static void handle_tx_complete(void *context, uint64_t ticket, bool success)
{
    AppLoader *loader = context;
    uint32_t saved_interrupt_mask;
    bool entry_permitted;
    if (!loader->requested || ticket == 0U || ticket != loader->request.boot_epoch ||
        loader->result != BOOT_ENTRY_READY)
    {
        return;
    }
    saved_interrupt_mask = BspSystem_EnterCritical();
    /* 最终二次复核后撤销票据使用权限；复位失败也不能再次消费旧TX结果。 */
    entry_permitted =
        success && loader->result == BOOT_ENTRY_READY &&
        (uint32_t)(BspSystem_TimeMs() - loader->entry_started_ms) < BOOT_ENTRY_TIMEOUT_MS &&
        MotorLifecycle_IsStopped(loader->lifecycle);
    loader->result = BOOT_ENTRY_FAILED;
    if (entry_permitted)
    {
        (void)BootHw_EnterLoader(loader->read_node_id(loader->identity_context));
    }
    BspSystem_ExitCritical(saved_interrupt_mask);
}

/** @brief 记录本次运行时读取的 Loader 可用状态，并刷新身份信息。 */
void AppLoader_Init(AppLoader *loader)
{
    loader->endpoint->loader =
        (BootEntryService){loader, handle_entry, handle_tx_complete, confirm_image};
    loader->supported = BootHw_IsAvailable();
    loader->publish_identity(loader->identity_context);
}
/**
 * @brief 推进Loader停机事务，并按100 ms间隔检查应用健康以维护继承看门狗。
 * @param now_ms 通信服务执行之后读取的当前毫秒计数，允许32位自然回绕。
 * @note 前台调用；健康要求监督拍数有进展、生命周期输入有效且通信就绪。
 *       连续11次健康才满足镜像确认门槛，任一次失败清零。
 */
void AppLoader_Process(AppLoader *loader, uint32_t now_ms)
{
    poll_entry(loader, now_ms);
    if ((uint32_t)(now_ms - loader->health_checked_ms) < BOOT_HEALTH_INTERVAL_MS)
    {
        return;
    }
    uint32_t supervisor_ticks = 0U;
    loader->health_checked_ms = now_ms;
    bool healthy = MotorLifecycle_ReadBootHealth(loader->lifecycle, &supervisor_ticks);
    healthy = healthy && supervisor_ticks != loader->observed_supervisor_ticks &&
              *loader->communication_ready && Protocol_IsReady(loader->endpoint);
    loader->observed_supervisor_ticks = supervisor_ticks;
    if (!healthy)
    {
        loader->healthy_intervals = 0U;
    }
    else if (loader->healthy_intervals < BOOT_REQUIRED_HEALTHY_INTERVALS)
    {
        ++loader->healthy_intervals;
    }
    else
    {
        loader->healthy_intervals = BOOT_REQUIRED_HEALTHY_INTERVALS;
    }
    BootHw_Watchdog(healthy);
}
#endif
