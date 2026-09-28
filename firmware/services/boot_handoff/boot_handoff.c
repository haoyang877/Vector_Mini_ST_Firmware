#include "boot_handoff.h"

#include <stddef.h>

#include "boot_hw.h"
#include "critical_hw.h"
#include "time_hw.h"

/* 交接超时只撤销ACK权限，不能恢复已经撤销的电机目标。 */
#define BOOT_HANDOFF_TIMEOUT_MS 1000U
#define BOOT_HEALTH_INTERVAL_MS 100U

static BootHandoffOwner owner;
static void *owner_context;
static BootEntryRequest entry_request;
static BootEntryResult entry_result;
static uint64_t entry_ticket;
static uint32_t entry_started_ms, health_checked_ms, observed_supervisor_epoch;
static bool initialized, entry_requested, confirmation_failed;

static bool same_request(const BootEntryRequest *left, const BootEntryRequest *right)
{
    return left->boot_epoch == right->boot_epoch && left->source_node == right->source_node &&
           left->sequence == right->sequence;
}

static void poll_entry(uint32_t now_ms)
{
    if (!entry_requested ||
        (entry_result != BOOT_ENTRY_PENDING && entry_result != BOOT_ENTRY_READY))
    {
        return;
    }
    if ((uint32_t)(now_ms - entry_started_ms) >= BOOT_HANDOFF_TIMEOUT_MS)
    {
        entry_result = BOOT_ENTRY_FAILED;
    }
    else if (owner.stop_ready(owner_context, entry_ticket))
    {
        entry_result = BOOT_ENTRY_READY;
    }
    else
    {
        entry_result = BOOT_ENTRY_PENDING;
    }
}

static void handle_entry(void *context, const BootEntryRequest *request, BootEntryReply *reply)
{
    (void)context;
    if (reply == NULL)
    {
        return;
    }
    *reply = (BootEntryReply){.result = BOOT_ENTRY_UNAVAILABLE};
    BootHwInfo info = {0};
    BootHw_GetInfo(&info);
    if (!initialized || !info.available || info.boot_epoch == 0U)
    {
        return;
    }
    if (request == NULL || request->boot_epoch != info.boot_epoch ||
        request->source_node == UINT8_MAX)
    {
        reply->result = BOOT_ENTRY_DENIED;
        return;
    }
    if (entry_requested && !same_request(request, &entry_request))
    {
        reply->result = BOOT_ENTRY_DENIED;
        return;
    }
    uint32_t now_ms = time_hw_now_ms();
    if (!entry_requested)
    {
        entry_request = *request;
        /* 本boot只允许一个交接事务；可信64位boot代号作为内部票据，不接受线上ACK票据。 */
        entry_ticket = info.boot_epoch;
        entry_started_ms = now_ms;
        entry_requested = true;
        entry_result = owner.request_stop(owner_context, entry_ticket) ? BOOT_ENTRY_PENDING
                                                                       : BOOT_ENTRY_DENIED;
    }
    poll_entry(now_ms);
    reply->result = entry_result;
    reply->ack_ticket = entry_result == BOOT_ENTRY_READY ? entry_ticket : 0U;
}

static void handle_tx_complete(void *context, uint64_t ticket, bool success)
{
    (void)context;
    if (!initialized || !entry_requested || ticket == 0U || ticket != entry_ticket ||
        (entry_result != BOOT_ENTRY_READY && entry_result != BOOT_ENTRY_PENDING))
    {
        return;
    }
    uint32_t mask = critical_hw_enter();
    BootHwInfo info = {0};
    BootHw_GetInfo(&info);
    bool permitted = success && entry_result == BOOT_ENTRY_READY && info.available &&
                     info.boot_epoch == entry_request.boot_epoch &&
                     (uint32_t)(time_hw_now_ms() - entry_started_ms) < BOOT_HANDOFF_TIMEOUT_MS &&
                     owner.stop_ready(owner_context, ticket);
    /* 在最终二次复核后撤销票据使用权限；复位失败也不能再次消费旧TX结果。 */
    entry_result = BOOT_ENTRY_FAILED;
    if (permitted)
    {
        (void)BootHw_EnterLoader(info.boot_epoch, info.node);
    }
    critical_hw_exit(mask);
}

bool BootHandoff_Init(const BootHandoffOwner *binding, void *context, uint32_t now_ms)
{
    if (initialized || binding == NULL || binding->request_stop == NULL ||
        binding->stop_ready == NULL || binding->read_health == NULL ||
        binding->confirmation_complete == NULL)
    {
        return false;
    }
    owner = *binding;
    owner_context = context;
    entry_requested = confirmation_failed = false;
    entry_ticket = 0U;
    observed_supervisor_epoch = 0U;
    health_checked_ms = now_ms;
    initialized = true;
    return true;
}

bool BootHandoff_GetService(BootEntryService *service)
{
    if (service == NULL)
    {
        return false;
    }
    *service = (BootEntryService){0};
    if (!initialized)
    {
        return false;
    }
    *service = (BootEntryService){NULL, handle_entry, handle_tx_complete};
    return true;
}

void BootHandoff_Service(uint32_t now_ms)
{
    if (!initialized)
    {
        return;
    }
    poll_entry(now_ms);
    if ((uint32_t)(now_ms - health_checked_ms) < BOOT_HEALTH_INTERVAL_MS)
    {
        return;
    }
    health_checked_ms = now_ms;
    BootHandoffHealth health = {0};
    owner.read_health(owner_context, &health);
    bool healthy = health.healthy && health.supervisor_epoch != observed_supervisor_epoch;
    observed_supervisor_epoch = health.supervisor_epoch;
    BootHwInfo info = {0};
    BootHw_GetInfo(&info);
    if (info.available && info.trial && !info.confirmed && !confirmation_failed &&
        !entry_requested && healthy && health.confirmation_ready)
    {
        uint32_t mask = critical_hw_enter();
        BootHandoffHealth current = {0};
        owner.read_health(owner_context, &current);
        bool permitted = current.healthy && current.confirmation_ready &&
                         current.supervisor_epoch == health.supervisor_epoch;
        if (permitted &&
            (!BootHw_Confirm(info.boot_epoch, info.image_token) ||
             !owner.confirmation_complete(owner_context, info.boot_epoch, info.image_token)))
        {
            confirmation_failed = true;
            /* 确认写入失败保持停机，等继承的看门狗返回Loader救援。 */
            (void)owner.request_stop(owner_context, info.boot_epoch);
        }
        critical_hw_exit(mask);
    }
    BootHw_Watchdog(healthy && !confirmation_failed);
}
