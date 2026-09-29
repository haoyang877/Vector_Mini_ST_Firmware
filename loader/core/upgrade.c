/* 升级租约的唯一所有者；外层公司帧、CAN 硬件和镜像存储保持独立。 */
#include "upgrade.h"
#include <string.h>

static bool gate(void *context)
{
    LoaderUpgrade *u = context;
    uint32_t now_ms = u->ports.now_ms(u->ports.context);
    return u->active && (uint32_t)(now_ms - u->last_activity_ms) < LOADER_UPGRADE_LEASE_MS &&
           u->ports.outputs_off_and_link_ready(u->ports.context);
}

static void watchdog(void *context)
{
    LoaderUpgrade *u = context;
    u->ports.watchdog(u->ports.context);
}

static bool same(const LoaderUpgrade *u, const LoaderUpgradeBinding *binding)
{
    return binding != NULL && binding->boot_epoch == u->binding.boot_epoch &&
           binding->session_id == u->binding.session_id && binding->session_id != 0 &&
           binding->source_node == u->binding.source_node;
}

static bool admitted(LoaderUpgrade *u, const LoaderUpgradeBinding *binding)
{
    return u != NULL && u->initialized && !u->in_call && same(u, binding) && gate(u);
}

static void capture(const LoaderUpgrade *u, LoaderUpgradeStatus *out)
{
    out->binding = u->binding;
    out->phase = u->image.phase;
    out->reason = u->image.reason;
    out->image_bytes = u->image.image.bytes;
    out->token = u->image.image.token;
    out->version = u->image.image.version;
    out->role = u->image.image.role;
    out->next_offset = u->written_bytes;
    out->activation_pending = u->pending_ticket != 0;
}

bool LoaderUpgrade_Init(LoaderUpgrade *u,
                        const LoaderImageConfig *image,
                        const LoaderUpgradePorts *ports)
{
    if (u == NULL || image == NULL || ports == NULL || ports->now_ms == NULL ||
        ports->outputs_off_and_link_ready == NULL || ports->watchdog == NULL ||
        ports->lock == NULL || ports->unlock == NULL || ports->boot == NULL)
    {
        return false;
    }
    memset(u, 0, sizeof(*u));
    u->config = *image;
    u->ports = *ports;
    u->config.owner = u;
    u->config.allowed = gate;
    u->config.watchdog = watchdog;
    u->initialized = LoaderImage_Init(&u->image, &u->config);
    return u->initialized;
}

LoaderImageResult LoaderUpgrade_Begin(LoaderUpgrade *u,
                                      uint8_t source_node,
                                      uint64_t expected_boot,
                                      const uint8_t manifest[96],
                                      LoaderUpgradeStatus *status)
{
    LoaderImageResult result;
    if (u == NULL || !u->initialized || manifest == NULL || status == NULL || u->in_call ||
        source_node == 255 || expected_boot == 0)
    {
        return YI_ARGUMENT;
    }
    LoaderUpgrade_Poll(u);
    if (u->active)
    {
        if (u->binding.source_node != source_node || u->binding.boot_epoch != expected_boot ||
            memcmp(u->image.manifest, manifest, 96) != 0)
        {
            return YI_BUSY;
        }
        capture(u, status);
        return u->image.phase == YI_ERROR ? u->image.reason : YI_OK;
    }
    if (u->sessions == UINT32_MAX || !u->ports.outputs_off_and_link_ready(u->ports.context))
    {
        return YI_DENIED;
    }
    result = LoaderImage_Begin(&u->image, manifest);
    if (result != YI_OK)
    {
        return result;
    }
    u->binding.boot_epoch = expected_boot;
    u->binding.source_node = source_node;
    u->binding.session_id = ++u->sessions;
    u->active = true;
    u->written_bytes = 0;
    u->last_activity_ms = u->ports.now_ms(u->ports.context);
    result = LoaderImage_Erase(&u->image);
    capture(u, status);
    return result;
}

LoaderImageResult LoaderUpgrade_Write(LoaderUpgrade *u,
                                      const LoaderUpgradeBinding *binding,
                                      uint32_t offset,
                                      const uint8_t *data,
                                      uint32_t bytes)
{
    LoaderImageResult result;
    bool was_receiving;
    if (!admitted(u, binding) || u->pending_ticket != 0)
    {
        return YI_DENIED;
    }
    was_receiving = u->image.phase == YI_RECEIVING;
    u->in_call = true;
    result = LoaderImage_Write(&u->image, offset, data, bytes);
    if (was_receiving)
    {
        u->written_bytes = u->image.next;
    }
    /* 写入可能跨过租约截止，不能通过操作结束的续租复活权限。 */
    if (result == YI_OK && !gate(u))
    {
        result = YI_DENIED;
        u->image.phase = YI_ERROR;
        u->image.reason = result;
        u->active = false;
    }
    if (result == YI_OK)
    {
        u->last_activity_ms = u->ports.now_ms(u->ports.context);
    }
    u->in_call = false;
    return result;
}

LoaderImageResult LoaderUpgrade_Finish(LoaderUpgrade *u, const LoaderUpgradeBinding *binding)
{
    LoaderImageResult result;
    if (!admitted(u, binding) || u->pending_ticket != 0)
    {
        return YI_DENIED;
    }
    result = LoaderImage_Verify(&u->image);
    if (result == YI_OK)
    {
        u->last_activity_ms = u->ports.now_ms(u->ports.context);
    }
    return result;
}

LoaderImageResult LoaderUpgrade_Status(LoaderUpgrade *u,
                                       const LoaderUpgradeBinding *binding,
                                       LoaderUpgradeStatus *status)
{
    if (u == NULL || !u->initialized || status == NULL || u->in_call)
    {
        return YI_ARGUMENT;
    }
    if (binding != NULL)
    {
        if (!admitted(u, binding))
        {
            return YI_DENIED;
        }
        if (u->pending_ticket == 0)
        {
            u->last_activity_ms = u->ports.now_ms(u->ports.context);
        }
    }
    capture(u, status);
    return YI_OK;
}

LoaderImageResult
LoaderUpgrade_Activate(LoaderUpgrade *u, const LoaderUpgradeBinding *binding, uint64_t *ticket)
{
    if (ticket == NULL || !admitted(u, binding) || u->image.phase != YI_VERIFIED)
    {
        return YI_DENIED;
    }
    if (u->pending_ticket == 0)
    {
        if (u->tickets == UINT64_MAX)
        {
            return YI_DENIED;
        }
        u->pending_ticket = ++u->tickets;
        u->ack_started_ms = u->ports.now_ms(u->ports.context);
    }
    if ((uint32_t)(u->ports.now_ms(u->ports.context) - u->ack_started_ms) >= LOADER_UPGRADE_ACK_MS)
    {
        LoaderUpgrade_LinkLost(u);
        return YI_DENIED;
    }
    *ticket = u->pending_ticket;
    return YI_OK;
}

bool LoaderUpgrade_TxComplete(LoaderUpgrade *u, uint64_t ticket, bool success)
{
    uint32_t key, now_ms;
    bool result = false;
    if (u == NULL || !u->initialized || u->in_call || ticket == 0 || ticket != u->pending_ticket)
    {
        return false;
    }
    u->pending_ticket = 0;
    now_ms = u->ports.now_ms(u->ports.context);
    if (!success || !gate(u) || (uint32_t)(now_ms - u->ack_started_ms) >= LOADER_UPGRADE_ACK_MS)
    {
        LoaderUpgrade_LinkLost(u);
        return false;
    }
    u->in_call = true;
    if (LoaderImage_Check(&u->config, &u->image.image))
    {
        key = u->ports.lock(u->ports.context);
        now_ms = u->ports.now_ms(u->ports.context);
        if (gate(u) && (uint32_t)(now_ms - u->ack_started_ms) < LOADER_UPGRADE_ACK_MS)
        {
            if (LoaderImage_Activate(&u->config, &u->image.image) && gate(u))
            {
                result = u->ports.boot(u->ports.context);
            }
            else
            {
                u->image.phase = YI_ERROR;
                u->image.reason = YI_FLASH;
            }
        }
        u->ports.unlock(u->ports.context, key);
    }
    else
    {
        u->image.phase = YI_ERROR;
        u->image.reason = YI_DIGEST;
    }
    u->active = false;
    u->in_call = false;
    return result;
}

LoaderImageResult LoaderUpgrade_Abort(LoaderUpgrade *u, const LoaderUpgradeBinding *binding)
{
    if (u == NULL || !u->initialized || u->in_call || !same(u, binding))
    {
        return YI_DENIED;
    }
    u->active = false;
    u->pending_ticket = 0;
    LoaderImage_Abort(&u->image);
    u->written_bytes = 0;
    return YI_OK;
}

void LoaderUpgrade_Poll(LoaderUpgrade *u)
{
    uint32_t now_ms;
    if (u == NULL || !u->initialized || u->in_call || !u->active)
    {
        return;
    }
    now_ms = u->ports.now_ms(u->ports.context);
    if (!gate(u) ||
        (u->pending_ticket != 0 && (uint32_t)(now_ms - u->ack_started_ms) >= LOADER_UPGRADE_ACK_MS))
    {
        LoaderUpgrade_LinkLost(u);
        return;
    }
    if (u->pending_ticket == 0)
    {
        (void)LoaderImage_Poll(&u->image);
    }
}

void LoaderUpgrade_LinkLost(LoaderUpgrade *u)
{
    if (u == NULL || !u->initialized || u->in_call)
    {
        return;
    }
    u->active = false;
    u->pending_ticket = 0;
    LoaderImage_Abort(&u->image);
    u->written_bytes = 0;
    u->image.reason = YI_DENIED;
}
