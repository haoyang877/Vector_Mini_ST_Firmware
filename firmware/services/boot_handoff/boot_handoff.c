/* 请求只写一次；升级锁直到复位，停止失败不得自动恢复运动。 */
#include "boot_handoff.h"
static volatile bool pending;
static volatile uint8_t requested_node;
static volatile uint32_t requested_ms;
static bool queued, failed;
static uint32_t queued_ms;

bool BootHandoff_Request(uint8_t node, uint32_t now_ms)
{
    if (pending || node > 15)
    {
        return false;
    }
    requested_node = node;
    requested_ms = now_ms;
    pending = true;
    return true;
}

bool BootHandoff_Pending(void) { return pending; }

uint8_t BootHandoff_Node(void) { return requested_node; }

void BootHandoff_ReplyQueued(uint32_t now_ms)
{
    queued_ms = now_ms;
    queued = true;
}

BootAction BootHandoff_Poll(uint32_t now_ms, bool stopped, bool phases_off, bool sent)
{
    if (!pending)
    {
        return BOOT_WAIT;
    }
    if (!stopped || !phases_off)
    {
        if ((uint32_t)(now_ms - requested_ms) > 100)
        {
            failed = true;
        }
        return queued ? BOOT_STOP : (failed ? BOOT_REPLY_FAILED : BOOT_STOP);
    }
    if (failed)
    {
        return queued ? BOOT_WAIT : BOOT_REPLY_FAILED;
    }
    if (!queued)
    {
        if ((uint32_t)(now_ms - requested_ms) > 100)
        {
            failed = true;
            return BOOT_REPLY_FAILED;
        }
        return BOOT_REPLY_OK;
    }
    if ((uint32_t)(now_ms - queued_ms) > 100)
    {
        failed = true;
        return BOOT_WAIT;
    }
    if (sent)
    {
        return BOOT_RESET;
    }
    return BOOT_WAIT;
}
