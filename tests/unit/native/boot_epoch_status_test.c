#include <assert.h>
#include <stdio.h>
#include "boot_epoch.h"
#include "boot_hw.h"
#include "yg_protocol_readonly_payload.h"

static uint64_t epoch;
static bool confirmed;
static yg_protocol_readonly_device_status_t s_boot_source;
uint64_t BootEpoch_Value(void)
{
    return epoch;
}
bool BootHw_IsConfirmed(void)
{
    return confirmed;
}

/* BOOT_EPOCH_STATUS_PRODUCTION：脚本插入真实状态源刷新体。 */

int main(void)
{
    uint8_t payload[16];
    refresh_boot_source();
    assert(!s_boot_source.epoch_valid && s_boot_source.boot_epoch == 0U);
    assert(yg_protocol_readonly_encode_device_status(
               &s_boot_source, payload, sizeof(payload), NULL) == YG_PROTOCOL_OK);
    assert(payload[0] == 6U && payload[5] == 0U && payload[6] == 0U);
    epoch = UINT64_C(0x100000001);
    refresh_boot_source();
    assert(s_boot_source.epoch_valid && s_boot_source.boot_epoch == epoch);
    assert(yg_protocol_readonly_encode_device_status(
               &s_boot_source, payload, sizeof(payload), NULL) == YG_PROTOCOL_OK);
    assert(payload[0] == 0U && payload[6] == 1U && payload[8] == 1U && payload[12] == 1U);
#if defined(YG_APP_LOADER_IMAGE) && YG_APP_LOADER_IMAGE
    assert(payload[5] == YG_PROTOCOL_BOOT_STATE_TRIAL_APP);
    confirmed = true;
    refresh_boot_source();
    assert(s_boot_source.boot_state == YG_PROTOCOL_BOOT_STATE_CONFIRMED_APP);
#else
    assert(payload[5] == YG_PROTOCOL_BOOT_STATE_CONFIRMED_APP);
#endif
    epoch = 0U;
    refresh_boot_source();
    assert(!s_boot_source.epoch_valid && s_boot_source.boot_epoch == 0U &&
           s_boot_source.boot_state == 0U);
    puts("PASS production Type3 source: full u64, invalid zero, phase distinct from identity");
    return 0;
}
