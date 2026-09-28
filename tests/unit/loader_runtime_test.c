/* 运行真实Loader组合根，以内存Flash和复位桩验证启动去向，不访问硬件。 */
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#define main LoaderRuntime_Main
#include "../../loader/runtime/main.c"
#undef main
#include "loader_image_v2_vector.h"

static jmp_buf finish;
static bool model_software, model_watchdog, model_mailbox, model_link;
static uint32_t ticks;
static uint8_t model_image[sizeof(LoaderV2Image)];

static bool read_model(void *context, uint32_t address, void *out, uint32_t bytes)
{
    (void)context;
    if (address < TARGET_APP_BASE || bytes > sizeof(model_image) ||
        address - TARGET_APP_BASE > sizeof(model_image) - bytes)
    {
        return false;
    }
    memcpy(out, model_image + address - TARGET_APP_BASE, bytes);
    return true;
}
static bool erase_model(void *context, uint32_t address, uint32_t bytes)
{
    (void)context;
    (void)address;
    (void)bytes;
    return false;
}
static bool program_model(void *context, uint32_t address, const void *data, uint32_t bytes)
{
    (void)context;
    (void)address;
    (void)data;
    (void)bytes;
    return false;
}
bool LoaderPlatform_Init(bool *software_reset, bool *watchdog_reset)
{
    *software_reset = model_software;
    *watchdog_reset = model_watchdog;
    return true;
}
bool LoaderPlatform_TakeMailbox(BootMailboxMessage *message)
{
    *message = (BootMailboxMessage){YG_BOOT_ENTER, 3U};
    return model_mailbox;
}
LoaderParameterState LoaderPlatform_Parameters(uint8_t *node_out)
{
    *node_out = 3U;
    return YI_PARAMS_RAW11;
}
void LoaderPlatform_ImageIo(LoaderFlashIo *io)
{
    *io = (LoaderFlashIo){NULL,
                          TARGET_APP_BASE,
                          TARGET_APP_BYTES,
                          TARGET_ERASE_BYTES,
                          TARGET_PROGRAM_BYTES,
                          read_model,
                          erase_model,
                          program_model};
}
bool LoaderPlatform_OutputsOff(void)
{
    return true;
}
uint32_t LoaderPlatform_NowMs(void)
{
    return ticks;
}
void LoaderPlatform_Watchdog(void)
{
    /* 完整CRC校验也会喂狗；上限覆盖真实包长度，停留路径有界结束用例。 */
    if (++ticks >= 100U + (sizeof(LoaderV2Image) + 63U) / 64U)
    {
        longjmp(finish, 1);
    }
}
uint32_t LoaderPlatform_Lock(void)
{
    return 0U;
}
void LoaderPlatform_Unlock(uint32_t key)
{
    (void)key;
}
void LoaderPlatform_Reset(void)
{
    longjmp(finish, 3);
}
void LoaderPlatform_Fatal(void)
{
    longjmp(finish, 4);
}
void LoaderPlatform_Jump(void)
{
    longjmp(finish, 2);
}
bool LoaderCan_Init(uint8_t node_in)
{
    return node_in == 3U;
}
bool LoaderCan_Ready(void)
{
    return model_link;
}
bool LoaderCan_Receive(LoaderCanFrame *frame)
{
    (void)frame;
    return false;
}
bool LoaderCan_TxEvent(LoaderCanTxEvent *event)
{
    (void)event;
    return false;
}
bool LoaderCan_Send(const LoaderCanFrame *frame, uint64_t ticket)
{
    (void)frame;
    (void)ticket;
    return true;
}
static int boot(bool software, bool watchdog, bool mailbox, bool valid_image, bool link)
{
    model_software = software;
    model_watchdog = watchdog;
    model_mailbox = mailbox;
    model_link = link;
    ticks = 0U;
    memcpy(model_image, LoaderV2Image, sizeof(model_image));
    if (!valid_image)
    {
        model_image[100] ^= 1U;
    }
    int result = setjmp(finish);
    if (result == 0)
    {
        (void)LoaderRuntime_Main();
        assert(false);
    }
    return result;
}
int main(void)
{
    assert(boot(false, false, false, true, true) == 2);
    assert(boot(true, false, false, true, true) == 2);
    assert(boot(false, false, true, true, true) == 2);
    assert(boot(true, false, true, true, true) == 1);
    assert(boot(false, true, false, true, true) == 1);
    assert(boot(false, false, false, false, true) == 1);
    assert(boot(false, false, false, true, false) == 1);
    YgProtocolUpgradeReply reply;
    status_reply(&reply);
    assert(reply.boot_epoch == 0U && (reply.flags & YG_PROTOCOL_UPGRADE_FLAG_HOST_NONCE) != 0U);
    puts("PASS cold boot, ENTER, watchdog rescue, corrupt image, link loss and v2 discovery");
    return 0;
}
