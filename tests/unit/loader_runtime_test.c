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
static bool model_discover, discovery_sent;
static uint32_t ticks, stop_ticks;
/* 0：正常参数；1：空白；2：格式损坏；3：不可读。 */
static unsigned parameter_case, marker_case;
static uint8_t parameters[TARGET_PARAM_BYTES];
static uint8_t parameters_before[TARGET_PARAM_BYTES];
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
    if (bytes != 8U || address < TARGET_APP_BASE ||
        address - TARGET_APP_BASE > sizeof(model_image) - bytes)
    {
        return false;
    }
    memcpy(model_image + address - TARGET_APP_BASE, data, bytes);
    return true;
}
void LoaderPlatform_ReadIdentity(uint8_t uid[12], DeviceIdentity *identity)
{
    memset(uid, 1, 12U);
    memset(identity, 0, sizeof(*identity));
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
bool LoaderPlatform_ReadParameters(uint32_t offset, void *destination, uint32_t bytes)
{
    const uint32_t capacity = TARGET_IDENTITY_BASE - TARGET_PARAM_BASE;
    assert(bytes != 0U && bytes <= capacity && offset <= capacity - bytes);
    if (parameter_case == 3U)
    {
        return false;
    }
    memcpy(destination, parameters + offset, bytes);
    return true;
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
    if (++ticks >= stop_ticks)
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
    stop_ticks = ticks + 100U + (sizeof(LoaderV2Image) + 63U) / 64U;
    return node_in == (parameter_case == 0U ? 3U : TARGET_RESCUE_NODE);
}
bool LoaderCan_Ready(void)
{
    return model_link;
}
bool LoaderCan_Receive(LoaderCanFrame *frame)
{
    if (model_discover && !discovery_sent)
    {
        const yg_protocol_message_t request = {.version = YG_PROTOCOL_VERSION,
                                               .flags = YG_PROTOCOL_FLAGS_ACK_REQUEST,
                                               .source_node = 2U,
                                               .destination_node = 3U,
                                               .message_type = 53U,
                                               .sequence = 77U};
        const yg_protocol_can_id_fields_t id = {3U, 2U, 3U};
        memset(frame, 0, sizeof(*frame));
        frame->bytes = 20U;
        assert(yg_protocol_can_id_encode(&id, &frame->identifier) == YG_PROTOCOL_OK);
        assert(yg_protocol_frame_encode(&request, frame->data, 64U, NULL) == YG_PROTOCOL_OK);
        discovery_sent = true;
        return true;
    }
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
    stop_ticks = UINT32_MAX;
    node = TARGET_RESCUE_NODE; /* 模拟真正复位时的静态初始化。 */
    memset(parameters, 255, sizeof(parameters));
    memset(parameters + TARGET_IDENTITY_BASE - TARGET_PARAM_BASE, 0x5A, TARGET_IDENTITY_BYTES);
    if (parameter_case != 1U)
    {
        const float can_node = 3.0f;
        const uint32_t schema = parameter_case == 2U ? 10U : TARGET_SCHEMA;
        const uint32_t magic = TARGET_RAW_MAGIC;
        memcpy(parameters, &can_node, sizeof(can_node));
        memcpy(parameters + TARGET_RAW_SCHEMA_OFFSET, &schema, sizeof(schema));
        memcpy(parameters + TARGET_RAW_MAGIC_OFFSET, &magic, sizeof(magic));
    }
    memcpy(parameters_before, parameters, sizeof(parameters));
    discovery_sent = false;
    memcpy(model_image, LoaderV2Image, sizeof(model_image));
    for (uint32_t i = 0U; i < 4U; ++i)
    {
        model_image[TARGET_DESCRIPTOR_OFFSET + YGI_ACTIVATION_OFFSET + i] =
            (uint8_t)(YGI_ACTIVATION_MAGIC >> (24U - i * 8U));
        model_image[TARGET_DESCRIPTOR_OFFSET + YGI_ACTIVATION_OFFSET + i + 4U] =
            (uint8_t)(~YGI_ACTIVATION_MAGIC >> (24U - i * 8U));
    }
    if (marker_case != 0U)
    {
        const uint32_t offsets[] = {YGI_ATTEMPT_OFFSET, YGI_CONFIRM_OFFSET};
        const uint32_t magics[] = {YGI_ATTEMPT_MAGIC, YGI_CONFIRM_MAGIC};
        for (unsigned j = 0; j < 2U; ++j)
        {
            if (j == 1U && marker_case != 2U)
            {
                continue;
            }
            for (unsigned i = 0; i < 4U; ++i)
            {
                model_image[TARGET_DESCRIPTOR_OFFSET + offsets[j] + i] =
                    (uint8_t)(magics[j] >> (24U - i * 8U));
                model_image[TARGET_DESCRIPTOR_OFFSET + offsets[j] + i + 4U] =
                    (uint8_t)(~magics[j] >> (24U - i * 8U));
            }
        }
    }
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
    assert(memcmp(parameters, parameters_before, sizeof(parameters)) == 0);
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
    /* 坏 APP 已试运行却未确认：普通冷启动也必须留在 Loader。 */
    marker_case = 1U;
    assert(boot(false, false, false, true, true) == 1);
    marker_case = 2U;
    assert(boot(false, false, false, true, true) == 2);
    marker_case = 0U;
    for (parameter_case = 1U; parameter_case <= 3U; ++parameter_case)
    {
        /* 参数异常保持救援节点，不能阻止恢复固件，也不能改写参数和 SN。 */
        assert(boot(true, false, true, true, true) == 1 && node == TARGET_RESCUE_NODE);
    }
    parameter_case = 0U;
    model_discover = true;
    assert(boot(false, false, false, true, true) == 1 && discovery_sent && resident);
    YgProtocolUpgradeReply reply;
    status_reply(&reply);
    assert(reply.boot_epoch == 0U && (reply.flags & YG_PROTOCOL_UPGRADE_FLAG_HOST_NONCE) != 0U);
    puts("PASS real G431 parameters, rescue node, unchanged parameters/SN, trial/confirmed boot, "
         "ENTER, watchdog, corrupt image, link loss and discovery");
    return 0;
}
