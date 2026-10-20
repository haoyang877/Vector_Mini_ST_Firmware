/* 主机CLI对真实Loader组合根的离线设备模型；APP健康/确认与硬件均为显式桩。 */
#include <stdint.h>
#include <string.h>
#define main LoaderRuntime_Main
#include "../../loader/runtime/main.c"
#undef main

static uint8_t s_flash[TARGET_APP_BYTES], s_programmed[TARGET_APP_BYTES / 8U];
static uint32_t s_ticks, s_reply_id, s_reply_bytes;
static uint8_t s_reply[64];
static LoaderCanFrame s_rx;
static bool s_rx_ready, s_app;
static uint32_t s_trial_ms;

static bool model_read(void *context, uint32_t address, void *out, uint32_t bytes)
{
    (void)context;
    if (address < TARGET_APP_BASE || bytes > TARGET_APP_BYTES ||
        address - TARGET_APP_BASE > TARGET_APP_BYTES - bytes)
    {
        return false;
    }
    memcpy(out, s_flash + (address - TARGET_APP_BASE), bytes);
    return true;
}
static bool model_erase(void *context, uint32_t address, uint32_t bytes)
{
    (void)context;
    if (bytes != TARGET_ERASE_BYTES || address % bytes != 0U || address < TARGET_APP_BASE ||
        address - TARGET_APP_BASE > TARGET_APP_BYTES - bytes)
    {
        return false;
    }
    memset(s_flash + (address - TARGET_APP_BASE), 255, bytes);
    memset(s_programmed + (address - TARGET_APP_BASE) / 8U, 0, bytes / 8U);
    return true;
}
static bool model_program(void *context, uint32_t address, const void *data, uint32_t bytes)
{
    (void)context;
    if (bytes != 8U || address % 8U != 0U || address < TARGET_APP_BASE ||
        address - TARGET_APP_BASE > TARGET_APP_BYTES - bytes ||
        s_programmed[(address - TARGET_APP_BASE) / 8U] != 0U)
    {
        return false;
    }
    s_programmed[(address - TARGET_APP_BASE) / 8U] = 1U;
    memcpy(s_flash + (address - TARGET_APP_BASE), data, bytes);
    return true;
}
void LoaderPlatform_ImageIo(LoaderFlashIo *io)
{
    *io = (LoaderFlashIo){NULL,
                          TARGET_APP_BASE,
                          TARGET_APP_BYTES,
                          TARGET_ERASE_BYTES,
                          8U,
                          model_read,
                          model_erase,
                          model_program};
}
bool LoaderPlatform_Init(bool *software, bool *watchdog_reset)
{
    *software = *watchdog_reset = false;
    return true;
}
bool LoaderPlatform_OutputsOff(void)
{
    return true;
}
uint32_t LoaderPlatform_NowMs(void)
{
    return s_ticks;
}
void LoaderPlatform_Watchdog(void)
{
}
uint32_t LoaderPlatform_Lock(void)
{
    return 0U;
}
void LoaderPlatform_Unlock(uint32_t key)
{
    (void)key;
}
void LoaderPlatform_Fatal(void)
{
}
bool LoaderPlatform_TakeMailbox(BootMailboxMessage *message)
{
    (void)message;
    return false;
}
LoaderParameterState LoaderParameters_ReadNode(uint8_t *value)
{
    *value = 3U;
    return YI_PARAMS_RAW11;
}
void LoaderPlatform_ReadIdentity(uint8_t uid[12], DeviceIdentity *identity)
{
    memset(uid, 1, 12U);
    memset(identity, 0, sizeof(*identity));
}
void LoaderPlatform_Jump(void)
{
    s_app = true;
}
void LoaderPlatform_Reset(void)
{
    s_app = LoaderImage_PrepareBoot(&image_config);
    s_trial_ms = s_ticks;
}
bool LoaderCan_Init(uint8_t value)
{
    return value == 3U;
}
bool LoaderCan_Ready(void)
{
    return true;
}
bool LoaderCan_TxEvent(LoaderCanTxEvent *event)
{
    (void)event;
    return false;
}
bool LoaderCan_Receive(LoaderCanFrame *frame)
{
    if (!s_rx_ready)
    {
        return false;
    }
    *frame = s_rx;
    s_rx_ready = false;
    return true;
}
bool LoaderCan_Send(const LoaderCanFrame *frame, uint64_t ticket)
{
    s_reply_id = frame->identifier;
    s_reply_bytes = frame->bytes;
    memcpy(s_reply, frame->data, frame->bytes);
    if (ticket != 0U)
    {
        (void)LoaderUpgrade_TxComplete(&upgrade, ticket, true);
    }
    return true;
}

/** @brief 重置离线模型为无APP的Loader救援状态，不操作硬件。 */
__declspec(dllexport) void Device_Reset(void)
{
    memset(s_flash, 255, sizeof(s_flash));
    memset(s_programmed, 0, sizeof(s_programmed));
    s_ticks = 0U;
    s_app = false;
    node = 3U;
    (void)initialize_image();
    (void)yg_protocol_fragment_init(&fragments, assembly, sizeof(assembly));
}
/** @brief 将模型切为旧APP查询/交接桩，Loader下载仍运行生产源码。 */
__declspec(dllexport) void Device_StartApp(void)
{
    s_app = true;
}
/**
 * @brief 把一帧送入真实Loader接收/重组/升级路径，返回对应实际编码回复。
 * @param identifier 输入CAN ID。
 * @param data 借用的输入帧。
 * @param bytes 输入帧长。
 * @param reply_id 输出CAN ID。
 * @param reply 输出帧，容量64B。
 * @return 回复帧字节数，0表示没有完整回复。
 * @note APP查询/确认是桩；Loader擦写、校验、票据和持久启动门槛运行生产源码。
 */
__declspec(dllexport) uint32_t Device_Exchange(
    uint32_t identifier, const uint8_t *data, uint32_t bytes, uint32_t *reply_id, uint8_t *reply)
{
    yg_protocol_message_t message;
    s_ticks += 10U;
    s_reply_bytes = 0U;
    if (bytes > 64U || bytes < 20U)
    {
        return 0U;
    }
    if (!s_app)
    {
        s_rx = (LoaderCanFrame){0};
        s_rx.identifier = identifier;
        s_rx.bytes = bytes;
        memcpy(s_rx.data, data, bytes);
        s_rx_ready = true;
        LoaderUpgrade_Poll(&upgrade);
        receive();
    }
    else
    {
        uint32_t payload_bytes = YgProtocol_ReadU32Le(data + 10U);
        if (payload_bytes > 46U || payload_bytes + 18U > bytes ||
            yg_protocol_frame_decode(data, payload_bytes + 18U, &message) != YG_PROTOCOL_OK)
        {
            return 0U;
        }
        if (message.message_type == 1U)
        {
            query_info(&message);
            if (YgProtocol_ReadU16Le(message.payload + 8U) == 1U)
            {
                uint8_t payload[32];
                memcpy(payload, s_reply + 16U, 32U);
                payload[22] = 1U;
                payload[23] = LoaderImage_ReadMarker(
                                  &image_config, YGI_CONFIRM_OFFSET, YGI_CONFIRM_MAGIC) == 1
                                  ? 1U
                                  : 2U;
                send_payload(&message, payload, 32U, 0U);
            }
        }
        else if (message.message_type == 56U && payload_bytes == 8U)
        {
            const uint8_t payload[4] = {1U, 0U, 0U, 0U};
            send_payload(&message, payload, 4U, 0U);
            s_app = false;
        }
        else if (message.message_type == 57U && payload_bytes == 8U)
        {
            uint8_t payload[4] = {5U, 0U, 0U, 0U}, raw[8];
            uint8_t *desc = s_flash + TARGET_DESCRIPTOR_OFFSET;
            uint32_t crc = (uint32_t)desc[48] << 24 | (uint32_t)desc[49] << 16 |
                           (uint32_t)desc[50] << 8 | desc[51];
            if (s_ticks - s_trial_ms >= 100U &&
                YgProtocol_ReadU32Le(message.payload) == TARGET_APP_VERSION &&
                YgProtocol_ReadU32Le(message.payload + 4U) == crc)
            {
                for (uint32_t i = 0; i < 4; ++i)
                {
                    raw[i] = (uint8_t)(YGI_CONFIRM_MAGIC >> (24U - i * 8U));
                    raw[i + 4U] = (uint8_t)(~YGI_CONFIRM_MAGIC >> (24U - i * 8U));
                }
                if (LoaderImage_ReadMarker(&image_config, YGI_CONFIRM_OFFSET, YGI_CONFIRM_MAGIC) ==
                        1 ||
                    model_program(NULL,
                                  TARGET_APP_BASE + TARGET_DESCRIPTOR_OFFSET + YGI_CONFIRM_OFFSET,
                                  raw,
                                  8U))
                {
                    payload[0] = 0U;
                }
            }
            send_payload(&message, payload, 4U, 0U);
        }
    }
    *reply_id = s_reply_id;
    memcpy(reply, s_reply, s_reply_bytes);
    return s_reply_bytes;
}
