/* 产品组合根：唯一前台所有者，冷启动驻留/升级/跳前校验；不链接APP运行态。 */
#include "upgrade.h"
#include "platform.h"
#include "can_port.h"
#include "vector_mini_st_g431.h"
#include "yg_protocol_frame_codec.h"
#include "yg_protocol_can_id.h"
#include "yg_protocol_fragment.h"
#include "yg_protocol_upgrade_wire.h"
#include <string.h>
static LoaderUpgrade upgrade;
static LoaderImageConfig image_config;
static yg_protocol_fragment_state_t fragments;
static uint8_t assembly[YG_PROTOCOL_UPGRADE_MAX_REQUEST_BYTES];
static uint8_t node = TARGET_RESCUE_NODE;
static bool resident;
static uint32_t started_ms;

static uint32_t now_ms(void *context)
{
    (void)context;
    return LoaderPlatform_NowMs();
}
static void watchdog(void *context)
{
    (void)context;
    LoaderPlatform_Watchdog();
}
static bool safe(void *context)
{
    (void)context;
    return LoaderPlatform_OutputsOff() && LoaderCan_Ready();
}
static uint32_t lock(void *context)
{
    (void)context;
    return LoaderPlatform_Lock();
}
static void unlock(void *context, uint32_t key)
{
    (void)context;
    LoaderPlatform_Unlock(key);
}
static bool boot_now(void *context)
{
    (void)context;
    LoaderPlatform_Reset();
    return false;
}
static uint8_t phase_code(LoaderImagePhase phase)
{
    switch (phase)
    {
    case YI_IDLE:
        return YG_PROTOCOL_UPGRADE_PHASE_IDLE;
    case YI_READY:
        return YG_PROTOCOL_UPGRADE_PHASE_READY;
    case YI_ERASING:
        return YG_PROTOCOL_UPGRADE_PHASE_ERASING;
    case YI_RECEIVING:
        return YG_PROTOCOL_UPGRADE_PHASE_RECEIVING;
    case YI_VERIFYING:
        return YG_PROTOCOL_UPGRADE_PHASE_VERIFYING;
    case YI_VERIFIED:
        return YG_PROTOCOL_UPGRADE_PHASE_VERIFIED;
    default:
        return YG_PROTOCOL_UPGRADE_PHASE_ERROR;
    }
}
static void result_code(LoaderImageResult result, YgProtocolUpgradeReply *reply)
{
    reply->detail = YG_PROTOCOL_UPGRADE_DETAIL_NONE;
    switch (result)
    {
    case YI_OK:
        reply->result = YG_PROTOCOL_UPGRADE_RESULT_OK;
        break;
    case YI_BUSY:
        reply->result = YG_PROTOCOL_UPGRADE_RESULT_BUSY;
        break;
    case YI_ARGUMENT:
        reply->result = YG_PROTOCOL_UPGRADE_RESULT_BAD_FIELD;
        reply->detail = YG_PROTOCOL_UPGRADE_DETAIL_BAD_FIELD;
        break;
    case YI_IDENTITY:
        reply->result = YG_PROTOCOL_UPGRADE_RESULT_DENIED;
        reply->detail = YG_PROTOCOL_UPGRADE_DETAIL_IDENTITY;
        break;
    case YI_RANGE:
        reply->result = YG_PROTOCOL_UPGRADE_RESULT_OUT_OF_RANGE;
        reply->detail = YG_PROTOCOL_UPGRADE_DETAIL_RANGE;
        break;
    case YI_CONFLICT:
        reply->result = YG_PROTOCOL_UPGRADE_RESULT_DENIED;
        reply->detail = YG_PROTOCOL_UPGRADE_DETAIL_DATA_CONFLICT;
        break;
    case YI_DENIED:
        reply->result = YG_PROTOCOL_UPGRADE_RESULT_DENIED;
        reply->detail = YG_PROTOCOL_UPGRADE_DETAIL_OUTPUTS_OR_LINK;
        break;
    case YI_STATE:
        reply->result = YG_PROTOCOL_UPGRADE_RESULT_DENIED;
        reply->detail = YG_PROTOCOL_UPGRADE_DETAIL_WRONG_STATE;
        break;
    case YI_FLASH:
        reply->result = YG_PROTOCOL_UPGRADE_RESULT_FAILED;
        reply->detail = YG_PROTOCOL_UPGRADE_DETAIL_FLASH;
        break;
    case YI_DIGEST:
        reply->result = YG_PROTOCOL_UPGRADE_RESULT_FAILED;
        reply->detail = YG_PROTOCOL_UPGRADE_DETAIL_DIGEST;
        break;
    default:
        reply->result = YG_PROTOCOL_UPGRADE_RESULT_FAILED;
        reply->detail = YG_PROTOCOL_UPGRADE_DETAIL_INTERNAL;
        break;
    }
}
static void status_reply(YgProtocolUpgradeReply *reply)
{
    LoaderUpgradeStatus status;
    memset(reply, 0, sizeof(*reply));
    /* 启动记录已移除；这里回显本次会话由主机提供的不透明标识，不代表持久启动号。 */
    reply->boot_epoch = upgrade.binding.boot_epoch;
    reply->max_block_bytes = LOADER_UPGRADE_MAX_BLOCK_BYTES;
    reply->flags = YG_PROTOCOL_UPGRADE_FLAG_INTEGRITY_ONLY;
    if (!upgrade.initialized || LoaderUpgrade_Status(&upgrade, NULL, &status) != YI_OK)
    {
        reply->phase = YG_PROTOCOL_UPGRADE_PHASE_ERROR;
        result_code(YI_FLASH, reply);
        return;
    }
    reply->session_id = status.binding.session_id;
    reply->next_offset_bytes = status.next_offset;
    reply->image_bytes = status.image_bytes;
    reply->image_token = status.token;
    reply->image_version = status.version;
    reply->image_role = status.role;
    reply->phase = phase_code(status.phase);
    if (status.activation_pending)
    {
        reply->flags |= YG_PROTOCOL_UPGRADE_FLAG_ACTIVATION_PENDING;
    }
    result_code(status.reason, reply);
}
static void send_reply(const yg_protocol_message_t *request,
                       const YgProtocolUpgradeReply *reply,
                       uint64_t ticket)
{
    LoaderCanFrame frame = {0};
    yg_protocol_message_t response = {0};
    yg_protocol_can_id_fields_t fields = {3, 0, 0};
    uint8_t payload[YG_PROTOCOL_UPGRADE_REPLY_BYTES];
    size_t written;
    fields.source_node = node;
    fields.destination_node = request->source_node;
    response.version = YG_PROTOCOL_VERSION;
    response.flags = YG_PROTOCOL_FLAGS_RESPONSE;
    response.source_node = node;
    response.destination_node = request->source_node;
    response.message_type = request->message_type;
    response.sequence = request->sequence;
    response.payload_length = sizeof(payload);
    response.payload = payload;
    frame.bytes = 64; /* 16+40+2=58，尾6B严格补0。 */
    if (YgProtocolUpgrade_EncodeReply(reply, payload, sizeof(payload), NULL) != YG_PROTOCOL_OK ||
        yg_protocol_can_id_encode(&fields, &frame.identifier) != YG_PROTOCOL_OK ||
        yg_protocol_frame_encode(&response, frame.data, sizeof(frame.data), &written) !=
            YG_PROTOCOL_OK ||
        !LoaderCan_Send(&frame, ticket))
    {
        if (ticket != 0)
        {
            (void)LoaderUpgrade_TxComplete(&upgrade, ticket, false);
        }
    }
}
static void dispatch(const yg_protocol_message_t *message)
{
    YgProtocolUpgradeRequest request;
    YgProtocolUpgradeReply reply;
    LoaderUpgradeStatus status;
    LoaderUpgradeBinding binding;
    LoaderImageResult result = YI_DENIED;
    uint64_t ticket = 0;
    yg_protocol_result_t decoded;
    decoded = YgProtocolUpgrade_DecodeRequest(message, &request);
    status_reply(&reply);
    if (decoded != YG_PROTOCOL_OK)
    {
        reply.result = decoded == YG_PROTOCOL_INVALID_LENGTH ? YG_PROTOCOL_UPGRADE_RESULT_BAD_LENGTH
                                                             : YG_PROTOCOL_UPGRADE_RESULT_BAD_FIELD;
        reply.detail = YG_PROTOCOL_UPGRADE_DETAIL_BAD_FIELD;
    }
    else if (!upgrade.initialized)
    {
        result_code(YI_FLASH, &reply);
    }
    else if (request.source_node != TARGET_HOST_NODE)
    {
        reply.result = YG_PROTOCOL_UPGRADE_RESULT_UNAUTHORIZED;
    }
    else if (message->destination_node != node && !request.discover)
    {
        reply.result = YG_PROTOCOL_UPGRADE_RESULT_DENIED;
    }
    else
    {
        binding.boot_epoch = request.boot_epoch;
        binding.session_id = request.session_id;
        binding.source_node = request.source_node;
        switch (request.message_type)
        {
        case YG_PROTOCOL_UPGRADE_TYPE_BEGIN:
            result = LoaderUpgrade_Begin(
                &upgrade, request.source_node, request.boot_epoch, request.data, &status);
            break;
        case YG_PROTOCOL_UPGRADE_TYPE_WRITE:
            result = LoaderUpgrade_Write(
                &upgrade, &binding, request.image_offset_bytes, request.data, request.data_bytes);
            break;
        case YG_PROTOCOL_UPGRADE_TYPE_FINISH:
            result = LoaderUpgrade_Finish(&upgrade, &binding);
            break;
        case YG_PROTOCOL_UPGRADE_TYPE_STATUS:
            result = LoaderUpgrade_Status(&upgrade, request.discover ? NULL : &binding, &status);
            break;
        case YG_PROTOCOL_UPGRADE_TYPE_ACTIVATE:
            result = LoaderUpgrade_Activate(&upgrade, &binding, &ticket);
            break;
        case YG_PROTOCOL_UPGRADE_TYPE_ABORT:
            result = LoaderUpgrade_Abort(&upgrade, &binding);
            break;
        default:
            result = YI_ARGUMENT;
            break;
        }
        status_reply(&reply);
        if (result != YI_OK || request.message_type != YG_PROTOCOL_UPGRADE_TYPE_STATUS)
        {
            result_code(result, &reply);
        }
        if (result == YI_OK && (request.message_type == YG_PROTOCOL_UPGRADE_TYPE_BEGIN ||
                                request.message_type == YG_PROTOCOL_UPGRADE_TYPE_FINISH ||
                                request.message_type == YG_PROTOCOL_UPGRADE_TYPE_ACTIVATE))
        {
            reply.result = YG_PROTOCOL_UPGRADE_RESULT_ACCEPTED;
        }
    }
    send_reply(message, &reply, ticket);
}
static void receive(void)
{
    LoaderCanFrame frame;
    yg_protocol_can_id_fields_t id;
    yg_protocol_message_t message, complete;
    uint32_t payload_bytes, length, i, padded;
    if (!LoaderCan_Receive(&frame) || frame.bytes < 20 ||
        yg_protocol_can_id_decode(frame.identifier, &id) != YG_PROTOCOL_OK || id.source_node == 255)
    {
        return;
    }
    payload_bytes = (uint32_t)frame.data[10] | ((uint32_t)frame.data[11] << 8) |
                    ((uint32_t)frame.data[12] << 16) | ((uint32_t)frame.data[13] << 24);
    if (payload_bytes > YG_PROTOCOL_MAX_PAYLOAD_SIZE)
    {
        return;
    }
    length = YG_PROTOCOL_HEADER_SIZE + payload_bytes + YG_PROTOCOL_TRAILER_SIZE;
    padded = length <= 20 ? 20 : length <= 24 ? 24 : length <= 32 ? 32 : length <= 48 ? 48 : 64;
    if (frame.bytes != padded ||
        yg_protocol_frame_decode(frame.data, length, &message) != YG_PROTOCOL_OK ||
        message.source_node != id.source_node || message.destination_node != id.destination_node ||
        (id.destination_node != node && id.destination_node != 255) ||
        (message.flags & YG_PROTOCOL_FLAGS_RESPONSE) || message.reserved != 0 ||
        (message.flags & ~(YG_PROTOCOL_FLAGS_FRAGMENT_MASK | YG_PROTOCOL_FLAGS_ACK_REQUEST |
                           YG_PROTOCOL_FLAGS_RETRY)) ||
        message.message_type < 50 || message.message_type > 55)
    {
        return;
    }
    for (i = length; i < frame.bytes; ++i)
    {
        if (frame.data[i] != 0)
        {
            return;
        }
    }
    /* 有效主站BEGIN首片抢占启动窗口，不能在重组96B清单期间跳进APP。 */
    if (message.source_node == TARGET_HOST_NODE && message.destination_node == node &&
        message.message_type == YG_PROTOCOL_UPGRADE_TYPE_BEGIN && message.sequence == 0 &&
        (message.flags & YG_PROTOCOL_FLAGS_FRAGMENT_MASK) == YG_PROTOCOL_FLAGS_FRAGMENT_FIRST)
    {
        resident = true;
    }
    if (message.source_node != TARGET_HOST_NODE)
    {
        return;
    }
    if (yg_protocol_fragment_accept(&fragments, &message, LoaderPlatform_NowMs(), 500, &complete) ==
        YG_PROTOCOL_OK)
    {
        dispatch(&complete);
    }
}
static bool initialize_image(void)
{
    LoaderUpgradePorts ports = {0};
    LoaderPlatform_ImageIo(&image_config.io);
    image_config.app.base = TARGET_APP_BASE;
    image_config.app.bytes = TARGET_APP_BYTES;
    image_config.layout = TARGET_LAYOUT;
    image_config.mcu = TARGET_MCU;
    image_config.product = TARGET_PRODUCT;
    image_config.hardware = TARGET_HARDWARE;
    image_config.param_abi = TARGET_PARAM_ABI;
    image_config.schema = TARGET_SCHEMA;
    image_config.ram_start = TARGET_RAM_BASE;
    image_config.ram_end = TARGET_RAM_END;
    image_config.descriptor_offset = TARGET_DESCRIPTOR_OFFSET;
    image_config.allowed = safe;
    image_config.watchdog = watchdog;
    ports.now_ms = now_ms;
    ports.outputs_off_and_link_ready = safe;
    ports.watchdog = watchdog;
    ports.lock = lock;
    ports.unlock = unlock;
    ports.boot = boot_now;
    return LoaderUpgrade_Init(&upgrade, &image_config, &ports);
}
static void try_boot(void)
{
    if (resident || upgrade.active || !upgrade.initialized || !safe(NULL) ||
        (uint32_t)(LoaderPlatform_NowMs() - started_ms) < TARGET_LOADER_WINDOW_MS)
    {
        return;
    }
    resident = true; /* 每次真实启动最多尝试一次；校验失败留在Loader。 */
    if (!LoaderImage_CheckImage(&image_config) || !safe(NULL))
    {
        return;
    }
    LoaderPlatform_Jump();
}
int main(void)
{
    LoaderCanTxEvent event;
    BootMailboxMessage message = {0};
    bool software_reset, mailbox_valid;
    if (!LoaderPlatform_Init(&software_reset))
    {
        LoaderPlatform_Fatal();
    }
    mailbox_valid = LoaderPlatform_TakeMailbox(&message);
    (void)LoaderPlatform_Parameters(&node);
    if (!LoaderCan_Init(node))
    {
        LoaderPlatform_Fatal();
    }
    (void)initialize_image();
    resident = software_reset && mailbox_valid && message.command == YG_BOOT_ENTER;
    (void)yg_protocol_fragment_init(&fragments, assembly, sizeof(assembly));
    started_ms = LoaderPlatform_NowMs();
    for (;;)
    {
        if (!LoaderPlatform_OutputsOff())
        {
            LoaderPlatform_Fatal();
        }
        if (!LoaderCan_Ready())
        {
            LoaderUpgrade_LinkLost(&upgrade);
            resident = true;
        }
        if (LoaderCan_TxEvent(&event) && event.ticket != 0)
        {
            (void)LoaderUpgrade_TxComplete(&upgrade, event.ticket, event.success);
        }
        LoaderUpgrade_Poll(&upgrade);
        receive();
        (void)yg_protocol_fragment_poll(&fragments, LoaderPlatform_NowMs(), 500);
        try_boot();
        LoaderPlatform_Watchdog();
    }
}
