#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "comm_hw.h"
#include "critical_hw.h"
#include "yg_protocol_canfd.h"
#include "yg_protocol_link.h"
#include "yg_protocol_readonly.h"
#include "yg_protocol_readonly_payload.h"

static CommHwCanFrame sent_frame;
static bool sent;
static uint32_t irq_mask;
static unsigned motor_calls;
static bool tx_blocked;
static CommHwCanFrame injected_frame;
static bool inject_on_unlock;
static uint16_t sent_sequences[32];
static size_t sent_count;

uint32_t critical_hw_enter(void)
{
    uint32_t previous = irq_mask;
    irq_mask = 1U;
    return previous;
}

void critical_hw_exit(uint32_t state)
{
    assert(irq_mask == 1U);
    irq_mask = state;
    if (state == 0U && inject_on_unlock)
    {
        inject_on_unlock = false;
        assert(YgProtocolLink_OnRxFrame(&injected_frame));
    }
}

/* 替代平台发送口，验证链路只读请求的编码、路由与响应，不接触真实 CAN 或电机。 */
bool comm_hw_can_try_send_frame(const CommHwCanFrame *frame)
{
    assert(irq_mask == 0U);
    if (frame == NULL || sent || tx_blocked)
    {
        return false;
    }
    sent_frame = *frame;
    sent = true;
    assert(sent_count < sizeof(sent_sequences) / sizeof(sent_sequences[0]));
    sent_sequences[sent_count++] = (uint16_t)frame->data[8] | ((uint16_t)frame->data[9] << 8U);
    return true;
}

static void motor_handler(void *context,
                          const yg_protocol_motor_request_t *request,
                          yg_protocol_service_reply_t *reply)
{
    (void)context;
    assert(irq_mask == 0U);
    assert(request->operation == YG_PROTOCOL_MOTOR_STOP);
    ++motor_calls;
    reply->status = YG_PROTOCOL_SERVICE_OK;
}

static void control_handler(void *context,
                            const yg_protocol_motor_request_t *request,
                            yg_protocol_service_reply_t *reply)
{
    (void)context;
    assert(request->operation == YG_PROTOCOL_MOTOR_SET_CONTROL && request->mode == 3U &&
           request->position_mrad == 1000);
    ++motor_calls;
    reply->status = YG_PROTOCOL_SERVICE_ACCEPTED;
}

/* 在发送持续忙时填满 TX，随后恢复发送：命令不得执行两次或乱序丢失。 */
static void backlog_and_interrupt_handoff(void)
{
    yg_protocol_motor_service_t service = {NULL, motor_handler};
    yg_protocol_message_t request = {.version = YG_PROTOCOL_VERSION,
                                     .flags = YG_PROTOCOL_FLAGS_ACK_REQUEST,
                                     .source_node = 2U,
                                     .destination_node = 7U,
                                     .message_type = YG_PROTOCOL_MOTOR_TYPE_STOP};
    yg_protocol_transfer_frame_t frame;
    assert(YgProtocolLink_Init(7U));
    assert(YgProtocolLink_BindMotorService(&service));
    motor_calls = 0U;
    sent_count = 0U;
    sent = false;
    tx_blocked = true;
    for (unsigned index = 0U; index < 8U; ++index)
    {
        request.sequence = (uint16_t)index;
        assert(yg_protocol_canfd_pack(&request, 3U, &frame) == YG_PROTOCOL_OK);
        injected_frame = (CommHwCanFrame){.identifier = frame.identifier,
                                          .length = frame.length,
                                          .extended = true,
                                          .fd = true,
                                          .bitrate_switch = true};
        memcpy(injected_frame.data, frame.data, frame.length);
        assert(YgProtocolLink_OnRxFrame(&injected_frame));
    }
    /* RX 队列 8 帧已满，最新帧丢弃；后台服务再处理这 8 帧。 */
    assert(!YgProtocolLink_OnRxFrame(&injected_frame));
    assert(motor_calls == 0U);
    YgProtocolLink_Service(10U);
    assert(motor_calls == 8U && sent_count == 0U);
    YgProtocolLink_Service(12U);
    tx_blocked = false;
    for (unsigned i = 0U; i < 20U; ++i)
    {
        sent = false;
        YgProtocolLink_Service(13U + i);
    }
    assert(motor_calls == 8U && sent_count == 8U);
    for (unsigned i = 0U; i < 8U; ++i)
    {
        assert(sent_sequences[i] == i);
    }
    /* 在后台解锁出队副本时模拟 RX 中断，验证新帧不会覆盖已出队的请求。 */
    assert(YgProtocolLink_OnRxFrame(&injected_frame));
    inject_on_unlock = true;
    sent = false;
    YgProtocolLink_Service(100U);
    assert(motor_calls == 10U && sent_count == 9U && !inject_on_unlock && irq_mask == 0U);
}

static void put_u16(uint8_t *buffer, uint16_t value)
{
    buffer[0] = (uint8_t)value;
    buffer[1] = (uint8_t)(value >> 8U);
}

static void put_u32(uint8_t *buffer, uint32_t value)
{
    buffer[0] = (uint8_t)value;
    buffer[1] = (uint8_t)(value >> 8U);
    buffer[2] = (uint8_t)(value >> 16U);
    buffer[3] = (uint8_t)(value >> 24U);
}

static void control_reply_and_feedback(void)
{
    uint8_t payload[YG_PROTOCOL_MOTOR_CONTROL_REQUEST_SIZE] = {3U, 0U, 0U, 0U, 0xE8U, 0x03U};
    yg_protocol_message_t request = {.version = YG_PROTOCOL_VERSION,
                                     .flags = YG_PROTOCOL_FLAGS_ACK_REQUEST,
                                     .source_node = 2U,
                                     .destination_node = 7U,
                                     .message_type = YG_PROTOCOL_MOTOR_TYPE_CONTROL,
                                     .sequence = 41U,
                                     .payload_length = sizeof(payload),
                                     .payload = payload};
    yg_protocol_transfer_frame_t packed, response_frame;
    yg_protocol_message_t response;
    CommHwCanFrame received = {0};
    assert(YgProtocolLink_Init(7U));
    assert(yg_protocol_canfd_pack(&request, 2U, &packed) == YG_PROTOCOL_OK);
    received.identifier = packed.identifier;
    received.length = packed.length;
    received.extended = true;
    received.fd = true;
    received.bitrate_switch = true;
    memcpy(received.data, packed.data, packed.length);
    sent = false;
    assert(YgProtocolLink_OnRxFrame(&received));
    YgProtocolLink_Service(1U);
    assert(sent);
    response_frame = (yg_protocol_transfer_frame_t){0};
    response_frame.identifier = sent_frame.identifier;
    response_frame.length = sent_frame.length;
    response_frame.extended = true;
    response_frame.fd = true;
    response_frame.bitrate_switch = true;
    memcpy(response_frame.data, sent_frame.data, sent_frame.length);
    assert(yg_protocol_canfd_unpack(&response_frame, 2U, &response) == YG_PROTOCOL_OK);
    assert(response.message_type == YG_PROTOCOL_MOTOR_TYPE_CONTROL && response.sequence == 41U);
    assert(response.payload_length == YG_PROTOCOL_MOTOR_REPLY_SIZE && response.payload[0] == 2U);

    sent = false;
    YgProtocolLink_Service(2U);
    assert(sent);
    response_frame.identifier = sent_frame.identifier;
    response_frame.length = sent_frame.length;
    memcpy(response_frame.data, sent_frame.data, sent_frame.length);
    assert(yg_protocol_canfd_unpack(&response_frame, 2U, &response) == YG_PROTOCOL_OK);
    assert(response.message_type == YG_PROTOCOL_READONLY_TYPE_MOTION_FEEDBACK &&
           response.payload_length == YG_PROTOCOL_MOTOR_FEEDBACK_PAYLOAD_SIZE &&
           response.payload[0] == 2U && response.payload[2] == 41U && response.payload[42] == 7U);
    assert(response.payload[24] == 0U && response.payload[25] == 0U);

    /* 同序号重传重发回复与反馈，但不得再次提交目标。 */
    {
        yg_protocol_motor_service_t service = {NULL, control_handler};
        assert(YgProtocolLink_Init(7U));
        assert(YgProtocolLink_BindMotorService(&service));
        motor_calls = 0U;
        sent = false;
        assert(YgProtocolLink_OnRxFrame(&received));
        YgProtocolLink_Service(3U);
        assert(motor_calls == 1U);
        request.flags |= YG_PROTOCOL_FLAGS_RETRY;
        assert(yg_protocol_canfd_pack(&request, 2U, &packed) == YG_PROTOCOL_OK);
        received.length = packed.length;
        memcpy(received.data, packed.data, packed.length);
        assert(YgProtocolLink_OnRxFrame(&received));
        YgProtocolLink_Service(4U);
        assert(motor_calls == 1U);
    }
}

static void enable_route(void)
{
    uint8_t payload[YG_PROTOCOL_MOTOR_ENABLE_REQUEST_SIZE] = {1U};
    yg_protocol_message_t request = {.version = YG_PROTOCOL_VERSION,
                                     .flags = YG_PROTOCOL_FLAGS_ACK_REQUEST,
                                     .source_node = 2U,
                                     .destination_node = 7U,
                                     .message_type = YG_PROTOCOL_MOTOR_TYPE_ENABLE,
                                     .sequence = 40U,
                                     .payload_length = sizeof(payload),
                                     .payload = payload};
    yg_protocol_transfer_frame_t frame;
    CommHwCanFrame received = {0};
    assert(YgProtocolLink_Init(7U));
    assert(yg_protocol_canfd_pack(&request, 2U, &frame) == YG_PROTOCOL_OK);
    received.identifier = frame.identifier;
    received.length = frame.length;
    received.extended = true;
    received.fd = true;
    received.bitrate_switch = true;
    memcpy(received.data, frame.data, frame.length);
    sent = false;
    assert(YgProtocolLink_OnRxFrame(&received));
    YgProtocolLink_Service(1U);
    assert(sent && sent_frame.data[6] == YG_PROTOCOL_MOTOR_TYPE_ENABLE &&
           sent_frame.data[8] == 40U && sent_frame.data[16] == 2U);
}

int main(void)
{
    uint8_t payload[YG_PROTOCOL_READONLY_PAGE_REQUEST_SIZE] = {0};
    yg_protocol_message_t request = {0};
    yg_protocol_transfer_frame_t request_frame = {0};
    yg_protocol_transfer_frame_t response_frame = {0};
    yg_protocol_message_t response = {0};
    CommHwCanFrame received = {0};
    uint32_t session_id = 0x11223344U;
    uint32_t request_id = 0x55667788U;

    assert(YgProtocolLink_Init(7U));
    assert(YgProtocolLink_IsReady());

    put_u32(payload, session_id);
    put_u32(payload + 4U, request_id);
    put_u16(payload + 8U, 0U);
    request.version = YG_PROTOCOL_VERSION;
    request.flags = YG_PROTOCOL_FLAGS_ACK_REQUEST;
    request.source_node = 1U;
    request.destination_node = 7U;
    request.message_type = YG_PROTOCOL_READONLY_TYPE_GET_CAPS;
    request.sequence = 12U;
    request.payload_length = sizeof(payload);
    request.payload = payload;
    assert(yg_protocol_canfd_pack(&request, 2U, &request_frame) == YG_PROTOCOL_OK);

    {
        received.identifier = request_frame.identifier;
        received.length = request_frame.length;
        received.extended = request_frame.extended;
        received.remote = request_frame.remote;
        received.fd = request_frame.fd;
        received.bitrate_switch = request_frame.bitrate_switch;
        memcpy(received.data, request_frame.data, received.length);
        assert(YgProtocolLink_OnRxFrame(&received));
    }

    YgProtocolLink_Service(100U);
    assert(sent);
    assert(sent_frame.extended && sent_frame.fd && sent_frame.bitrate_switch);
    response_frame.identifier = sent_frame.identifier;
    response_frame.length = sent_frame.length;
    response_frame.extended = sent_frame.extended;
    response_frame.remote = sent_frame.remote;
    response_frame.fd = sent_frame.fd;
    response_frame.bitrate_switch = sent_frame.bitrate_switch;
    memcpy(response_frame.data, sent_frame.data, sent_frame.length);
    assert(yg_protocol_canfd_unpack(&response_frame, 1U, &response) == YG_PROTOCOL_OK);
    assert(response.source_node == 7U);
    assert(response.destination_node == 1U);
    assert(response.message_type == YG_PROTOCOL_READONLY_TYPE_GET_CAPS);
    assert(response.sequence == 12U);
    assert((response.flags & YG_PROTOCOL_FLAGS_RESPONSE) != 0U);
    assert(response.payload_length == YG_PROTOCOL_READONLY_CAPS_PAGE0_SIZE);
    assert(response.payload[0] == 1U);
    assert(response.payload[1] == 0U);

    sent = false;
    assert(YgProtocolLink_Init(7U));
    request.flags = YG_PROTOCOL_FLAGS_ACK_REQUEST;
    request.source_node = 2U;
    request.message_type = YG_PROTOCOL_MOTOR_TYPE_STOP;
    request.payload_length = 0U;
    request.payload = NULL;
    assert(yg_protocol_canfd_pack(&request, 2U, &request_frame) == YG_PROTOCOL_OK);
    received.identifier = request_frame.identifier;
    received.length = request_frame.length;
    received.extended = request_frame.extended;
    received.remote = request_frame.remote;
    received.fd = request_frame.fd;
    received.bitrate_switch = request_frame.bitrate_switch;
    memcpy(received.data, request_frame.data, received.length);
    assert(YgProtocolLink_OnRxFrame(&received));
    YgProtocolLink_Service(200U);
    assert(sent);
    response_frame.identifier = sent_frame.identifier;
    response_frame.length = sent_frame.length;
    response_frame.extended = sent_frame.extended;
    response_frame.remote = sent_frame.remote;
    response_frame.fd = sent_frame.fd;
    response_frame.bitrate_switch = sent_frame.bitrate_switch;
    memcpy(response_frame.data, sent_frame.data, sent_frame.length);
    assert(yg_protocol_canfd_unpack(&response_frame, 2U, &response) == YG_PROTOCOL_OK);
    assert(response.message_type == YG_PROTOCOL_MOTOR_TYPE_STOP);
    assert(response.payload_length == YG_PROTOCOL_MOTOR_REPLY_SIZE);
    assert(response.payload[0] == YG_PROTOCOL_SERVICE_UNSUPPORTED);
    assert(response.payload[1] == 0U && response.payload[2] == 0U && response.payload[3] == 0U);

    /* 旧 111 不再注册；即使帧校验正确也不能触发控制或产生成功回复。 */
    sent = false;
    request.message_type = 111U;
    assert(yg_protocol_canfd_pack(&request, 2U, &request_frame) == YG_PROTOCOL_OK);
    received.identifier = request_frame.identifier;
    received.length = request_frame.length;
    memcpy(received.data, request_frame.data, received.length);
    assert(YgProtocolLink_OnRxFrame(&received));
    YgProtocolLink_Service(201U);
    assert(!sent);

    sent = false;
    assert(YgProtocolLink_Init(7U));
    request.flags = YG_PROTOCOL_FLAGS_ACK_REQUEST;
    request.source_node = 1U;
    request.message_type = YG_PROTOCOL_READONLY_TYPE_GET_MOTOR_STATE;
    request.sequence = 13U;
    request.payload_length = 0U;
    request.payload = NULL;
    assert(yg_protocol_canfd_pack(&request, 2U, &request_frame) == YG_PROTOCOL_OK);
    received.identifier = request_frame.identifier;
    received.length = request_frame.length;
    received.extended = request_frame.extended;
    received.remote = request_frame.remote;
    received.fd = request_frame.fd;
    received.bitrate_switch = request_frame.bitrate_switch;
    memcpy(received.data, request_frame.data, received.length);
    assert(YgProtocolLink_OnRxFrame(&received));
    YgProtocolLink_Service(300U);
    assert(sent);
    response_frame.identifier = sent_frame.identifier;
    response_frame.length = sent_frame.length;
    response_frame.extended = sent_frame.extended;
    response_frame.remote = sent_frame.remote;
    response_frame.fd = sent_frame.fd;
    response_frame.bitrate_switch = sent_frame.bitrate_switch;
    memcpy(response_frame.data, sent_frame.data, sent_frame.length);
    assert(yg_protocol_canfd_unpack(&response_frame, 1U, &response) == YG_PROTOCOL_OK);
    assert(response.message_type == YG_PROTOCOL_READONLY_TYPE_GET_MOTOR_STATE);
    assert(response.payload_length == YG_PROTOCOL_MOTOR_FEEDBACK_PAYLOAD_SIZE);
    assert(response.payload[0] == 6U && response.payload[1] == 0U && response.payload[24] == 0U &&
           response.payload[42] == 7U);
    enable_route();
    control_reply_and_feedback();
    backlog_and_interrupt_handoff();
    return 0;
}
