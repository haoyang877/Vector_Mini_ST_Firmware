#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "comm_hw.h"
#include "yg_protocol_canfd.h"
#include "yg_protocol_link.h"
#include "yg_protocol_readonly.h"
#include "yg_protocol_readonly_payload.h"

static CommHwCanFrame sent_frame;
static bool sent;

/* 替代平台发送口，验证链路只读请求的编码、路由与响应，不接触真实 CAN 或电机。 */
bool comm_hw_can_try_send_frame(const CommHwCanFrame *frame)
{
    if (frame == NULL || sent)
    {
        return false;
    }
    sent_frame = *frame;
    sent = true;
    return true;
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
    assert(yg_protocol_canfd_unpack(&response_frame, 1U, &response) == YG_PROTOCOL_OK);
    assert(response.message_type == YG_PROTOCOL_MOTOR_TYPE_STOP);
    assert(response.payload_length == YG_PROTOCOL_MOTOR_REPLY_SIZE);
    assert(response.payload[0] == YG_PROTOCOL_SERVICE_UNSUPPORTED);
    return 0;
}
