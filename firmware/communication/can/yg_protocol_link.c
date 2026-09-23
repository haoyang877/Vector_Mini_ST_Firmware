/* CAN FD 组合层：后台路由，电机业务经显式绑定注入；不直接访问电机控制。 */
#include "yg_protocol_link.h"

#include <stddef.h>
#include <string.h>

#include "critical_hw.h"

#include "yg_protocol_endpoint.h"
#include "yg_protocol_crc.h"
#include "yg_protocol_readonly.h"
#include "yg_protocol_readonly_payload.h"
#include "yg_protocol_transfer.h"
#include "yg_protocol_motor.h"

/* 队列一次覆盖最大 1767B 逻辑消息的 39 个 46B 分片，并保留一个空槽。
 * 重组区与协议上限一致，避免声明支持长消息却在组合层静默截短。 */
#define YG_LINK_RX_QUEUE_CAPACITY 8U
#define YG_LINK_TX_QUEUE_CAPACITY 40U
#define YG_LINK_FRAGMENT_CAPACITY YG_PROTOCOL_MAX_MESSAGE_PAYLOAD
#define YG_LINK_PROTOCOL_INFO_TYPE 1U
#define YG_LINK_CAPABILITIES_TYPE 2U
#define YG_LINK_MASTER_NODE_ID 2U
#define YG_LINK_CONTROL_HISTORY_SIZE 256U

typedef struct
{
    uint16_t sequence;
    uint16_t request_crc;
    uint16_t result;
    uint16_t detail;
    bool valid;
} yg_link_control_history_t;

typedef struct
{
    uint8_t node_id;
    yg_protocol_readonly_info_page0_t info;
    yg_protocol_readonly_caps_page0_t capabilities;
    yg_protocol_motor_service_t motor_service;
    yg_protocol_motor_status_source_t empty_motor_status;
    const yg_protocol_motor_status_source_t *motor_status_source;
    yg_link_control_history_t control_history[YG_LINK_CONTROL_HISTORY_SIZE];
    uint16_t last_control_sequence;
    uint16_t feedback_sequence;
    bool control_sequence_seen;
    bool feedback_pending;
    uint8_t feedback_destination;
    uint8_t feedback_payload[YG_PROTOCOL_MOTOR_FEEDBACK_PAYLOAD_SIZE];
} yg_protocol_link_context_t;

static yg_protocol_link_context_t link_context;
static yg_protocol_transfer_frame_t rx_storage[YG_LINK_RX_QUEUE_CAPACITY];
static yg_protocol_transfer_frame_t tx_storage[YG_LINK_TX_QUEUE_CAPACITY];
static uint8_t fragment_storage[YG_LINK_FRAGMENT_CAPACITY];
static yg_protocol_transfer_queue_t rx_queue;
static yg_protocol_transfer_queue_t tx_queue;
static yg_protocol_message_registry_t registry;
static yg_protocol_router_t router;
static yg_protocol_readonly_service_t readonly_service;
static yg_protocol_endpoint_t endpoint;
static bool ready;

static bool motor_request_valid(const yg_protocol_message_t *request);
static void
write_motor_result(yg_protocol_service_result_t *response, uint16_t code, uint16_t detail);

static yg_protocol_result_t provide_info(void *context,
                                         const yg_protocol_message_t *request,
                                         uint8_t *payload,
                                         uint32_t *payload_length)
{
    yg_protocol_readonly_page_request_t page_request;
    yg_protocol_link_context_t *link = context;
    size_t written = 0U;

    if (link == NULL ||
        yg_protocol_readonly_decode_page_request(request, &page_request) != YG_PROTOCOL_OK ||
        page_request.page != 0U)
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    if (yg_protocol_readonly_encode_info_page0(
            &link->info, payload, YG_PROTOCOL_MAX_PAYLOAD_SIZE, &written) != YG_PROTOCOL_OK)
    {
        return YG_PROTOCOL_BUFFER_TOO_SMALL;
    }
    *payload_length = (uint32_t)written;
    return YG_PROTOCOL_OK;
}

static yg_protocol_result_t provide_capabilities(void *context,
                                                 const yg_protocol_message_t *request,
                                                 uint8_t *payload,
                                                 uint32_t *payload_length)
{
    yg_protocol_readonly_page_request_t page_request;
    yg_protocol_link_context_t *link = context;
    size_t written = 0U;

    if (link == NULL ||
        yg_protocol_readonly_decode_page_request(request, &page_request) != YG_PROTOCOL_OK ||
        page_request.page != 0U)
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    if (yg_protocol_readonly_encode_caps_page0(
            &link->capabilities, payload, YG_PROTOCOL_MAX_PAYLOAD_SIZE, &written) != YG_PROTOCOL_OK)
    {
        return YG_PROTOCOL_BUFFER_TOO_SMALL;
    }
    *payload_length = (uint32_t)written;
    return YG_PROTOCOL_OK;
}

static yg_protocol_result_t handle_motor_stop(void *context,
                                              const yg_protocol_message_t *request,
                                              yg_protocol_service_result_t *result)
{
    yg_protocol_motor_request_t motor_request = {.operation = YG_PROTOCOL_MOTOR_STOP};
    yg_protocol_service_reply_t motor_reply;
    yg_protocol_link_context_t *link = context;

    if (link == NULL || request == NULL || result == NULL || request->payload_length != 0U ||
        !motor_request_valid(request))
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    if (request->source_node != YG_LINK_MASTER_NODE_ID)
    {
        write_motor_result(result, 7U, 0U);
        return YG_PROTOCOL_OK;
    }
    motor_request.source_node = request->source_node;
    motor_request.sequence = request->sequence;
    (void)yg_protocol_motor_call(&link->motor_service, &motor_request, &motor_reply);
    if (yg_protocol_motor_encode_reply(
            &motor_reply, result->response_payload, sizeof(result->response_payload), NULL) !=
        YG_PROTOCOL_OK)
    {
        return YG_PROTOCOL_BUFFER_TOO_SMALL;
    }
    result->response_payload_length = YG_PROTOCOL_MOTOR_REPLY_SIZE;
    result->response_flags = YG_PROTOCOL_FLAGS_RESPONSE;
    return YG_PROTOCOL_OK;
}

static void
write_motor_result(yg_protocol_service_result_t *response, uint16_t code, uint16_t detail)
{
    response->response_payload[0] = (uint8_t)code;
    response->response_payload[1] = (uint8_t)(code >> 8U);
    response->response_payload[2] = (uint8_t)detail;
    response->response_payload[3] = (uint8_t)(detail >> 8U);
    response->response_payload_length = YG_PROTOCOL_MOTOR_REPLY_SIZE;
}

static bool motor_request_valid(const yg_protocol_message_t *request)
{
    return request->version == YG_PROTOCOL_VERSION && request->reserved == 0U &&
           request->source_node != YG_PROTOCOL_BROADCAST_NODE_ID &&
           request->destination_node != YG_PROTOCOL_BROADCAST_NODE_ID &&
           (request->flags == YG_PROTOCOL_FLAGS_ACK_REQUEST ||
            request->flags == (YG_PROTOCOL_FLAGS_ACK_REQUEST | YG_PROTOCOL_FLAGS_RETRY));
}

static yg_protocol_result_t handle_motor_enable(void *context,
                                                const yg_protocol_message_t *request,
                                                yg_protocol_service_result_t *result)
{
    yg_protocol_link_context_t *link = context;
    yg_protocol_motor_request_t command = {.operation = YG_PROTOCOL_MOTOR_STOP};
    yg_protocol_service_reply_t reply;
    yg_protocol_result_t decoded;
    bool enabled;
    if (link == NULL || request == NULL || result == NULL || !motor_request_valid(request))
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    if (request->source_node != YG_LINK_MASTER_NODE_ID)
    {
        write_motor_result(result, 7U, 0U);
        return YG_PROTOCOL_OK;
    }
    decoded = yg_protocol_motor_decode_enable(request, &enabled);
    if (decoded != YG_PROTOCOL_OK)
    {
        write_motor_result(result, decoded == YG_PROTOCOL_INVALID_LENGTH ? 3U : 4U, 0U);
        return YG_PROTOCOL_OK;
    }
    command.operation = enabled ? YG_PROTOCOL_MOTOR_ENABLE : YG_PROTOCOL_MOTOR_DISABLE;
    command.source_node = request->source_node;
    command.sequence = request->sequence;
    (void)yg_protocol_motor_call(&link->motor_service, &command, &reply);
    write_motor_result(result, yg_protocol_motor_result_code(reply.status), reply.detail);
    return YG_PROTOCOL_OK;
}

static yg_protocol_result_t handle_motor_control(void *context,
                                                 const yg_protocol_message_t *request,
                                                 yg_protocol_service_result_t *result)
{
    yg_protocol_link_context_t *link = context;
    yg_protocol_motor_request_t command;
    yg_protocol_motor_feedback_t feedback;
    yg_protocol_service_reply_t reply;
    yg_link_control_history_t *entry;
    yg_protocol_result_t decoded;
    uint16_t code, detail = 0U, digest = 0U;
    if (link == NULL || request == NULL || result == NULL || !motor_request_valid(request))
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    code = 7U;
    if (request->source_node == YG_LINK_MASTER_NODE_ID)
    {
        decoded = yg_protocol_motor_decode_control(request, &command);
        code = decoded == YG_PROTOCOL_INVALID_LENGTH ? 3U : 4U;
        if (decoded == YG_PROTOCOL_OK)
        {
            digest = yg_protocol_crc16(request->payload, request->payload_length);
            entry = &link->control_history[request->sequence % YG_LINK_CONTROL_HISTORY_SIZE];
            if (entry->valid && entry->sequence == request->sequence)
            {
                code = entry->request_crc == digest ? entry->result : 4U;
                detail = entry->request_crc == digest ? entry->detail : 0U;
            }
            else if (link->control_sequence_seen &&
                     (uint16_t)(request->sequence - link->last_control_sequence) >= 0x8000U)
            {
                code = 4U; /* 历史窗口外的旧序号不得重新执行。 */
            }
            else
            {
                (void)yg_protocol_motor_call(&link->motor_service, &command, &reply);
                code = yg_protocol_motor_result_code(reply.status);
                detail = reply.detail;
                *entry = (yg_link_control_history_t){request->sequence, digest, code, detail, true};
                link->last_control_sequence = request->sequence;
                link->control_sequence_seen = true;
            }
        }
    }
    if (yg_protocol_motor_feedback_from_source(
            link->motor_status_source, link->node_id, code, request->sequence, &feedback) !=
            YG_PROTOCOL_OK ||
        yg_protocol_motor_feedback_encode(
            &feedback, link->feedback_payload, sizeof(link->feedback_payload)) != YG_PROTOCOL_OK)
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    link->feedback_destination = request->source_node;
    link->feedback_pending = true;
    write_motor_result(result, code, detail);
    return YG_PROTOCOL_OK;
}

static yg_protocol_result_t provide_motor_status(void *context,
                                                 const yg_protocol_message_t *request,
                                                 uint8_t *payload,
                                                 uint32_t *payload_length)
{
    yg_protocol_link_context_t *link = context;
    const yg_protocol_motor_status_source_t *source;

    if (link == NULL)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    source =
        link->motor_status_source != NULL ? link->motor_status_source : &link->empty_motor_status;
    return yg_protocol_motor_status_provider((void *)source, request, payload, payload_length);
}

bool YgProtocolLink_Init(uint8_t node_id)
{
    static const yg_protocol_route_t routes[] = {
        {YG_LINK_PROTOCOL_INFO_TYPE, yg_protocol_readonly_handle, &readonly_service},
        {YG_LINK_CAPABILITIES_TYPE, yg_protocol_readonly_handle, &readonly_service},
        {YG_PROTOCOL_READONLY_TYPE_GET_MOTOR_STATE, yg_protocol_readonly_handle, &readonly_service},
        {YG_PROTOCOL_MOTOR_TYPE_ENABLE, handle_motor_enable, &link_context},
        {YG_PROTOCOL_MOTOR_TYPE_STOP, handle_motor_stop, &link_context},
        {YG_PROTOCOL_MOTOR_TYPE_CONTROL, handle_motor_control, &link_context},
    };
    yg_protocol_readonly_service_t configuration = {0};
    yg_protocol_endpoint_config_t endpoint_config;

    ready = false;
    if (node_id == YG_PROTOCOL_BROADCAST_NODE_ID ||
        !yg_protocol_transfer_queue_init(&rx_queue, rx_storage, YG_LINK_RX_QUEUE_CAPACITY) ||
        !yg_protocol_transfer_queue_init(&tx_queue, tx_storage, YG_LINK_TX_QUEUE_CAPACITY) ||
        yg_protocol_product_registry_init(&registry) != YG_PROTOCOL_OK)
    {
        return false;
    }

    link_context.node_id = node_id;
    link_context.motor_service = (yg_protocol_motor_service_t){0};
    memset(&link_context.empty_motor_status, 0, sizeof(link_context.empty_motor_status));
    link_context.motor_status_source = NULL;
    memset(link_context.control_history, 0, sizeof(link_context.control_history));
    link_context.control_sequence_seen = false;
    link_context.feedback_sequence = 0U;
    link_context.feedback_pending = false;
    memset(&link_context.info, 0, sizeof(link_context.info));
    link_context.info.product_id = 0x5947U;
    link_context.info.hardware_revision = 1U;
    link_context.info.boot_api = 1U;
    link_context.info.application_version = 1U;
    link_context.info.boot_version = 0U;
    link_context.info.boot_id = 0U;
    link_context.capabilities.features = 0x00000001U;
    link_context.capabilities.supported_modes = 0U;
    link_context.capabilities.max_full_frame = YG_PROTOCOL_MAX_FRAME_SIZE;
    link_context.capabilities.max_block_data = YG_PROTOCOL_MAX_PAYLOAD_SIZE;
    link_context.capabilities.write_alignment = 1U;
    link_context.capabilities.default_watchdog_ms = 0U;
    link_context.capabilities.max_watchdog_ms = 0U;

    configuration.context = &link_context;
    configuration.protocol_info_type = YG_LINK_PROTOCOL_INFO_TYPE;
    configuration.capabilities_type = YG_LINK_CAPABILITIES_TYPE;
    configuration.device_info_type = YG_PROTOCOL_READONLY_TYPE_DISABLED;
    configuration.motor_status_type = YG_PROTOCOL_READONLY_TYPE_GET_MOTOR_STATE;
    configuration.protocol_info = provide_info;
    configuration.capabilities = provide_capabilities;
    configuration.motor_status = provide_motor_status;
    if (yg_protocol_readonly_init(&readonly_service, &configuration) != YG_PROTOCOL_OK ||
        yg_protocol_router_init(&router, &registry, routes, sizeof(routes) / sizeof(routes[0])) !=
            YG_PROTOCOL_OK)
    {
        return false;
    }

    endpoint_config.local_node = node_id;
    endpoint_config.reply_priority = 3U;
    endpoint_config.fragment_timeout_ms = 200U;
    endpoint_config.rx = &rx_queue;
    endpoint_config.tx = &tx_queue;
    endpoint_config.router = &router;
    endpoint_config.fragment_storage = fragment_storage;
    endpoint_config.fragment_capacity = sizeof(fragment_storage);
    if (yg_protocol_endpoint_init(&endpoint, &endpoint_config) != YG_PROTOCOL_OK)
    {
        return false;
    }
    ready = true;
    return true;
}

bool YgProtocolLink_BindMotorService(const yg_protocol_motor_service_t *service)
{
    if (!ready)
    {
        return false;
    }
    link_context.motor_service = service != NULL ? *service : (yg_protocol_motor_service_t){0};
    return true;
}

bool YgProtocolLink_BindMotorStatusSource(const yg_protocol_motor_status_source_t *source)
{
    if (!ready)
    {
        return false;
    }
    link_context.motor_status_source = source;
    return true;
}

bool YgProtocolLink_OnRxFrame(const CommHwCanFrame *frame)
{
    uint32_t state;
    bool accepted;
    if (!ready || frame == NULL || !frame->extended || !frame->fd || !frame->bitrate_switch ||
        frame->remote)
    {
        return false;
    }
    state = critical_hw_enter();
    accepted = yg_protocol_canfd_receive(&rx_queue, frame) == YG_PROTOCOL_OK;
    critical_hw_exit(state);
    return accepted;
}

static yg_protocol_result_t send_pending_feedback(void)
{
    yg_protocol_message_t feedback = {0};
    yg_protocol_result_t result;
    if (!link_context.feedback_pending)
    {
        return YG_PROTOCOL_OK;
    }
    feedback.version = YG_PROTOCOL_VERSION;
    feedback.source_node = link_context.node_id;
    feedback.destination_node = link_context.feedback_destination;
    feedback.message_type = YG_PROTOCOL_READONLY_TYPE_MOTION_FEEDBACK;
    feedback.sequence = link_context.feedback_sequence;
    feedback.payload_length = YG_PROTOCOL_MOTOR_FEEDBACK_PAYLOAD_SIZE;
    feedback.payload = link_context.feedback_payload;
    result = yg_protocol_endpoint_send(&endpoint, &feedback, 3U);
    if (result == YG_PROTOCOL_OK)
    {
        ++link_context.feedback_sequence;
        link_context.feedback_pending = false;
    }
    return result;
}

void YgProtocolLink_Service(uint32_t now_ms)
{
    uint8_t processed = 0U;
    uint8_t sent = 0U;

    if (!ready)
    {
        return;
    }
    while (processed < 8U)
    {
        yg_protocol_transfer_frame_t frame;
        yg_protocol_result_t result;
        if (!endpoint.response_pending && send_pending_feedback() != YG_PROTOCOL_OK)
        {
            break;
        }
        if (endpoint.response_pending)
        {
            result = yg_protocol_endpoint_process_frame(&endpoint, NULL, now_ms);
        }
        else
        {
            /* 116 执行业务前，为同 Type 回复和 124 反馈各留一帧。 */
            if (YG_LINK_TX_QUEUE_CAPACITY - yg_protocol_transfer_queue_count(&tx_queue) < 2U)
            {
                break;
            }
            uint32_t state = critical_hw_enter();
            bool received = yg_protocol_transfer_queue_pop(&rx_queue, &frame);
            critical_hw_exit(state);
            if (!received)
            {
                break;
            }
            /* 临界区只复制帧；CRC、路由和业务处理始终允许电机中断抢占。 */
            result = yg_protocol_endpoint_process_frame(&endpoint, &frame, now_ms);
        }
        if (result == YG_PROTOCOL_QUEUE_FULL)
        {
            break;
        }
        if (!endpoint.response_pending && send_pending_feedback() != YG_PROTOCOL_OK)
        {
            break;
        }
        ++processed;
    }
    while (sent < 8U && yg_protocol_canfd_send_one(&tx_queue) == YG_PROTOCOL_OK)
    {
        ++sent;
    }
}

bool YgProtocolLink_IsReady(void)
{
    return ready;
}
