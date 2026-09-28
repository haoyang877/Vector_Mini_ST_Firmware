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
#include "yg_protocol_parameter.h"
#include "yg_protocol_boot.h"

/* 队列一次覆盖最大 1767B 逻辑消息的 39 个 46B 分片，并保留一个空槽。
 * 重组区与协议上限一致，避免声明支持长消息却在组合层静默截短。 */
#define YG_LINK_RX_QUEUE_CAPACITY 8U
#define YG_LINK_TX_QUEUE_CAPACITY 40U
#define YG_LINK_FRAGMENT_CAPACITY YG_PROTOCOL_MAX_MESSAGE_PAYLOAD
#define YG_LINK_MASTER_NODE_ID 2U
#define YG_LINK_CONTROL_HISTORY_SIZE 256U
#define YG_LINK_BOOT_QUEUE_TIMEOUT_MS 100U

/* 仅56需要精确发送身份，旁记录不扩大每一条普通CAN帧。 */
typedef enum
{
    YG_BOOT_TX_NONE,
    YG_BOOT_TX_PREPARED,
    YG_BOOT_TX_QUEUED,
    YG_BOOT_TX_IN_FLIGHT
} yg_link_boot_tx_state_t;

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
    const yg_protocol_readonly_device_status_t *device_status_source;
    yg_protocol_motor_service_t motor_service;
    yg_protocol_parameter_service_t parameter_service;
    yg_protocol_job_service_t job_service;
    BootEntryService boot_service;
    uint64_t boot_ticket;
    uint32_t service_now_ms;
    uint32_t boot_ready_ms;
    size_t boot_tx_slot;
    size_t base_route_count;
    yg_link_boot_tx_state_t boot_tx_state;
    uint16_t boot_reply_sequence;
    uint8_t boot_reply_destination;
    yg_protocol_motor_status_source_t empty_motor_status;
    const yg_protocol_motor_status_source_t *motor_status_source;
    yg_link_control_history_t control_history[YG_LINK_CONTROL_HISTORY_SIZE];
    uint16_t last_control_sequence;
    uint16_t feedback_sequence;
    bool device_description_available;
    bool control_sequence_seen;
    bool feedback_pending;
    uint8_t feedback_destination;
    uint8_t feedback_payload[YG_PROTOCOL_MOTOR_FEEDBACK_PAYLOAD_SIZE];
} yg_protocol_link_context_t;

static yg_protocol_link_context_t link_context;
static const yg_protocol_readonly_device_status_t empty_device_status = {
    .role = YG_PROTOCOL_DEVICE_ROLE_APP};
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

static bool request_is_valid(const yg_protocol_message_t *request);
static void write_result(yg_protocol_service_result_t *response, uint16_t code, uint16_t detail);

static yg_protocol_result_t provide_unavailable(uint8_t *payload, uint32_t *payload_length)
{
    /* 缺少真实身份/能力也须给业务错误回复，不能让端点将内部错误当作丢帧。 */
    payload[0] = 6U;
    payload[1] = 0U;
    payload[2] = 0U;
    payload[3] = 0U;
    *payload_length = 4U;
    return YG_PROTOCOL_OK;
}

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
    if (!link->device_description_available)
    {
        return provide_unavailable(payload, payload_length);
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
    if (!link->device_description_available)
    {
        return provide_unavailable(payload, payload_length);
    }
    if (yg_protocol_readonly_encode_caps_page0(
            &link->capabilities, payload, YG_PROTOCOL_MAX_PAYLOAD_SIZE, &written) != YG_PROTOCOL_OK)
    {
        return YG_PROTOCOL_BUFFER_TOO_SMALL;
    }
    *payload_length = (uint32_t)written;
    return YG_PROTOCOL_OK;
}

static yg_protocol_result_t handle_device_status(void *context,
                                                 const yg_protocol_message_t *request,
                                                 yg_protocol_service_result_t *result)
{
    yg_protocol_link_context_t *link = context;
    if (link == NULL || request == NULL || result == NULL || !request_is_valid(request))
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    if (request->payload_length != 0U)
    {
        write_result(result, 3U, 0U);
        return YG_PROTOCOL_OK;
    }
    if (yg_protocol_readonly_encode_device_status(link->device_status_source,
                                                  result->response_payload,
                                                  sizeof(result->response_payload),
                                                  NULL) != YG_PROTOCOL_OK)
    {
        write_result(result, 8U, 0U);
        return YG_PROTOCOL_OK;
    }
    result->response_payload_length = YG_PROTOCOL_READONLY_DEVICE_STATUS_SIZE;
    return YG_PROTOCOL_OK;
}

static yg_protocol_result_t handle_motor_action(void *context,
                                                const yg_protocol_message_t *request,
                                                yg_protocol_service_result_t *result)
{
    yg_protocol_motor_request_t command = {.operation = YG_PROTOCOL_MOTOR_STOP};
    yg_protocol_service_reply_t reply;
    yg_protocol_link_context_t *link = context;
    yg_protocol_result_t decoded;
    bool enabled;

    /* STOP 的非空请求在来源检查前拒绝，保留既有线路处理顺序。 */
    if (link == NULL || request == NULL || result == NULL || !request_is_valid(request) ||
        (request->message_type != YG_PROTOCOL_MOTOR_TYPE_ENABLE && request->payload_length != 0U))
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    if (request->source_node != YG_LINK_MASTER_NODE_ID)
    {
        write_result(result, 7U, 0U);
        return YG_PROTOCOL_OK;
    }
    if (request->message_type == YG_PROTOCOL_MOTOR_TYPE_ENABLE)
    {
        decoded = yg_protocol_motor_decode_enable(request, &enabled);
        if (decoded != YG_PROTOCOL_OK)
        {
            write_result(result, decoded == YG_PROTOCOL_INVALID_LENGTH ? 3U : 4U, 0U);
            return YG_PROTOCOL_OK;
        }
        command.operation = enabled ? YG_PROTOCOL_MOTOR_ENABLE : YG_PROTOCOL_MOTOR_DISABLE;
    }
    else if (request->message_type == YG_PROTOCOL_MOTOR_TYPE_CLEAR_FAULT)
    {
        command.operation = YG_PROTOCOL_MOTOR_CLEAR_FAULT;
    }
    command.source_node = request->source_node;
    command.sequence = request->sequence;
    (void)yg_protocol_motor_call(&link->motor_service, &command, &reply);
    if (yg_protocol_motor_encode_reply(
            &reply, result->response_payload, sizeof(result->response_payload), NULL) !=
        YG_PROTOCOL_OK)
    {
        return YG_PROTOCOL_BUFFER_TOO_SMALL;
    }
    result->response_payload_length = YG_PROTOCOL_MOTOR_REPLY_SIZE;
    result->response_flags = YG_PROTOCOL_FLAGS_RESPONSE;
    return YG_PROTOCOL_OK;
}

static void write_result(yg_protocol_service_result_t *response, uint16_t code, uint16_t detail)
{
    response->response_payload[0] = (uint8_t)code;
    response->response_payload[1] = (uint8_t)(code >> 8U);
    response->response_payload[2] = (uint8_t)detail;
    response->response_payload[3] = (uint8_t)(detail >> 8U);
    response->response_payload_length = YG_PROTOCOL_MOTOR_REPLY_SIZE;
}

static bool request_is_valid(const yg_protocol_message_t *request)
{
    return request->version == YG_PROTOCOL_VERSION && request->reserved == 0U &&
           request->source_node != YG_PROTOCOL_BROADCAST_NODE_ID &&
           request->destination_node != YG_PROTOCOL_BROADCAST_NODE_ID &&
           (request->flags == YG_PROTOCOL_FLAGS_ACK_REQUEST ||
            request->flags == (YG_PROTOCOL_FLAGS_ACK_REQUEST | YG_PROTOCOL_FLAGS_RETRY));
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
    if (link == NULL || request == NULL || result == NULL || !request_is_valid(request))
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
    write_result(result, code, detail);
    return YG_PROTOCOL_OK;
}

static yg_protocol_result_t provide_motor_status(void *context,
                                                 const yg_protocol_message_t *request,
                                                 uint8_t *payload,
                                                 uint32_t *payload_length)
{
    yg_protocol_link_context_t *link = context;

    if (link == NULL)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    return yg_protocol_motor_status_provider(
        (void *)link->motor_status_source, request, payload, payload_length);
}

static yg_protocol_result_t handle_parameter(void *context,
                                             const yg_protocol_message_t *request,
                                             yg_protocol_service_result_t *result)
{
    yg_protocol_link_context_t *link = context;
    yg_protocol_parameter_request_t command;
    yg_protocol_service_reply_t reply;
    yg_protocol_result_t decoded;
    size_t written;
    if (link == NULL || request == NULL || result == NULL || !request_is_valid(request))
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    if (request->source_node != YG_LINK_MASTER_NODE_ID)
    {
        write_result(result, 7U, 0U);
        return YG_PROTOCOL_OK;
    }
    decoded = yg_protocol_parameter_decode(request, &command);
    if (decoded != YG_PROTOCOL_OK)
    {
        write_result(
            result,
            decoded == YG_PROTOCOL_INVALID_LENGTH ? 3U : 4U,
            decoded == YG_PROTOCOL_INVALID_LENGTH ? 0U : YG_PROTOCOL_PARAMETER_DETAIL_BAD_FIELD);
        return YG_PROTOCOL_OK;
    }
    /* 幂等与持久作业由参数所有者掌握，通信不缓存一个尚未完成的“成功”。 */
    (void)yg_protocol_parameter_call(&link->parameter_service, &command, &reply);
    if (yg_protocol_parameter_encode_reply(&command,
                                           &reply,
                                           result->response_payload,
                                           sizeof(result->response_payload),
                                           &written) != YG_PROTOCOL_OK)
    {
        write_result(result, 8U, 0U);
        return YG_PROTOCOL_OK;
    }
    result->response_payload_length = (uint32_t)written;
    return YG_PROTOCOL_OK;
}

static void write_job_error(yg_protocol_service_result_t *response, uint16_t code, uint16_t detail)
{
    write_result(response, code, detail);
    memset(response->response_payload + 4U, 0, YG_PROTOCOL_JOB_REPLY_SIZE - 4U);
    response->response_payload_length = YG_PROTOCOL_JOB_REPLY_SIZE;
}

static yg_protocol_result_t handle_job(void *context,
                                       const yg_protocol_message_t *request,
                                       yg_protocol_service_result_t *result)
{
    yg_protocol_link_context_t *link = context;
    yg_protocol_job_request_t command;
    yg_protocol_job_reply_t reply;
    yg_protocol_result_t decoded;
    if (link == NULL || request == NULL || result == NULL || !request_is_valid(request))
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    if (request->source_node != YG_LINK_MASTER_NODE_ID)
    {
        write_job_error(result, 7U, 0U);
        return YG_PROTOCOL_OK;
    }
    decoded = yg_protocol_job_decode(request, &command);
    if (decoded != YG_PROTOCOL_OK)
    {
        write_job_error(result,
                        decoded == YG_PROTOCOL_INVALID_LENGTH ? 3U : 4U,
                        YG_PROTOCOL_JOB_DETAIL_BAD_FIELD);
        return YG_PROTOCOL_OK;
    }
    (void)yg_protocol_job_call(&link->job_service, &command, &reply);
    if (yg_protocol_job_encode_reply(
            &reply, result->response_payload, sizeof(result->response_payload), NULL) !=
        YG_PROTOCOL_OK)
    {
        write_job_error(result, 8U, 0U);
        return YG_PROTOCOL_OK;
    }
    result->response_payload_length = YG_PROTOCOL_JOB_REPLY_SIZE;
    return YG_PROTOCOL_OK;
}

static yg_protocol_result_t handle_boot(void *context,
                                        const yg_protocol_message_t *request,
                                        yg_protocol_service_result_t *result)
{
    yg_protocol_link_context_t *link = context;
    BootEntryRequest command;
    BootEntryReply reply = {.result = BOOT_ENTRY_FAILED};
    yg_protocol_result_t decoded;
    if (link == NULL || request == NULL || result == NULL || !request_is_valid(request))
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    if (request->source_node != YG_LINK_MASTER_NODE_ID)
    {
        write_result(result, 7U, 0U);
        return YG_PROTOCOL_OK;
    }
    decoded = yg_protocol_boot_decode(request, &command);
    if (decoded != YG_PROTOCOL_OK)
    {
        write_result(result, decoded == YG_PROTOCOL_INVALID_LENGTH ? 3U : 4U, 0U);
        return YG_PROTOCOL_OK;
    }
    if (link->boot_tx_state != YG_BOOT_TX_NONE)
    {
        /* 一次只发送一个带票据的READY；重发不能替换原票据或续长其截止。 */
        write_result(result, 6U, 0U);
        return YG_PROTOCOL_OK;
    }
    if (link->boot_service.request == NULL || link->boot_service.tx_complete == NULL)
    {
        write_result(result, 2U, 0U);
        return YG_PROTOCOL_OK;
    }
    link->boot_service.request(link->boot_service.context, &command, &reply);
    if (yg_protocol_boot_encode_reply(
            &reply, result->response_payload, sizeof(result->response_payload), NULL) !=
        YG_PROTOCOL_OK)
    {
        if (reply.ack_ticket != 0U)
        {
            link->boot_service.tx_complete(link->boot_service.context, reply.ack_ticket, false);
        }
        write_result(result, 8U, 0U);
        return YG_PROTOCOL_OK;
    }
    if (reply.result == BOOT_ENTRY_READY)
    {
        link->boot_ticket = reply.ack_ticket;
        link->boot_ready_ms = link->service_now_ms;
        link->boot_reply_sequence = request->sequence;
        link->boot_reply_destination = request->source_node;
        link->boot_tx_state = YG_BOOT_TX_PREPARED;
    }
    result->response_payload_length = YG_PROTOCOL_BOOT_REPLY_SIZE;
    return YG_PROTOCOL_OK;
}

bool YgProtocolLink_Init(uint8_t node_id)
{
    static const yg_protocol_route_t routes[] = {
        {YG_PROTOCOL_READONLY_TYPE_GET_INFO, yg_protocol_readonly_handle, &readonly_service},
        {YG_PROTOCOL_READONLY_TYPE_GET_CAPS, yg_protocol_readonly_handle, &readonly_service},
        {YG_PROTOCOL_READONLY_TYPE_GET_DEVICE_STATUS, handle_device_status, &link_context},
        {YG_PROTOCOL_READONLY_TYPE_GET_MOTOR_STATE, yg_protocol_readonly_handle, &readonly_service},
        {YG_PROTOCOL_MOTOR_TYPE_ENABLE, handle_motor_action, &link_context},
        {YG_PROTOCOL_MOTOR_TYPE_CLEAR_FAULT, handle_motor_action, &link_context},
        {YG_PROTOCOL_MOTOR_TYPE_STOP, handle_motor_action, &link_context},
        {YG_PROTOCOL_MOTOR_TYPE_CONTROL, handle_motor_control, &link_context},
        {YG_PROTOCOL_PARAMETER_TYPE_READ, handle_parameter, &link_context},
        {YG_PROTOCOL_PARAMETER_TYPE_WRITE, handle_parameter, &link_context},
        {YG_PROTOCOL_PARAMETER_TYPE_SAVE, handle_parameter, &link_context},
        {YG_PROTOCOL_JOB_TYPE_START, handle_job, &link_context},
        {YG_PROTOCOL_JOB_TYPE_STATUS, handle_job, &link_context},
        {YG_PROTOCOL_JOB_TYPE_CANCEL, handle_job, &link_context},
        {YG_PROTOCOL_BOOT_TYPE_ENTER, handle_boot, &link_context},
    };
    yg_protocol_readonly_service_t configuration = {0};
    yg_protocol_endpoint_config_t endpoint_config;

    if (ready && link_context.boot_tx_state != YG_BOOT_TX_NONE)
    {
        /* 初始化不能丢弃尚未完成的物理票据；由所有者/平台先确认失败或完成。 */
        return false;
    }
    ready = false;
    if (node_id == YG_PROTOCOL_BROADCAST_NODE_ID ||
        !yg_protocol_transfer_queue_init(&rx_queue, rx_storage, YG_LINK_RX_QUEUE_CAPACITY) ||
        !yg_protocol_transfer_queue_init(&tx_queue, tx_storage, YG_LINK_TX_QUEUE_CAPACITY) ||
        yg_protocol_product_registry_init(&registry) != YG_PROTOCOL_OK)
    {
        return false;
    }

    /* 初始化一次撤销全部旧绑定、历史和待发状态；此时 RX 尚未启动。 */
    memset(&link_context, 0, sizeof(link_context));
    link_context.node_id = node_id;
    link_context.motor_status_source = &link_context.empty_motor_status;
    link_context.device_status_source = &empty_device_status;
    link_context.base_route_count = sizeof(routes) / sizeof(routes[0]) - 1U;

    configuration.context = &link_context;
    configuration.protocol_info_type = YG_PROTOCOL_READONLY_TYPE_GET_INFO;
    configuration.capabilities_type = YG_PROTOCOL_READONLY_TYPE_GET_CAPS;
    configuration.device_info_type = YG_PROTOCOL_READONLY_TYPE_DISABLED;
    configuration.motor_status_type = YG_PROTOCOL_READONLY_TYPE_GET_MOTOR_STATE;
    configuration.protocol_info = provide_info;
    configuration.capabilities = provide_capabilities;
    configuration.motor_status = provide_motor_status;
    if (yg_protocol_readonly_init(&readonly_service, &configuration) != YG_PROTOCOL_OK ||
        yg_protocol_router_init(&router, &registry, routes, link_context.base_route_count) !=
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

bool YgProtocolLink_SetDeviceDescription(const yg_protocol_readonly_info_page0_t *info,
                                         const yg_protocol_readonly_caps_page0_t *capabilities)
{
    if (!ready || (info == NULL) != (capabilities == NULL))
    {
        return false;
    }
    link_context.info = info != NULL ? *info : (yg_protocol_readonly_info_page0_t){0};
    link_context.capabilities =
        capabilities != NULL ? *capabilities : (yg_protocol_readonly_caps_page0_t){0};
    link_context.device_description_available = info != NULL;
    return true;
}

bool YgProtocolLink_BindDeviceStatusSource(const yg_protocol_readonly_device_status_t *source)
{
    if (!ready)
    {
        return false;
    }
    link_context.device_status_source = source != NULL ? source : &empty_device_status;
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

bool YgProtocolLink_BindParameterService(const yg_protocol_parameter_service_t *service)
{
    if (!ready)
    {
        return false;
    }
    link_context.parameter_service =
        service != NULL ? *service : (yg_protocol_parameter_service_t){0};
    return true;
}

bool YgProtocolLink_BindJobService(const yg_protocol_job_service_t *service)
{
    if (!ready)
    {
        return false;
    }
    link_context.job_service = service != NULL ? *service : (yg_protocol_job_service_t){0};
    return true;
}

bool YgProtocolLink_BindBootService(const BootEntryService *service)
{
    if (!ready || link_context.boot_tx_state != YG_BOOT_TX_NONE || endpoint.response_pending ||
        yg_protocol_transfer_queue_count(&rx_queue) != 0U ||
        yg_protocol_transfer_queue_count(&tx_queue) != 0U ||
        (service != NULL && (service->request == NULL || service->tx_complete == NULL)))
    {
        return false;
    }
    const size_t count = link_context.base_route_count + (service != NULL ? 1U : 0U);
    if (yg_protocol_router_init(&router, &registry, router.routes, count) != YG_PROTOCOL_OK)
    {
        return false;
    }
    link_context.boot_service = service != NULL ? *service : (BootEntryService){0};
    return true;
}

bool YgProtocolLink_BindMotorStatusSource(const yg_protocol_motor_status_source_t *source)
{
    if (!ready)
    {
        return false;
    }
    link_context.motor_status_source = source != NULL ? source : &link_context.empty_motor_status;
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

static void finish_boot_ticket(bool success)
{
    const uint64_t ticket = link_context.boot_ticket;
    link_context.boot_ticket = 0U;
    link_context.boot_tx_state = YG_BOOT_TX_NONE;
    if (ticket != 0U && link_context.boot_service.tx_complete != NULL)
    {
        link_context.boot_service.tx_complete(link_context.boot_service.context, ticket, success);
    }
}

static void service_boot_completion(void)
{
    CommHwCanTxCompletion completion;
    for (uint8_t received = 0U; received < 8U; ++received)
    {
        if (!comm_hw_can_take_tx_completion(&completion))
        {
            break;
        }
        if (link_context.boot_tx_state == YG_BOOT_TX_IN_FLIGHT &&
            completion.ticket == link_context.boot_ticket && completion.ticket != 0U)
        {
            finish_boot_ticket(completion.success);
        }
        /* 旧票据或其他帧事件不能使当前交接完成。 */
    }
}

static void capture_boot_reply_slot(size_t previous_tail, yg_protocol_result_t result)
{
    if (link_context.boot_tx_state != YG_BOOT_TX_PREPARED || endpoint.response_pending)
    {
        return;
    }
    if (result != YG_PROTOCOL_OK || tx_queue.tail == previous_tail)
    {
        finish_boot_ticket(false);
        return;
    }
    /* endpoint每次只排一个4B回复，此位置即当前READY，不搜索相似ID/seq的旧帧。 */
    link_context.boot_tx_slot = previous_tail;
    link_context.boot_tx_state = YG_BOOT_TX_QUEUED;
}

static bool is_boot_ready_frame(const yg_protocol_transfer_frame_t *frame)
{
    yg_protocol_message_t response;
    return yg_protocol_canfd_unpack(frame, link_context.boot_reply_destination, &response) ==
               YG_PROTOCOL_OK &&
           response.message_type == YG_PROTOCOL_BOOT_TYPE_ENTER &&
           response.flags == YG_PROTOCOL_FLAGS_RESPONSE &&
           response.source_node == link_context.node_id &&
           response.sequence == link_context.boot_reply_sequence &&
           response.payload_length == YG_PROTOCOL_BOOT_REPLY_SIZE && response.payload[0] == 1U &&
           response.payload[1] == 0U && response.payload[2] == 0U && response.payload[3] == 0U;
}

static void discard_queued_boot_reply(void)
{
    size_t slot = tx_queue.head;
    for (size_t i = 0U; i < tx_queue.count; ++i)
    {
        if (slot == link_context.boot_tx_slot)
        {
            if (!is_boot_ready_frame(&tx_queue.storage[slot]))
            {
                return;
            }
            const size_t last = (tx_queue.tail + tx_queue.capacity - 1U) % tx_queue.capacity;
            /* 只有一条带票据回复；最多移39条普通帧，保持它们原顺序且不改丢帧计数。 */
            while (slot != last)
            {
                size_t next = (slot + 1U) % tx_queue.capacity;
                tx_queue.storage[slot] = tx_queue.storage[next];
                slot = next;
            }
            tx_queue.tail = last;
            --tx_queue.count;
            return;
        }
        slot = (slot + 1U) % tx_queue.capacity;
    }
}

static void expire_boot_reply(uint32_t now_ms)
{
    if ((link_context.boot_tx_state != YG_BOOT_TX_PREPARED &&
         link_context.boot_tx_state != YG_BOOT_TX_QUEUED) ||
        (uint32_t)(now_ms - link_context.boot_ready_ms) < YG_LINK_BOOT_QUEUE_TIMEOUT_MS)
    {
        return;
    }
    if (link_context.boot_tx_state == YG_BOOT_TX_PREPARED)
    {
        if (endpoint.response_pending &&
            endpoint.pending_response.message_type == YG_PROTOCOL_BOOT_TYPE_ENTER &&
            endpoint.pending_response.sequence == link_context.boot_reply_sequence &&
            endpoint.pending_response.destination_node == link_context.boot_reply_destination)
        {
            endpoint.response_pending = false;
        }
    }
    else
    {
        discard_queued_boot_reply();
    }
    finish_boot_ticket(false);
}

static yg_protocol_result_t send_one(void)
{
    yg_protocol_transfer_frame_t frame;
    CommHwCanFrame hardware = {0};
    if (link_context.boot_tx_state != YG_BOOT_TX_QUEUED ||
        tx_queue.head != link_context.boot_tx_slot)
    {
        return yg_protocol_canfd_send_one(&tx_queue);
    }
    if (!yg_protocol_transfer_queue_peek(&tx_queue, &frame))
    {
        finish_boot_ticket(false);
        return YG_PROTOCOL_QUEUE_EMPTY;
    }
    /* 复用既有承载校验，避免位置关联被后续队列改动错误指向普通回复。 */
    if (!is_boot_ready_frame(&frame))
    {
        (void)yg_protocol_transfer_queue_pop(&tx_queue, &frame);
        finish_boot_ticket(false);
        return YG_PROTOCOL_INVALID_CAN_FRAME;
    }
    hardware.identifier = frame.identifier;
    hardware.length = frame.length;
    hardware.extended = frame.extended;
    hardware.remote = frame.remote;
    hardware.fd = frame.fd;
    hardware.bitrate_switch = frame.bitrate_switch;
    memcpy(hardware.data, frame.data, frame.length);
    if (!comm_hw_can_try_send_tracked_frame(&hardware, link_context.boot_ticket))
    {
        return YG_PROTOCOL_TRANSPORT_BUSY;
    }
    (void)yg_protocol_transfer_queue_pop(&tx_queue, &frame);
    link_context.boot_tx_state = YG_BOOT_TX_IN_FLIGHT;
    return YG_PROTOCOL_OK;
}

void YgProtocolLink_Service(uint32_t now_ms)
{
    uint8_t processed = 0U;
    uint8_t sent = 0U;

    if (!ready)
    {
        return;
    }
    service_boot_completion();
    link_context.service_now_ms = now_ms;
    expire_boot_reply(now_ms);
    while (processed < 8U)
    {
        yg_protocol_transfer_frame_t frame;
        yg_protocol_result_t result;
        size_t previous_tail;
        if (!endpoint.response_pending && send_pending_feedback() != YG_PROTOCOL_OK)
        {
            break;
        }
        if (endpoint.response_pending)
        {
            previous_tail = tx_queue.tail;
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
            previous_tail = tx_queue.tail;
            result = yg_protocol_endpoint_process_frame(&endpoint, &frame, now_ms);
        }
        capture_boot_reply_slot(previous_tail, result);
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
    while (sent < 8U && send_one() == YG_PROTOCOL_OK)
    {
        ++sent;
    }
}

bool YgProtocolLink_IsReady(void)
{
    return ready;
}
