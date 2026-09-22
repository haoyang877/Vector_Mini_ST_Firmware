/* 只读 CAN FD 链路诊断端点：验证扩展帧、CRC、路由和响应闭环，不接触电机控制。 */
#include "yg_protocol_link.h"

#include <stddef.h>
#include <string.h>

#include "critical_hw.h"

#include "yg_protocol_endpoint.h"
#include "yg_protocol_readonly.h"
#include "yg_protocol_readonly_payload.h"
#include "yg_protocol_transfer.h"
#include "yg_protocol_motor.h"

#define YG_LINK_QUEUE_CAPACITY 4U
#define YG_LINK_FRAGMENT_CAPACITY 128U
#define YG_LINK_PROTOCOL_INFO_TYPE 1U
#define YG_LINK_CAPABILITIES_TYPE 2U

typedef struct
{
    uint8_t node_id;
    yg_protocol_readonly_info_page0_t info;
    yg_protocol_readonly_caps_page0_t capabilities;
    yg_protocol_motor_service_t motor_service;
    yg_protocol_motor_status_source_t empty_motor_status;
    const yg_protocol_motor_status_source_t *motor_status_source;
} yg_protocol_link_context_t;

static yg_protocol_link_context_t link_context;
static yg_protocol_transfer_frame_t rx_storage[YG_LINK_QUEUE_CAPACITY];
static yg_protocol_transfer_frame_t tx_storage[YG_LINK_QUEUE_CAPACITY];
static uint8_t fragment_storage[YG_LINK_FRAGMENT_CAPACITY];
static yg_protocol_transfer_queue_t rx_queue;
static yg_protocol_transfer_queue_t tx_queue;
static yg_protocol_message_registry_t registry;
static yg_protocol_router_t router;
static yg_protocol_readonly_service_t readonly_service;
static yg_protocol_endpoint_t endpoint;
static bool ready;

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

    if (link == NULL || request == NULL || result == NULL || request->payload_length != 0U)
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    motor_request.operation =
        (yg_protocol_motor_operation_t)(request->message_type == YG_PROTOCOL_MOTOR_TYPE_STOP
                                            ? YG_PROTOCOL_MOTOR_STOP
                                            : YG_PROTOCOL_MOTOR_DISABLE);
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
        {YG_PROTOCOL_MOTOR_TYPE_STOP, handle_motor_stop, &link_context},
        {YG_PROTOCOL_MOTOR_TYPE_DISABLE, handle_motor_stop, &link_context},
    };
    yg_protocol_readonly_service_t configuration = {0};
    yg_protocol_endpoint_config_t endpoint_config;

    ready = false;
    if (node_id == YG_PROTOCOL_BROADCAST_NODE_ID ||
        !yg_protocol_transfer_queue_init(&rx_queue, rx_storage, YG_LINK_QUEUE_CAPACITY) ||
        !yg_protocol_transfer_queue_init(&tx_queue, tx_storage, YG_LINK_QUEUE_CAPACITY) ||
        yg_protocol_readonly_registry_init(&registry) != YG_PROTOCOL_OK)
    {
        return false;
    }

    link_context.node_id = node_id;
    link_context.motor_service = (yg_protocol_motor_service_t){0};
    memset(&link_context.empty_motor_status, 0, sizeof(link_context.empty_motor_status));
    link_context.motor_status_source = NULL;
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
    if (!ready || frame == NULL || !frame->extended || !frame->fd || frame->remote)
    {
        return false;
    }
    state = critical_hw_enter();
    accepted = yg_protocol_canfd_receive(&rx_queue, frame) == YG_PROTOCOL_OK;
    critical_hw_exit(state);
    return accepted;
}

void YgProtocolLink_Service(uint32_t now_ms)
{
    uint8_t processed = 0U;
    uint8_t sent = 0U;

    if (!ready)
    {
        return;
    }
    while (processed < 4U)
    {
        yg_protocol_transfer_frame_t frame;
        yg_protocol_result_t result;
        if (endpoint.response_pending)
        {
            result = yg_protocol_endpoint_process_frame(&endpoint, NULL, now_ms);
        }
        else
        {
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
        ++processed;
    }
    while (sent < 4U && yg_protocol_canfd_send_one(&tx_queue) == YG_PROTOCOL_OK)
    {
        ++sent;
    }
}

bool YgProtocolLink_IsReady(void)
{
    return ready;
}
