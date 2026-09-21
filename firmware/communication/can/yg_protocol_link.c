/* 只读 CAN FD 链路诊断端点：验证扩展帧、CRC、路由和响应闭环，不接触电机控制。 */
#include "yg_protocol_link.h"

#include <stddef.h>
#include <string.h>

#include "yg_protocol_endpoint.h"
#include "yg_protocol_readonly.h"
#include "yg_protocol_readonly_payload.h"
#include "yg_protocol_transfer.h"

#define YG_LINK_QUEUE_CAPACITY 8U
#define YG_LINK_FRAGMENT_CAPACITY 1767U
#define YG_LINK_PROTOCOL_INFO_TYPE 1U
#define YG_LINK_CAPABILITIES_TYPE 2U

typedef struct
{
    uint8_t node_id;
    yg_protocol_readonly_info_page0_t info;
    yg_protocol_readonly_caps_page0_t capabilities;
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

bool YgProtocolLink_Init(uint8_t node_id)
{
    static const yg_protocol_route_t routes[] = {
        {YG_LINK_PROTOCOL_INFO_TYPE, yg_protocol_readonly_handle, &readonly_service},
        {YG_LINK_CAPABILITIES_TYPE, yg_protocol_readonly_handle, &readonly_service},
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
    configuration.motor_status_type = YG_PROTOCOL_READONLY_TYPE_DISABLED;
    configuration.protocol_info = provide_info;
    configuration.capabilities = provide_capabilities;
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

bool YgProtocolLink_OnRxFrame(const CommHwCanFrame *frame)
{
    if (!ready || frame == NULL || !frame->extended || !frame->fd || frame->remote)
    {
        return false;
    }
    return yg_protocol_canfd_receive(&rx_queue, frame) == YG_PROTOCOL_OK;
}

void YgProtocolLink_Service(uint32_t now_ms)
{
    uint8_t processed = 0U;
    uint8_t sent = 0U;

    if (!ready)
    {
        return;
    }
    while (processed < 4U &&
           yg_protocol_endpoint_process_one(&endpoint, now_ms) != YG_PROTOCOL_QUEUE_EMPTY)
    {
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
