#include "yg_protocol_service.h"
#include "yg_protocol_readonly.h"
#include "yg_protocol_common.h"
#include <string.h>

/* 查询校验与有界回复；提供者已在命令注册时确定。 */

yg_protocol_result_t yg_protocol_readonly_handle(void *context,
                                                 const yg_protocol_message_t *request,
                                                 yg_protocol_service_result_t *result)
{
    yg_protocol_readonly_service_t *service = (yg_protocol_readonly_service_t *)context;
    uint32_t payload_length = YG_PROTOCOL_MAX_PAYLOAD_SIZE;
    yg_protocol_result_t provider_result;

    if (service == NULL || service->provider == NULL || request == NULL || result == NULL)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    result->response_payload_length = 0U;
    if (request->source_node == YG_PROTOCOL_BROADCAST_NODE_ID ||
        request->destination_node == YG_PROTOCOL_BROADCAST_NODE_ID)
    {
        return YG_PROTOCOL_INVALID_NODE_ID;
    }
    if (request->version != YG_PROTOCOL_VERSION || request->reserved != 0U ||
        (request->flags != YG_PROTOCOL_FLAGS_ACK_REQUEST &&
         request->flags != (YG_PROTOCOL_FLAGS_ACK_REQUEST | YG_PROTOCOL_FLAGS_RETRY)))
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    provider_result =
        service->provider(service->context, request, result->response_payload, &payload_length);
    if (provider_result != YG_PROTOCOL_OK)
    {
        return provider_result;
    }
    if (payload_length > YG_PROTOCOL_MAX_PAYLOAD_SIZE)
    {
        return YG_PROTOCOL_INVALID_LENGTH;
    }
    result->response_payload_length = payload_length;
    result->response_flags = YG_PROTOCOL_FLAGS_RESPONSE;
    return YG_PROTOCOL_OK;
}

/* 只读查询 payload 的显式小端编解码，不访问设备状态或硬件。 */

yg_protocol_result_t
yg_protocol_readonly_encode_device_status(const yg_protocol_readonly_device_status_t *status,
                                          uint8_t *payload,
                                          size_t capacity,
                                          size_t *written)
{
    if (status == NULL || payload == NULL)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    if ((status->role != YG_PROTOCOL_DEVICE_ROLE_APP &&
         status->role != YG_PROTOCOL_DEVICE_ROLE_LOADER) ||
        status->boot_state > YG_PROTOCOL_BOOT_STATE_LOADER ||
        (status->role == YG_PROTOCOL_DEVICE_ROLE_APP &&
         status->boot_state == YG_PROTOCOL_BOOT_STATE_LOADER) ||
        (status->role == YG_PROTOCOL_DEVICE_ROLE_LOADER &&
         status->boot_state != YG_PROTOCOL_BOOT_STATE_LOADER) ||
        (status->epoch_valid &&
         (status->boot_epoch == 0U || status->boot_state == YG_PROTOCOL_BOOT_STATE_UNAVAILABLE)) ||
        (!status->epoch_valid &&
         (status->boot_epoch != 0U || (status->role == YG_PROTOCOL_DEVICE_ROLE_APP &&
                                       status->boot_state != YG_PROTOCOL_BOOT_STATE_UNAVAILABLE))))
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    if (capacity < YG_PROTOCOL_READONLY_DEVICE_STATUS_SIZE)
    {
        return YG_PROTOCOL_BUFFER_TOO_SMALL;
    }
    YgProtocol_WriteU16Le(payload, status->epoch_valid ? 0U : 6U);
    YgProtocol_WriteU16Le(payload + 2U, 0U);
    payload[4] = status->role;
    payload[5] = status->boot_state;
    YgProtocol_WriteU16Le(payload + 6U, status->epoch_valid ? 1U : 0U);
    YgProtocol_WriteU32Le(payload + 8U, (uint32_t)status->boot_epoch);
    YgProtocol_WriteU32Le(payload + 12U, (uint32_t)(status->boot_epoch >> 32U));
    if (written != NULL)
    {
        *written = YG_PROTOCOL_READONLY_DEVICE_STATUS_SIZE;
    }
    return YG_PROTOCOL_OK;
}

yg_protocol_result_t
yg_protocol_readonly_decode_page_request(const yg_protocol_message_t *message,
                                         yg_protocol_readonly_page_request_t *request)
{
    const uint8_t *payload;

    if (message == NULL || request == NULL || message->payload == NULL)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    if (message->payload_length != YG_PROTOCOL_READONLY_PAGE_REQUEST_SIZE)
    {
        return YG_PROTOCOL_INVALID_LENGTH;
    }
    payload = message->payload;
    if (YgProtocol_ReadU16Le(payload + 10U) != 0U)
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    request->session_id = YgProtocol_ReadU32Le(payload);
    request->request_id = YgProtocol_ReadU32Le(payload + 4U);
    request->page = YgProtocol_ReadU16Le(payload + 8U);
    return YG_PROTOCOL_OK;
}

yg_protocol_result_t
yg_protocol_readonly_encode_info_page0(const yg_protocol_readonly_info_page0_t *page,
                                       uint8_t *payload,
                                       size_t capacity,
                                       size_t *written)
{
    if (page == NULL || payload == NULL)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    if (capacity < YG_PROTOCOL_READONLY_INFO_PAGE0_SIZE)
    {
        return YG_PROTOCOL_BUFFER_TOO_SMALL;
    }
    memcpy(payload, page->uid, sizeof(page->uid));
    YgProtocol_WriteU32Le(payload + 12U, page->product_id);
    YgProtocol_WriteU16Le(payload + 16U, page->hardware_revision);
    YgProtocol_WriteU16Le(payload + 18U, page->boot_api);
    YgProtocol_WriteU32Le(payload + 20U, page->application_version);
    YgProtocol_WriteU32Le(payload + 24U, page->boot_version);
    YgProtocol_WriteU32Le(payload + 28U, page->boot_id);
    if (written != NULL)
    {
        *written = YG_PROTOCOL_READONLY_INFO_PAGE0_SIZE;
    }
    return YG_PROTOCOL_OK;
}

yg_protocol_result_t
yg_protocol_readonly_encode_caps_page0(const yg_protocol_readonly_caps_page0_t *page,
                                       uint8_t *payload,
                                       size_t capacity,
                                       size_t *written)
{
    if (page == NULL || payload == NULL)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    if (capacity < YG_PROTOCOL_READONLY_CAPS_PAGE0_SIZE)
    {
        return YG_PROTOCOL_BUFFER_TOO_SMALL;
    }
    YgProtocol_WriteU32Le(payload, page->features);
    YgProtocol_WriteU16Le(payload + 4U, page->supported_modes);
    YgProtocol_WriteU16Le(payload + 6U, page->max_full_frame);
    YgProtocol_WriteU16Le(payload + 8U, page->max_block_data);
    YgProtocol_WriteU16Le(payload + 10U, page->write_alignment);
    YgProtocol_WriteU16Le(payload + 12U, page->default_watchdog_ms);
    YgProtocol_WriteU16Le(payload + 14U, page->max_watchdog_ms);
    if (written != NULL)
    {
        *written = YG_PROTOCOL_READONLY_CAPS_PAGE0_SIZE;
    }
    return YG_PROTOCOL_OK;
}

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
    yg_protocol_readonly_context_t *query = context;
    size_t written = 0U;

    if (query == NULL ||
        yg_protocol_readonly_decode_page_request(request, &page_request) != YG_PROTOCOL_OK ||
        page_request.page > 2U)
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    if (!query->device_description_available)
    {
        return provide_unavailable(payload, payload_length);
    }
    if (page_request.page != 0U)
    {
        memcpy(payload, query->maintenance_pages[page_request.page - 1U], 32U);
        *payload_length = 32U;
        return YG_PROTOCOL_OK;
    }
    if (yg_protocol_readonly_encode_info_page0(
            &query->info, payload, YG_PROTOCOL_MAX_PAYLOAD_SIZE, &written) != YG_PROTOCOL_OK)
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
    yg_protocol_readonly_context_t *query = context;
    size_t written = 0U;

    if (query == NULL ||
        yg_protocol_readonly_decode_page_request(request, &page_request) != YG_PROTOCOL_OK ||
        page_request.page != 0U)
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    if (!query->device_description_available)
    {
        return provide_unavailable(payload, payload_length);
    }
    if (yg_protocol_readonly_encode_caps_page0(
            &query->capabilities, payload, YG_PROTOCOL_MAX_PAYLOAD_SIZE, &written) !=
        YG_PROTOCOL_OK)
    {
        return YG_PROTOCOL_BUFFER_TOO_SMALL;
    }
    *payload_length = (uint32_t)written;
    return YG_PROTOCOL_OK;
}

yg_protocol_result_t yg_protocol_readonly_handle_device_status(void *context,
                                                               const yg_protocol_message_t *request,
                                                               yg_protocol_service_result_t *result)
{
    yg_protocol_readonly_context_t *query = context;
    if (!yg_protocol_service_request_valid(query, request, result))
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    if (request->payload_length != 0U)
    {
        yg_protocol_service_write_result(result, 3U, 0U);
        return YG_PROTOCOL_OK;
    }
    if (yg_protocol_readonly_encode_device_status(query->device_status_source,
                                                  result->response_payload,
                                                  sizeof(result->response_payload),
                                                  NULL) != YG_PROTOCOL_OK)
    {
        yg_protocol_service_write_result(result, 8U, 0U);
        return YG_PROTOCOL_OK;
    }
    result->response_payload_length = YG_PROTOCOL_READONLY_DEVICE_STATUS_SIZE;
    return YG_PROTOCOL_OK;
}
yg_protocol_result_t yg_protocol_readonly_handle_info(void *context,
                                                      const yg_protocol_message_t *request,
                                                      yg_protocol_service_result_t *result)
{
    yg_protocol_readonly_service_t service = {context, provide_info};
    return yg_protocol_readonly_handle(&service, request, result);
}

yg_protocol_result_t yg_protocol_readonly_handle_caps(void *context,
                                                      const yg_protocol_message_t *request,
                                                      yg_protocol_service_result_t *result)
{
    yg_protocol_readonly_service_t service = {context, provide_capabilities};
    return yg_protocol_readonly_handle(&service, request, result);
}
