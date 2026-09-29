/* 只读查询 payload 的显式小端编解码，不访问设备状态或硬件。 */
#include "yg_protocol_readonly_payload.h"
#include "yg_protocol_byte_order.h"

#include <string.h>

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
