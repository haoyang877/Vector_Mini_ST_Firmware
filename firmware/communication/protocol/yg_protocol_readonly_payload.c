/* 只读查询 payload 的显式小端编解码，不访问设备状态或硬件。 */
#include "yg_protocol_readonly_payload.h"

#include <string.h>

static uint16_t read_u16(const uint8_t *bytes)
{
    return (uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8U);
}

static uint32_t read_u32(const uint8_t *bytes)
{
    return (uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8U) | ((uint32_t)bytes[2] << 16U) |
           ((uint32_t)bytes[3] << 24U);
}

static void write_u16(uint8_t *bytes, uint16_t value)
{
    bytes[0] = (uint8_t)value;
    bytes[1] = (uint8_t)(value >> 8U);
}

static void write_u32(uint8_t *bytes, uint32_t value)
{
    bytes[0] = (uint8_t)value;
    bytes[1] = (uint8_t)(value >> 8U);
    bytes[2] = (uint8_t)(value >> 16U);
    bytes[3] = (uint8_t)(value >> 24U);
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
    if (read_u16(payload + 10U) != 0U)
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    request->session_id = read_u32(payload);
    request->request_id = read_u32(payload + 4U);
    request->page = read_u16(payload + 8U);
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
    write_u32(payload + 12U, page->product_id);
    write_u16(payload + 16U, page->hardware_revision);
    write_u16(payload + 18U, page->boot_api);
    write_u32(payload + 20U, page->application_version);
    write_u32(payload + 24U, page->boot_version);
    write_u32(payload + 28U, page->boot_id);
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
    write_u32(payload, page->features);
    write_u16(payload + 4U, page->supported_modes);
    write_u16(payload + 6U, page->max_full_frame);
    write_u16(payload + 8U, page->max_block_data);
    write_u16(payload + 10U, page->write_alignment);
    write_u16(payload + 12U, page->default_watchdog_ms);
    write_u16(payload + 14U, page->max_watchdog_ms);
    if (written != NULL)
    {
        *written = YG_PROTOCOL_READONLY_CAPS_PAGE0_SIZE;
    }
    return YG_PROTOCOL_OK;
}

yg_protocol_result_t
yg_protocol_readonly_encode_motor_state(const yg_protocol_readonly_motor_state_t *state,
                                        uint8_t *payload,
                                        size_t capacity,
                                        size_t *written)
{
    if (state == NULL || payload == NULL)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    if (capacity < YG_PROTOCOL_READONLY_MOTOR_STATE_SIZE)
    {
        return YG_PROTOCOL_BUFFER_TOO_SMALL;
    }
    write_u32(payload, state->boot_id);
    write_u32(payload + 4U, state->sample_counter);
    write_u32(payload + 8U, (uint32_t)state->position_mrad);
    write_u32(payload + 12U, (uint32_t)state->speed_mrad_s);
    write_u32(payload + 16U, (uint32_t)state->iq_mA);
    write_u32(payload + 20U, state->faults);
    write_u16(payload + 24U, state->bus_mV);
    write_u16(payload + 26U, (uint16_t)state->temperature_centi_c);
    payload[28] = state->state;
    payload[29] = state->mode;
    payload[30] = state->last_applied_sequence;
    payload[31] = state->valid_bits;
    if (written != NULL)
    {
        *written = YG_PROTOCOL_READONLY_MOTOR_STATE_SIZE;
    }
    return YG_PROTOCOL_OK;
}
