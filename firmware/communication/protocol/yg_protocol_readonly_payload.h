#ifndef YG_PROTOCOL_READONLY_PAYLOAD_H
#define YG_PROTOCOL_READONLY_PAYLOAD_H

#include <stddef.h>
#include <stdint.h>

#include "yg_protocol_wire_types.h"

#define YG_PROTOCOL_READONLY_PAGE_REQUEST_SIZE 12U
#define YG_PROTOCOL_READONLY_INFO_PAGE0_SIZE 32U
#define YG_PROTOCOL_READONLY_CAPS_PAGE0_SIZE 16U
#define YG_PROTOCOL_READONLY_MOTOR_STATE_SIZE 34U

/** @brief GET_INFO/GET_CAPS 的页查询前缀，数值均为小端。 */
typedef struct
{
    uint32_t session_id;
    uint32_t request_id;
    uint16_t page;
} yg_protocol_readonly_page_request_t;

/** @brief GET_INFO page 0 的设备身份和版本字段。 */
typedef struct
{
    uint8_t uid[12];
    uint32_t product_id;
    uint16_t hardware_revision;
    uint16_t boot_api;
    uint32_t application_version;
    uint32_t boot_version;
    uint32_t boot_id;
} yg_protocol_readonly_info_page0_t;

/** @brief GET_CAPS page 0 的设备能力字段。 */
typedef struct
{
    uint32_t features;
    uint16_t supported_modes;
    uint16_t max_full_frame;
    uint16_t max_block_data;
    uint16_t write_alignment;
    uint16_t default_watchdog_ms;
    uint16_t max_watchdog_ms;
} yg_protocol_readonly_caps_page0_t;

/** @brief GET_MOTOR_STATE 的 34 字节同快照状态字段。 */
typedef struct
{
    uint32_t boot_id;
    uint32_t sample_counter;
    int32_t position_mrad;
    int32_t speed_mrad_s;
    int32_t iq_mA;
    uint32_t faults;
    uint16_t bus_mV;
    int16_t bus_mA;
    int16_t temperature_centi_c;
    uint8_t state;
    uint8_t mode;
    uint8_t last_applied_sequence;
    uint8_t valid_bits;
} yg_protocol_readonly_motor_state_t;

/**
 * @brief 解码 GET_INFO/GET_CAPS 的页查询请求。
 * @param message 已完成统一帧校验的查询消息。
 * @param request 输出的会话、请求号和页号。
 * @return 解码结果。
 * @note 只接受恰好 12 字节 payload，保留字段必须为 0。
 */
yg_protocol_result_t
yg_protocol_readonly_decode_page_request(const yg_protocol_message_t *message,
                                         yg_protocol_readonly_page_request_t *request);

/**
 * @brief 编码 GET_INFO page 0 内容。
 * @param page 输入的设备身份快照。
 * @param payload 输出缓冲区。
 * @param capacity 输出容量，至少 32 字节。
 * @param written 成功时写入字节数，可为 NULL。
 * @return 编码结果。
 */
yg_protocol_result_t
yg_protocol_readonly_encode_info_page0(const yg_protocol_readonly_info_page0_t *page,
                                       uint8_t *payload,
                                       size_t capacity,
                                       size_t *written);

/**
 * @brief 编码 GET_CAPS page 0 内容。
 * @param page 输入的能力快照。
 * @param payload 输出缓冲区。
 * @param capacity 输出容量，至少 16 字节。
 * @param written 成功时写入字节数，可为 NULL。
 * @return 编码结果。
 */
yg_protocol_result_t
yg_protocol_readonly_encode_caps_page0(const yg_protocol_readonly_caps_page0_t *page,
                                       uint8_t *payload,
                                       size_t capacity,
                                       size_t *written);

/**
 * @brief 编码 GET_MOTOR_STATE 的详细状态字段。
 * @param state 输入的同快照电机状态。
 * @param payload 输出缓冲区。
 * @param capacity 输出容量，至少 34 字节。
 * @param written 成功时写入字节数，可为 NULL。
 * @return 编码结果。
 */
yg_protocol_result_t
yg_protocol_readonly_encode_motor_state(const yg_protocol_readonly_motor_state_t *state,
                                        uint8_t *payload,
                                        size_t capacity,
                                        size_t *written);

#endif
