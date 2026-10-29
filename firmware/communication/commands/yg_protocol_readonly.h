#ifndef YG_PROTOCOL_READONLY_H
#define YG_PROTOCOL_READONLY_H

#include <stddef.h>
#include <stdint.h>
#include "yg_protocol_common.h"

#define YG_PROTOCOL_READONLY_PAGE_REQUEST_SIZE 12U
#define YG_PROTOCOL_READONLY_INFO_PAGE0_SIZE 32U
#define YG_PROTOCOL_READONLY_CAPS_PAGE0_SIZE 16U
#define YG_PROTOCOL_READONLY_DEVICE_STATUS_SIZE 16U
#define YG_PROTOCOL_DEVICE_ROLE_APP 1U
#define YG_PROTOCOL_DEVICE_ROLE_LOADER 2U
#define YG_PROTOCOL_BOOT_STATE_UNAVAILABLE 0U
#define YG_PROTOCOL_BOOT_STATE_CONFIRMED_APP 1U
#define YG_PROTOCOL_BOOT_STATE_TRIAL_APP 2U
#define YG_PROTOCOL_BOOT_STATE_LOADER 3U

/** @brief Type3真实启动身份快照，boot_epoch不能由GET_INFO的u32计数补造。 */
typedef struct
{
    uint64_t boot_epoch;
    uint8_t role;
    uint8_t boot_state;
    bool epoch_valid;
} yg_protocol_readonly_device_status_t;

/**
 * @brief 显式编码Type3的16B角色/启动状态与完整u64身份。
 * @param status 产品所有者提供的稳定快照，epoch_valid与真实交接资格一致。
 * @param payload 输出缓冲，不得与status重叠。
 * @param capacity 输出容量，至少16B。
 * @param written 成功时写入16，可为NULL；失败保持原值。
 * @return 编码成功返回OK；角色、阶段或身份不一致时返回INVALID_FIELD，不写输出。
 * @note epoch无效时线路result6，epoch与有效位为0；不触发确认、停机或复位。
 */
yg_protocol_result_t
yg_protocol_readonly_encode_device_status(const yg_protocol_readonly_device_status_t *status,
                                          uint8_t *payload,
                                          size_t capacity,
                                          size_t *written);

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

/* 以下编号来自项目中的公司消息登记稿；最终以公司发布的注册表为准。 */
#define YG_PROTOCOL_READONLY_TYPE_GET_INFO 0x0001U
#define YG_PROTOCOL_READONLY_TYPE_GET_CAPS 0x0002U
#define YG_PROTOCOL_READONLY_TYPE_GET_DEVICE_STATUS 0x0003U
#define YG_PROTOCOL_READONLY_TYPE_GET_MOTOR_STATE 0x006CU
#define YG_PROTOCOL_READONLY_TYPE_MOTION_FEEDBACK 0x007CU

/**
 * @brief 只读信息提供者；把设备状态转换为协议 payload。
 * @param context 提供者上下文，由调用方拥有。
 * @param request 已完成校验的请求消息。
 * @param payload 输出 payload 缓冲区，最大 46 字节。
 * @param payload_length 输入为缓冲区容量，输出为实际长度。
 * @return 提供结果。
 * @note 回调不得访问 CAN、HAL 或修改电机控制状态。
 */
typedef yg_protocol_result_t (*yg_protocol_readonly_provider_t)(
    void *context,
    const yg_protocol_message_t *request,
    uint8_t *payload,
    uint32_t *payload_length);

/** 一条查询直接绑定一个提供者；CMD ID 只在命令表中登记。 */
typedef struct
{
    void *context;
    yg_protocol_readonly_provider_t provider;
} yg_protocol_readonly_service_t;

/**
 * @brief 处理一个只读查询。
 * @param context 指向 yg_protocol_readonly_service_t。
 * @param request 已完成协议校验的请求消息。
 * @param result 输出响应元数据和 payload。
 * @return 提供者处理结果。
 * @note 作为命令处理函数使用；每项 context 指向各自的提供者配置。
 */
yg_protocol_result_t yg_protocol_readonly_handle(void *context,
                                                 const yg_protocol_message_t *request,
                                                 yg_protocol_service_result_t *result);

/** @brief 查询命令拥有的身份信息及应用状态快照引用。 */
typedef struct
{
    yg_protocol_readonly_info_page0_t info;
    uint8_t maintenance_pages[2][32];
    yg_protocol_readonly_caps_page0_t capabilities;
    const yg_protocol_readonly_device_status_t *device_status_source;
    bool device_description_available;
} yg_protocol_readonly_context_t;
/**
 * @brief 处理本模块命令并生成回复内容；不直接操作收发队列或硬件。
 * @param context 本模块的上下文或服务绑定，由初始化时注入。
 * @param request 已完成外层校验的消息。
 * @param result 输出回复内容，实际发送由统一端点完成。
 * @return 解析处理结果；业务拒绝通过回复中的结果码报告。
 */
yg_protocol_result_t yg_protocol_readonly_handle_info(void *context,
                                                      const yg_protocol_message_t *request,
                                                      yg_protocol_service_result_t *result);
/**
 * @brief 处理本模块命令并生成回复内容；不直接操作收发队列或硬件。
 * @param context 本模块的上下文或服务绑定，由初始化时注入。
 * @param request 已完成外层校验的消息。
 * @param result 输出回复内容，实际发送由统一端点完成。
 * @return 解析处理结果；业务拒绝通过回复中的结果码报告。
 */
yg_protocol_result_t yg_protocol_readonly_handle_caps(void *context,
                                                      const yg_protocol_message_t *request,
                                                      yg_protocol_service_result_t *result);
/**
 * @brief 处理本模块命令并生成回复内容；不直接操作收发队列或硬件。
 * @param context 本模块的上下文或服务绑定，由初始化时注入。
 * @param request 已完成外层校验的消息。
 * @param result 输出回复内容，实际发送由统一端点完成。
 * @return 解析处理结果；业务拒绝通过回复中的结果码报告。
 */
yg_protocol_result_t yg_protocol_readonly_handle_device_status(
    void *context, const yg_protocol_message_t *request, yg_protocol_service_result_t *result);

#endif
