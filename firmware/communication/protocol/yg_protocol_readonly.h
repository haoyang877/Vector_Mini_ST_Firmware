#ifndef YG_PROTOCOL_READONLY_H
#define YG_PROTOCOL_READONLY_H

#include "yg_protocol_router.h"

/* 以下编号来自项目中的公司消息登记稿；最终以公司发布的注册表为准。 */
#define YG_PROTOCOL_READONLY_TYPE_GET_INFO 0x0001U
#define YG_PROTOCOL_READONLY_TYPE_GET_CAPS 0x0002U
#define YG_PROTOCOL_READONLY_TYPE_GET_MOTOR_STATE 0x006CU
#define YG_PROTOCOL_READONLY_TYPE_MOTION_FEEDBACK 0x007CU
#define YG_PROTOCOL_READONLY_TYPE_DISABLED 0xFFFFU

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

/** @brief 只读服务的四类信息提供者配置。未配置的 type 不可路由。 */
typedef struct
{
    void *context;
    uint16_t protocol_info_type;
    uint16_t device_info_type;
    uint16_t capabilities_type;
    uint16_t motor_status_type;
    yg_protocol_readonly_provider_t protocol_info;
    yg_protocol_readonly_provider_t device_info;
    yg_protocol_readonly_provider_t capabilities;
    yg_protocol_readonly_provider_t motor_status;
} yg_protocol_readonly_service_t;

/**
 * @brief 初始化只读服务配置。
 * @param service 输出服务对象。
 * @param configuration 提供者配置；配置内容由调用方保持有效。
 * @return 初始化结果。
 */
yg_protocol_result_t yg_protocol_readonly_init(yg_protocol_readonly_service_t *service,
                                               const yg_protocol_readonly_service_t *configuration);

/**
 * @brief 处理一个只读查询路由。
 * @param context 指向 yg_protocol_readonly_service_t。
 * @param request 已完成协议校验的请求消息。
 * @param result 输出响应元数据和 payload。
 * @return 提供者或路由处理结果。
 * @note 该函数可直接作为多个只读 type 的路由 handler。
 */
yg_protocol_result_t yg_protocol_readonly_handle(void *context,
                                                 const yg_protocol_message_t *request,
                                                 yg_protocol_service_result_t *result);

#endif
