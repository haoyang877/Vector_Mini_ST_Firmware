#ifndef YG_PROTOCOL_ROUTER_H
#define YG_PROTOCOL_ROUTER_H

#include <stddef.h>
#include <stdint.h>

#include "yg_protocol_message_registry.h"
#include "yg_protocol_wire_types.h"

typedef yg_protocol_result_t (*yg_protocol_route_handler_t)(void *context,
                                                            const yg_protocol_message_t *message,
                                                            yg_protocol_service_result_t *result);

/** @brief 一条按消息 type 匹配的业务路由。 */
typedef struct
{
    uint16_t message_type;
    yg_protocol_route_handler_t handler;
    void *context;
} yg_protocol_route_t;

/** @brief 路由器视图；路由表由调用方持有，不能在运行期修改。 */
typedef struct
{
    const yg_protocol_message_registry_t *registry;
    const yg_protocol_route_t *routes;
    size_t route_count;
} yg_protocol_router_t;

/**
 * @brief 检查并绑定一张协议消息路由表。
 * @param router 输出路由器对象。
 * @param registry 已登记的消息类型表。
 * @param routes 调用方持有的静态路由表。
 * @param route_count 路由数量。
 * @return 初始化结果。
 * @note 路由器不复制路由表；表和处理函数必须在路由器使用期间保持有效。
 */
yg_protocol_result_t yg_protocol_router_init(yg_protocol_router_t *router,
                                             const yg_protocol_message_registry_t *registry,
                                             const yg_protocol_route_t *routes,
                                             size_t route_count);

/**
 * @brief 按消息 type 路由一条已完成协议校验的消息。
 * @param router 已初始化的路由器。
 * @param message 已解码消息；payload 仍由调用方拥有。
 * @param result 输出服务结果。
 * @return 路由或服务处理结果。
 * @note 本函数只选择并调用服务，不直接访问电机、HAL 或持久化存储。
 */
yg_protocol_result_t yg_protocol_router_handle(const yg_protocol_router_t *router,
                                               const yg_protocol_message_t *message,
                                               yg_protocol_service_result_t *result);

/**
 * @brief 根据服务结果构造可再次编码的响应消息视图。
 * @param request 原始请求消息。
 * @param service_result 已由路由服务填充的结果。
 * @param response 输出响应消息视图；payload 指向 service_result 内部缓冲区。
 * @return 构造结果。
 * @note 返回视图不拥有 payload；service_result 在响应编码完成前必须保持有效。
 */
yg_protocol_result_t
yg_protocol_router_build_response(const yg_protocol_message_t *request,
                                  const yg_protocol_service_result_t *service_result,
                                  yg_protocol_message_t *response);

#endif
