/* 完整消息到业务服务的有限路由；本模块不访问硬件或电机全局状态。 */
#include "yg_protocol_router.h"

yg_protocol_result_t yg_protocol_router_init(yg_protocol_router_t *router,
                                             const yg_protocol_message_registry_t *registry,
                                             const yg_protocol_route_t *routes,
                                             size_t route_count)
{
    if (router == NULL || registry == NULL || (route_count > 0U && routes == NULL))
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    for (size_t index = 0U; index < route_count; ++index)
    {
        if (routes[index].handler == NULL)
        {
            return YG_PROTOCOL_INVALID_ARGUMENT;
        }
        if (yg_protocol_message_registry_find(registry, routes[index].message_type) == NULL)
        {
            return YG_PROTOCOL_INVALID_MESSAGE_TYPE;
        }
        for (size_t next = index + 1U; next < route_count; ++next)
        {
            if (routes[index].message_type == routes[next].message_type)
            {
                return YG_PROTOCOL_INVALID_MESSAGE_TYPE;
            }
        }
    }

    router->registry = registry;
    router->routes = routes;
    router->route_count = route_count;
    return YG_PROTOCOL_OK;
}

yg_protocol_result_t yg_protocol_router_handle(const yg_protocol_router_t *router,
                                               const yg_protocol_message_t *message,
                                               yg_protocol_service_result_t *result)
{
    if (router == NULL || message == NULL || result == NULL)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }

    result->result = YG_PROTOCOL_ROUTE_NOT_FOUND;
    result->applied_sequence = message->sequence;
    result->response_type = message->message_type;
    result->response_flags = YG_PROTOCOL_FLAGS_RESPONSE;
    result->response_source_node = message->destination_node;
    result->response_destination_node = message->source_node;
    result->response_reserved = 0U;
    result->response_payload_length = 0U;
    if (yg_protocol_message_registry_find(router->registry, message->message_type) == NULL)
    {
        return YG_PROTOCOL_ROUTE_NOT_FOUND;
    }
    for (size_t index = 0U; index < router->route_count; ++index)
    {
        if (router->routes[index].message_type == message->message_type)
        {
            result->result =
                router->routes[index].handler(router->routes[index].context, message, result);
            return result->result;
        }
    }
    return YG_PROTOCOL_ROUTE_NOT_FOUND;
}

yg_protocol_result_t
yg_protocol_router_build_response(const yg_protocol_message_t *request,
                                  const yg_protocol_service_result_t *service_result,
                                  yg_protocol_message_t *response)
{
    if (request == NULL || service_result == NULL || response == NULL)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    if (service_result->result != YG_PROTOCOL_OK)
    {
        return service_result->result;
    }
    if (service_result->response_source_node == YG_PROTOCOL_BROADCAST_NODE_ID ||
        service_result->response_destination_node == YG_PROTOCOL_BROADCAST_NODE_ID)
    {
        return YG_PROTOCOL_INVALID_NODE_ID;
    }
    if (service_result->response_payload_length > YG_PROTOCOL_MAX_PAYLOAD_SIZE)
    {
        return YG_PROTOCOL_INVALID_LENGTH;
    }
    response->version = request->version;
    response->flags = service_result->response_flags;
    response->source_node = service_result->response_source_node;
    response->destination_node = service_result->response_destination_node;
    response->message_type = service_result->response_type;
    response->sequence = service_result->applied_sequence;
    response->payload_length = service_result->response_payload_length;
    response->reserved = service_result->response_reserved;
    response->payload = service_result->response_payload;
    return YG_PROTOCOL_OK;
}
