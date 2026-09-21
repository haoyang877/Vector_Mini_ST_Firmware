/* 只读查询服务适配器；只负责 provider 选择和有界响应 payload。 */
#include "yg_protocol_readonly.h"

#include "yg_protocol_motor.h"

static yg_protocol_readonly_provider_t
select_provider(const yg_protocol_readonly_service_t *service, uint16_t message_type)
{
    if (message_type == YG_PROTOCOL_READONLY_TYPE_DISABLED)
    {
        return NULL;
    }
    if (service->protocol_info_type == message_type)
    {
        return service->protocol_info;
    }
    if (service->device_info_type == message_type)
    {
        return service->device_info;
    }
    if (service->capabilities_type == message_type)
    {
        return service->capabilities;
    }
    if (service->motor_status_type == message_type)
    {
        return service->motor_status;
    }
    return NULL;
}

static bool type_is_enabled(uint16_t message_type)
{
    return message_type != YG_PROTOCOL_READONLY_TYPE_DISABLED;
}

yg_protocol_result_t yg_protocol_readonly_registry_init(yg_protocol_message_registry_t *registry)
{
    static const yg_protocol_message_descriptor_t descriptors[] = {
        {YG_PROTOCOL_READONLY_TYPE_GET_INFO, true, false},
        {YG_PROTOCOL_READONLY_TYPE_GET_CAPS, false, false},
        {YG_PROTOCOL_READONLY_TYPE_GET_MOTOR_STATE, false, false},
        {YG_PROTOCOL_READONLY_TYPE_MOTION_FEEDBACK, false, true},
        {YG_PROTOCOL_MOTOR_TYPE_STOP, false, false},
        {YG_PROTOCOL_MOTOR_TYPE_DISABLE, false, false},
    };

    if (registry == NULL)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    return yg_protocol_message_registry_init(
        registry, descriptors, sizeof(descriptors) / sizeof(descriptors[0]));
}

yg_protocol_result_t yg_protocol_readonly_init(yg_protocol_readonly_service_t *service,
                                               const yg_protocol_readonly_service_t *configuration)
{
    if (service == NULL || configuration == NULL)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    if ((type_is_enabled(configuration->protocol_info_type) &&
         configuration->protocol_info == NULL) ||
        (type_is_enabled(configuration->device_info_type) && configuration->device_info == NULL) ||
        (type_is_enabled(configuration->capabilities_type) &&
         configuration->capabilities == NULL) ||
        (type_is_enabled(configuration->motor_status_type) && configuration->motor_status == NULL))
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    if ((type_is_enabled(configuration->protocol_info_type) &&
         configuration->protocol_info_type == configuration->device_info_type) ||
        (type_is_enabled(configuration->protocol_info_type) &&
         configuration->protocol_info_type == configuration->capabilities_type) ||
        (type_is_enabled(configuration->protocol_info_type) &&
         configuration->protocol_info_type == configuration->motor_status_type) ||
        (type_is_enabled(configuration->device_info_type) &&
         configuration->device_info_type == configuration->capabilities_type) ||
        (type_is_enabled(configuration->device_info_type) &&
         configuration->device_info_type == configuration->motor_status_type) ||
        (type_is_enabled(configuration->capabilities_type) &&
         configuration->capabilities_type == configuration->motor_status_type))
    {
        return YG_PROTOCOL_INVALID_MESSAGE_TYPE;
    }
    *service = *configuration;
    return YG_PROTOCOL_OK;
}

yg_protocol_result_t yg_protocol_readonly_handle(void *context,
                                                 const yg_protocol_message_t *request,
                                                 yg_protocol_service_result_t *result)
{
    yg_protocol_readonly_service_t *service = (yg_protocol_readonly_service_t *)context;
    yg_protocol_readonly_provider_t provider;
    uint32_t payload_length = YG_PROTOCOL_MAX_PAYLOAD_SIZE;
    yg_protocol_result_t provider_result;

    if (service == NULL || request == NULL || result == NULL)
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
    provider = select_provider(service, request->message_type);
    if (provider == NULL)
    {
        return YG_PROTOCOL_ROUTE_NOT_FOUND;
    }
    provider_result =
        provider(service->context, request, result->response_payload, &payload_length);
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
