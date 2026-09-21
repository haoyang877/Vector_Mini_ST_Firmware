/* 消息类型的静态登记和查找；不承担业务执行和硬件访问。 */
#include "yg_protocol_message_registry.h"

yg_protocol_result_t
yg_protocol_message_registry_init(yg_protocol_message_registry_t *registry,
                                  const yg_protocol_message_descriptor_t *descriptors,
                                  size_t descriptor_count)
{
    if (registry == NULL || (descriptor_count > 0U && descriptors == NULL))
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    for (size_t index = 0U; index < descriptor_count; ++index)
    {
        for (size_t next = index + 1U; next < descriptor_count; ++next)
        {
            if (descriptors[index].message_type == descriptors[next].message_type)
            {
                return YG_PROTOCOL_INVALID_MESSAGE_TYPE;
            }
        }
    }
    registry->descriptors = descriptors;
    registry->descriptor_count = descriptor_count;
    return YG_PROTOCOL_OK;
}

const yg_protocol_message_descriptor_t *
yg_protocol_message_registry_find(const yg_protocol_message_registry_t *registry,
                                  uint16_t message_type)
{
    if (registry == NULL)
    {
        return NULL;
    }
    for (size_t index = 0U; index < registry->descriptor_count; ++index)
    {
        if (registry->descriptors[index].message_type == message_type)
        {
            return &registry->descriptors[index];
        }
    }
    return NULL;
}
