/* 消息类型的静态登记和查找；不承担业务执行和硬件访问。 */
#include "yg_protocol_message_registry.h"

#include "yg_protocol_job.h"
#include "yg_protocol_boot.h"
#include "yg_protocol_motor.h"
#include "yg_protocol_parameter.h"
#include "yg_protocol_readonly.h"

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

yg_protocol_result_t yg_protocol_product_registry_init(yg_protocol_message_registry_t *registry)
{
    static const yg_protocol_message_descriptor_t descriptors[] = {
        {YG_PROTOCOL_READONLY_TYPE_GET_INFO, true, false},
        {YG_PROTOCOL_READONLY_TYPE_GET_CAPS, false, false},
        {YG_PROTOCOL_READONLY_TYPE_GET_MOTOR_STATE, false, false},
        {YG_PROTOCOL_READONLY_TYPE_MOTION_FEEDBACK, false, true},
        {YG_PROTOCOL_MOTOR_TYPE_ENABLE, false, false},
        {YG_PROTOCOL_MOTOR_TYPE_CLEAR_FAULT, false, false},
        {YG_PROTOCOL_MOTOR_TYPE_STOP, false, false},
        {YG_PROTOCOL_MOTOR_TYPE_CONTROL, false, false},
        {YG_PROTOCOL_PARAMETER_TYPE_READ, false, false},
        {YG_PROTOCOL_PARAMETER_TYPE_WRITE, false, false},
        {YG_PROTOCOL_PARAMETER_TYPE_SAVE, false, false},
        {YG_PROTOCOL_JOB_TYPE_START, false, false},
        {YG_PROTOCOL_JOB_TYPE_STATUS, false, false},
        {YG_PROTOCOL_JOB_TYPE_CANCEL, false, false},
        /* 描述不开放业务；56只有两个真实回调绑定后才进入产品路由。 */
        {YG_PROTOCOL_BOOT_TYPE_ENTER, false, false},
    };
    return yg_protocol_message_registry_init(
        registry, descriptors, sizeof(descriptors) / sizeof(descriptors[0]));
}
