#ifndef YG_PROTOCOL_MESSAGE_REGISTRY_H
#define YG_PROTOCOL_MESSAGE_REGISTRY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "yg_protocol_wire_types.h"

/** @brief 一条消息类型的静态属性；表由产品协议登记文件生成或维护。 */
typedef struct
{
    uint16_t message_type;
    bool allow_fragment;
    bool periodic;
} yg_protocol_message_descriptor_t;

/** @brief 消息注册表视图；不拥有 descriptors 存储。 */
typedef struct
{
    const yg_protocol_message_descriptor_t *descriptors;
    size_t descriptor_count;
} yg_protocol_message_registry_t;

/**
 * @brief 检查并绑定一张消息类型注册表。
 * @param registry 输出注册表对象。
 * @param descriptors 调用方拥有的静态描述符数组。
 * @param descriptor_count 描述符数量。
 * @return 初始化结果。
 * @note 注册表不复制数组；数组在使用期间必须保持有效且不可变。
 */
yg_protocol_result_t
yg_protocol_message_registry_init(yg_protocol_message_registry_t *registry,
                                  const yg_protocol_message_descriptor_t *descriptors,
                                  size_t descriptor_count);

/**
 * @brief 按消息类型查找描述符。
 * @param registry 已初始化的注册表。
 * @param message_type 消息类型。
 * @return 找到返回描述符指针；未找到返回 NULL。
 * @note 返回指针由注册表拥有，调用方不能修改。
 */
const yg_protocol_message_descriptor_t *
yg_protocol_message_registry_find(const yg_protocol_message_registry_t *registry,
                                  uint16_t message_type);

#endif
