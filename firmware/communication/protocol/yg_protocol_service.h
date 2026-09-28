#ifndef YG_PROTOCOL_SERVICE_H
#define YG_PROTOCOL_SERVICE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** @brief 内部业务执行结果；不是线上错误码，不得直接序列化枚举。 */
typedef enum
{
    YG_PROTOCOL_SERVICE_OK = 0,
    YG_PROTOCOL_SERVICE_ACCEPTED,
    YG_PROTOCOL_SERVICE_UNSUPPORTED,
    YG_PROTOCOL_SERVICE_INVALID_ARGUMENT,
    YG_PROTOCOL_SERVICE_BUSY,
    YG_PROTOCOL_SERVICE_DENIED,
    YG_PROTOCOL_SERVICE_FAILED,
    YG_PROTOCOL_SERVICE_OUT_OF_RANGE,
    YG_PROTOCOL_SERVICE_FAULT
} yg_protocol_service_status_t;

/** @brief 后端结果值对象；异步受理通过 token 关联后续查询，value 单位由具体服务定义。 */
typedef struct
{
    yg_protocol_service_status_t status;
    uint32_t token;
    uint32_t revision;
    int32_t value;
    uint16_t detail;
    /* 参数标量的实际类型与权限；其他服务保持0，不推断尚未实现的写权限。 */
    uint8_t value_type;
    uint8_t access;
} yg_protocol_service_reply_t;

/**
 * @brief 将内部业务结果显式映射为公司应用层的公共结果码。
 * @param status 服务结果，不允许直接序列化C枚举值。
 * @return 公共线路结果码；未知状态返回8（内部失败）。
 */
static inline uint16_t yg_protocol_service_result_code(yg_protocol_service_status_t status)
{
    switch (status)
    {
    case YG_PROTOCOL_SERVICE_OK:
        return 0U;
    case YG_PROTOCOL_SERVICE_ACCEPTED:
        return 1U;
    case YG_PROTOCOL_SERVICE_UNSUPPORTED:
        return 2U;
    case YG_PROTOCOL_SERVICE_INVALID_ARGUMENT:
        return 4U;
    case YG_PROTOCOL_SERVICE_BUSY:
        return 6U;
    case YG_PROTOCOL_SERVICE_DENIED:
        return 5U;
    case YG_PROTOCOL_SERVICE_OUT_OF_RANGE:
        return 11U;
    case YG_PROTOCOL_SERVICE_FAULT:
        return 17U;
    case YG_PROTOCOL_SERVICE_FAILED:
    default:
        return 8U;
    }
}

#endif
