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
    YG_PROTOCOL_SERVICE_FAILED
} yg_protocol_service_status_t;

/** @brief 后端结果值对象；异步受理通过 token 关联后续查询，value 单位由具体服务定义。 */
typedef struct
{
    yg_protocol_service_status_t status;
    uint32_t token;
    uint32_t revision;
    int32_t value;
    uint16_t detail;
} yg_protocol_service_reply_t;

#endif
