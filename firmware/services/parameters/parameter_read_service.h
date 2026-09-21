#ifndef PARAMETER_READ_SERVICE_H
#define PARAMETER_READ_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "foc_param.h"

/* 参数只读业务服务：把协议评审稿的参数 ID 映射到调用方提供的活动参数只读副本。
 * 本模块不读取可变全局量、不访问 Flash、不依赖任何通信协议头，也不写任何参数。 */

/** 只读查询结果码；未知 ID、数据未就绪与非法数值必须分开报告。 */
typedef enum
{
    PARAMETER_READ_OK = 0,
    PARAMETER_READ_UNSUPPORTED,
    PARAMETER_READ_NOT_READY,
    PARAMETER_READ_INVALID_VALUE,
    PARAMETER_READ_INVALID_ARGUMENT,
} ParameterReadStatus;

/* 与 READ_PARAM 契约的 value_type 对齐：1=i32，2=u32。 */
#define PARAMETER_READ_VALUE_TYPE_I32 1U
#define PARAMETER_READ_VALUE_TYPE_U32 2U

/* 参数 ID 取自协议评审稿 motor_protocol_v1.md §8.4 最小注册表；
 * 只登记本轮能明确映射到 InterfaceParam_TypeDef 的项，其余留作缺口。 */
#define PARAMETER_READ_ID_ACTIVE_REVISION 0U
#define PARAMETER_READ_ID_MAX_SPEED_RAD_S 1U
#define PARAMETER_READ_ID_MAX_CURRENT_A 2U

/** 调用方提供的只读副本；本模块只读取，不保存指针。 */
typedef struct
{
    InterfaceParam_TypeDef values;
    /* 参数所有者提供的活动配置版本计数，不是 Flash schema 版本。 */
    uint32_t config_revision;
    /* false 表示尚无一致快照，任何读取都必须返回 NOT_READY。 */
    bool snapshot_ready;
} ParameterReadSource;

/** 查询结果；value 的单位与 value_type 由 parameter_id 决定。 */
typedef struct
{
    uint8_t value_type;
    int32_t value;
} ParameterReadResult;

/**
 * @brief 读取一个已登记参数的活动值。
 * @param source 调用方提供的只读参数副本与活动配置版本，只读。
 * @param parameter_id 协议评审稿参数 ID。
 * @param result 输出结果，任何失败都保持全零；缓冲区归调用方所有。
 * @return 成功返回 OK；ID 未登记返回 UNSUPPORTED；快照未就绪返回 NOT_READY；
 *         字段非有限、非正或缩放后越界返回 INVALID_VALUE；空指针返回 INVALID_ARGUMENT。
 * @note 只读纯函数，不分配、不阻塞、不访问硬件；可从前台串行调用。
 */
ParameterReadStatus ParameterRead_Get(const ParameterReadSource *source,
                                      uint16_t parameter_id,
                                      ParameterReadResult *result);

#endif
