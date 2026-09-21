#ifndef YG_PROTOCOL_PARAMETER_ADAPTER_H
#define YG_PROTOCOL_PARAMETER_ADAPTER_H

#include "yg_protocol_parameter.h"
#include "parameter_read_service.h"

/* 通信适配器：把 parameter 内部请求转成 parameter_read_service 的调用。
 * 只实现 READ；WRITE/SAVE/RESTORE_DEFAULTS 一律 UNSUPPORTED。
 * 本模块不读取可变全局量，只使用调用方在 context 中提供的只读副本。 */

/** 适配器上下文；由组合层初始化并覆盖 handler 的调用生命周期。 */
typedef struct
{
    ParameterReadSource source;
} yg_protocol_parameter_adapter_t;

/**
 * @brief 实现 yg_protocol_parameter_adapter_t 的 READ 处理器。
 * @param context 调用方持有的 yg_protocol_parameter_adapter_t，生命周期覆盖调用。
 * @param request 只读内部请求；返回后不可保留其指针。
 * @param reply 输出内部结果，成功时 value 的单位由 parameter_id 决定。
 * @note 前台串行调用，不分配、不阻塞、不访问硬件；写操作直接返回 UNSUPPORTED，
 *       未就绪映射为 BUSY，非法数值映射为 FAILED，绝不以空操作冒充成功。
 */
void yg_protocol_parameter_adapter_handle(void *context,
                                          const yg_protocol_parameter_request_t *request,
                                          yg_protocol_service_reply_t *reply);

#endif
