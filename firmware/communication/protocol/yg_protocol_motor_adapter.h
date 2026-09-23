#ifndef YG_PROTOCOL_MOTOR_ADAPTER_H
#define YG_PROTOCOL_MOTOR_ADAPTER_H

#include "motor_stop_service.h"
#include "yg_protocol_motor.h"

/* 通信侧适配器：STOP/失能接停机所有者；使能与控制在功率行为验证前拒绝。
 * 处理器不持有请求指针，也不访问硬件。 */

/** 组合层绑定对象；停止服务由业务所有者维护，生命周期必须覆盖所有 handler 调用。 */
typedef struct
{
    motor_stop_service_t *stop_service;
} yg_protocol_motor_adapter_t;

/**
 * @brief 处理一个线格式电机请求并填充内部结果，可直接作为 motor handler 使用。
 * @param context 组合层注入的 yg_protocol_motor_adapter_t，允许为空表示未接线。
 * @param request 只读请求；函数返回后不保留其指针，允许为空表示 API 误用。
 * @param reply 输出结果，始终被稳定初始化；允许为空时直接返回。
 * @note 前台串行调用，不阻塞、不分配；STOP/DISABLE 经所有者端口异步确认，
 *       未确认时返回 ACCEPTED 与 token，只有所有者确认功率输出禁止才返回 OK。
 */
void yg_protocol_motor_adapter_handle(void *context,
                                      const yg_protocol_motor_request_t *request,
                                      yg_protocol_service_reply_t *reply);

#endif
