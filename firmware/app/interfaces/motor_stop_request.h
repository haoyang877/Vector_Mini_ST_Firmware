#ifndef MOTOR_STOP_REQUEST_H
#define MOTOR_STOP_REQUEST_H

#include <stdbool.h>
#include <stdint.h>

/* 只请求停机并核对关相；状态迁移和功率动作仍归运行所有者。 */

/** 停机业务结果；不是线上错误码，映射由通信适配器完成。 */
typedef enum
{
    MOTOR_STOP_RESULT_OK = 0,      /* 所有者已确认功率输出禁止 */
    MOTOR_STOP_RESULT_ACCEPTED,    /* 请求已受理，等待所有者异步确认 */
    MOTOR_STOP_RESULT_DENIED,      /* 所有者按当前状态拒绝停机 */
    MOTOR_STOP_RESULT_UNAVAILABLE, /* 未注入所有者或缺少停机确认通道 */
    MOTOR_STOP_RESULT_FAILED       /* 内部状态非法（API 误用） */
} motor_stop_result_t;

/**
 * @brief 所有者提供的停机请求与关相查询端口。
 * @note request_stop 与 is_power_disabled 都在调用方上下文同步执行，不得阻塞或分配；
 *       所有者负责与自身调度（如 2 kHz 慢拍）同步其内部状态上下文。两个回调都必须
 *       有效，否则停机无法确认，服务拒绝初始化而不是猜测成功。
 */
typedef struct
{
    bool (*request_stop)(void *owner_context, uint8_t source_node, uint16_t sequence);
    bool (*is_power_disabled)(void *owner_context);
} motor_stop_owner_port_t;

/** 调用方持有的停机服务状态；本模块不分配内存，端口与上下文均只借用作调用期引用。 */
typedef struct
{
    const motor_stop_owner_port_t *port;
    void *owner_context;
    bool confirmed;
    uint32_t token;
} motor_stop_service_t;

/**
 * @brief 绑定唯一所有者端口并把服务复位为未请求状态。
 * @param service 调用方持有的服务状态，不允许为空。
 * @param port 所有者窄端口；请求与确认回调缺一不可。
 * @param owner_context 透传给端口回调的所有者上下文，可为空表示无状态所有者。
 * @return 端口与回调完整返回 true；service 为空或缺少确认通道返回 false 且服务保持不可用。
 * @note 不访问硬件、不分配内存；返回 false 后调用 RequestStop 只能得到 UNAVAILABLE。
 */
bool MotorStopService_Init(motor_stop_service_t *service,
                           const motor_stop_owner_port_t *port,
                           void *owner_context);

/**
 * @brief 每次请求停机并复核关相，未确认时仅报告ACCEPTED。
 * @param service 由 Init 成功初始化的服务状态。
 * @param source_node 已校验请求来源；原样传给唯一所有者。
 * @param sequence 原始请求序号，每次有效请求都原样传给所有者。
 * @param token_out 输出请求 token；ACCEPTED/OK 时非零，未受理时为 0，不允许为空。
 * @return OK 表示所有者已确认功率输出禁止；ACCEPTED 表示已受理待确认；
 *         DENIED/UNAVAILABLE/FAILED 分别表示拒绝、缺少通道和 API 误用。
 * @note 每次有效调用都先转交来源和序号；在途重复调用复用同一 token 并轮询确认；已确认后仍逐次核查关断证据，
 *       所有者后来重新使能时提交新的停机请求与 token，不能复用旧成功；
 *       未确认的关断始终停留在 ACCEPTED，绝不报告 OK。同一服务只允许单一上下文串行调用。
 */
motor_stop_result_t MotorStopService_RequestStop(motor_stop_service_t *service,
                                                 uint8_t source_node,
                                                 uint16_t sequence,
                                                 uint32_t *token_out);

#endif
