#ifndef MOTOR_STOP_SERVICE_H
#define MOTOR_STOP_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

/* 平台无关的 STOP/DISABLE 业务接口：只向“唯一运行所有者”发出停机请求并跟踪其停机确认，
 * 不直接写 MotorControl，也不自行实现第二套生命周期状态机。运行所有者仍是唯一调用
 * AppLifecycle_Step 的一侧；通信侧只能经本接口的窄端口请求停止，不得自行驱动同一实例。 */

/** 停机业务结果；不是线上错误码，映射由通信适配器完成。 */
typedef enum
{
    MOTOR_STOP_RESULT_OK = 0,      /* 所有者已确认功率输出禁止 */
    MOTOR_STOP_RESULT_ACCEPTED,    /* 请求已受理，等待所有者异步确认 */
    MOTOR_STOP_RESULT_DENIED,      /* 所有者按当前状态拒绝停机 */
    MOTOR_STOP_RESULT_UNAVAILABLE, /* 未注入所有者或缺少停机确认通道 */
    MOTOR_STOP_RESULT_FAILED       /* 内部状态非法（API 误用） */
} motor_stop_result_t;

/** 唯一所有者窄端口的请求结果；确认只由 is_power_disabled 提供。 */
typedef enum
{
    MOTOR_STOP_OWNER_ACCEPTED = 0, /* 所有者已受理，稍后达到功率输出禁止 */
    MOTOR_STOP_OWNER_DENIED        /* 所有者当前状态拒绝停机 */
} motor_stop_owner_result_t;

/**
 * @brief 停机请求相位；仅由 MotorStopService 内部读写。
 */
typedef enum
{
    MOTOR_STOP_PHASE_IDLE = 0,
    MOTOR_STOP_PHASE_PENDING,
    MOTOR_STOP_PHASE_CONFIRMED
} motor_stop_phase_t;

/**
 * @brief 唯一运行所有者对外暴露的唯一窄端口。
 * @note request_stop 与 is_power_disabled 都在调用方上下文同步执行，不得阻塞或分配；
 *       所有者负责与自身调度（如 2 kHz 慢拍）同步其内部 AppLifecycle。两个回调都必须
 *       有效，否则停机无法确认，服务拒绝初始化而不是猜测成功。
 */
typedef struct
{
    motor_stop_owner_result_t (*request_stop)(void *owner_context);
    bool (*is_power_disabled)(void *owner_context);
} motor_stop_owner_port_t;

/** 调用方持有的停机服务状态；本模块不分配内存，端口与上下文均只借用作调用期引用。 */
typedef struct
{
    const motor_stop_owner_port_t *port;
    void *owner_context;
    motor_stop_phase_t phase;
    uint32_t token;
    uint32_t next_token;
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
 * @brief 幂等地请求停机，并在所有者确认后报告完成。
 * @param service 由 Init 成功初始化的服务状态。
 * @param token_out 输出请求 token；ACCEPTED/OK 时非零，未受理时为 0，不允许为空。
 * @return OK 表示所有者已确认功率输出禁止；ACCEPTED 表示已受理待确认；
 *         DENIED/UNAVAILABLE/FAILED 分别表示拒绝、缺少通道和 API 误用。
 * @note 在途重复调用复用同一 token 并轮询确认；已确认后仍逐次核查关断证据，
 *       所有者后来重新使能时提交新的停机请求与 token，不能复用旧成功；
 *       未确认的关断始终停留在 ACCEPTED，绝不报告 OK。同一服务只允许单一上下文串行调用。
 */
motor_stop_result_t MotorStopService_RequestStop(motor_stop_service_t *service,
                                                 uint32_t *token_out);

#endif
