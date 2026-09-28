#ifndef YG_PROTOCOL_OWNER_ADAPTER_H
#define YG_PROTOCOL_OWNER_ADAPTER_H

#include "yg_protocol_motor.h"

/**
 * @brief 把协议生命周期值请求翻译为 APP 所有者业务请求。
 * @param context 保留组合上下文，当前不使用。
 * @param request 解码后的只读请求，返回后不保留。
 * @param reply 输出实际受理或执行结果，始终初始化。
 * @note 不接触硬件、Watch 邮箱或控制对象；101 低侧制动语义未定，继续不支持。
 */
void YgProtocolOwner_HandleLifecycle(void *context,
                                     const yg_protocol_motor_request_t *request,
                                     yg_protocol_service_reply_t *reply);
/**
 * @brief 绑定参数单值、异步保存和原始身份维护服务。
 * @return 链路已经初始化且绑定成功时返回 true。
 * @note 启动组合层调用；不读写 Flash、不分配对象副本。
 */
bool YgProtocolOwner_BindServices(void);

#endif
