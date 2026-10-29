#ifndef YG_PROTOCOL_BOOT_H
#define YG_PROTOCOL_BOOT_H

#include "loader_entry.h"
#include "yg_protocol_common.h"

#define YG_PROTOCOL_BOOT_TYPE_ENTER 56U
#define YG_PROTOCOL_BOOT_TYPE_CONFIRM 57U
#define YG_PROTOCOL_BOOT_REQUEST_SIZE 8U
#define YG_PROTOCOL_BOOT_REPLY_SIZE 4U

/**
 * @brief 将56的非零v2事务nonce与公司头身份转换为共享Loader进入请求。
 * @param message 已校验的单帧公司消息；主站授权与目标检查由组合层负责。
 * @param request 成功时写入共享值请求，失败保持原值。
 * @return 字段合法返回OK；参数、Type、长度或零启动代号返回对应错误。
 * @note 不调用停机、Flash或复位，不把请求中的boot当作可信当前启动身份。
 */
yg_protocol_result_t yg_protocol_boot_decode(const yg_protocol_message_t *message,
                                             BootEntryRequest *request);

/**
 * @brief 编码56的4B受理或拒绝回复，内部u64票据不进入报文。
 * @param reply APP唯一所有者的真实结果，READY须携带非零ack_ticket。
 * @param payload 输出缓冲，不得与reply重叠。
 * @param capacity 输出容量，至少4B。
 * @param written 成功时写入4，可为NULL。
 * @return 编码成功返回OK；结果或票据非法时不写输出。
 * @note READY只回复ACCEPTED，不宣称已复位；通信仍须绑定精确物理发送完成。
 */
yg_protocol_result_t yg_protocol_boot_encode_reply(const BootEntryReply *reply,
                                                   uint8_t *payload,
                                                   size_t capacity,
                                                   size_t *written);

/** @brief 升级请求与待确认票据；CAN 队列位置由链路层独立管理。 */
typedef struct
{
    BootEntryService boot_service;
    uint64_t boot_ticket;
    uint32_t service_now_ms;
    uint32_t boot_ready_ms;
    uint16_t boot_reply_sequence;
    uint8_t boot_reply_destination;
} yg_protocol_boot_context_t;
/**
 * @brief 处理本模块命令并生成回复内容；不直接操作收发队列或硬件。
 * @param context 本模块的上下文或服务绑定，由初始化时注入。
 * @param request 已完成外层校验的消息。
 * @param result 输出回复内容，实际发送由统一端点完成。
 * @return 解析处理结果；业务拒绝通过回复中的结果码报告。
 */
yg_protocol_result_t yg_protocol_boot_handle(void *context,
                                             const yg_protocol_message_t *request,
                                             yg_protocol_service_result_t *result);

/**
 * @brief 将匹配票据的实际发送结果交给升级服务；重复或过期结果无效。
 * @param boot 升级通信上下文。
 * @param ticket 已跟踪的发送票据。
 * @param success 是否实际发送成功。
 */
void yg_protocol_boot_complete(yg_protocol_boot_context_t *boot, uint64_t ticket, bool success);

#endif
