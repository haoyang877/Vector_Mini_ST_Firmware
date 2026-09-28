#ifndef YG_PROTOCOL_BOOT_H
#define YG_PROTOCOL_BOOT_H

#include "loader_entry.h"
#include "yg_protocol_wire_types.h"

#define YG_PROTOCOL_BOOT_TYPE_ENTER 56U
#define YG_PROTOCOL_BOOT_REQUEST_SIZE 8U
#define YG_PROTOCOL_BOOT_REPLY_SIZE 4U

/**
 * @brief 将56的非零boot64与公司头身份转换为共享Loader进入请求。
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

#endif
