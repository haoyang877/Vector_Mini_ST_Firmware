#ifndef COMM_HW_H
#define COMM_HW_H
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

typedef struct {
    uint32_t identifier;
    uint8_t length;
    bool extended, remote;
    uint8_t data[64];
} CommHwCanFrame;

/** Pop one received frame into a full-size bounded buffer, without waiting. */
bool comm_hw_can_receive(CommHwCanFrame *frame);
/** Submit optional CAN FD telemetry only when no transmission is queued.
 * Never retry/wait; reserves room for subsequent priority replies. */
bool comm_hw_can_try_send_status(uint16_t identifier, const uint8_t *data, size_t length);
/** @brief 空队列中提交4/8/16字节维护回复。
 * @param identifier 标准CAN ID。
 * @param data 只读回复字节。
 * @param length 4、8或16字节。
 * @return 入队成功为true。
 * @note 不等待；成功后用sent接口确认本次实际发送。
 */
bool comm_hw_can_try_send_reply(uint16_t identifier, const uint8_t *data, size_t length);
/** @brief 检查维护回复已完成且无待发送消息。
 * @return 实际完成为true。
 */
bool comm_hw_can_reply_sent(void);
#endif
