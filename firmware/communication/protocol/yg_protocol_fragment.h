#ifndef YG_PROTOCOL_FRAGMENT_H
#define YG_PROTOCOL_FRAGMENT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "yg_protocol_wire_types.h"

/* 12.1：首片起整条重组绝对上限，新片不得延长该时限。 */
#define YG_PROTOCOL_FRAGMENT_ABSOLUTE_TIMEOUT_MS 2000U

/** @brief 有界分片重组状态；storage 由调用方提供，单会话，不分配内存。 */
typedef struct
{
    uint8_t *storage;
    size_t capacity;
    size_t length;
    uint8_t version;
    uint8_t base_flags;
    uint8_t reserved;
    uint8_t source_node;
    uint8_t destination_node;
    uint16_t message_type;
    uint16_t next_sequence;
    uint32_t first_activity_ms;
    uint32_t last_activity_ms;
    bool active;
} yg_protocol_fragment_state_t;

/**
 * @brief 初始化分片重组状态。
 * @param state 输出状态对象。
 * @param storage 调用方提供的固定重组缓冲区。
 * @param capacity 缓冲区容量，不能为零。
 * @return 初始化结果。
 * @note 不分配内存；重组消息的最大长度由 capacity 决定。
 */
yg_protocol_result_t
yg_protocol_fragment_init(yg_protocol_fragment_state_t *state, uint8_t *storage, size_t capacity);

/**
 * @brief 清除当前分片会话并使其不可被后续片段续接。
 * @param state 待清除的重组状态。
 * @return 清除结果。
 * @note 不清除 storage 内容，只清除会话所有权、长度和时限基准。
 */
yg_protocol_result_t yg_protocol_fragment_reset(yg_protocol_fragment_state_t *state);

/**
 * @brief 接收一个已完成 CRC 校验的消息片段，按 12.1 规则重组单条逻辑消息。
 * @param state 重组状态。
 * @param message 输入片段；payload 由调用方拥有。
 * @param now_ms 当前单调时间，单位 ms；允许回绕，差值按 uint32 模运算。
 * @param timeout_ms 调用方片间超时，单位 ms，不能为零。
 * @param completed 输出完整消息；任何错误时内容保持不变。
 * @return 完整消息返回 YG_PROTOCOL_OK；等待后续片段返回 FRAGMENT_IN_PROGRESS；
 *         超时返回 FRAGMENT_TIMEOUT；跳号/无首尾/内容冲突返回 FRAGMENT_STATE_ERROR 或
 *         FRAGMENT_SEQUENCE_ERROR；片长非法返回 YG_PROTOCOL_INVALID_LENGTH。
 * @note 分片规则：首片与中片 payload 恰为 46B，尾片 1..46B，首片 seq 必须为 0；
 *       各片的 version、非分片标志、reserved 及 src/dst/type 必须一致；
 *       ACK_REQ 例外：仅请求尾片可设置，重组输出保留该尾片标志。
 * @note 首片起 2000ms 绝对超时与调用方片间超时同时生效，恰好到达 deadline 即超时。
 * @note 上一片元数据与 payload 完全一致的重复片不累加也不再刷新片间超时；单帧直接交付，
 *       不接触活动会话；不相关分片不混入会话也不破坏会话；任何终止性错误都清除会话，
 *       不留下可被后包续接的状态。不提供 ACK 关联或整消息重传。
 */
yg_protocol_result_t yg_protocol_fragment_accept(yg_protocol_fragment_state_t *state,
                                                 const yg_protocol_message_t *message,
                                                 uint32_t now_ms,
                                                 uint32_t timeout_ms,
                                                 yg_protocol_message_t *completed);

/**
 * @brief 无需新片段即可检查并清除已超时的重组会话。
 * @param state 重组状态。
 * @param now_ms 当前单调时间，单位 ms。
 * @param timeout_ms 调用方片间超时，单位 ms，不能为零。
 * @return 无活动会话返回 YG_PROTOCOL_OK；活动且未超时返回 FRAGMENT_IN_PROGRESS；
 *         已超时并丢弃会话返回 FRAGMENT_TIMEOUT。
 * @note 绝对超时从首片开始，poll 不会延长任何时限；超时判定与 accept 完全一致。
 */
yg_protocol_result_t yg_protocol_fragment_poll(yg_protocol_fragment_state_t *state,
                                               uint32_t now_ms,
                                               uint32_t timeout_ms);

#endif
