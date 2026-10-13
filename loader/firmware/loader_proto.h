
#ifndef LOADER_PROTO_H
#define LOADER_PROTO_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint8_t opcode;
    uint8_t flags;
    uint16_t seq;
    uint16_t plen;
    uint32_t session;
    const uint8_t *payload;
    const uint8_t *raw;
    uint8_t raw_len;
} ldr_request_t;

/** @brief 校验v2帧头、长度、CRC及填充。
 * @param raw 原始帧。
 * @param len 实际FD字节数。
 * @param out 成功时输出借用raw的字段视图。
 * @return 帧完整有效为true。
 */
bool ldr_parse_request(const uint8_t *raw, uint8_t len, ldr_request_t *out);

/** @brief 编码大端帧并填充合法FD长度。
 * @param seq 本次请求序号。
 * @param session 本次会话。
 * @param opcode 操作码。
 * @param payload 只读载荷，0长度可为空。
 * @param plen 载荷字节数，最多48。
 * @param out 至少64字节输出。
 * @return 帧字节数；超容量返回0。
 */
uint8_t ldr_build_reply(uint16_t seq, uint32_t session, uint8_t opcode, const uint8_t *payload,
                        uint16_t plen, uint8_t *out);

/** @brief 读取大端16位字段。
 * @param p 至少2字节。
 * @return 字段值。
 */
uint16_t ldr_get_u16(const uint8_t *p);
/** @brief 读取大端32位字段。
 * @param p 至少4字节。
 * @return 字段值。
 */
uint32_t ldr_get_u32(const uint8_t *p);
/** @brief 写入大端16位字段。
 * @param p 至少2字节输出。
 * @param v 字段值。
 */
void ldr_put_u16(uint8_t *p, uint16_t v);
/** @brief 写入大端32位字段。
 * @param p 至少4字节输出。
 * @param v 字段值。
 */
void ldr_put_u32(uint8_t *p, uint32_t v);

#endif
