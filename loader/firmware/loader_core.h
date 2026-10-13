#ifndef LOADER_CORE_H
#define LOADER_CORE_H
#include "loader_cfg.h"

/** @brief 单个前台所有者的升级状态；缓存只记录变更请求。 */
typedef struct
{
    uint8_t state, last_error, node, cache_length, reply_length;
    bool cache_valid, activate_pending;
    uint16_t last_sequence;
    uint32_t session, offset, last_activity_ms;
    BootImage image;
    uint8_t cache_request[64], cache_reply[64];
} LoaderCore;

/** @brief 初始化单台Loader会话。
 * @param core 状态所有者。
 * @param node 当前0～7节点。
 */
void LoaderCore_Init(LoaderCore *core, uint8_t node);
/** @brief 前台处理完整CAN-FD请求；坏帧无副作用。
 * @param core 状态所有者。
 * @param raw 输入帧。
 * @param size 实际DLC字节数。
 * @param now_ms 单调毫秒计时。
 * @param reply 输出最多64字节。
 * @return 回复字节数；0表示丢弃无效帧。
 * @note 同步Flash操作，无动态分配；禁止ISR调用。
 */
uint8_t LoaderCore_Request(LoaderCore *core, const uint8_t *raw, uint8_t size, uint32_t now_ms,
                           uint8_t reply[64]);
/** @brief 处理会话超时，保留恢复入口。
 * @param core 状态所有者。
 * @param now_ms 单调毫秒计时。
 */
void LoaderCore_Tick(LoaderCore *core, uint32_t now_ms);
/** @brief 验证持久记录、全APP CRC和向量。
 * @return 可以启动APP为true。
 */
bool LoaderCore_BootValid(void);
#endif
