#ifndef BOOT_HANDOFF_H
#define BOOT_HANDOFF_H
#include <stdbool.h>
#include <stdint.h>

/** @brief 前台交接动作，不直接访问电机和硬件。 */
typedef enum
{
    BOOT_WAIT,
    BOOT_STOP,
    BOOT_REPLY_OK,
    BOOT_REPLY_FAILED,
    BOOT_RESET
} BootAction;
/** @brief CAN中断提交一次升级请求。
 * @param node 当前节点。
 * @param now_ms 请求接收毫秒。
 * @return 本次首次请求被接受为true；重复请求不重置时限。
 * @note ISR只写入请求，前台独占消费；不执行停止或复位。
 */
bool BootHandoff_Request(uint8_t node, uint32_t now_ms);
/** @brief 查询运动锁存。
 * @return 已受理升级时为true，直到系统复位。
 */
bool BootHandoff_Pending(void);
/** @brief 前台推进停止、关相、回复和复位准入。
 * @param now_ms 单调毫秒。
 * @param stopped 模式所有者已确认停机。
 * @param phases_off 六路输出已关闭。
 * @param sent 本次回复已实际发送完成。
 * @return 组合层需要执行的动作。
 */
BootAction BootHandoff_Poll(uint32_t now_ms, bool stopped, bool phases_off, bool sent);
/** @brief 回复入队后开始等待实际发送。
 * @param now_ms 入队毫秒。
 */
void BootHandoff_ReplyQueued(uint32_t now_ms);
/** @brief 返回已锁存目标节点。
 * @return 当前节点。
 */
uint8_t BootHandoff_Node(void);
#endif
