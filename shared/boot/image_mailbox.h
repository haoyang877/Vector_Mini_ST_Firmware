#ifndef BOOT_IMAGE_MAILBOX_H
#define BOOT_IMAGE_MAILBOX_H
#include <stdbool.h>
#include <stdint.h>
#define YGIM_BYTES 32U
#define YGIM_MAGIC 0x33494759U
#define YGIM_VERSION 3U
#define YG_BOOT_ENTER 1U
#define YG_BOOT_TRIAL 2U
#define YG_BOOT_NORMAL 4U
/** LE布局magic/version/command/node/token/epoch低32/高32/CRC32，末字覆盖前28B。 */
typedef struct
{
    uint32_t command, node, token;
    uint64_t epoch;
} BootMailboxMessage;
/**
 * @brief 编码一次性APP/Loader交接值。
 * @param message 命令和0..254节点；TRIAL/NORMAL/START要求非零token和epoch。
 * @param out 成功输出32B，调用方负责保留RAM、内存屏障和最终magic发布。
 * @return 格式有效返回true；不分配epoch或授予硬件写权限。
 */
bool BootMailbox_Encode(const BootMailboxMessage *message, uint8_t out[YGIM_BYTES]);
/**
 * @brief 检查邮箱版本、CRC与命令字段。
 * @param raw 完整32B输入，函数不清除实际邮箱。
 * @param message 成功输出值，失败保持原值。
 * @return 格式有效返回true；调用方仍须检查真实启动来源并立即消费邮箱。
 */
bool BootMailbox_Decode(const uint8_t raw[YGIM_BYTES], BootMailboxMessage *message);
#endif
