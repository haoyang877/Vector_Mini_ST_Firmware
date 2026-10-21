#ifndef BOOT_CONTRACT_H
#define BOOT_CONTRACT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define BOOT_APP_BASE 0x08004000UL
#define BOOT_APP_CAPACITY 0x18000UL
#define BOOT_RECORD_BASE 0x08003000UL
#define BOOT_PARAM_BASE 0x0801C000UL
#define BOOT_PARAM_CAPACITY 0x4000UL
#define BOOT_PAGE_BYTES 2048UL
#define BOOT_MAILBOX_BASE 0x20000000UL
#define BOOT_LAYOUT_ID 0x4F564C31UL
#define BOOT_HARDWARE_ID 1UL
#define BOOT_PARAM_ABI 0x4F563130UL
#define BOOT_ENTER_KEY 0x424F4F54UL
#define BOOT_LOADER_VERSION 0x01000000UL
#define BOOT_RECORD_BYTES 48U
#define BOOT_MAILBOX_BYTES 32U

/** @brief 镜像描述；长度单位字节，版本编码0xMMmmppbb。 */
typedef struct
{
    uint32_t size;
    uint32_t crc;
    uint32_t version;
} BootImage;

/**
 * @brief 计算CRC-32/ISO-HDLC，与zlib.crc32一致。
 * @param data 只读字节串。
 * @param size 字节数，0允许data为空。
 * @return 完成初值与最终异或的CRC。
 */
uint32_t Boot_Crc32(const uint8_t *data, size_t size);
/**
 * @brief 编码48字节小端记录，提交标记位于最后8字节。
 * @param image 已验证的镜像描述。
 * @param out 输出48字节。
 */
void Boot_RecordEncode(const BootImage *image, uint8_t out[BOOT_RECORD_BYTES]);
/**
 * @brief 检查记录字段、CRC和提交标记；不访问实际Flash。
 * @param raw 48字节记录。
 * @param image 成功时输出镜像描述。
 * @return 完整有效记录为true。
 */
bool Boot_RecordDecode(const uint8_t raw[BOOT_RECORD_BYTES], BootImage *image);
/**
 * @brief 检查实际APP前8字节向量与镜像边界。
 * @param vectors 只读8字节小端MSP和Reset_Handler。
 * @param size 实际APP字节数。
 * @return 栈及Thumb复位入口合法为true。
 */
bool Boot_VectorsValid(const uint8_t vectors[8], uint32_t size);
/**
 * @brief 编码32字节软复位交接邮箱。
 * @param node 当前节点，0～15。
 * @param token 主机交接标识。
 * @param out 输出32字节；最后4字节为有效标记。
 */
void Boot_MailboxEncode(uint32_t node, uint32_t token, uint8_t out[BOOT_MAILBOX_BYTES]);
/**
 * @brief 校验邮箱；复位原因检查由硬件端口负责。
 * @param raw 32字节邮箱。
 * @param node 成功时输出节点。
 * @return 版本、命令、CRC和有效标记均合法为true。
 */
bool Boot_MailboxDecode(const uint8_t raw[BOOT_MAILBOX_BYTES], uint32_t *node);

#endif
