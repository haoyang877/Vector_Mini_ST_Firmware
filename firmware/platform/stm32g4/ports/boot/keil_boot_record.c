/* Keil 链接占位段；构建后由离线工具填写，APP 不读取或写入该记录。 */
#include "boot_contract.h"

/** @brief 48 B 无效占位记录；后处理失败时 Loader 必须拒绝启动。 */
__attribute__((section("BOOT_RECORD"), used))
const uint8_t g_keil_boot_record[BOOT_RECORD_BYTES] = {0};
