#ifndef FIRMWARE_VERSION_H
#define FIRMWARE_VERSION_H

#include <stdint.h>

/** @brief 当前 APP 软件版本；每项占一个字节，修改后须重新构建。 */
#define FIRMWARE_VERSION_MAJOR 0
#define FIRMWARE_VERSION_MINOR 6
#define FIRMWARE_VERSION_PATCH 1
#define FIRMWARE_VERSION_BUILD 0

#if FIRMWARE_VERSION_MAJOR > 255U || FIRMWARE_VERSION_MINOR > 255U || \
    FIRMWARE_VERSION_PATCH > 255U || FIRMWARE_VERSION_BUILD > 255U
#error "Firmware version components must fit in one byte"
#endif

#define FIRMWARE_VERSION_CODE \
    (((uint32_t)FIRMWARE_VERSION_MAJOR << 24) | \
     ((uint32_t)FIRMWARE_VERSION_MINOR << 16) | \
     ((uint32_t)FIRMWARE_VERSION_PATCH << 8) | (uint32_t)FIRMWARE_VERSION_BUILD)

/** @brief Flash 中只读版本数值，编码为0xMMmmppbb；供 CAN 和调试器读取。 */
extern const uint32_t g_firmware_software_version;
/** @brief Flash 中只读版本字符串，形式为v主.次.修订；构建号见数字字段。 */
extern const char g_firmware_software_version_string[];

#endif
