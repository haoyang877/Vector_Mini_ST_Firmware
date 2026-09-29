#ifndef DEVICE_IDENTITY_H
#define DEVICE_IDENTITY_H
#include <stdbool.h>
#include <stdint.h>
#define DEVICE_IDENTITY_RECORD_BYTES 64U
#define DEVICE_IDENTITY_INFO_BYTES 32U
#define DEVICE_IDENTITY_MAGIC 0x59475331U
/** 生产身份独立于电机校准；SN为1..16个ASCII字母、数字或连字符，短串以0补齐。 */
typedef struct
{
    uint8_t sn[16];
    uint32_t motor_type;
    bool valid;
} DeviceIdentity;
/**
 * @brief 校验64B生产记录及完整CRC；失败输出未配置身份。
 * @param record 借用的Flash读回副本。
 * @param identity 调用方持有的输出值。
 * @return 格式、SN、非零类型、保留字段与CRC均合法返回true。
 * @note 纯计算，无硬件读写；空或半写记录不能授予生产权限。
 */
bool DeviceIdentity_Decode(const uint8_t record[DEVICE_IDENTITY_RECORD_BYTES],
                           DeviceIdentity *identity);
/**
 * @brief 编码GET_INFO page1的32B身份、启动状态与镜像CRC。
 * @param identity 已校验身份，valid=false时SN和类型输出0。
 * @param node 当前CAN ID，来源是参数所有者。
 * @param role APP=1或Loader=2。
 * @param boot_state 未确认=0、已确认=1、试运行=2、Loader=3。
 * @param image_crc32 当前合法镜像CRC，未知为0。
 * @param payload 调用方拥有的32B输出。
 * @note 所有数值小端，profile输出当前升级容器版本；不暴露写权限。
 */
void DeviceIdentity_EncodeInfo(const DeviceIdentity *identity,
                               uint8_t node,
                               uint8_t role,
                               uint8_t boot_state,
                               uint32_t image_crc32,
                               uint8_t payload[DEVICE_IDENTITY_INFO_BYTES]);
#endif
