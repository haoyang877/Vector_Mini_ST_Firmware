#ifndef LOADER_PLATFORM_H
#define LOADER_PLATFORM_H
#include "image_transfer.h"
#include "device_identity.h"
#include "image_mailbox.h"
/**
 * @brief 初始化独立 Loader 平台并核对芯片和 Flash 容量。
 * @param software_reset 输出是否为无冷启动/看门狗标志的纯软件复位。
 * @param watchdog_reset 输出是否为无冷启动标志的看门狗复位，必须驻留救援。
 * @return 硬件符合固定目标且安全输出已关闭返回 true。
 * @note 启动看门狗和独立时钟，不配置 APP 的 PWM/ADC。
 */
bool LoaderPlatform_Init(bool *software_reset, bool *watchdog_reset);
/** @brief 读取包含 Flash 停顿的单调毫秒时钟。 @return 模32位毫秒值。 */
uint32_t LoaderPlatform_NowMs(void);
/** @brief 喂独立看门狗；前台有界工作之间调用。 */
void LoaderPlatform_Watchdog(void);
/** @brief 回读六路门极GPIO和TIM1主输出关闭。 @return 数字关断满足返回 true。 */
bool LoaderPlatform_OutputsOff(void);
/** @brief 保存PRIMASK并屏蔽普通中断。 @return 原PRIMASK。 */
uint32_t LoaderPlatform_Lock(void);
/** @brief 恢复PRIMASK。 @param key Lock返回值，不能猜测。 */
void LoaderPlatform_Unlock(uint32_t key);
/** @brief 发起系统复位，调用前必须已确认输出关闭。 */
void LoaderPlatform_Reset(void);
/** @brief 安全关断并停喂看门狗，不返回。 */
void LoaderPlatform_Fatal(void);
/**
 * @brief 建立仅允许APP槽写入的端口。
 * @param io 输出端口，指向平台固定范围而非主机地址。
 */
void LoaderPlatform_ImageIo(LoaderFlashIo *io);
/**
 * @brief 读取芯片实际UID和独立生产身份，不访问APP运行态。
 * @param uid 调用方提供的12B输出。
 * @param identity CRC与格式合法时输出生产身份，否则为无效值。
 * @note 前台调用；ECC损坏只使身份无效，不能改变生产权限。
 */
void LoaderPlatform_ReadIdentity(uint8_t uid[12], DeviceIdentity *identity);
/**
 * @brief 只读识别raw11存储外形并解析CAN节点。
 * @param node 成功识别后输出0..254节点，否则保持原值。
 * @return RAW11、完整空区或不可信；不代表标定有效或允许使能。
 */
LoaderParameterState LoaderPlatform_Parameters(uint8_t *node);
/**
 * @brief 读取后立即清除交接邮箱。
 * @param message 有效时输出值，失败不修改。
 * @return CRC与命令格式有效返回true。
 */
bool LoaderPlatform_TakeMailbox(BootMailboxMessage *message);
/**
 * @brief 停外设并通过已经校验的向量进入固定APP槽。
 * @note 仅启动所有者完成完整镜像校验后调用；失败安全停机，不接受任意入口地址。
 */
void LoaderPlatform_Jump(void);
#endif
