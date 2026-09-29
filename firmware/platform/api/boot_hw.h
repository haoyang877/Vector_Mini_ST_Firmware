#ifndef BOOT_HW_H
#define BOOT_HW_H
#include <stdbool.h>
#include <stdint.h>
/**
 * @brief 启动最早期确认本次是否由 Loader 按 APP 槽启动并设置 VTOR。
 * @return 实际链接基址等于 APP 槽返回 true；独立 APP 返回 false。
 * @note 在 HAL_Init 和开中断前调用；不读取任何持久记录，不授权电机使能。
 */
bool BootHw_Init(void);
/**
 * @brief 请求进入 Loader：写 RAM ENTER 邮箱并触发系统复位。
 * @param node 当前合法节点 0..254。
 * @return 未由 Loader 启动或写邮箱失败返回 false；成功复位不返回。
 * @note 由通信所有者复核停机与数字关相后调用；邮箱失败不得复位，不得自动恢复运动。
 */
bool BootHw_EnterLoader(uint8_t node);
/**
 * @brief 喂 Loader 启动时留下的继承看门狗。
 * @param healthy 调用方确认的基础健康；禁止由任意循环无条件传 true。
 * @note 未由 Loader 启动时不操作 IWDG；健康失败停止喂狗以退回 Loader。
 */
void BootHw_Watchdog(bool healthy);
/**
 * @brief 数字关相时一次写入当前试运行镜像的确认双字。
 * @param version 主机已核对的发布版本。
 * @param crc32 主机已核对的整包CRC32。
 * @return 身份匹配且确认标记读回有效返回true；半写不补写。
 * @note 前台调用；短暂屏蔽中断，不擦除APP或参数。
 */
bool BootHw_Confirm(uint32_t version, uint32_t crc32);
/**
 * @brief 查询当前APP持久确认状态。
 * @return 配套APP确认双字完整返回true；独立APP或未确认返回false。
 */
bool BootHw_IsConfirmed(void);
/**
 * @brief 返回配套APP描述符声明的整包CRC32，供维护身份查询。
 * @return 配套APP的CRC32；独立APP或启动端口不可用时为0。
 * @note 只读元数据，不代替Loader的整包校验。
 */
uint32_t BootHw_ImageCrc32(void);
#endif
