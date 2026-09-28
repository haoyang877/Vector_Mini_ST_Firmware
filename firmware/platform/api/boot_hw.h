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
#endif
