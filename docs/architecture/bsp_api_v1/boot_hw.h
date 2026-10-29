/**
 * @file boot_hw.h
 * @brief 沿用现有APP与Loader交接签名。
 * @note 保留APP与Loader交接行为；可用性查询已单独通过评审。
 */
#ifndef BOOT_HW_H
#define BOOT_HW_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief 启动最早期识别APP槽布局并设置VTOR。
     * @return 链接于Loader APP槽为true；独立APP或无Loader支持为false。
     * @note HAL_Init及开中断前调用一次；不读取持久记录，不授权电机使能。
     */
    bool BootHw_Init(void);

    /**
     * @brief 写RAM ENTER邮箱并复位进入Loader。
     * @param node 当前合法节点，0..254。
     * @return 未由Loader启动、节点非法或邮箱失败为false；成功复位不返回。
     * @note 通信所有者先核对维护授权、停机和数字关相；邮箱失败不复位或恢复运动。
     */
    bool BootHw_EnterLoader(uint8_t node);

    /**
     * @brief 按健康状态处理Loader留下的继承看门狗。
     * @param healthy 所有者确认的基础健康结果，禁止无条件传true。
     * @note 保持原前台健康节拍；非Loader启动不操作IWDG，健康失败停止喂狗以回到Loader。
     */
    void BootHw_Watchdog(bool healthy);

    /**
     * @brief 数字关相时一次写入试运行镜像确认双字。
     * @param version 主机已核对的发布版本。
     * @param crc32 主机已核对的整包CRC32。
     * @return 身份匹配且读回有效为true；不支持、身份不符或失败为false，半写不补写。
     * @note 前台调用，保留短中断屏蔽及原写入顺序；不擦APP/参数，不恢复运动。
     */
    bool BootHw_Confirm(uint32_t version, uint32_t crc32);

    /**
     * @brief 只读查询当前APP持久确认状态。
     * @return 配套APP确认双字完整为true；独立APP、未确认或不支持为false。
     * @note 前台只读，不触发任何确认写入。
     */
    bool BootHw_IsConfirmed(void);

    /**
     * @brief 只读取得APP描述符声明的整包CRC32。
     * @return 配套APP声明的CRC32；独立APP或不支持为0。
     * @note 只读元数据，不代替整包校验，不据此自行授权更新或启相。
     */
    uint32_t BootHw_ImageCrc32(void);

    /**
     * @brief 查询本次启动是否已完成有效的Loader APP交接。
     * @return BootHw_Init成功建立交接后为true；未初始化、独立APP或不支持为false。
     * @note 前台有界只读，不写VTOR/邮箱/Flash，不重置试运行时限，不授权运动。
     */
    bool BootHw_IsAvailable(void);

#ifdef __cplusplus
}
#endif

#endif /* BOOT_HW_H */
