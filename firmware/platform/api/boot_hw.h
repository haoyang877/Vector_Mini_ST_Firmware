#ifndef BOOT_HW_H
#define BOOT_HW_H
#include <stdbool.h>
#include <stdint.h>
/** 本次真实Loader交接的只读值；无交接时available为false且epoch为0。 */
typedef struct
{
    uint64_t boot_epoch;
    uint32_t image_token;
    uint32_t image_role;
    uint8_t node;
    bool available;
    bool trial;
    bool confirmed;
} BootHwInfo;
/**
 * @brief 启动最早期一次性消费邮箱并验证本次APP镜像绑定。
 * @return 当前APP确由兼容Loader启动返回true；独立APP返回false，不猜epoch。
 * @note 在HAL_Init和开中断前调用；确认实际链接向量地址后才触碰保留邮箱/VTOR。
 *       完整镜像只读校验，不擦写，不授权电机使能；失败不得重试消费。
 */
bool BootHw_Init(void);
/** @brief 获取本次交接快照。 @param info 完整输出，不能修改平台状态。 */
void BootHw_GetInfo(BootHwInfo *info);
/**
 * @brief 为自检和通信健康已通过的本次试启动写入持久确认。
 * @param boot_epoch 精确匹配启动观察的64位代号。
 * @param image_token 精确匹配启动镜像token。
 * @return 完整镜像和参数外形有效、关相且确认页写入读回成功返回true。
 * @note E唯一前台owner在worker归还、保存空闲、健康成立后调用；内部临界区阻止ISR启相。
 *       仅允许记录页写，不改变APP槽/参数。掉电可能留下无效记录，Loader将救援。
 */
bool BootHw_Confirm(uint64_t boot_epoch, uint32_t image_token);
/**
 * @brief 在精确回复物理发送后写ENTER邮箱并复位。
 * @param boot_epoch 当前启动代号。
 * @param node 当前合法节点0..254。
 * @return 拒绝返回false；成功复位不返回。
 * @note E所有者已复核票据、截止和停机资源条件；本函数再次检查身份和数字关相。
 */
bool BootHw_EnterLoader(uint64_t boot_epoch, uint8_t node);
/**
 * @brief 由APP健康监督者喂继承的看门狗。
 * @param healthy 本次监督确认初始化/调度/通信健康，禁止由任意循环无条件传true。
 * @note 试启动未确认时不延长看门狗；Loader初始约32s期限供启动并确认。
 *       独立APP无Loader交接时不操作IWDG。
 */
void BootHw_Watchdog(bool healthy);
#endif
