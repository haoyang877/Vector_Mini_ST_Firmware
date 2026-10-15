#ifndef PARAMETER_PERSISTENCE_H
#define PARAMETER_PERSISTENCE_H

/** @brief 最近一次参数保存结果，仅供本机诊断，不扩展参数或CAN ABI。 */
typedef enum
{
    FLASH_PARAM_SAVE_NOT_ATTEMPTED = 0,
    FLASH_PARAM_SAVE_WRITTEN,
    FLASH_PARAM_SAVE_UNCHANGED,
    FLASH_PARAM_SAVE_BUSY,
    FLASH_PARAM_SAVE_OUTPUTS_ACTIVE,
    FLASH_PARAM_SAVE_UNLOCK_FAILED,
    FLASH_PARAM_SAVE_ERASE_FAILED,
    FLASH_PARAM_SAVE_PROGRAM_FAILED,
    FLASH_PARAM_SAVE_VERIFY_FAILED,
    FLASH_PARAM_SAVE_LOCK_FAILED
} FlashParamSaveResult;

/**
 * @brief 保存当前参数快照，记录相同时跳过擦写，写入后完整回读校验。
 * @return 保存结果；仅WRITTEN和UNCHANGED表示成功。
 * @note 保留既有入口名；仅由已停机的前台调用，拒绝未关相或缓冲忙。
 *       整段操作屏蔽中断并恢复原状态；停机和故障发布由Poll协调。
 */
FlashParamSaveResult flash_write_param(void);

/**
 * @brief 读取最近一次保存诊断，不触发Flash访问或清除结果。
 * @return 最近一次保存结果；启动后尚未保存时为NOT_ATTEMPTED。
 */
FlashParamSaveResult flash_get_last_save_result(void);

/**
 * @brief 启动时读取参数到RAM，兼容旧记录。
 * @note 保留既有入口名；仅在电机控制启动前调用，忙时不改变RAM参数。
 *       APP_LOADER_LAYOUT下不自动改写Flash，旧布局保留启动迁移。
 */
void flash_read_param(void);

/**
 * @brief 前台处理保存模式，等待快环关相确认，再保存并发布失败故障。
 * @note 不改变CAN或模式编号；失败先关相，发布MotorParam_Error并停机。
 *       使用模块私有缓冲，不自动重试，恢复调用前中断状态。
 */
void ParameterPersistence_Poll(void);

#endif
