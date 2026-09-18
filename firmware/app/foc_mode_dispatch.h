#ifndef FOC_MODE_DISPATCH_H
#define FOC_MODE_DISPATCH_H

/**
 * @brief 按当前运行模式分发到对应控制任务，并处理模式退出清理。
 * @note 仅由 20 kHz 快速中断调用；覆盖禁用预载、全部控制/校准模式与参数/清障命令。
 */
void FocMode_Dispatch(void);

#endif
