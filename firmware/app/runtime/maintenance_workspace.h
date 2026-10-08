#ifndef MAINTENANCE_WORKSPACE_H
#define MAINTENANCE_WORKSPACE_H

#include <stdbool.h>
#include "encoder_calibration_task.h"
#include "foc_param.h"

/* APP 唯一维护工作区；参数和编码器采样互斥，不分配、不保留块链表。 */
#define MAINTENANCE_WORKSPACE_CAPACITY_BYTES (1024U * 6U)

/**
 * @brief 借用参数记录空间，已有任何借用时返回失败。
 * @return 独占记录指针；繁忙返回 NULL。
 * @note 启动或前台使用；调用者归还前不得启动标定，不清空已有内容。
 */
InterfaceParam *MaintenanceWorkspace_BorrowParameter(void);
/**
 * @brief 归还实际持有的参数记录，错误指针或其他占用不产生效果。
 * @param record 此工作区借出的参数记录。
 * @note 前台在实际存储操作结束后调用，不根据取消意图提前归还。
 */
void MaintenanceWorkspace_ReleaseParameter(InterfaceParam *record);
/**
 * @brief 借用编码器采样区；同一活动标定的重复调用保持原指针。
 * @return 固定采样对象；其他任务占用时返回 NULL。
 * @note APP 维护组合层在快速拍调用，只有常量时间的占用更新，不清空采样。
 */
EncoderCalibrationWorkspace *MaintenanceWorkspace_BorrowEncoder(void);
/**
 * @brief 归还编码器占用，其他任务占用不产生效果。
 * @note APP 必须先确认 EncoderCalibrationTask_IsReleased；不能由 STOP 意图直接调用。
 */
void MaintenanceWorkspace_ReleaseEncoder(void);
/**
 * @brief 查询共享维护空间是否已实际归还。
 * @return 无任何借用返回 true，否则返回 false。
 * @note 不替代任务归还和后台身份检查，不修改占用。
 */
bool MaintenanceWorkspace_IsReleased(void);

#endif
