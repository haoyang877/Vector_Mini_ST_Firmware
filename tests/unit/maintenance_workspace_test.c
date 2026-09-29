#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "critical_hw.h"
#include "maintenance_workspace.h"

/* ENCODER_WORKSPACE_REFERENCE：脚本插入真实任务的采样借用和取消函数。 */

/* 离线临界区桩：核对嵌套进入后恢复原屏蔽状态，不接触硬件。 */
static uint32_t s_mask;
static unsigned s_depth;

uint32_t critical_hw_enter(void)
{
    uint32_t previous = s_mask;
    s_mask = 1U;
    ++s_depth;
    return previous;
}

void critical_hw_exit(uint32_t state)
{
    s_mask = state;
    --s_depth;
}

/**
 * @brief 检查实际类型容量、共享地址、互斥、错误归还和内容保持。
 * @return 全部满足契约返回 0，否则返回失败编号。
 */
int main(void)
{
    if (sizeof(EncoderCalibrationWorkspace) != MAINTENANCE_WORKSPACE_CAPACITY_BYTES ||
        sizeof(InterfaceParam) != 4304U || !MaintenanceWorkspace_IsReleased())
    {
        return 1;
    }
    for (unsigned pass = 0U; pass < 4U; ++pass)
    {
        EncoderCalibrationWorkspace *encoder = MaintenanceWorkspace_BorrowEncoder();
        if (encoder == NULL || (uintptr_t)encoder % 8U != 0U ||
            MaintenanceWorkspace_BorrowParameter() != NULL ||
            MaintenanceWorkspace_BorrowCogging() != NULL)
        {
            return 2;
        }
        memset(encoder, 0xA5, sizeof(*encoder));
        if (!encoder_calib_prepare_samples(encoder) || EncoderCalibrationTask_IsReleased())
        {
            return 8;
        }
        memset(encoder, 0xA5, sizeof(*encoder));
        MaintenanceWorkspace_ReleaseParameter((InterfaceParam *)encoder);
        MaintenanceWorkspace_ReleaseCogging((CoggingMapRecord *)encoder);
        if (MaintenanceWorkspace_IsReleased() || MaintenanceWorkspace_BorrowEncoder() != encoder ||
            encoder->bins.samples[ENCODER_OFFSET_LUT_SIZE - 1U] != 0xA5A5U)
        {
            return 3;
        }
        EncoderCalibrationTask_Cancel();
        if (!EncoderCalibrationTask_IsReleased() || MaintenanceWorkspace_IsReleased())
        {
            return 9;
        }
        MaintenanceWorkspace_ReleaseEncoder();
        InterfaceParam *parameter = MaintenanceWorkspace_BorrowParameter();
        if ((void *)parameter != (void *)encoder ||
            MaintenanceWorkspace_BorrowParameter() != NULL ||
            MaintenanceWorkspace_BorrowEncoder() != NULL)
        {
            return 4;
        }
        memset(parameter, 0x5A, sizeof(*parameter));
        MaintenanceWorkspace_ReleaseParameter(NULL);
        MaintenanceWorkspace_ReleaseEncoder();
        if (MaintenanceWorkspace_IsReleased() || parameter->record_crc32 != 0x5A5A5A5AU)
        {
            return 5;
        }
        MaintenanceWorkspace_ReleaseParameter(parameter);
        CoggingMapRecord *cogging = MaintenanceWorkspace_BorrowCogging();
        if ((void *)cogging != (void *)encoder || MaintenanceWorkspace_BorrowCogging() != NULL ||
            MaintenanceWorkspace_BorrowParameter() != NULL)
        {
            return 6;
        }
        MaintenanceWorkspace_ReleaseCogging(cogging);
    }
    s_mask = 1U;
    if (!MaintenanceWorkspace_IsReleased() || s_mask != 1U || s_depth != 0U)
    {
        return 7;
    }
    printf("workspace PASS: encoder=%zu parameter=%zu cogging=%zu; one aligned shared storage\n",
           sizeof(EncoderCalibrationWorkspace),
           sizeof(InterfaceParam),
           sizeof(CoggingMapRecord));
    return 0;
}
