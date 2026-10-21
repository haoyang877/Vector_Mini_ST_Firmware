#include "parameter_persistence.h"

#include <stdbool.h>
#include <stddef.h>
#include "foc_param.h"
#include "foc_errhandle.h"
#include "motor_hw.h"
#include "parameter_storage_hw.h"

extern ModeNow_TypeDef ModeLast;

/* 启动读取与停机保存共享独占快照，BSP不持有应用参数或缓冲所有权。 */
static InterfaceParam_TypeDef parameter_buffer;
static volatile bool parameter_buffer_busy;
static volatile FlashParamSaveResult last_save_result = FLASH_PARAM_SAVE_NOT_ATTEMPTED;
typedef char ParameterBufferSize[(sizeof(parameter_buffer) == PARAMETER_STORAGE_RECORD_BYTES) ? 1 : -1];

/** @brief 获取参数快照独占权，重入时返回NULL且不改现有参数。 */
static InterfaceParam_TypeDef *parameter_buffer_acquire(void)
{
    uint32_t state = parameter_storage_hw_enter();
    InterfaceParam_TypeDef *param = NULL;
    if (!parameter_buffer_busy)
    {
        parameter_buffer_busy = true;
        param = &parameter_buffer;
    }
    parameter_storage_hw_exit(state);
    return param;
}

/** @brief 将存储层结果映射为既有本机诊断值，不将硬件类型传到应用接口。 */
static FlashParamSaveResult store_snapshot(const InterfaceParam_TypeDef *param)
{
    switch (parameter_storage_hw_write(param))
    {
        case PARAMETER_STORAGE_WRITTEN: return FLASH_PARAM_SAVE_WRITTEN;
        case PARAMETER_STORAGE_UNCHANGED: return FLASH_PARAM_SAVE_UNCHANGED;
        case PARAMETER_STORAGE_UNLOCK_FAILED: return FLASH_PARAM_SAVE_UNLOCK_FAILED;
        case PARAMETER_STORAGE_ERASE_FAILED: return FLASH_PARAM_SAVE_ERASE_FAILED;
        case PARAMETER_STORAGE_PROGRAM_FAILED: return FLASH_PARAM_SAVE_PROGRAM_FAILED;
        case PARAMETER_STORAGE_VERIFY_FAILED: return FLASH_PARAM_SAVE_VERIFY_FAILED;
        case PARAMETER_STORAGE_LOCK_FAILED: return FLASH_PARAM_SAVE_LOCK_FAILED;
        default: return FLASH_PARAM_SAVE_VERIFY_FAILED;
    }
}

FlashParamSaveResult flash_write_param(void)
{
    uint32_t state = parameter_storage_hw_enter();
    FlashParamSaveResult result;
    InterfaceParam_TypeDef *param;
    param = parameter_buffer_acquire();
    if (param == NULL)
    {
        result = FLASH_PARAM_SAVE_BUSY;
    }
    else
    {
        if (!motor_hw_phase_outputs_disabled())
        {
            result = FLASH_PARAM_SAVE_OUTPUTS_ACTIVE;
        }
        else
        {
            Param_Upload(param);
            param->magic_word = MAGIC_WORD;
            result = store_snapshot(param);
        }
        parameter_buffer_busy = false;
    }
    last_save_result = result;
    parameter_storage_hw_exit(state);
    return result;
}

FlashParamSaveResult flash_get_last_save_result(void)
{
    return last_save_result;
}

void flash_read_param(void)
{
    InterfaceParam_TypeDef *param = parameter_buffer_acquire();
    bool rewrite_legacy_param;
    if (param == NULL)
    {
        return;
    }

    parameter_storage_hw_read(param);
    rewrite_legacy_param = Param_Download(param);
#ifdef APP_LOADER_LAYOUT
    /* 升级版只在RAM解释旧参数；启动迁移不能改写个体标定区。 */
    (void)rewrite_legacy_param;
#else
    if (rewrite_legacy_param)
    {
        uint32_t state;

        /* 电机控制启动前保存已通过解析的旧记录迁移，不走运行状态准入。 */
        Param_Upload(param);
        param->magic_word = MAGIC_WORD;
        state = parameter_storage_hw_enter();
        last_save_result = store_snapshot(param);
        parameter_buffer_busy = false;
        parameter_storage_hw_exit(state);
        return;
    }
#endif
    parameter_buffer_busy = false;
}

void ParameterPersistence_Poll(void)
{
    uint32_t state;
    FlashParamSaveResult result;
    if (Get_ModeNow() != Save_Param)
    {
        return;
    }
    state = parameter_storage_hw_enter();
    /* CAN可在两次快拍间提交STOP和SAVE；ModeLast是完成关相后的快环确认。 */
    if (Get_ModeNow() == Save_Param && ModeLast == Save_Param)
    {
        result = flash_write_param();
        if (result != FLASH_PARAM_SAVE_WRITTEN && result != FLASH_PARAM_SAVE_UNCHANGED)
        {
            Stop_PWM_Generate();
            Set_ErrorNow(MotorParam_Error);
        }
        Set_ModeNow(Motor_Disable);
    }
    parameter_storage_hw_exit(state);
}
