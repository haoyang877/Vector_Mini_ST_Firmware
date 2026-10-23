#include "board_hw.h"

#include <stddef.h>
#include <string.h>
#include "at32f45x.h"
#include "power_stage_hw.h"
#include "r40_adc.h"
#include "r40_platform.h"

/* main已完成时钟及安全GPIO初始化；此处按APP启动契约校准ADC再启用采样时基。 */
void board_hw_start(bool motor_phases_enabled)
{
    if (!R40_AdcInit())
    {
        __disable_irq();
        power_stage_hw_stop();
        for (;;)
        {
            __WFI();
        }
    }
    if (motor_phases_enabled)
    {
        power_stage_hw_start();
    }
    if (!R40_AdcStart() || !R40_TimingStart())
    {
        __disable_irq();
        power_stage_hw_stop();
        for (;;)
        {
            __WFI();
        }
    }
}

bool board_hw_read_uid(uint8_t uid[BOARD_HW_UID_BYTES])
{
    if (uid == NULL)
    {
        return false;
    }
    /* AT32F45x手册6.1：96位UID起始0x1FFFF7E8，按三个小端字输出。 */
    const volatile uint32_t *words = (const volatile uint32_t *)0x1FFFF7E8U;
    for (uint32_t word = 0U; word < 3U; ++word)
    {
        uint32_t value = words[word];
        for (uint32_t byte = 0U; byte < 4U; ++byte)
        {
            uid[word * 4U + byte] = (uint8_t)(value >> (byte * 8U));
        }
    }
    return true;
}

bool board_hw_read_identity(uint8_t record[64])
{
    if (record == NULL)
    {
        return false;
    }
    /* 新板生产身份布局尚未分配；明确无有效记录，不借用旧板或参数区。 */
    memset(record, 0xFF, 64U);
    return false;
}
