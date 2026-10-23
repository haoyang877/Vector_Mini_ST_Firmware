#ifndef APP_BUS_CURRENT_H
#define APP_BUS_CURRENT_H

#include <stdint.h>
#include "board_profile_defaults.h"

/* APP装配扩展，不属于BSP契约；按板级硬件能力选择状态来源。
 * 不修改FOC估算对象、控制算法、参数记录或协议布局。 */
#if BOARD_HAS_BUS_CURRENT_SENSOR
/**
 * @brief 初始化本次上电的母线电流偏置及滤波记录。
 * @note 仅板级APP启动时、采样调度开始前调用，不读取或写入Flash。
 */
void AppBusCurrent_Init(void);

/**
 * @brief 消费独立母线电流的新鲜样本，关相时完成一次零点校准。
 * @note 仅由板级APP的2kHz监督中断调用；不启相、不阻塞、不保存参数。
 */
void AppBusCurrent_Sample(void);

/**
 * @brief 用硬件反馈覆盖APP状态中的母线电流和对应有效位。
 * @param current_a 输入估算值，输出有符号实测滤波电流，单位A；无效时为NAN。
 * @param valid_bits APP测量有效位，仅修改母线电流位0x40。
 * @note 前台调用；未校零、过期或异常时置无效，不回退为估算；空指针不操作。
 */
void AppBusCurrent_Apply(float *current_a, uint8_t *valid_bits);
#else
/**
 * @brief 无独立采样硬件时保留原估算值及所有有效位。
 * @param current_a APP已有的估算值，不修改。
 * @param valid_bits APP已有的测量有效位，不修改。
 * @note 旧板编译为无操作，不引入任何硬件依赖。
 */
static inline void AppBusCurrent_Apply(float *current_a, uint8_t *valid_bits)
{
    (void)current_a;
    (void)valid_bits;
}
#endif

#endif
