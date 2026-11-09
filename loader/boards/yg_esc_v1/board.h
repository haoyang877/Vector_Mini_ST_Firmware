#ifndef LOADER_YG_ESC_BOARD_H
#define LOADER_YG_ESC_BOARD_H
#include <stdbool.h>
#define LOADER_AT32_CLOCK_HZ 180000000U
/** @brief 冷启动建立180MHz时钟并保持六路功率关闭。 @return 时钟与关断有效返回true。 */
bool LoaderBoard_Init(void);
/** @brief 将PA9/10/11与PB0/1/4置为推挽低电平，不初始化PWM。 */
void LoaderBoard_Safe(void);
/** @brief 回读六路配置、电平和定时器主输出。 @return 全部处于关闭条件返回true。 */
bool LoaderBoard_OutputsOff(void);
/** @brief 配置CAN1的PB8/PB9引脚和时钟。 */
void LoaderBoard_CanInit(void);
#endif
