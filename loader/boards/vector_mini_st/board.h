#ifndef LOADER_VECTOR_MINI_BOARD_H
#define LOADER_VECTOR_MINI_BOARD_H
#include <stdbool.h>
/* 板级170MHz CAN内核，仲裁1M、数据5M；引脚和时序不进入通用升级模块。 */
#define YG_CAN_PRESCALER 10U
#define YG_CAN_DATA_PRESCALER 2U
#define YG_CAN_SEG1 12U
#define YG_CAN_SEG2 4U
#define YG_CAN_SJW 1U
/** @brief 六路门极改为推挽低电平，冷启动和致命失败调用。 */
void LoaderBoard_Safe(void);
/** @brief 从板载8MHz HSE建立170MHz时钟，失败禁止中断并停留。 */
void SystemClock_Config(void);
/** @brief 配置FDCAN1时钟及PB8/PB9引脚。 @return 时钟配置成功返回true。 */
bool LoaderBoard_CanInit(void);
#endif
