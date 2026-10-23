#ifndef R40_STORAGE_FAKE_H
#define R40_STORAGE_FAKE_H

#include "../r40_fake/at32f45x.h"
#define __IO volatile
#undef FLASH_WAIT_CYCLE_5
typedef enum
{
    ERROR = 0,
    SUCCESS = 1
} error_status;
#include "at32f45x_flash.h"
#undef FLASH
#define FLASH (&sim_flash)
extern flash_type sim_flash;
/** @brief 读取离线中断上下文编号。
 * @return 测试指定的异常号，0表示前台。
 */
uint32_t __get_IPSR(void);

#endif
