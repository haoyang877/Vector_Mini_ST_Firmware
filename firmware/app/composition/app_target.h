#ifndef APP_TARGET_H
#define APP_TARGET_H

#include "app_feature_requirements.h"

/* 唯一的APP目标身份选择；算法与协议不判断MCU。默认仍使用旧板目标。 */
#if defined(APP_TARGET_R40_F456) && APP_TARGET_R40_F456
#include "r40_f456_bringup.h"
#else
#include "vector_mini_st_g431.h"
#endif

#endif
