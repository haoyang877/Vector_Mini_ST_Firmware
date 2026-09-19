#ifndef __COMMON_INC_H__
#define __COMMON_INC_H__

/* 应用层公共聚合头：集中引入 HAL/驱动/FOC/通信/工具头文件供各模式与任务模块使用；
 * 新增依赖应放入最窄的模块头，不要扩到此处。 */

#include "main.h"

/*Lib*/
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/*Core*/
#include "gpio.h"
#include "adc.h"
#include "fdcan.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"

/*Bsp*/
#include "delay.h"
#include "led.h"
#include "encoder.h"
#include "rgb.h"
#include "flash.h"
#include "hw_conf.h"
#include "bsp_task.h"

/*Foc*/
#include "foc_algorithm.h"
#include "foc_sensing.h"
#include "foc_sensorless.h"
#include "foc_param.h"
#include "foc_pid.h"
#include "foc_traptraj.h"
#include "foc_run.h"
#include "foc_errhandle.h"
#include "foc_task.h"

/*Communication*/
#include "interface_can.h"

/*System*/
#include "utils.h"
#include "heap.h"
#include "board_config.h"
#include "data_type.h"

#endif
