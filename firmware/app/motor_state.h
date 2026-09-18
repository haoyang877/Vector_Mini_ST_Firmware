#ifndef MOTOR_STATE_H
#define MOTOR_STATE_H

#include <stdbool.h>
#include <stdint.h>

#include "data_type.h"
#include "encoder.h"
#include "foc_algorithm.h"
#include "foc_pid.h"
#include "foc_sensorless.h"

/* 电机运行状态实例：定义在本模块，其他模块通过本地 extern 或本头文件引用。 */
extern MotorControl_TypeDef MotorControl;
extern PI_Controller_TypeDef PI_Speed;
extern Encoder_TypeDef OnBoard_Encoder;
extern Fluxobserver_TypeDef Fluxobserver;
extern SensorlessStartup_TypeDef SensorlessStartup;
extern ModeNow_TypeDef ModeLast;
extern ErrorNow_TypeDef ErrorLast;
extern FOC_TypeDef FOC;

/**
 * @brief 查询启动所需配置是否已通过校验。
 * @return 轴身份、参数和必要校准均有效时返回 true，否则返回 false。
 * @note 只读查询，不清除故障、不写参数，也不使能电机。
 */
bool MotorControl_IsConfigurationValid(void);
/**
 * @brief 初始化电机控制对象、参数缓存和控制调度状态。
 * @note 由启动组合层调用；完成初始化不代表配置有效，也不会自动使能功率输出。
 */
void MotorControl_Init(void);

#endif
