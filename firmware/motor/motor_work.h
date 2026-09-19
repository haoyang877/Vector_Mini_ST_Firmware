#ifndef MOTOR_WORK_H
#define MOTOR_WORK_H

#include "data_type.h"

/* 模式 worker 的周期结果：由控制/标定任务返回，运行状态机是唯一消费者。
 * 迁移期约定：返回 SWITCH_MODE/STOP/FAULT 的任务不再自行写 ModeNow；
 * power_off 用于声明"切换前先关断功率级"（等价于模块原先自行 Stop_PWM），
 * 尚未迁移的任务仍按旧方式自行写入，状态机按兼容差分处理（见运行状态机计划）。 */
typedef enum
{
    MOTOR_WORK_RUNNING = 0, /* 继续当前模式 */
    MOTOR_WORK_SWITCH_MODE, /* 请求切换到 next_mode（内部完成/阶段推进） */
    MOTOR_WORK_STOP,        /* 请求停机，等价于原自行写入 Motor_Disable */
    MOTOR_WORK_FAULT        /* 上报故障码，由状态机锁存并在同拍停机 */
} MotorWorkResult_TypeDef;

/* worker 结果载荷：除 result 外，仅对应分支读取的字段有效。 */
typedef struct
{
    MotorWorkResult_TypeDef result;
    ModeNow_TypeDef next_mode; /* 仅 MOTOR_WORK_SWITCH_MODE 有效 */
    ErrorNow_TypeDef error;    /* 仅 MOTOR_WORK_FAULT 有效 */
    bool power_off;            /* 仅 MOTOR_WORK_SWITCH_MODE 有效：应用模式前先 Stop_PWM */
} MotorWorkOutcome_TypeDef;

#endif
