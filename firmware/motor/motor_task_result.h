#ifndef MOTOR_TASK_RESULT_H
#define MOTOR_TASK_RESULT_H

#include "data_type.h"

/* 功能只报告事实，下一任务和功率许可由 APP 所有者决定；无全局模式字段。 */
typedef enum
{
    MOTOR_TASK_RUNNING = 0,
    MOTOR_TASK_PREPARED,       /* 零矢量已准备，申请所有者复核输出许可 */
    MOTOR_TASK_COMPLETED,      /* 算法完成，不表示参数已持久化 */
    MOTOR_TASK_FINALIZE,       /* 采集结束，等待关相后前台计算 */
    MOTOR_TASK_STOP_REQUESTED, /* 控制保护要求关断输出 */
    MOTOR_TASK_FAILED
} MotorTaskState;

typedef struct
{
    MotorTaskState result;
    ErrorNow_TypeDef error; /* 仅 FAILED 有效 */
} MotorTaskResult;

#endif
