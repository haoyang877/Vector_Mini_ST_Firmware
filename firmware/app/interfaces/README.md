# APP核心值与停机接口

本目录只保留当前启停、标定和状态查询实际使用的契约，不持有运行对象或访问硬件。

| 文件 | 职责 |
| --- | --- |
| motor_state_types.h | 五态、请求结果、三类标定和状态/模式/故障值 |
| motor_stop_request.c/h | 向所有者请求停止，复核实际关相并返回受理编号 |

生命周期和功率动作由[MotorLifecycle](../runtime/motor_lifecycle.c)执行，
运行对象由[MotorAxis](../runtime/motor_axis.c)持有。
没有目标命令、进度查询、启动资格明细或ISR状态邮箱。
