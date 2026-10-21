# APP公共请求与快照接口

通信和启动组装通过本目录的窄契约调用APP，不包含runtime或主状态机私有头文件。
本目录只使用值对象、显式借用的所有者端口及有界邮箱，不读取Watch全局或直接访问硬件。

| 文件 | 职责 |
| --- | --- |
| motor_control_command.h | SI控制目标、来源/序号、受理与应用进展 |
| motor_lifecycle_request.h | 单项维护请求、票据、阶段及提交效果 |
| motor_readiness.h | 自检事实与启动阻断原因 |
| motor_stop_request.c/h | 转交停机请求，真实关闭确认后才报告完成 |
| motor_status_mailbox.c/h | ISR生产、前台消费的单槽只读状态快照 |

主生命周期和功率动作仍由[主状态机](../motor_state_machine/motor_state_machine.c)执行，
运行对象仍由[MotorAxis](../runtime/motor_axis.c)持有。这里没有第二个状态机或可写全局访问器。
