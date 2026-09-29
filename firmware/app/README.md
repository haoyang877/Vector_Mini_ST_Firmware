# APP 组装与运行

| 目录 | 职责 | 从哪里读起 |
| --- | --- | --- |
| composition | 启动、任务入口、协议到业务的绑定 | [Board_Init](composition/board_config.c)、bsp_task、foc_task |
| runtime | 单轴对象、快慢环执行、维护和当前 RTT 适配 | [motor_axis](runtime/motor_axis.c)、motor_execution、motor_maintenance |
| motor_state_machine | 唯一主生命周期、请求与动作；规则是内部实现 | [motor_state_machine](motor_state_machine/motor_state_machine.c) |

composition 绑定端口和选择实现，不计算电机算法；runtime 调度真实功能并发布结果。
运行对象由 motor_axis 私有持有，主状态机的状态和事务由其私有上下文持有。
其他模块通过请求与值快照交流，不公开可写实例。
maintenance_workspace 私有持有一份固定联合工作区，参数、齿槽候选与编码器采样互斥。
标定任务显式借用采样对象；实际归还后才允许复用，不使用堆分配器。

motor_execution 仍含外环协调，RTT 适配仍读取外环就绪事实，因此属于 APP 运行组装。
参数已按 model/application/device 分组，硬件配置编译期只读。
通信只接收所有者传出的节点值，不作为参数存储；不增加完整记录或 LUT 副本。
入口与兼容边界见[参数所有权](../../docs/architecture/parameter_ownership_v1.md)。
