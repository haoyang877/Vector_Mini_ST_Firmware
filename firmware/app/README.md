# APP组装与运行

当前v1.3.0核心范围；[完整模块导航](../../docs/architecture/module_core_responsibilities.md)。

| 目录 | 职责 | 阅读入口 |
| --- | --- | --- |
| composition | 初始化、快慢拍入口、协议与所有者绑定 | [board_config.c](composition/board_config.c) |
| runtime | 单轴对象、控制/维护调度、参数事务和固定工作区 | [motor_axis.c](runtime/motor_axis.c) |
| motor_state_machine | 唯一生命周期、启停准入、故障及资源归还 | [motor_state_machine.c](motor_state_machine/motor_state_machine.c) |

20kHz快拍执行采样、保护及一拍控制/标定；2kHz慢拍执行估计、外环和主状态监督。
前台处理通信和停机SAVE，Flash不在ISR执行。
单轴对象保留Watch可见全局符号，跨模块写入仍通过请求或显式借用对象。
状态机上下文拥有迁移；控制器拥有积分/滤波；维护任务拥有标定内部阶段。
固定工作区仅由编码器采样与参数记录互斥使用，不包含齿槽候选或heap。
RTT、组合计划、运行租约、摩擦/电阻辨识和齿槽任务已删除。

详见[主状态机](../../docs/architecture/motor_state_machine_v1.md)、
[控制执行](../../docs/architecture/motor_execution_core_v3.md)、
[维护](../../docs/architecture/motor_maintenance_core_v3.md)、
[四类参数](../../docs/architecture/parameter_ownership_v1.md)。
