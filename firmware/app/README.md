# APP组装与运行

当前核心范围；[完整模块导航](../../docs/architecture/module_core_responsibilities.md)。

从[app.h](composition/app.h)开始读；只向芯片入口公开四个应用调用：

完整调用流程及2kHz执行次序评审统一维护在[软件架构主流程](../../docs/architecture/software_architecture.md)。

| 入口 | 实现 | 调用方与职责 |
| --- | --- | --- |
| App_Init | app_startup.c前部 | main完成外设初始化后调用一次：参数/电机、采样、通信依次启动 |
| App_Process | app_startup.c前部 | main的while循环：后台事务、状态刷新、通信及私有Loader交接 |
| App_FastTick | app_tasks.c | ADC完整JEOS回调，执行既有20kHz采样、保护和控制/标定 |
| App_SupervisorTick | app_tasks.c | TIM7回调，执行既有2kHz监督、主状态推进和指示器分频 |

随后阅读runtime/motor_axis.c的Init、FastTick、SupervisorTick和ServiceBackground，
再从MotorStateMachine_Tick进入主状态机。四入口是执行上下文边界，不增加线程或调度器。

| 目录 | 职责 | 阅读入口 |
| --- | --- | --- |
| composition | 四个公开入口；后台编排、协议与所有者绑定 | [app.h](composition/app.h)、[app_startup.c](composition/app_startup.c) |
| runtime | 单轴对象、控制/维护调度、参数事务和固定工作区 | [motor_axis.c](runtime/motor_axis.c) |
| motor_state_machine | 唯一生命周期、启停准入、故障及资源归还 | [motor_state_machine.c](motor_state_machine/motor_state_machine.c) |

20kHz快拍执行采样、保护及一拍控制/标定；2kHz慢拍执行估计、外环和主状态监督。
前台处理通信和停机SAVE，Flash不在ISR执行。
runtime不实现外环、热模型或ADC计数处理：分别调用motor控制执行、保护及采样接口，
只传入所属对象和当前事实。命令格式/范围校验仍归APP，不反向引入motor算法。
单轴对象保留Watch可见全局符号，跨模块写入仍通过请求或显式借用对象。
状态机上下文拥有迁移；控制器拥有积分/滤波；维护任务拥有标定内部阶段。
固定工作区仅由编码器采样与参数记录互斥使用，不包含齿槽候选或heap。
RTT、组合计划、运行租约、摩擦/电阻辨识和齿槽任务已删除。

详见[主状态机](../../docs/architecture/motor_state_machine_v1.md)、
[控制执行](../../docs/architecture/motor_execution_core_v3.md)、
[维护](../../docs/architecture/motor_maintenance_core_v3.md)、
[四类参数](../../docs/architecture/parameter_ownership_v1.md)。
