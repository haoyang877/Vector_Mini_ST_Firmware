# APP应用层

应用层保留启停、同模式目标、三类标定、实时状态、开机参数装载和必要保护。
116直接提交现有目标，108/124共用状态读取；见[接口v1.9](../../docs/protocols/README.md#control)。
在线参数保存和任务进度仍不提供。

| 模块 | 职责 | 入口 |
| --- | --- | --- |
| composition/app.c | 初始化、前台、快慢调度、看门狗与Loader交接 | [app.h](composition/app.h) |
| composition/app_protocol.c | 协议绑定、请求/结果映射和只读状态刷新 | [app_protocol.h](composition/app_protocol.h) |
| runtime/motor_axis.c | 持有单轴对象，执行采样和控制/标定调度 | [motor_axis.h](runtime/motor_axis.h) |
| runtime/motor_lifecycle.c | 启停、同模式目标、标定会话、状态读取与功率保护 | [motor_lifecycle.h](runtime/motor_lifecycle.h) |
| motor_state_machine | 五态纯迁移，自检占位 | [motor_state_machine.h](motor_state_machine/motor_state_machine.h) |
| interfaces | 核心状态值、停机请求和实际关相确认 | [说明](interfaces/README.md) |

两板main依次调用Bsp_Init、App_Init、Bsp_Start、App_Start，前台循环调用App_Process。
AT32在main的ADC入口确认完成后直接执行轴快环和健康计数；STM32保留HAL完成回调。
两板共享g_app_fast_samples供前台看门狗观察，监督回调进入App_SupervisorTick。
20kHz快环先完成采样，再执行当前控制或标定；2 kHz 监督拍执行状态检查。AT32F456/YQ-5708 每五个有效 ADC 快拍派发一次 4 kHz 外环，其他配置在监督拍执行外环。
前台依次处理标定候选发布、状态刷新、通信、Loader交接和健康监督。

三类标定通过通信160的kind 1、2、6进入生命周期，完成后关相回空闲；
编码器LUT候选在前台核对原会话并发布。标定结果仅更新RAM，不自动写Flash。
看门狗独立核对快慢拍是否推进；Loader确认和交接保留健康、超时及最终关相复核。

运行对象仅由Axis持有，生命周期拥有运行状态；控制算法留在motor，硬件操作经BSP。
协议层只接收值对象和明确服务，不读取Watch对象或持有可写电机对象。

在线参数读写和修订号已删除，协议返回UNSUPPORTED；见[核心v1.8](../../docs/protocols/README.md#implementation)。
