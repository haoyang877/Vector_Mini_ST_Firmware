# APP 文件职责与阅读顺序

更新：2026-10-30。以当前源码为准；执行过程见[软件架构](software_architecture.md)。

| 阅读顺序 | 文件 | 责任 |
| --- | --- | --- |
| 1 | projects/<board>/application/main.c | 唯一主入口和 ADC 应用回调；按同一模板启动 BSP 与 APP |
| 2 | firmware/app/composition/app.c/h | 生命周期组装、快慢调度、前台、看门狗和 Loader 交接 |
| 3 | firmware/app/runtime/motor_axis.c/h | 单轴对象、启动装载、采样/控制/标定调度、前台候选发布 |
| 4 | firmware/app/runtime/motor_lifecycle.c/h | 启停和标定准入、代际、故障与输出动作 |
| 5 | firmware/app/motor_state_machine/motor_state_machine.c/h | 五态纯迁移，不保存 s_context 或访问硬件 |
| 6 | firmware/app/composition/app_protocol.c/h | 服务绑定、请求结果映射、实时状态缩放 |
| 7 | firmware/app/interfaces | 状态/请求值与 STOP 实际关相确认 |
| 8 | firmware/app/composition/app_rtt.c | RTT缓冲与发送；板级初始化调用RTT_Init，快环调用RTT_Sampling |

配置与运行对象分离；算法位于 motor，记录和值校验位于 parameters，
线上编码位于 communication，寄存器和引脚操作位于 platform。
通信只依赖服务/值契约，不依赖 APP runtime。

当前没有 app_tasks、独立 startup、命令邮箱、参数事务、维护所有者或功率窗口模块。
不要从历史文件名推导出另一条运行链路。标定结果只更新 RAM；116通过生命周期提交同模式目标，108/124返回实时状态，参数服务仍不开放。

s_motor_configuration、s_motor_state、s_foc_state、s_rotor_encoder 位于 motor_axis.c；
s_context 位于 motor_lifecycle.c，均保留只读 Watch 名称。
