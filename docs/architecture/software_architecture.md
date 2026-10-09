# 电机驱动软件架构

更新：2026-10-09。当前v1.3.0以核心控制、单项标定、参数、保护及升级恢复为范围。

## 产品与层级

```text
host_app：独立主机工具，通过通信访问设备，不链接进固件
APP：composition组装
       ├─ runtime单轴调度 + 主状态机
       ├─ services参数/请求/快照 + communication业务
       ├─ motor控制、反馈、标定、保护
       └─ platform能力接口 → STM32G4 BSP/ports/CubeMX
Loader：独立启动/升级产品
shared：APP/Loader共用的纯codec、CRC、镜像、启动邮箱和目标契约
```

APP决定什么时候执行，motor计算结果，platform完成I/O，communication转换字节与请求。
主状态机独占启停许可，标定任务只管内部阶段；它们不互相复制状态机。
MotorAxis拥有单份活动配置和运行对象，控制器拥有积分/滤波，参数事务拥有修订与保存身份。
部分对象是Watch可见全局；模块间仍通过请求、快照及显式对象交互，未宣称所有状态都是私有变量。

## 上电、运行和维护

配套Loader校验并跳APP；独立APP直接从芯片入口启动。
外设初始化后装载配置与LUT，在关相下采集电流零偏并完成INIT自检，进入IDLE等待。
明确START选择一种模式，RUNNING只更新同模式目标；STOP关相后回IDLE。
单项LUT/零位由任务内部推进，成功回待机后另行SAVE；Flash只在前台执行。
故障先关相，再收敛FAULT；恢复与CLEAR成功后仅回待机，不重新启动。

没有自动全套标定、轨迹管理、齿槽补偿、摩擦/电阻辨识、RTT、运行租约或heap。
硬件、电机、应用和设备配置按来源与职责分组，持久布局仍为schema12。

## 深入阅读

- [模块输入、输出与全局符号归属](module_core_responsibilities.md)
- [八状态及启停/故障流程](motor_state_machine_v1.md)
- [控制执行和直接位置串级](motor_execution_core_v3.md)
- [单项维护、编码器标定和真实保存](motor_maintenance_core_v3.md)
- [四类参数与兼容限制](parameter_ownership_v1.md)
- [通信分层](communication_layering.md)、[Loader设计](loader/loader_product_v3.md)
- [实际目录归属](code_structure.md)、[候选及未测项](../releases/architecture-app-v1.3.0-candidate.md)

本页描述源码职责，不代表当前候选已通过实机验收。
