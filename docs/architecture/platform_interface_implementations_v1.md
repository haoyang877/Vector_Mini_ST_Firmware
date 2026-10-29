# BSP公共接口与实现对应

完整文件树、每个文件的职责和数据所有权统一维护在[BSP架构总图](bsp_architecture_v1.md)。
函数签名、单位、失败和上下文统一维护在[BSP接口](bsp_interfaces_v1.md)及[公开头文件](../../firmware/platform/bsp/include)。

两板共享ADC、PWM、编码器、CAN、LED的实际功能流程；MCU目录实现寄存器/SDK操作，devices实现器件协议。
UART当前未绑定，统一返回未支持。Flash、系统、启动交接根据MCU直接实现相同契约。
应用限流与功率窗口在app/config和app/runtime，BSP只消费目标提供的永久关相输入。

CAN命令和低频监督的共享状态使用`BspSystem_EnterControlCritical`，保持ADC与载波可抢占，不使用PRIMASK全局锁。邮箱发布使用内存屏障，故障/共享工作区使用32位排他访问；这些原语不替代数据所有权。实际启相由ADC快环固定入口消费完整请求，状态快照持续冲突时明确清除测量有效位，不能为查询阻塞快环。关相Flash擦写仍按存储契约暂停采样。实现与实测证据见[CAN抢占隔离报告](../reports/2026-10/can_fast_loop_preemption_20261029.md)。

当前未完成实机验收，离线通过和上电通过分别记录，不再维护另一份易失效的逐函数表。
