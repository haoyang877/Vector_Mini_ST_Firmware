# BSP公共接口与实现对应

include仅含8个公开功能头；bsp_internal.h位于BSP根目录。
未使用的微秒时基、独立复位和快速环阶段诊断已删除，单相PWM仅供底层台架。

当前范围：保留PWM、ADC、Encoder（SPI）、CAN与Flash及必要系统支撑。LED、UART已删除，
不保留接口、占位实现或工程引用。见[删除记录](../plans/active/2026-10-30-remove-led-uart-v1.md)。

完整文件树、每个文件的职责和数据所有权统一维护在[BSP架构总图](bsp_architecture_v1.md)。
函数签名、单位、失败和上下文统一维护在[BSP接口](bsp_interfaces_v1.md)及[公开头文件](../../firmware/platform/bsp/include)。

两板编码器由各自MCU encoder_spi.c直接实现统一Begin/ReadAngle，负责初始化和SPI/DMA收发，返回65536计数域。
BSP仅保留公开头，不增加转发层；无旧事务、诊断缓存或devices包装。
CAN两板共用同一公开头；DLC转换直接放入各MCU收发函数，MCU实现统一入口、启停和IRQ。
Start返回void，其余收发与忙/完成查询返回bool，没有状态枚举、软件票据或自动恢复。
其它功能的共享边界见下述模块约定。
两板ADC的初始化、原码、换算、零偏、启停和中断均在MCU实现，统一接口直接返回float。
Flash、系统、启动交接根据MCU直接实现相同契约。
两板PWM的MCU实现周期读取、比较值写入、启停和通道查询；占空比换算也在MCU中，无状态/结果码/永久锁。
应用限流与功率窗口仍在原应用模块，限时窗口不再依赖BSP关相序号。

CAN命令和低频监督的共享状态使用`BspSystem_EnterControlCritical`，保持ADC与载波可抢占，不使用PRIMASK全局锁。邮箱发布使用内存屏障，故障/共享工作区使用32位排他访问；这些原语不替代数据所有权。实际启相由ADC快环固定入口消费完整请求，状态快照持续冲突时明确清除测量有效位，不能为查询阻塞快环。关相Flash擦写仍按存储契约暂停采样。实现与实测证据见[CAN抢占隔离报告](../reports/2026-10/can_fast_loop_preemption_20261029.md)。

当前未完成实机验收，离线通过和上电通过分别记录，不再维护另一份易失效的逐函数表。
