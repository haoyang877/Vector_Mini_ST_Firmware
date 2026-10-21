# 虚拟平台第一、二阶段

## 目标与边界

从system-refactor的3a099dc821695491836064d811e6faa4389e728e独立开发。
分支codex/virtual-platform；不复制主线未提交文档整理和参数注释。
本批只冻结接口契约、接通原APP的静态虚拟外设，保持控制数学、保护阈值、
通信/参数ABI、控制状态所有权不变。不实现动力学、不接硬件、不修改Keil输入。

## 接口和目录

保留platform/api及stm32g4目录，新增platform/virtual：
virtual_platform.h只对台架开放初始化、推进、只读快照和CAN队列；
virtual_platform.c实现现有API。tests/virtual_bench/main.c使用四个真实APP入口。
tools/build/virtual_bench.py复用normal工程的可移植C源码清单，直接编译原文件，
不抽取函数体、不复制控制算法、不引入HAL寄存器替身。
独立APP的Boot能力直接复用boot_hw_unavailable.c，不再复制相同空实现。

| 契约 | 真实实现 | 本批虚拟实现 |
| --- | --- | --- |
| motor_sensing | ADC注入采样及温度 | 三相不同静态零点、母线量化、固定温度 |
| encoder_sensor | TLE5012B分阶段SPI | 静态角度，begin/readback/complete同步回退 |
| motor_hw/power_stage_hw | PWM及六路使能位 | 保存占空比、使能事实；启相即台架失败并停止推进 |
| board_hw | 采样启动、UID及身份页 | 默认关相、虚拟UID、空白身份记录 |
| comm_hw | CAN FD及物理完成槽 | 有界队列；消费跟踪帧才完成，100ms超时失败 |
| param_store | Flash字节读写与回读 | 初始空存储回退真实默认值；内存写入与回读 |
| time_hw/critical_hw | MCU时钟和中断屏蔽 | 单线程虚拟时钟与嵌套配对，不覆盖抢占竞争 |
| boot_hw | 邮箱、复位、确认 | 独立APP明确不支持，不模拟升级成功 |
| indicator_hw | LED/RGB | 无光学模型，不参与闭环判断 |

电流/电压原始值为12位转换四次和0..16380；零点输入为单次计数。
编码器沿用angle_q15编码，当前真实TLE驱动最低位为0；不修订现有命名。
PWM写入不隐含使能。20kHz每50us，2kHz每10拍；同刻先接收、快环、监督、前台。
前台每快拍执行一次是可重复的台架调度，不声称等效MCU执行耗时。
模型真值接口留到动力学阶段；本批静态输入不支持任何运动结论。

## 验收、证据与回退

一进程只初始化一次；推进1.6秒虚拟时间，原APP完成上电电流校正进入IDLE，
无故障、无目标电流、从未启相，时钟与快慢拍数匹配。
编译记录完整原生产源码清单、头文件哈希、日志及结果，归outputs/virtual_bench。
检查原控制源码/ABI/Keil输入无差异；执行仓库PR检查，失败如实记录。
不宣称速度闭环、标定、CAN物理时序、Loader、MCU耗时或实机PASS。
后续加入独立逆变器/电机模型，分开控制器参数与对象真实参数。
回退只移除此批新增平台、台架、命令与文档，无主线运行态迁移。

实际首轮1秒时仍INIT：原上电校正本身需要1秒，随后还需稳定窗口；
台架改为先检查0.5秒INIT、再推进至1.5秒IDLE，不改生产守卫和状态。
最后100ms用于发送超时检查，总32000快拍、3200监督拍。

## 本批结果

- 启动场景：32000快拍/3200监督拍，虚拟时间1600000us，IDLE、NONE、fault=0，
  自检八项全部满足；配置未确认的启动限制仍在，数字输出始终关闭、从未启相。
- 虚拟CAN：64位票据不截断、入队不算完成、完成一次消费；超时由原协议前台
  消费失败事件，过期帧撤销且发送槽释放；非法DLC拒绝。
- 原APP/控制/协议/参数/STM32工程输入无修改，只新增虚拟实现与台架。
- 仓库PR：outputs/runs/virtual-platform-stage12/summary.json全部PASS；
  最后台架调整后重新执行场景、format --check和lint，均PASS。
- 最终直接证据：outputs/virtual_bench/startup/summary.json、compile.log、run.log。
  原生产源码和头文件以该summary哈希为准；未进行Keil构建或任何实机操作。

后续范围仍为第三阶段动力学闭环；本批不提前创建模型占位接口。
