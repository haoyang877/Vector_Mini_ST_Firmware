# CAN命令与ADC快环抢占隔离

## 改动

维护、启动、STOP、目标、参数和状态查询的应用互斥改用`BspSystem_EnterControlCritical`：BASEPRI阈值2，不写PRIMASK。AT32的ADC优先级0，载波与采样DMA优先级1；STM32的ADC优先级0，低频监督优先级3。快环始终可抢占前台和监督。

CAN前台只完成任务准备并最后发布原代际。`MotorStateMachine_BeginFastTick`在本拍感测和故障检查之后、worker之前重核动态保护，消费一次启相请求。STOP先发布撤销；故障、取消及过时代际不能启相。普通启动的STARTED仍须监督回读实际输出后确认，受理不等于物理启相。

CAN帧队列由唯一生产者和消费者各自维护累计序号，完整槽复制后才发布；保留原槽索引，因此任意容量在32位序号回绕后仍正常。公共BSP发送票据仍只有前台一个所有者；硬件互斥只屏蔽CAN相关中断，STM32的ADC仍能抢占HAL操作。

撤销全局锁后同步处理的共享数据问题：

- 故障以32位排他访问锁存，低频故障报告与CLEAR不能覆盖抢占期间的新故障。内部运行对象布局改变，线上枚举和持久化格式不变。
- 维护工作区原子认领、归还；状态查询不再通过工作区查询间接关闭全局中断。
- LUT后台复制前暂时撤销有效资格，复制完整后发布，ADC不读取半张表。发布只允许关相维护状态。
- 运行状态快照保持三次有界重试；持续抢占时返回测量有效位0，不再退回全局锁。生命周期仍读取当前所有者。
- 功率窗口检查在不一致时复读，遥测SPSC消费不再复制整帧时屏蔽ADC。

STOP、快故障关断、PWM提交期限、50us快环超时、一次性功率窗口和保护阈值均保留。停机Flash擦写仍使用BSP内部独占与采样暂停；它是关相后的持久化过程，不属于普通命令受理临界区。不能声称Flash擦写期间快采样连续。

## 离线证据

基线清单：`outputs/bsp-v1-implementation-20261026/can-preemptible-baseline-20261029.json`。

固件快照：`outputs/bsp-v1-implementation-20261026/can-preemptible-final-20261029.json`。

双板重建：`can-preemptible-final-build/builds.json`与`can-preemptible-final-stm32-build/builds.json`，均位于上述outputs父目录。AT32 Commissioning与STM32G431 APP均为0错误、0警告。

完整PR回归：`outputs/runs/can-preemptible-verified-20261029/summary.json`为PASS，全部42项离线测试通过；架构、接口、格式及文档检查通过。当前生产固件文件与双板构建快照逐文件SHA-256一致。

## AT32实测

时序观测仅位于独立outputs冻结树，未加入生产工程；首次主机采集误把未初始化观测帧计入最小值，该次不作为通过证据。修正主机初始化后，固件未改，连续三次有界零位及CAN查询负载测试通过：

| 项目 | 本轮观测 |
| --- | --- |
| 零位控制帧 | 15,107拍 |
| PWM提交失败 | 0 |
| CAN维护受理时PRIMASK | 三次均0 |
| CAN受理期间ADC实际抢占 | 第一次1次、第二次0次、第三次1次 |
| 首拍到PWM检查 | 33.456～33.833us |
| ADC中断最长时间，含观测开销 | 37.389us，保留原50us保护 |
| PWM检查时最小剩余时间 | 14.772us |
| 扣除原256计数保护后的最小余量 | 13.350us |
| 有界测试持久化参数 | 三次均未改写 |

这证明本轮负载下，前台正在受理CAN命令时ADC仍可进入；原先全局屏蔽导致首拍延后的路径已移除。上述为有限实测的观察值，不替代所有工况的WCET证明。

时序证据：`outputs/bsp-v1-implementation-20261026/hil-can-preemptible-20261029/summary.json`及`preemption-v2-0.json`、`preemption-v2-1.json`、`preemption-v2-2.json`。

随后已下载不含观测代码的正式修改版，整片读回候选字节差异0、未写页变化0，原参数保留。HEX SHA-256：`9eb91d2f57ca8a4e4906b6e229c1cd1d11153c4398ad52a03f8d9dba311d6de5`。

正式修改版完整电零位任务通过，耗时1.606s；任务原身份完成，未再触发PWM提交超时。电零位18681已保存，CRC、RAM/Flash及复位重载一致；LUT哈希不变、无无关参数修改。随后100次CAN状态查询通过，快环约19,997.62Hz、监督2,000Hz，ADC故障位0、已初始化且运行，看门狗调试冻结位0。最终CPU运行、IDLE、无故障、实际关相。

正式修改版证据：`outputs/bsp-v1-implementation-20261026/hil-can-preemptible-production-20261029/program.json`、`can-zero3-result.json`和`final-check.json`。本轮没有扩展到速度闭环品质验证；STM32只有离线回归和重建，没有实板测试。
