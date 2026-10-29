# 共用电机BSP与AT32关相验收

日期：2026-10-26。验证源码为冻结快照提交 `d1922f581d1d4dfa96995fb294e472b58e38d56b`，不是当前工作树提交。
关联[完成计划](../../plans/completed/2026-10-26-common-motor-bsp-v1.md)和
[现行接口说明](../../architecture/platform_interface_implementations_v1.md)。

## 实现与设计复核

采样、PWM、编码器、CAN分别只有一份公共BSP实现，两板九个工程均链接这四个文件。
公共BSP持有逻辑通道/完整辅助帧、输出许可协调、编码器分段流程和CAN软件票据；
mcu目录负责寄存器、SDK、DMA、SPI事务和中断确认，devices目录只负责编码器命令与解码。
使用编译期选择与直接调用，没有运行时注册表。板配置每板一份，应用使能和试验目标仍在app/config。

共用层不包含厂商头、不按板名分支；两个MCU实现不再重复导出四类BSP公共入口。
原有快帧四次累加计数、温度消费时点、PWM六路操作顺序、立即关相、编码器失败收尾、
CAN物理槽差异、完整64位票据和错误恢复保持。配置/保护阈值的五组编译探针对比逐位相同。
STM32生成外设配置仍须与板配置同步，不能仅凭更改宏就支持任意外设组合。

151个SDK文件和19个生成文件逐字节不变；167个受保护控制/参数/协议等源文件接受正文检查。
其中APP描述符文件仅增加独立镜像尾对象，原96字节描述符初始化正文保持。
12个公共头的声明与数据布局保持，检查忽略包含路径与注释的变化。

## 离线验收

九个APP/台架工程和独立STM32 Loader均为0错误、0警告。
工作树PR与干净冻结快照release均通过：119项单元、14项集成、37组原生回归。
覆盖ADC/DMA/存储恢复、编码器CRC与SPI失败收尾、真实HAL的PWM启停轨迹、CAN超时/迟到完成/bus-off，
以及23种能力配置、16种ADC组合、两板禁用CAN与27种非法配置拒绝。
控制算法继续与既有基线运行等价回归；硬件热函数仍位于AT32的SRAM。

重排代码后STM32 LoaderApp的装载长度不再天然满足8字节对齐。
修复是在链接脚本尾部放置对齐的独立8字节常量区，同步生成器并分离RW/ZI区域供地址计算；
未放宽打包校验，未修改描述符ABI或Flash/RAM可用边界。
最终真实打包通过，装载跨度83992字节。规则依据为
[Arm scatter表达式文档](https://support.arm.com/documentation/dui0377/g/Scatter-File-Syntax/Expression-evaluation-in-scatter-files/Expression-rules-in-scatter-files?lang=en)，实际结果以本批构建日志为准。

| 目标 | ROM变化/字节 | RAM变化/字节 | 静态最大栈/字节 |
| --- | --- | --- | --- |
| ESC-APP-AT32F456-Commissioning | +152 | +0 | 608 → 612 |
| ESC-APP-AT32F456 | +104 | +16 | 608 → 612 |
| R40_F456_ClosedLoop | +728 | -8 | 280 → 280 |
| R40_F456_Foundation | +740 | +0 | 416 → 424 |
| R40_F456_OpenLoop_Combined | +728 | -8 | 536 → 536 |
| R40_F456_OpenLoop_Dry | +680 | -8 | 176 → 168 |
| R40_F456_OpenLoop_Power | +724 | -8 | 176 → 168 |
| ESC-APP-STM32G431-LoaderApp | +500 | +16 | 568 → 580 |
| ESC-APP-STM32G431 | +484 | +16 | 568 → 580 |

ROM来自MAP的Total ROM Size，包含实际装载压缩影响；不是简单相加Code/RO/RW。
静态最大栈报告均含Unknown项，不能作为完整上界。AT32采样、编码器完成/解码、PWM写入和CAN接收关键符号的SRAM位置已逐项核对。

## AT32实机验证

沿用用户对AT32永久关相镜像烧录、复位和只读验证的明确授权。
先核对芯片/探针/CAN适配器和关相状态，备份256KiB Flash，并对原镜像做30秒只读通信基线。
候选有效装载字节：77100；HEX SHA-256：`67f7660e06046414e90a44c530770c6634522e59bd413c1d8e31131992e0c870`。
只改APP涉及扇区，逐字节读回并验证其余扇区及末尾16KiB参数区保持后复位。

| 项目 | 结果 |
| --- | --- |
| 只读CAN窗口 | 30.003秒 |
| 有效回复 | 588次，无查询超时失败 |
| 坏CRC查询 | 拒绝，随后合法查询正常 |
| 采样平均频率 | 20001.815 Hz |
| 监督平均频率 | 2000.698 Hz |
| ADC故障/编码器传输错误 | 检查点均为0 |
| 参数 | 七项CAN读数及完整参数区烧录前后相同 |
| 最终状态 | CPU运行，关相，Flash与烧录读回一致 |

未启相、未转动，没有发送控制或参数写入命令。设备保留本次永久关相镜像。
STM32仅编译和离线回归，未连接实机；没有验证PWM波形、带载控制、动态编码器或采样精度。
主机时间只给出平均节拍，不是WCET或中断延迟结论。

## 剩余精简项

已完成本轮共用逻辑收敛，但文件级精简尚有候选：STM32的param_store_stm32g4.c只转发读写，
pwm_setup.c只转发初始化；can_disabled.h包含禁用实现，可收回bsp_can.c；
pwm_output_permit只由bsp_pwm.c使用，可作为其私有逻辑。此次冻结后未继续修改这些文件。
合并许可函数必须保留拒绝统计、永久关相和限时输出规则。
SPI事务、ADC存储暂停恢复、器件协议和实际限时窗口状态仍有独立职责，不能为减少文件数删除。
bus_current_feedback目前有两个实际调用方，也不是无意义的转发。

## 原始证据

根目录outputs/platform-common-20261026：baseline/candidate清单记录源码，builds-release保存九目标产物，
design-audit、constant-comparison、resource-comparison分别记录边界、常量及资源核对，summary.json聚合最终结果。
PR在outputs/runs/common-bsp-pr-release-20261026；release在candidate指定快照的outputs/runs/common-bsp-release-20261026。
hil保存脚本、候选清单、基线/烧录读回/最终Flash与通信回复。保留的早期失败日志不作为通过证据。
