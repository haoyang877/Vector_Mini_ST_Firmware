# BSP最小直接实现与AT32关相验收

日期：2026-10-26。源码为冻结快照`01d1ea1f6849dd13a02c3ed5277aabdf887868eb`，不是当前工作树提交。
关联[完成计划](../../plans/completed/2026-10-26-minimal-bsp-v1.md)和
[现行接口说明](../../architecture/platform_interface_implementations_v1.md)。

## 完成的简化

手写BSP的C/H文件从81减为64，源码从6462行减为5927行；
不计SDK、生成文件、README。删除17个文件，未增加生产代码文件或运行时转发层。

| 原边界 | 最终实现 |
| --- | --- |
| pwm_output_permit.c/.h | 许可函数收为公共bsp_pwm.c私有函数，拒绝计数、关相锁及限时规则保留 |
| 两板pwm_setup.c | McuPwm_Init与本板运行操作归同一pwm_backend.c |
| can_disabled.h | 禁用分支直接在公共bsp_can.c中定义，仍有稳定接口和失败返回 |
| STM32 flash.c/.h和参数转发 | param_store_stm32g4.c直接实现公共接口，Flash错误与忙超时锁存保留 |
| STM32 led.c/.h、rgb.c/.h | indicator_stm32g4.c直接更新LED/RGB状态和DMA缓冲；删除重复枚举、颜色映射及转发函数 |
| STM32 delay.c/.h | 没有实际延时调用；删除未使用实现及仅为它赋值的初始化 |
| STM32 hw_conf.h、pwm_timer_config.h | 删除重复聚合/别名；控制时基显式引用原配置，ADC按原寄存器读取 |
| AT32 sampling_diagnostics.c/.h | sin/cos发布归sampling_backend私有函数，版本顺序、分频与对外快照保留 |

限时AT32链接脚本删除旧许可对象选择，合并后的PWM初始化显式留在Flash；运行期热代码仍在SRAM。
板配置、功能使能和应用策略位置保持，没有把目标策略复制到另一套配置。

## 保留文件的理由

公共目录的四个BSP是应用契约与共享流程；四个internal头明确真实MCU/协议边界。
两个编码器协议文件负责实际不同的MT6835/TLE5012B帧；SPI文件拥有物理事务及清理。
AT32 ADC配置、运行采集和存储暂停恢复分工不同，但共享同一状态对象，不新增影子状态。
温度换算、启动时钟、系统原语、参数存储和Boot接口分别有实际调用方或硬件状态。
bus_current_feedback有两个实际调用方，can_frame_codec为两个MCU复用，pwm_power_window持有一次性限时状态。
这些边界保留；后续新增代码先做私有函数，出现独立状态或真实复用需求再拆文件。

## 等价性和离线结果

151个SDK文件、19个生成文件、179个控制/参数/通信/目标配置等受保护源文件逐字节不变；
12个公共头逐字节不变。72个保留/迁移函数按规范化正文比较通过，迁移只归并位置或私有名称。
STM32指示器的状态逻辑简化单独验证：以重构前完整生产代码为基线，覆盖512组闪烁配置及28672次
LED/GPIO和RGB/DMA输出，固定签名`e6b1ec75 28672`保持。签名由基线生成，候选检查不能重写。

九个APP/台架工程、独立STM32 Loader均0错误、0警告；STM32 LoaderApp真实打包通过。
PR和干净快照release通过：119项单元、14项集成、38组原生回归。架构检查为零新增反向依赖。
关相锁、限时过期/不可续期、PWM真实HAL轨迹、采样/DMA/恢复失败、编码器CRC/超时、CAN故障及Flash错误路径继续运行。

| 目标 | ROM变化/字节 | RAM变化/字节 | 静态最大栈/字节 |
| --- | --- | --- | --- |
| ESC-APP-AT32F456-Commissioning | +0 | +0 | 612 → 612 |
| ESC-APP-AT32F456 | +0 | +0 | 612 → 612 |
| R40_F456_ClosedLoop | -60 | +0 | 280 → 280 |
| R40_F456_Foundation | -8 | +0 | 424 → 424 |
| R40_F456_OpenLoop_Combined | -60 | +0 | 536 → 536 |
| R40_F456_OpenLoop_Dry | -24 | +0 | 168 → 168 |
| R40_F456_OpenLoop_Power | -44 | +0 | 168 → 168 |
| ESC-APP-STM32G431-LoaderApp | -376 | +0 | 580 → 580 |
| ESC-APP-STM32G431 | -376 | +0 | 580 → 580 |

ROM按MAP的实际Total ROM Size计算。两板APP RAM和静态最大栈没有增加；静态报告含Unknown项，
不视为完整上界。AT32九个关键采样、编码器、PWM和CAN函数的SRAM地址逐项核对。

## AT32实机

沿用用户已批准的永久关相烧录和只读验证范围。核对设备/探针/适配器，备份完整Flash，
对原固件进行30秒只读基线后，仅更新候选APP涉及的扇区。读回镜像并比较其他扇区及参数区。
候选HEX SHA-256：`bb968aa259c62261183da5dc25a185bf4f76b030ce1c6b33d2d29eaf1f677adc`，装载字节77100。

| 检查 | 结果 |
| --- | --- |
| 只读验证窗口 | 30.001秒 |
| 有效CAN回复 | 594次；无查询超时失败 |
| 坏CRC查询 | 拒绝；随后合法查询正常 |
| 采样平均频率 | 19997.339Hz |
| 监督平均频率 | 1999.667Hz |
| ADC/编码器错误 | 检查点为0 |
| 参数保留 | 七项CAN参数和完整参数区前后相同 |
| 最终状态 | CPU运行、关相、Flash与烧录读回一致 |

全程不启相、不转动、不发送控制或参数写入命令。设备保留本次永久关相镜像。
STM32未连接实机；未测PWM波形、动态精度、带载控制。平均节拍不等于WCET或中断延迟。

## 原始证据

outputs/bsp-minimal-20261026保存baseline/release源码清单、删除列表、设计与资源比较、九目标构建、
Loader日志和hil全Flash读回/通信记录，summary.json为聚合结论。
PR为outputs/runs/minimal-bsp-pr-20261026；release在冻结快照的outputs/runs/minimal-bsp-release-20261026。
早期旧文件链接、测试桩和链接脚本警告的失败日志保留；仅最终结果用于通过结论。
