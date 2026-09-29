# 当前 APP 按功能的 Flash 占用（2026-09-28）

本表保留 `f0c2517` 的基线。后续第一批优化已回收 4080 B，最新结果见
[残留参数、数值编码与模块编译优化](app_flash_batch1_20260928.md)。

统计对象：清理旧模式、主状态机空间优化及正弦半表后的 Loader 配套 APP。
前两项已提交于 `568dd04`；半表源码、黄金数据和测试随本报告所在提交保存。
工具链为 ARM Compiler 5.06 update 7 build 960。本次 map 来自当前对象的诊断链接，
虚拟扩大 Flash 上限以取得完整分布；正式 78 KiB 区域仍链接失败，不能烧录。

## 统计口径与总量

- Code 为 106200 B，RO Data 为 4288 B，压缩后的 RW 初始化为 16 B。
- 实际 ROM 数据合计 **110504 B（107.91 KiB）**；固定描述符前还有 40 B 地址间隙。
- Flash 加载跨度 **110544 B（107.95 KiB）**，超过 79872 B 区域 **30672 B（29.95 KiB）**。
- 当前普通 APP 为 102240 B ROM；两种布局的增量分布在启动、状态、通信与公共库，
  不能只用某一个 boot 对象代表全部增量。
- 下表按功能所有者分配每个对象一次，计 Code + RO；共享驱动和公共库单独列出。
  这是已链接的占用，不是删除某功能必然可回收的字节数。
- RAM 的 ZI（31632 B）、调试符号和 AXF 文件大小不计入 Flash。Code 中的
  inc. data 已经包含在 Code 内，不重复累加。

## 1. 完整功能账单

| 功能 | 字节 | KiB | ROM 占比 |
| --- | ---: | ---: | ---: |
| HAL、外设初始化及平台驱动 | 24848 | 24.27 | 22.49% |
| CAN通信、协议与业务适配 | 14467 | 14.13 | 13.09% |
| 主状态机与停机生命周期 | 10232 | 9.99 | 9.26% |
| 位置级联控制与连续轨迹 | 7962 | 7.78 | 7.21% |
| Loader接入、镜像及记录校验 | 7368 | 7.20 | 6.67% |
| 调度、控制适配与电机状态 | 5840 | 5.70 | 5.28% |
| C与数学运行库 | 5550 | 5.42 | 5.02% |
| 齿槽标定、补偿和数据表管理 | 5176 | 5.05 | 4.68% |
| 参数、轴配置及存储服务 | 4700 | 4.59 | 4.25% |
| 编码器标定 | 4442 | 4.34 | 4.02% |
| 观测器与无感启动 | 3832 | 3.74 | 3.47% |
| 摩擦辨识 | 3494 | 3.41 | 3.16% |
| FOC电流环、采样和故障检测 | 3232 | 3.16 | 2.92% |
| 数学、内存池与CRC辅助 | 3034 | 2.96 | 2.75% |
| 相电阻辨识 | 3000 | 2.93 | 2.71% |
| 编码器角度反馈 | 1228 | 1.20 | 1.11% |
| RTT遥测 | 804 | 0.79 | 0.73% |
| 速度环与PI公共控制器 | 578 | 0.56 | 0.52% |
| 板级启动组合 | 530 | 0.52 | 0.48% |
| 链接填充和压缩RW初始化 | 187 | 0.18 | 0.17% |
| **ROM 合计** | **110504** | **107.91** | **100%** |

其中“数学、内存池与 CRC 辅助”中的正弦表为 **2048 B**，其余 986 B 为代码/常量。
四类标定/辨识（编码器、齿槽、摩擦、相电阻）合计 **16112 B（15.73 KiB）**；
观测器与无感启动另有 3832 B，编码器标定依赖这部分，不能重复加到多个标定项。

## 2. 大对象与功能组成

| 对象 | Code + RO | 用途 |
| --- | ---: | --- |
| `foc_run_state.o` | 8198 B | 主状态机与停机生命周期 |
| `position_cascade.o` | 6426 B | 位置级联控制与连续轨迹 |
| `foc_encoder_calibration.o` | 4442 B | 编码器标定 |
| `stm32g4xx_hal_tim.o` | 3984 B | HAL、外设初始化及平台驱动 |
| `yg_protocol_link.o` | 3700 B | CAN通信、协议与业务适配 |
| `foc_cogging_calibration.o` | 3264 B | 齿槽标定、补偿和数据表管理 |
| `foc_sensorless_run.o` | 3072 B | 观测器与无感启动 |
| `stm32g4xx_hal_adc.o` | 2928 B | HAL、外设初始化及平台驱动 |
| `utils.o` | 2630 B | 数学、内存池与CRC辅助 |
| `friction_identification.o` | 2404 B | 摩擦辨识 |
| `motor_state.o` | 2340 B | 调度、控制适配与电机状态 |
| `foc_run.o` | 2248 B | 调度、控制适配与电机状态 |
| `foc_param.o` | 2216 B | 参数、轴配置及存储服务 |
| `stm32g4xx_hal_adc_ex.o` | 2046 B | HAL、外设初始化及平台驱动 |
| `foc_algorithm.o` | 1890 B | FOC电流环、采样和故障检测 |
| `image_transfer.o` | 1884 B | Loader接入、镜像及记录校验 |

主状态机项包含 `foc_run_state` 8198 B、纯生命周期核心 1832 B、停止服务 202 B。
调度/适配项包含 `foc_run` 2248 B、`motor_state` 2340 B、模式准入调度及监督入口。
位置控制项为 `position_cascade` 6426 B 加连续加速度轨迹 1536 B；旧阻抗控制器
和旧梯形轨迹已经不在 map 中。齿槽项包含标定、补偿和记录管理程序，不能把其
约 4 KiB 的 RAM 工作区再加到 Flash 占用。

## 3. 平台驱动进一步拆分

按真实源路径统计，ST HAL 驱动本体 **18654 B（18.22 KiB）**；其余平台启动、
初始化、向量表、中断与端口合计 **6194 B（6.05 KiB）**。早期报告仅按对象名
前缀统计 HAL，漏计 `stm32g4xx_hal.o` 并包含了项目 MSP；此处以 vendor 源路径
明确边界，Flash 总量未因此发生变化。

| HAL 功能 | 字节 |
| --- | ---: |
| ADC（含扩展） | 4974 B |
| TIM（含扩展） | 4630 B |
| RCC 时钟（含扩展） | 2504 B |
| FDCAN | 1826 B |
| UART（含扩展） | 1768 B |
| DMA | 824 B |
| Flash 擦写（含扩展） | 824 B |
| GPIO | 444 B |
| SPI | 236 B |
| PWR、Cortex 与 HAL 基础 | 624 B |

这些是共享驱动占用。例如 TIM 同时支持电机采样/PWM 和板级定时需求，不能按调用
次数重复分摊。UART 驱动虽然没有当前业务服务，也因生成初始化调用仍被链接；
仅删除未被引用的 `.c` 工程条目不会得到上述收益。

## 4. 实际大函数

下面是 map 的函数段大小（含段内常量池），不递归累加被调函数；内联代码已计入
调用者，不能再用另一份函数体重复计算。

| 函数 | 字节 |
| --- | ---: |
| `Task_Calib_EncoderObserver` | 3088 B |
| `PositionCascade_RunControl` | 1780 B |
| `PositionCascade_UpdateFriction` | 1440 B |
| `PositionCascade_CheckConfig` | 1432 B |
| `HAL_ADCEx_InjectedConfigChannel` | 1332 B |
| `AppLifecycle_Step` | 1298 B |
| `HAL_ADC_ConfigChannel` | 1292 B |
| `Param_Download` | 1240 B |
| `MotorOuterLoop_SlowTick` | 1224 B |
| `HAL_RCC_OscConfig` | 1104 B |
| `PositionCascade_UpdateTrajectory` | 1074 B |
| `RunState_ServiceBenchRequest` | 984 B |
| `FrictionIdentification_Update` | 856 B |
| `CoggingCalibration_Update` | 744 B |
| `UART_SetConfig` | 724 B |
| `FOC_CurrentWithReference` | 712 B |
| `FocMode_Admit` | 700 B |
| `FocRunState_Tick` | 692 B |
| `HAL_ADC_IRQHandler` | 684 B |
| `HAL_RCCEx_PeriphCLKConfig` | 664 B |

`RunState_ServiceBenchRequest` 为当前 Watch 台架请求处理，并不是已退役的旧模式。
位置配置检查、状态守卫和 ADC 初始化都有明确用途；大函数不等于可以直接删除。

## 5. 优化结论与证据

相对本轮开始时的 120160 B 加载跨度，当前减少 **9616 B**：旧模式和状态机批次
减少 7648 B，正弦半表再减少 1968 B。后者表数据减少 2048 B，同时查表及内联
代码增加 80 B，必须看整包净收益。半表全部 1024 个黄金项、204102 个角度的
正余弦位模式对照通过，并覆盖原索引 1024 越界的周期端点；不是改变控制采样精度。

后续优先检查平台驱动实际初始化需求、协议/状态机的大对象及启动契约实现；
当前控制、标定与保护仍有真实调用，不能把表中数字当作可随意删除的预算。

完整对象清单、803 个函数段和可逐项求和的 JSON/CSV 位于
`outputs/capacity-fit/functional-map/`。当前 map、构建日志、数学测试和产物位于
`outputs/capacity-fit/sine-half-table/`；真实区域仍超限，所有诊断 AXF 均为
NONDEPLOYABLE，无 HEX 输出，不可部署。普通 APP 独立构建成功不代表配套 APP
容量或全库 release 验收成功。没有进行硬件操作。

执行记录见 [容量适配计划](../../plans/completed/2026-09-28-capacity-fit-v1.md)。
