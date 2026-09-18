# D-loader：USB 移除与空间优化分析

日期：2026-09-17。状态：设计与现有链接产物分析；尚未删除源码或重新构建。

## 1. 已确定的设计选择

下一步实现移除 MCU 侧 USB CDC 通信及专用代码，APP 的现有业务使用 CAN，通用 Loader 同时设计 CAN-FD 和 UART 升级，CAN-FD 优先实现。PC 通过 USB-CAN 适配器接入 CAN 的能力保留：这是主机到适配器的 USB，与 MCU USB Device 协议栈是两回事。

继续保留现有 12 KiB Loader、96 KiB APP、16 KiB 参数/标定区布局。USB 移除预计已能提供足够 APP 空间，其他项按收益和验证成本排序，不为了压缩体积削减控制/保护功能。

## 2. 当前 map 的空间账目

分析输入：`MDK-ARM/Vector_Mini_ST/Vector_Mini_ST.map`，本地时间 2026-09-17 11:41:28 的产物。初始设计审查时 HEAD 为 `7a2d235`；本次空间分析时 HEAD 已为 `68c984c`。该 map 仍包含完整 USB，未将工作区或历史产物误认作已移除 USB 的新构建。

- Total ROM：107,968 B，约 105.44 KiB。
- Total RW（含 ZI）：30,992 B，约 30.27 KiB，已包含 map 中的栈和堆保留。
- 表格按链接器实际保留对象统计，不按源文件大小估算；`Code (inc. data)` 中的子列已经包含在 Code 内，不重复相加。
- RW 初值在此产物中经过压缩，不能把对象的未压缩 RW 大小直接视作最终 Flash 收益；下表 Flash 列仅计 Code+RO。

### USB 专用部分

| 模块 | Code+RO / B | RW+ZI / B |
| --- | ---: | ---: |
| `interface_usb.o` | 8,532 | 305 |
| `ring_buffer.o`，本工程调用方仅 USB | 170 | 260 |
| HAL PCD + PCD EX + LL USB | 5,742 | 0 |
| `usb_device.o` | 84 | 720 |
| `usbd_cdc.o` | 912 | 271 |
| `usbd_cdc_if.o` | 142 | 2,068 |
| `usbd_conf.o` | 716 | 1,276 |
| `usbd_core.o` | 684 | 0 |
| `usbd_ctlreq.o` | 1,822 | 1 |
| `usbd_desc.o` | 358 | 594 |
| `usbd_ioreq.o` | 132 | 0 |
| **合计** | **19,294（18.84 KiB）** | **5,495（5.37 KiB）** |

只扣除上述 Code+RO，粗估 APP 为 `107968 - 19294 = 88674 B`，约 **86.60 KiB**，距 96 KiB 上限约余 **9.40 KiB**。RAM 静态归属粗估降至 25,497 B，约 **24.90 KiB**。这些是对象归属估算，未包括新增 CAN 业务、APP 交接代码、布局变化、压缩及对齐变化。

### USB 带入的格式化库

map 明确显示 `interface_usb.o` 调用 `__2sprintf` 和浮点格式化链。`printf` 系列、`bigflt0.o`、`btod.o`、`_sputc.o`、`noretval__2sprintf.o` 等当前 Code+RO 合计 **4,414 B（4.31 KiB）**，未计入上面的 19,294 B。

如果删除 USB 后这些库成员没有其他根引用，链接器可进一步移除；对应粗估 APP 降至 **82.29 KiB**。必须用新 map 确认，不能同时把所有 C 库体积都算作收益。后续 CAN 状态和日志优先用现有定长二进制格式，避免为了诊断文字重新引入浮点 `sprintf`。

解析结果保存在 `outputs/d_loader_design_20260917/space_audit.json`，包含输入 map 的 SHA-256、USB/UART/RTT 对象明细及全部对象统计。该目录为本地分析产物，不参与固件构建。

## 3. USB 删除范围及业务迁移

只去掉 `MX_USB_Device_Init()` 不足以可靠释放协议栈；必须移除全部链接入口和依赖。

| 层次 | 下一步修改范围 |
| --- | --- |
| APP 入口 | `Core/Src/main.c` 中 USB 头文件、`MX_USB_Device_Init()`、主循环 `USB_SendMessage()` |
| 周期任务 | `Bsp/bsp_task.c` 的 `USB_PrintProfile()`，同步更新调度注释 |
| 公共头 | `System/common_inc.h` 中 `usbd_cdc_if.h`、`interface_usb.h`、USB 专用 `ring_buffer.h` 依赖 |
| 应用协议 | 删除 `Communication/interface_usb.c/.h` 与经全局引用复核后的 `ring_buffer.c/.h` |
| Device 层 | 删除 `USB_Device/App`、`USB_Device/Target` 的项目实现 |
| USB 中间件 | 移除 `Middlewares/ST/STM32_USB_Device_Library` 工程输入和专用 include path；不再维护 MCU USB 目标 |
| HAL 输入 | 从目标移除 `stm32g4xx_hal_pcd.c`、`stm32g4xx_hal_pcd_ex.c`、`stm32g4xx_ll_usb.c`，关闭 `HAL_PCD_MODULE_ENABLED`；共享 vendor SDK 可保留未引用源文件 |
| 中断 | 删除 USB HP/LP 的自定义处理器及 `hpcd_USB_FS` 引用；保留芯片启动文件固定向量槽位，不能删向量表条目造成后续 IRQ 错位 |
| 时钟/引脚 | 无其他使用者时关闭 HSI48 和 USB 专用配置；PA11/PA12 回到板级定义的闲置状态，不影响 HSE/PLL、CAN、ADC 时钟 |
| 生成配置 | 同步 `Vector_Mini_ST.ioc`、普通/HIL 工程、IncludePath 和目标源文件，避免重新生成后 USB 回来 |
| 工具与说明 | 更新 README 的 USB/VOFA 使用说明及依赖 USB 的测试工具；历史报告保留为历史记录 |

移除的是 USB 传输及命令适配，不是业务功能。实施前将 USB 命令分成三组：

1. CAN 已有的启停、模式、参数、状态、摩擦模型接口，使用既有 CAN 路径。
2. USB 独有但仍需保留的能力，例如 LUT/摩擦采样数据导出，改由有版本和分块规则的 CAN 服务提供；不能删除 USB 后默默丢功能，也不能直接套用 Flash PROGRAM 命令传业务数据。
3. VOFA 打印布局、USB 收发错误等仅与旧 USB 界面有关的能力随接口退出。

准确功能对照表在实施阶段按命令逐项完成；本次尚未宣称 CAN 已覆盖每一个 USB 命令。

验收要求：普通/HIL 都可构建；链接产物无 USB/PCD 依赖、无 USB 专用缓冲、无应用 USB 中断实现；CAN 参数/控制/校准流程可用；Flash 参数 ABI 与已有标定保持兼容；生成配置不会恢复 USB。

## 4. 其他优化候选

| 顺序 | 候选 | 当前占用或潜在收益 | 建议与边界 |
| --- | --- | --- | --- |
| 1 | USB 带入的格式化库 | 当前 Code+RO 4,414 B | 随 USB 删除后检查链接结果；优先收益，不手工删除共享 C 运行库 |
| 2 | APP 当前仅初始化的 USART1 | UART/HAL UART/EX 共 Code+RO 1,996 B、RAM 148 B | 可从无需 UART 业务的 APP 目标移除；保留通用 MCU UART port 以实现 Loader UART 升级。两个独立固件分别决定是否链接 |
| 3 | RTT 生产配置 | `segger_rtt.o` Code 520 B、RAM 3,256 B；采样编码还在 `rtt_telemetry.o` | 优先保留调试能力并缩小未使用缓冲；工程/HIL 版保留完整采样，生产版可选关闭。不能把整个 `rtt_telemetry.o` 算作可删除空间 |
| 4 | 正弦 LUT | `sin_tab[1024]` RO 4,096 B；四分之一周期 float 表约 1,028 B，表本体理论省 3,068 B | 涉及 20 kHz FOC 快速路径，低优先级。额外索引/符号代码减少净收益；先验证数值误差、边界、周期和实机噪声，不直接切换 |
| 5 | 自定义堆 | `heap.o` RAM 12,312 B，其中池 12 KiB；Code 356 B | RAM 优化，不是大额 Flash 来源。校准数组共 8 KiB，加参数记录及分配开销，不能因移除 USB 而一并缩减校准内存 |
| 6 | 工厂/诊断功能构建配置 | `foc_calibration.o` Code 7,362 B；辨识、观测器等还跨多个对象 | 可为明确不需现场校准的产品另建精简配置；当前默认保留。不能将文件总大小直接当可省空间，运行控制和校准可能共用代码 |
| 7 | 冷路径按体积优化 | 数量由同版本 A/B 构建测量 | 针对初始化、参数管理等冷路径，不统一改变已调优的 FOC/位置环编译选项；每次以 map 和实时性测量决定 |

### UART

已检索 APP 业务源码，`huart1`、`MX_USART1_UART_Init()` 和 `HAL_UART*` 调用局限于 `Core/Src/usart.c` 与 main 初始化，没有发现业务收发路径。可列为 APP 的精简候选；用户已要求 Loader 支持 UART 升级，因此不能从公共框架删除 UART 能力或以此阻断 UART port。APP 不用 UART 与 Loader 使用 UART 可以同时成立。

### RTT

当前 `bRttBuf` 为 2048 B，默认终端上行缓冲 1024 B、下行 16 B，连同控制块等合计 3256 B。业务采样使用 channel 1；优先核查 channel 0 是否需要 1 KiB。若缩至 128 B，单项 RAM 可省 896 B，同时保持 channel 1 的工程采样能力。进一步禁用整个 RTT 时需同步 main 初始化和 `RTT_Sampling()`，并调整依赖 RTT 的诊断流程。

### 堆与栈

校准分配 `1024×int32 + 1024×uint16 + 1024×int16 = 8192 B`，参数操作还使用独立记录缓冲。需要验证全调用生命周期、失败清理和并发条件后，才考虑专用工作区或池缩减。把堆换成同尺寸静态数组不会自动节约 RAM；互斥复用只有在生命周期确实不重叠时才成立。

map 另有启动文件的 512 B C 堆和 3072 B 栈。C 堆可在确认运行库没有分配需求后评估；栈必须以最坏中断嵌套和运行高水位测量为依据，不直接削减默认值。参数/标定区的 Flash 保留空间不作为本轮优化对象。

### 不优先改动的部分

位置伺服、位置阻抗、ADC/TIM 驱动、观测器及校准虽在 map 前列，但承担实际功能或时序约束。USB 去除后已有预算，不需要为容量优先重写这些模块，也不删除 schema 兼容路径。

源码和调试信息体积不等于设备 Flash：删除注释、头文件或 map 中很大的 Debug 列不能当作 ROM 优化收益。最终镜像中已被链接器剔除的函数，再删源码也不会释放第二次空间。

## 5. 实施和验收顺序

1. 保存含 USB 的普通/HIL 构建基线和功能清单。
2. 完整移除 MCU USB；对仍需保留的 USB 独有业务补 CAN 接口。
3. 重建并记录 ROM、RAM、加载跨度和最大栈；确认 USB 及格式化库实际释放量。
4. 在这份无 USB APP 上接入 Loader 布局、邮箱与进入升级命令；重新构建，不沿用未重定位镜像。
5. 若仍需更多余量，先评估未使用 UART 和 RTT 缓冲；正弦表、校准工作区等另列独立优化，不混入第一轮集成。
6. 发布门槛仍为 APP ≤96 KiB，目标 ≤92 KiB；Loader ≤12 KiB。APP 移除 USB 并不会扩大 Loader 自己的分区，新增 Loader 功能必须单独监控体积。

全部数字为本地 map 归属统计或明确注明的估算；本轮没有构建删除 USB 后的镜像，也未进行板上试验。
