# APP 容量继续优化（2026-09-28）

基准 `cb21b25d`，ARMCC 5.06 update 7 build 960。仅离线构建与测试，未访问硬件。
遵循 [容量计划](../../plans/active/2026-09-28-capacity-fit-v1.md)，保持统一正余弦查表。

## 标定与平台编译选项

当次重建复现普通 APP 96808 B、Loader APP 诊断 ROM 104632 B / 跨度 104672 B。
对尚未比较的电机、应用和平台对象分别试验 O2/O3 空间选项；每次仅替换一个对象
重新链接。保存源文件哈希、实际编译参数、链接参数、日志和 MAP。没有改全局浮点
语义、第三方源码、生成初始化或正式内存区域。

采用如下 16 个对象，表中收益为单独替换时的整包差值，不能累加充当组合结果。
相同收益优先 O2；O3 至少再减少 16 B 才采用。快速 FOC、位置控制、角度反馈和
主状态机保持基准选项；O2 状态机实验反而增加 32 B，未采用。

| 对象 | 选项 | 单独收益 |
| --- | --- | ---: |
| foc_encoder_calibration | O3/空间 | 712 B |
| stm32g4xx_hal_adc | O2/空间 | 376 B |
| stm32g4xx_hal_rcc | O2/空间 | 176 B |
| friction_identification | O2/空间 | 152 B |
| stm32g4xx_hal_tim | O3/空间 | 128 B |
| phase_resistance、stm32g4xx_hal_uart | O2/空间 | 各 112 B |
| stm32g4xx_hal_fdcan | O2/空间 | 104 B |
| foc_phase_resistance | O3/空间 | 96 B |
| foc_friction_identification | O2/空间 | 88 B |
| foc_cogging_calibration、boot_hw_stm32g4 | O2/空间 | 各 72 B |
| cogging_calibration、stm32g4xx_hal_adc_ex | O2/空间 | 各 64 B |
| comm_status_stm32g4 | O2/空间 | 56 B |
| rgb | O2/空间 | 48 B |

APP 生成器沿用普通工程文件配置，Loader 专用启动端口继承对应普通端口的 O2
选项。114 个源文件的名称、路径、顺序未改变。独立 Loader 工程不受影响。

| 镜像 | 基准 ROM / 跨度 | 本批 ROM / 跨度 | 净收益 | RAM |
| --- | ---: | ---: | ---: | ---: |
| 普通 APP | 96808 B | 94472 B | 2336 B | 31896 B |
| Loader APP | 104632 / 104672 B | 102224 / 102264 B | 2408 B | 32128 B |

真实 78 KiB APP 分区仍超 **22392 B（21.87 KiB）**，容量适配没有完成。
诊断链接只有 NONDEPLOYABLE AXF/MAP，没有 HEX；不能部署。RAM 不变，配套 APP
静态预算只剩 608 B，不能解释为运行栈水位。独立 Loader 当次重建 0 错误/0 警告，
仍为 ROM 20036 B / RAM 5152 B。

22 项原生检查、17 项集成、项目布局和架构通过。完整 release 总结果 FAIL：
既有格式/lint、接口文档和 10 个历史产物断链；39 项 Python 单元中 1 项因断链
失败。工作区含保留的用户修改，且本次门禁未重新调用 Keil，release-manifest
因此不通过；当次构建由独立的离线测量步骤完成，不宣称门禁或发布通过。

编译选项会影响标定、初始化及中断驱动时序。当前证据仅覆盖离线逻辑与空间；
MCU 快环时间、栈水位和实机电流仍未验收，按本批提交逆向回退。

证据：`outputs/capacity-fit/batch1/continuation-baseline/`、
`outputs/capacity-fit/batch1/continuation-cold-space/`、
`outputs/capacity-fit/continuation/compiler/`、`outputs/capacity-fit/continuation/loader.log`；
源码与产物哈希为 `outputs/capacity-fit/continuation/summary-cold.json`，完整验证为
`outputs/runs/capacity-continuation-cold-20260928/summary.json`。
