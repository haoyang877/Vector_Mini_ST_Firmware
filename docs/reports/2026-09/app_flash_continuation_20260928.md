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

## 配置有限值校验集中扫描

以 `a41ac00` 为基准，将位置配置 33 个 float 字段的有限值检查集中为有界偏移表
扫描。表使用 `offsetof`，通过 `memcpy` 读取真实 float 字段，忽略结构填充；
编译期检查结构大小，防止 byte 偏移截断。阈值、字段间大小关系、积分率与周期
乘积的溢出检查全部保留。只在原有完整配置验证分支运行，缓存命中与逐拍目标
检查不变。没有引入跨层接口或额外可变全局状态，原配置/状态结构与 ABI 不变。

源码同时按仓库要求统一格式、补齐控制流花括号和中文注释。测试不再匹配英文
注释原文，改为检查原有方向锁存表达式；现有逐拍测试继续保护真实行为。

- 2464409 次配置判定与冻结旧实现一致：33 字段分别覆盖 ±0、次正规数、最大
  有限值、±Inf、正负 quiet/signaling NaN、阈值及相邻位模式、随机值和多字段污染。
- 两组位置 API 各 790068 拍的完整私有状态和输出逐字节一致；缓存填充、调参
  变化、失效输入、20 kHz/2 kHz 分频、HOLD/恢复/饱和/反转用例通过。
- 23 项原生、17 项集成、逐拍对照、架构、项目布局通过；修改文件局部格式/lint
  通过。完整 PR 仍因原有格式/lint/接口和历史文档断链失败，不宣称全库通过。

当次普通 APP 0 错误/0 警告，ROM **94024 B**；Loader APP 诊断 ROM **101776 B**、
跨度 **101816 B**。本步两个 APP 均净减少 **448 B**，RAM 分别保持 31896/32128 B。
本会话累计回收 Loader APP **2856 B**，真实分区仍缺 **21944 B（21.43 KiB）**。
正式 Loader APP 链接仍失败，NONDEPLOYABLE 测量不输出 HEX。校验首次/变更路径
的 MCU 时间与栈仍待实机测量；没有更改快环调度或宣称实时性已经验收。

证据在 `outputs/capacity-fit/continuation/config-validation/`、`config-equivalence/`、
`scoped-style.json`、`summary-config.json`；构建在
`outputs/capacity-fit/batch1/continuation-config-final/`；最新 PR 清单为
`outputs/runs/capacity-continuation-config-final-20260928/summary.json`。
第一次 PR 中依赖英文注释的集成误报已修复并完整重跑，保留原失败日志。

## 未采用的编译实验与后续范围

八个大对象逐一禁用内联的实验没有净收益（0 至增加 208 B），未改正式选项。
ARMCC 的 `--ltcg` 探测被现有许可证拒绝（C3921U），没有将其作为容量方案。
原始实验在 `outputs/capacity-fit/continuation/no-inline/` 与 `ltcg-probe/`。

用户再次明确要求保留 APP 全部功能，继续等价优化；不将标定移入维护 APP，
不裁剪当前协议、标定和保护，不扩大正式分区。后续继续对平台与状态协调做
隔离测量和等价验证。当前剩余缺口真实存在，不能把后续潜在收益算入已完成结果。

## 逐函数 001：母线重新使能窗口

用户要求逐函数优化并熟悉代码，后续每次按职责、调用者、不变量、候选、测量与
测试展开，不再批量修改不相关函数。

从 `RunState_BuildGuards` 开始梳理：它在临界区收集故障、数字关相、采样新鲜度、
前台/标定资源释放、启动及维护资格，输出 `AppLifecycleGuards`。状态迁移归
`AppLifecycle_Step`，功率动作归 `RunState_ApplyActions`；收集守卫本身不启相。
其段大小 512 B（含常量池）。调用者包含启动确认、故障恢复、维护和启动交接；
温度与资源状态具有时效，未把多次调用改成跨调用缓存，也未删除关相或释放证明。

第一个实际精简点是它调用的 `RunState_VbusInEnableWindow`。该函数只判断滤波
母线电压是否在闭区间 `[25.6, 33.8] V`；两端是既有有限 float 常量。NaN 无法
满足有序比较，±Inf 无法同时满足两端，因而前置 `isfinite` 对返回值是冗余的。
删除这一分支，保护阈值、重新使能迟滞、故障锁存与其他调用者保持。

ARMCC 当次汇编确认：新旧实现都使用整数位模式区间比较，不执行浮点比较指令，
没有新引入 signaling NaN 浮点异常状态副作用。函数指令由 34 B 减至 24 B，
含常量池的段由 48 B 减至 36 B；因整包对齐，普通和配套 APP 实际各减少 **8 B**。

新增 1018946 个输入的独立整数位模式参考，覆盖所有正负指数类别、±0、次正规数、
Inf、quiet/signaling NaN、两端各 ±4096 个相邻表示和百万随机值。母线延时保护、
准入、运行所有者的 STOP/故障/取消释放原生用例通过；局部格式/lint 通过。

当前普通 APP ROM **94016 B**；配套 APP 诊断 ROM **101768 B**、跨度 **101808 B**，
RAM 不变。相对本会话基准累计回收 **2864 B**，真实容量仍缺 **21936 B**。
当次真实配套 APP 链接仍失败，诊断输出仍不可部署。构建证据在
`outputs/capacity-fit/batch1/function-vbus-window/`，测试、汇编和哈希在
`outputs/capacity-fit/continuation/function-review/`；完整 PR 清单为
`outputs/runs/function-vbus-window-20260928/summary.json`。

ADC 三文件 `--multifile` 额外实验先遇到 CMSIS 内嵌汇编符号冲突；仅在隔离副本
使用 CMSIS 已有宏排除未使用内嵌汇编后链接成功，但只有 24 B 收益，未采用这套
跨文件构建改法。没有修改 vendor 或 CubeMX 初始化源码。
