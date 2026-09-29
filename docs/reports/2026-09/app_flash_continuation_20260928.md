# APP 容量继续优化（2026-09-28）

基准 `cb21b25d`，ARMCC 5.06 update 7 build 960。仅离线构建与测试，未访问硬件。
遵循 [容量计划](../../plans/completed/2026-09-28-capacity-fit-v1.md)，保持统一正余弦查表。

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

## 逐函数 002：运行所有者请求准入与事件返回值

用户按新增内容排序，指定先优化 `foc_run_state.c`。基准 `76c18107`；本次先重新
构建基准，普通 APP ROM 94016 B、配套 APP 诊断 ROM 101768 B，与上一批一致。
生产代码只修改该文件，生命周期核心、平台驱动、协议和全部现行业务功能保持。

### 请求入口：先判断占用，再由各入口决定资格

`FocRunState_RequestEnable` 把业务模式转换为内部模式，检查故障、占用和启动资格，
最后以零目标或当前位置受理模式；实际启相由后续监督调度完成。Watch 速度请求
调用它；CAN 目标提交不能借此隐式启动。`RequestClear` 服务 Watch 和协议清错，
`RequestMaintenance` 服务 Watch、带来源维护请求及参数保存。

三者重复检查升级停机票据、后台借用、配置借用、待消费结果、待应用目标、Watch
速度会话和维护操作。提取为文件内 `request_busy`，在原有临界区和条件位置调用；
顺序保持。Enable/Maintenance 先返回 FAULT，Clear 先返回 BUSY；Clear/Maintenance
仍额外核对真实资源归还，Enable 仍通过配置许可判定。函数不缓存守卫、不改变状态。

### 事件提交：直接初始化结构体返回值

`RunState_Send` 构造普通控制/启动/故障事件，使用当前生命周期代号；
`RunState_SendOperation` 使用原始操作或取消代号，并在动作执行前保存维护终态；
`RunState_TryClearFault` 先核对恢复矩阵，再为本次 CLEAR 覆盖已获准的故障守卫。
这些差异全部保留，尤其不合并故障恢复的多次守卫采集。

三处将先声明、后赋值的 guards/result 改为在原赋值位置直接初始化。ARMCC5
实测消除了返回结构体的中间复制，无需增加公共提交函数或新的模式标志。
下表是各函数段大小，含常量池；不是每项独立的整包收益。

| 函数 | 基准段 / B | 当前段 / B |
| --- | ---: | ---: |
| FocRunState_RequestEnable | 200 | 172 |
| FocRunState_RequestClear | 116 | 84 |
| FocRunState_RequestMaintenance | 216 | 176 |
| request_busy（新增公共局部判断） | 0 | 56 |
| RunState_Send | 140 | 116 |
| RunState_SendOperation | 232 | 212 |
| RunState_TryClearFault | 232 | 212 |

对象代码加只读数据由 8186 B 降至 8078 B，减少 108 B；结合整包对齐，普通与
配套 APP 各净减少 **112 B**。逐步隔离测量为公共忙闲判断 48 B、直接初始化
再减少 64 B。三个事件函数的本地栈帧（含保存寄存器，不含被调函数）分别为
120→96 B、120→88 B、144→144 B。没有新增快环工作，也没有实机栈水位或周期证明。

未采用：控制模式映射抽取没有额外容量收益；只抽取守卫/Step 的公共函数在原写法
上仅额外省 8 B，直接初始化后反而比不抽取多 8 B；事件改用指定成员初始化比
保留清零赋值多 16 B。这些候选仅存于输出证据中，不进入最终实现。

### 验证和容量结论

新增特征测试先在原生产实现通过，再对最终实现运行：四种控制模式受理及目标
提交、七类占用来源、故障/非法请求优先级、维护资源未归还、失败票据清零、
带身份请求重试、派生 SAVE 不覆盖原标定终态。既有 STOP、故障快关断、取消
等待前台归还、迟到完成和关相失败等检查继续通过，所有测试均为离线模型。

当次完整 PR 的 23 项原生、17 项集成及架构/布局/卫生检查通过；修改文件局部
格式/lint 通过。整体仍 FAIL：format、lint、interfaces、docs 的失败日志与
上一批逐字一致；39 项 Python 单元中仍有 1 项因同样的 10 个历史产物断链失败。
没有把全库门禁或发布验收标为通过。

普通 APP ROM **93904 B** / RAM **31896 B**，构建 0 错误/0 警告；配套 APP
诊断 ROM **101656 B**、跨度 **101696 B** / RAM **32128 B**。两者 RAM 不变。
78 KiB 正式分区仍缺 **21824 B**，真实配套 APP 链接仍失败；诊断链接不可部署，
没有生成诊断 HEX。独立 Loader 源码和构建输入未改，本批没有重新构建独立 Loader。

构建证据为 `outputs/capacity-fit/batch1/run-state-baseline/` 与 `run-state-final/`；
逐项候选、原生测试、汇编和源/产物哈希为 `outputs/capacity-fit/run-state-refactor/`，
最终清单为该目录 `summary.json`。完整 PR 清单为
`outputs/runs/run-state-refactor-20260928/summary.json`。按本批差异逆向回退，保留
两处原有用户修改。未进行硬件操作。

## 逐函数 003：运行所有者残留重复与生命周期停止/故障入口合并

基准 `eef6e3ca`。先当次重建普通 APP 与 Loader APP，普通 APP ROM 93904 B、
Loader APP 诊断 ROM 101656 B / 跨度 101696 B，与交接值逐位一致。用记录的实际编译参数
对单文件做隔离替换链接（`foc_run_state.c` 为 `-O3`，`app_lifecycle.c` 为 `-O2`，
与工程 `Optim=4/3` 对应），先验证未改源码可复现基准，再逐一测量候选；只有整包净收益
明确才采用。诊断链接仍只是 NONDEPLOYABLE 测量，不可部署、无 HEX。

### foc_run_state.c：三处可证明重复

- `RunState_ServiceBenchRequest` 曾内联一段台架结果映射，四分支与文件内
  `bench_result()` 完全相同（ACCEPTED→ACCEPTED、BUSY→BUSY、INVALID/OUT_OF_RANGE→
  INVALID，其余 DENIED），改为调用该函数，函数体 984→964 B。
- `RunState_ServiceOperations` 的 ZERO 与 DEFAULTS 分支除目标常量外处理体相同，
  合并为一次 `RunState_SendOperation(..., lifecycle.snapshot.operation, ...)`，
  248→236 B。`sent`、`save_finish`、派生 SAVE 不覆盖原标定终态、短路顺序均保持。
- 提取文件内 `RunState_OwnerQuiescent()`（模式失能、无目标/占用、资源释放判据），
  供 `configuration_idle`（112→60 B）与 `FocRunState_BootHandoffReady`（140→84 B）
  复用，新增 76 B，净减 32 B。`configuration_idle` 中 `ErrorNow` 与 `ModeNow`
  读取次序交换，对纯布尔合取结果无影响，已在注释标明。对照：`ModeNow` 留在调用方
  或重复检查各少省 8 B，未采用。

### app_lifecycle.c：STOP/CANCEL 合并无收益，改合并停止/故障入口

交接建议的 STOP 与 CANCEL 相同处理体合并，在 `-O2` 下整包差值 0 B——ARMCC 已把两段
相同调用序列合并，故不采用该改法。改取提取 `enter_stopped_state(a, state, actions)`，
供 `fault()`（60→40 B）与 `stopping()`（68→34 B）复用"cancel_operation → 撤销作业 →
advance → 置目标状态并清 control_mode"序列；`advance` 耗尽代际转 FATAL 时两者都不再
改状态的原有分支保持。`cancel_operation`（24 B）并入该辅助函数后不再单独保留，
新增辅助 50 B，净减 24 B。

### 段落与整包

下表为本批两模块函数段大小（含常量池），不是各项独立的整包收益。

| 函数 | 基准段 / B | 本批段 / B |
| --- | ---: | ---: |
| FocRunState_ServiceBenchRequest | 984 | 964 |
| RunState_ServiceOperations | 248 | 236 |
| configuration_idle | 112 | 60 |
| FocRunState_BootHandoffReady | 140 | 84 |
| RunState_OwnerQuiescent（新增） | 0 | 76 |
| fault | 60 | 40 |
| stopping | 68 | 34 |
| cancel_operation | 24 | 0（并入辅助） |
| enter_stopped_state（新增） | 0 | 50 |

当次普通 APP ROM **93872 B**（-32 B）/ RAM 31896 B；Loader 配套 APP 诊断
ROM **101568 B**、跨度 **101608 B**（-88 B）/ RAM 32128 B，两者 RAM 不变。函数段合计
减少 92 B，整包对齐后 Loader 净减 88 B。78 KiB 正式分区仍缺 **21736 B**，真实配套
APP 链接仍失败，诊断镜像不可部署。独立 Loader 未改共享实现，本批未重建。

### 验证与限制

`test_run_state.py` 14 项特征用例（四种控制模式、七类占用、故障/非法优先级、恢复
不回启、派生 SAVE 身份、取消释放与限时关断等）通过；23 项原生、17 项集成及架构/
布局/卫生检查通过；两个修改文件局部格式/lint 通过。完整 PR 仍 FAIL：format、lint、
interfaces、docs 失败日志与上一批逐字一致，39 项 Python 单元中 1 项仍为 10 个历史
产物断链，无新增门禁问题。本轮只覆盖离线逻辑与空间；MCU 周期、实机栈水位和电流
仍未验收。证据见 `outputs/capacity-fit/deepseek/summary.json`、候选隔离尺寸目录与
构建目录 `outputs/capacity-fit/batch1/deepseek-01-baseline/`、`deepseek-02-final/`；
完整 PR 清单为 `outputs/runs/deepseek-runstate-lifecycle-20260928/summary.json`。
按模块提交逆向回退，保留原有 `angle_feedback.c` 尾空格与 Keil GUI 用户修改。

## 逐函数 004：参数读写/保存请求身份冲突合同

交接顺序 C。基准上一批提交 `c8aca9f1`（Loader APP 诊断跨度 101608 B）。`motor_state.c`
按目标默认 `-O1` 编译（工程 `Optim=2`）；先以记录的实际编译参数做单文件隔离替换链接
校准到当前基准，再逐一测量。

### 问题与改动

`MotorControl_ReadParameter`、`MotorControl_WriteParameter`、`MotorControl_RequestSave`
都用"来源节点+序号"识别同一请求身份，但三处各自展开相同的判据，且覆盖的缓存集合不同：
读入口排除 `last_write`/`save_job`，写入口排除 `save_job`/`last_read`，保存入口排除
`last_write`/`last_read`。新增文件内：

```c
static bool matches_cached_request(uint8_t source_node, uint16_t sequence);
```

返回是否命中 `last_read`/`last_write`/`save_job` 任一已缓存票据。三个入口保留各自的主缓存
结果重放分支（读→last_read、写→last_write、保存→save_job），其后统一调用该判据返回
`PARAMETER_CONFLICTING_RETRY`。因为调用点都在"主缓存已判定未命中"的 `else` 分支内，
判据集合包含主缓存不改变结果；各入口的返回优先级、原结果重放、配置修订号/持久化修订号
以及 `stale_sequence` 的位置都保持。`parameter_value_service.c` 本批未改。

### 段落与整包

| 函数 | 基准段 / B | 本批段 / B |
| --- | ---: | ---: |
| MotorControl_ReadParameter | 276 | 240 |
| MotorControl_WriteParameter | 388 | 332 |
| MotorControl_RequestSave | 276 | 228 |
| matches_cached_request（新增） | 0 | 72 |

函数段合计减少 68 B。当次普通 APP ROM **93808 B**（-64 B）/ RAM 31896 B；Loader 配套
APP 诊断 ROM **101504 B**、跨度 **101544 B**（-64 B）/ RAM 32128 B 不变。78 KiB 正式
分区仍缺 **21672 B**，真实配套 APP 链接仍失败。

### 验证与限制

新增 `tests/unit/native/motor_state_request_test.c`：夹具复刻 `motor_state.c` 私有状态，
由 runner `tests/unit/native/test_motor_state_request.py` 从生产源码逐函数切片真实实现
（旧实现无 `matches_cached_request` 时自动跳过），桩化临界区、运行所有者与参数值服务。
用例覆盖：读身份同参数重放、同身份换参数冲突；写身份成功应用并缓存、读复用写身份冲突、
同字段重放、不同值冲突且不重复应用；保存受理后读/写身份冲突、同身份保存重放原结果与
票据、不同身份保存 BUSY 且无票据。该测试先在旧实现（`git stash` 后）通过，再对最终实现
通过；`test_run_state.py` 14 项、23 项原生、17 项集成与架构/布局/卫生通过。完整 PR 仍
FAIL：format/lint/interfaces/docs 日志与上一批逐字一致，1 项 Python 单元仍为 10 个历史
产物断链，无新增门禁问题。仍只有离线证据，无 MCU 周期、栈水位或实机电流。

证据：`outputs/capacity-fit/deepseek/summary-motor-state.json`、构建目录
`deepseek-02-final/`、`deepseek-03-final/`；PR 清单
`outputs/runs/deepseek-motor-state-20260928/summary.json`。按本批差异逆向回退，保留
两处用户修改。没有硬件操作。
