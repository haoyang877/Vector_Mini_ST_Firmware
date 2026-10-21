# Runtime职责收口 v1

日期：2026-10-21。范围：重构主线APP；本批只调整职责、调用和工程输入，不新增功能。

## 目标与不变项

将runtime中的控制计算、上电电流零偏采样细节及热保护计算归还motor所属模块。
runtime保留单轴对象所有权、调度、请求准入适配、维护协调和后台参数事务。
先复用既有模块，不引入注册器、通用回调框架、配置副本或另一套运行路径。

保持控制公式、浮点求值顺序、保护阈值、状态/故障语义、快慢拍先后顺序、功率使能所有者、
参数/通信ABI和Keil Watch入口不变。包括所有既有Watch符号、字段及可观测对象；
跨模块只通过显式对象和既有命令交互，不开放新的可写全局接口。
本批不自动连接硬件、下载、复位或运动，也不复用旧实机结果声称新源码已实测。

## 基线与实施边界

- 开始HEAD：`d31001740cb0a8e23f70d1742debda4a24ca8c0e`。
- 对应最近固件：`b320d605be3fddeed30100e54a0fab6e0062ae49`，APP1.5.4。
- 本批候选独立标识APP1.5.5，Loader仍为2.1.0；版本变化不表示已发布或实机通过。
- 已有PR证据：`outputs/runs/20261021T044511774499Z-b320d605/summary.json`。
- 既有未提交的参数说明、RAM/时序计划和编辑器配置不属于本批，原样保留。
- 控制数学先核对所有调用及现有夹具，再移动完整职责；状态机临界区不在本批扩大。
- 不修改CubeMX生成初始化；仅在必要时同步受影响Keil工程源文件路径/分组，保留调试设置。

## 验收与回退

核对移动前后的函数体、调用顺序、Watch对象定义和ABI头，使用已有快环分发、
上电零偏、外环、热保护与状态所有者夹具覆盖本次实际移动的代码。
执行仓库PR检查；工程输入调整执行release检查并如实记录环境/现场改动限制。
比较适用的代码/参数边界、状态/故障、输出轨迹及构建资源；未做新硬件时序测试的项明确延期，
不将历史1.5.4时序当成本批新二进制的证据。

回退只撤回本批独立差异，以开始HEAD为参照；不reset/clean，不覆盖其它工作或现场数据。
若出现计算、输出、故障或时序的意外变化，先停止收口并定位，不改变保护阈值绕过。

## 实施与证据

### 已落地的职责

| 原runtime内容 | 当前归属 | runtime保留 |
| --- | --- | --- |
| 外环、位置控制装配与快环模式分发 | motor/foc/motor_control_execution | 命令格式/范围校验、执行调度 |
| 电流零偏累计与ADC计数校验 | 既有motor/foc/foc_sensing | 单份Watch实例、完成时修订与电流换算顺序 |
| PI配置绑定及动态电压额度 | 既有motor/foc/foc_algorithm | 初始化/提交/母线采样后的调用 |
| 绕组热历史、降额及VQ向量电流判断 | motor/protection/motor_thermal_control | 时间/功率/维护事实采集与故障快车道调度 |

控制计算与热历史没有复制。所有既有外部Watch对象仍在原所有者定义；
零偏对象字段及20字节布局不变，motor只借用显式对象，不自行extern访问Axis。
架构守卫扩展到自有平台代码和shared，防止驱动反向读取APP全局；
runtime不得重新定义已迁走的算法接口。热保护的功率事实读取仍遵循原短路门禁，
新增一次只读CanRun判断不推进模型，20kHz内的原始采样/计算/故障/输出顺序保持。

### 直接证据

- `outputs/runtime-responsibilities-v1/startup-offsets/`：2/6mΩ上电零偏、PI及Watch布局通过。
- `outputs/runtime-responsibilities-v1/parameter-request/`：参数/保存与开始HEAD调用摘要相同。
- `outputs/runtime-responsibilities-v1/dispatch-comparison/`：5760个调度场景与开始HEAD轨迹相同。
- `outputs/runtime-responsibilities-v1/state-comparison/summary.json`：所有者事件/功率轨迹及262144规则场景与开始HEAD相同。
- `outputs/runtime-responsibilities-v1/thermal/summary.json`：真实热模型、调度、VQ/降额和故障边界五场景通过。
- 两份Keil工程同步源归属；逐文件编译选项按文件名逐项比较保持，新增执行源继承原执行源选项。

### 最终检查与候选

生产源提交：`05528f86437ddeec44433965baa211e06e2f139e`，分支`codex/system-refactor`。
runtime中C源从1594行降至1068行；其中控制与保护功能迁回motor，不表示删掉526行功能。
19个迁移函数去除注释/空白后与基线相同；11个状态机、控制核心、配置类型和保护边界文件保持。

| 检查 | 结果及证据 |
| --- | --- |
| 最终PR | PASS：`outputs/runs/20261021T074856548080Z-05528f86/summary.json` |
| 最终文档收尾quick | PASS：`outputs/runs/20261021T075259803053Z-05528f86/summary.json` |
| 主工程normal | 0 errors/0 warnings：`outputs/build/logs/YG_ESC_APP.log` |
| Loader/配套APP构建与冻结 | 成功：`outputs/delivery/runtime-responsibilities-05528f86-20261021/summary.json` |
| Watch | 9个关键符号的大小和所属对象一致：同冻结目录`watch-comparison.json` |
| 源码/产物复核 | PASS：`outputs/runtime-responsibilities-v1/review_summary.json` |
| release profile | 未通过：`outputs/runs/20261021T075114976017Z-05528f86/summary.json`；doctor及离线检查通过，保留的原现场修改使clean-source失败，因此未启动该profile内构建/正式发布清单 |

配套APP Flash从81112增至81376字节（+264，约0.33%），RAM仍18552字节；这是跨源职责拆分的实际
编译代价，不宣称本批缩减镜像或提速。Loader仍14520字节且BIN与基线相同。
调试候选APP1.5.5/Loader2.1.0、layout6、schema14，冻结产物均按summary内SHA256复核。
Watch符号地址因重新链接改变，使用本候选匹配AXF，不复用历史固定地址。

本候选的真实20kHz耗时、标定/速度和升级尚未复测；既有1.5.4计时不能作为1.5.5结果。
新增跨源调用可能改变内联和指令开销。本轮未接硬件、未下载，冻结成功不代表正式release或实机PASS。
