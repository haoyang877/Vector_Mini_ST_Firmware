# MotorAxis 参数事务所有权解耦 v2

日期：2026-10-08。实现完成，等待 ROOT 接线和集成审查；上一批交付未集成，在原树续作。

基点与回滚：`e96e0fe230ed8ffc4a53dd86e8532d47df77f88e`。
继承上电/PI/Watch 修复及上一批单次准入查询不重复计收益；集成现场 MotorAxis 仍为
SHA256 `f12472f452b602aaf991a9eb08332b2ad877c32269dfc0a88bc64ffea6fac0bb`。

## 目标与不变行为

Axis 只装配单份电机/FOC/编码器/控制器对象、执行固定 20 kHz / 2 kHz 调度、采集状态快照。
新增一个小型 APP 参数事务模块，唯一持有活动修订、维护修订 epoch、最近读/写缓存和保存票据。
这些原有非 static Watch 符号保持名称、类型和数量；不在头文件暴露可写地址。
对象通过显式参数借用，不复制完整电机状态、不保存对象指针、不增加通用框架或回调。

已有 ParameterValue 服务继续负责参数字段、类型、单位缩放和校验；foc_param 负责 schema12
装载/导出；固定工作区和 param_store 继续负责记录借用和平台存储。主状态机仍唯一决定
准入、取消、功率使能、维护身份、真实释放及后台 claim。Axis 的公开入口保持明确转发。

保持来源+序号+原始完整内容重放、冲突/陈旧拒绝、输出失败清零、成功写修订一次、
控制器复位顺序、取消票据收敛、真实保存及回读完成才 OK、失效 claim 不写存储。
启动初始化、零点采样/失败、PI 初始配置、母线额度刷新、快慢环调用次序与临界区边界不变。

## 一页职责与数据流

| 流程 | 核心输入 → 输出 | 状态写入者与边界 |
| --- | --- | --- |
| 启动 | 借用参数记录 → 单份运行对象配置，活动 revision=1 | 参数事务重置自身票据并复用 Param_Download/默认值；Axis 随后按原顺序初始化零点、反馈及 PI |
| 上电零点 | ADC 四次和 → 1 秒 counts → 相电流 A | Axis 持有累计值；完成调用事务 AdvanceRevision，失败门禁保持 |
| 快拍/慢拍 | 采样与任务事实 → 控制/维护输出与状态快照 | Axis 固定调度；执行/维护/反馈各写现有显式对象；主状态机拥有生命周期 |
| 维护配置完成 | 已准入、无故障、COMPLETED/FINALIZE + epoch → 修订一次 | Axis 保留事实条件与提交位置；参数事务独占 epoch 去重与修订写入 |
| 参数读/写 | source、sequence、ID、整数位模式、expected revision → 值/状态 | 参数事务独占缓存及 revision；字段服务纯转换；成功应用后按原边界复位控制器 |
| 保存 | expected revision → 原 ticket → ACCEPTED/持久 revision/取消或失败 | 参数事务独占票据；主状态机受理任务；后台借记录、核对 claim、存储并归还，Axis 按原顺序 FinishBackground 后交付事实 |
| 状态观测 | 单份对象 → rad、rad/s、A、V、℃、有效位和生命周期 | Axis 同一短临界区快照；不向参数模块迁移测量或增加缓存 |

## 实施、删除与验收

1. 冻结本轮基点源及直接夹具；只读比较集成现场和直接接口。
2. 从 Axis 删除参数状态定义、字段重放/修订/保存逻辑及参数记录 I/O 依赖，
   由单一事务所有者实现；原 Watch 符号定义转移而非复制。
3. 直接编译真实事务生产文件与 Axis 转发；扩展已有请求夹具覆盖修订、失败无副作用、
   持久完成、取消/旧票据、存储与临界区次序；与冻结基点参数行为对照。
4. 仅变化直接检查和 PR；复用上一批未变控制算法证据。
5. Keil 公共工程不写：输出只添加一个生产 .c 的接线补丁；在冻结副本上确认单个主工程链接，
   保持原优化、Watch、调试和链接设置，容量增减据实记录，不运行其他工程/硬件。

无运行功能被删除；删除清单记录真实搜索和替代所有者。最终记录完整 SHA、实现接线、
离线/复用/未测证据与未完成项。回退仅撤回本轮提交，不回退集成现场和继承修复。

## 实际实现、状态和删除清单

源码提交：`4718d417d1b5917b84bcbed5e694a3b644bc77a1`。
实际入口：[Axis](../../../firmware/app/runtime/motor_axis.c) →
[参数事务](../../../firmware/app/runtime/motor_parameter_transaction.c)。
原协议、board_config 和主状态机调用 Axis 的公开参数入口；Axis 显式传入原对象，
不保留第二份状态或对象指针。参数事务不引用 Axis 全局运行对象或 CubeMX/HAL。

| 从 Axis 删除的实现/状态 | 保留理由与新归属 |
| --- | --- |
| s_config_revision / s_revision_work_epoch | 活动修订/维护完成去重仍使用；只有事务模块写入，原 Watch 符号保留 |
| s_last_read / s_last_write / s_save_job | 正在使用的身份缓存与原保存票据；定义唯一移入事务模块，合计状态仍为 80 B |
| same_write / stale_sequence / matches_cached_request / refresh_save_result | 字段与半圈规则有真实请求调用，事务模块保留，不改字段或拒绝顺序 |
| load_parameters / 参数借用和写记录 / 保存终态推进 | 复用固定工作区、Param_*、param_store 和原 claim；从调度层移出，无多余参数副本 |
| 公开入口内的参数策略及参数 I/O 直接 include | 原入口只转发；策略和 I/O 只归事务所有者，测量快照仍归 Axis |

搜索覆盖 firmware/tests/tools 的上述符号及全部 MotorAxis 参数调用。
未发现可直接退役的现用参数状态或功能，本轮不编造死代码删除收益。

## 接线、证据与未完成

新增一个生产源：`firmware/app/runtime/motor_parameter_transaction.c`。
YG_ESC_APP / Vector_Mini_ST_LoaderApp 的 00_App 组在 motor_axis.c 后各追加该文件，
继承现有组/目标选项；include 路径已覆盖 runtime。补丁为
`outputs/motor-axis-v2/keil-wiring.patch`，公共工程、逐文件优化和用户 Watch 配置未写入。
单个普通 APP 在冻结副本应用对应节点后只编译两个变化对象，0 Error / 0 Warning。

参数/保存夹具直接编译完整事务文件及 Axis 转发/后台生产体，新增修订饱和、完整字段冲突、
序号回绕、错误无副作用、七种保存/取消场景、原票据和边界调用序列检查；
冻结基点与候选摘要相同：trace=2310377353，read=7，apply=4，reset=9，revision=5。
启动零点/PI（2/6 mΩ）通过，诊断开/关各 19440 个调度组合与基点一致。
PR：29 native、22 integration 通过，Python 57/58；format/lint/布局/架构/接口/hygiene 通过。
总体仍 FAIL：基点两处普通启动提案断链导致 docs 和一个 harness 单测失败。

复用上一批匹配冻结输入的整包基线，不重建未变对象；111 个其它对象哈希一致。
整包 Code+RO 93348 → 93452 B（+104 B），静态 RAM 26904 → 26904 B。
Axis+事务对象合计 Code+RO 3064 → 3164 B；不把 Axis 单对象变小写成整包收益。
静态调用图：快拍 464 B、慢拍 420 B、后台 344 B 未变；Init 216 → 224 B，
参数写入口 192 → 232 B。无动态时序、抖动或中断叠加栈证据。

证据：`outputs/motor-axis-v2/` 的参数/调度/启动夹具、source-equivalence.json、
capacity-summary.json、接线补丁及产物；`outputs/runs/motor-axis-v2-pr/summary.json`。
未完成：ROOT 在共享工程统一接线、集成与组合验证；不自动集成。
未测：其它工程、实机时序/动态栈、Flash 掉电和物理 STOP；无硬件授权且未接触硬件。
