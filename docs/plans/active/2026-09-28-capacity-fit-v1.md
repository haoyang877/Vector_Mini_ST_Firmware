# APP / Loader 容量适配 v1

日期：2026-09-28。状态：执行中。
基线：集成提交 `7280e257c0a0e411b15638f9436799e9a5b682f7`。
本批使用现有 E 隔离工作树，分支 `codex/capacity-fit`；不修改现场或其他工作树。

## 目标与范围

用户要求先完成容量适配。本要求允许为容量定位与确认进行构建，覆盖此前本阶段
不构建的限制；功能回归和实机验收仍单独记录。禁止烧录、复位、设备参数写入和运动。
用当前三个工程的真实 map、日志和镜像确认 ROM/RAM/区域范围，不能套用历史占用。

优先修复构建缺口、去除无真实调用的链接输入、调整可追溯的编译优化和显式资源
预算。保留电机功能、控制数学、保护阈值、20 kHz/2 kHz 调度、schema11 RAW 4304 B
与 16 KiB 参数区。镜像读回、完整性校验、提交、试启动及失败救援不可删减。
如需改变布局，必须更新唯一目标输入及对应契约，不能只改链接脚本绕过边界。

## 子任务

1. 核对普通 APP、Loader 配套 APP、Loader 的工程钩子与实际工具链，构建当前基线。
2. 根据链接错误和 map 区分编译问题、Flash 缺口、RAM 缺口与重复保留资源。
3. 最小化修改容量来源；每次保留前后构建日志和 map，避免隐式功能裁剪。
4. 三工程在真实边界内成功链接，核对向量/描述符/邮箱/参数保护与镜像输出范围。
5. 记录实际尺寸、余量、工具链、源码身份、产物哈希及未测行为，提交可回退改动。

## 验收、证据与回退

构建不等于行为验收。容量完成要求当前三工程成功，APP/Loader 各自落在声明
区域内、参数区不重叠、RAM 保留邮箱且给出栈和静态数据预算。优化可能影响
代码时序；离线等价性和实机实时性留到统一测试，不冒充已经通过。

原始构建证据和局部工程实验只放 `outputs/`，不得追踪绝对工具链路径。
正式构建和范围核对结果记录在本计划或相邻报告；不将失败日志写成成功证据。
按本批提交逆序回退至基线，保留现场和历史标定工程，不 reset/clean 他人工作树。

## 阶段记录：先编译、分析，再缩减

用户进一步明确先编译新的 APP，分析占用原因，再开始缩减。当前停止采用编译器
和 Flash 优化实验，恢复原工程配置，先完成新的测量基线。

- 普通 APP：112016 B Flash、32080 B RAM，0 错误/0 警告；Flash 余 2672 B。
- 独立 Loader：20036 B Flash、5152 B RAM，0 错误/0 警告。
- 配套 APP：正式 78 KiB 区域链接失败。临时虚拟区域测量得到 120448 B ROM，
  含描述符地址间隙的跨度 120488 B，真实缺口为 40616 B。
- 额外发现描述符与代码区域缺少 FIXED 加载地址；测量链接临时修正，正式生成器
  未改。它与 Flash 缺口分别处理，不能因扩大区域而遗漏根区域错误。
- 构建前置修复包含缺失头文件、告警和 896 B 维护池 RAM 余量回收；没有更改
  栈、控制算法、参数 ABI、目标布局或采用正式 Flash 缩减。
- 最大维护借用的局部原生容量检查通过；全库格式和 lint 未通过，完整功能与
  release 验证仍未执行。没有硬件操作。

分析报告及后续顺序见
[新 APP 容量基线](../../reports/2026-09/app_capacity_baseline_20260928.md)。
子任务 1/2 的测量与原因分析已完成，子任务 3/4/5 仍待后续容量修改及验收。

## 删除历史兼容路径：参数装载第一批

用户确认老旧功能无需兼容适配，可以完全删除。先删除有明确版本边界的
schema 4～10 装载、历史单位/默认值迁移及启动自动回写；仅接受 schema 11。
无效/过时记录沿用默认配置回退，编码器标定标志清零，不启动运动、不改写 Flash。
当前 schema 11 的 4304 B 布局、节点限速、标定数据、参数校验和齿槽 CRC 保持。
跨电流采样配置的保护性失效处理属于当前硬件约束，继续保留。

这批不是完全行为等价重构：对旧参数的处理从迁移改为拒绝，版本化说明与拒绝用例
随代码提交；当前版本用例验证原有加载结果。移除仅为自动迁移服务的返回值和调用方
回写分支，不提供旧签名包装。Watch 和现有控制模式仍有真实调用，不按名称删除。
验收为针对性原生测试、当前工程构建和同口径 map 比较；不触碰设备。

第一批结果：普通 APP 从 112016 B 减至 111680 B（336 B）；配套 APP 的 ROM
从 120448 B 减至 120120 B，含地址间隙的跨度从 120488 B 减至 120160 B
（减少 328 B，仍超出 78 KiB 区域 40288 B）。两种布局的差额相差 8 B 来自
链接对齐，不能将普通 APP 的收益直接套到配套 APP。

当前 schema 黄金布局、参数/齿槽往返、无效/旧版本拒绝、硬件采样配置失配和
维护内存池容量测试通过。普通 APP 0 错误/0 警告；配套 APP 编译无警告，真实
Flash 区域仍链接失败，采用与基线相同的临时 FIXED/虚拟容量配置进行尺寸比较。
修改文件的格式及 lint 检查通过。新内存池测试已接入统一原生测试列表。

全库 PR 已执行，未通过：含现有测试夹具缺失依赖/旧头文件、历史文档产物链接、
格式和接口契约问题；项目布局、架构与本批参数测试通过。没有恢复被删的旧头文件
来使历史测试通过，也不把局部通过描述成全库通过。当前证据位于
`outputs/capacity-fit/remove-legacy/`，PR 清单位于相应 `outputs/runs/` 目录。
旧参数接受策略见 [schema11 加载策略 v1](../../protocols/parameter_schema11_loading_policy_v1.md)。

## 容量根因复核与隔离实验

用户要求重新认真核对大额 Flash 超限。当前 40288 B 缺口由新布局减少 34816 B
预算、APP 启动集成增加 8480 B 跨度、抵扣原有 3008 B 余量形成。
隔离构建复现当前基准；仅去掉无新入口的三个旧调度分支实测减少 6432 B，
仅状态机 O3 时间优化改为空间优化减少 1088 B，组合减少 7512 B，仍缺 32776 B。

前一阶段“现有控制模式有真实调用”的表述不能证明产品入口可达，已补充从使能、
维护与内部转换入口核对；正式源码尚未采用实验修改。后续优先清理这类旧功能
闭包，再评估编译选项、表数据、启动边界和布局。不得将虚拟链接成功作为容量完成。
详细证据见 [Flash 超限原因复核](../../reports/2026-09/app_flash_root_cause_20260928.md)。

## 实施批次：旧模式闭包删除与状态机空间优化

用户接受根因报告建议，开始正式实施。基准为 `0c1309c`（固件与 `e34deec` 相同），
普通 APP 111680 B、Loader APP 跨度 120160 B，实际缺口 40288 B。

1. 删除 12/16/18 三个无当前入口的内部模式及专用函数、旧阻抗和梯形轨迹模块，
   同步清理投影、复位、工程清单和夹具。其他模式编号显式固定，删除编号不复用；
   内部准入改为白名单，拒绝删去的数值与未知值。
2. 当前级联位置控制仍使用的摩擦回退值、参数边界迁入实际所有者，数值保持。
   schema11 的 4304 B 字段、编码器/齿槽数据、CAN 线格式、四个当前控制入口、
   STOP/FAULT/取消处理、20 kHz/2 kHz 调度和标定算法不变。
3. 两个 APP 工程的主状态机采用 O3/Ospace，其他文件选项保持；生成工程必须一致。
4. 验证删除模式拒绝、母线保护、运行所有者、外环/配置缓存、位置控制、参数黄金
   布局；执行 release 检查及真实普通 APP、Loader APP 链接，超限另用不可部署
   虚拟链接测量，核对资源/产物哈希。没有硬件授权操作，不宣称时序或实机验收。
5. 按实测保留这批变更后再测下一批。正式容量验收仍要求真实布局内链接成功。

回退用本批提交的逆向变更，保留 Keil GUI 自有选项及现场工作树。编译选项可能
改变时序，当前只能确认资源和离线逻辑；以后需补充实机快速环耗时与栈水位证据。

实施结果：普通 APP 104208 B ROM / 31912 B RAM；Loader 配套 APP 112472 B ROM、
112512 B 加载跨度 / 32144 B RAM，较基准回收 7648 B Flash，仍缺 32640 B。
独立 Loader 仍为 20036 B ROM / 5152 B RAM。普通 APP 和 Loader 均 0 错误/0 警告；
配套 APP 真实链接仍因容量失败，虚拟测量链接通过。正式生成器已补齐描述符和
代码区 FIXED 加载地址，没有扩大真实区域。

母线/删除编号拒绝、位置逐拍对照、外环、运行所有者、schema11 黄金布局、无感
标定状态迁移、齿槽、采样/数学等原生检查通过；17 项集成检查通过。修复了相关
运行所有者和启动顺序夹具缺失的现行接口桩，没有恢复旧头文件。
release 检查已执行但未通过：仍有全库风格、接口文档、历史文档链接和旧协议
夹具问题。证据目录 `outputs/capacity-fit/retire-modes/`；初次 release 清单为
`outputs/runs/retire-modes-20260928/summary.json`，其中位置/集成夹具问题已局部
修复并重跑通过，后续最终检查单独记录。没有硬件操作。

## 实施批次：正弦表无损半表存储

基准 `568dd04`：普通 APP 104208 B，Loader APP 跨度 112512 B，缺口 32640 B。
现有 1024 点表除两个正零端点外，后半段与前半段倒序取反的 float32 位模式相同。
保留前 512 点，查表时反射索引并取反，端点显式返回正零。角度归一化、LUT_MULT、
采样点值与余弦入口保持。浮点舍入使近零负角度归一化至 2π 时，原索引可能为
1024（旧表越界），本批显式返回周期端点正零，作为独立边界修复记录。

将基准整表以黄金位模式固定在测试数据中，逐点核对全部1024项，扫描正负多圈、
边界相邻浮点值及每个索引附近，比较 fast_sin/fast_cos 输出位模式；旧越界输入
单独验证周期端点。保持原快速环计算接口和调度，增加的查表分支耗时待实机测量。
验收包含原生数值检查、当前 APP 重建、前后 map、release 检查与产物哈希。
按本批提交逆向回退，不依赖设备参数或旧头文件。仅离线，不烧录。

实施结果：正弦表从 4096 B 降为 2048 B，查表/内联代码增加 80 B，整包净减少
1968 B。普通 APP 102240 B ROM；Loader APP 110504 B ROM、110544 B 加载跨度，
仍缺 30672 B。本轮累计回收 9616 B。204102 个角度的正余弦位模式、1024 个黄金
表项和 3 次周期端点修复检查通过，原平方根 2137118 个输入检查继续通过。
最新 release 中集成、位置伺服、外环、状态所有者、参数、数学、无感和齿槽等
相关检查通过；全库仍因工作区非干净、既有风格/文档/接口及旧协议夹具未通过，
不宣称 release 成功。没有执行硬件动作。

用户追加要求按功能分析 map，完整可求和功能账单见
[当前 APP 功能 Flash 占用](../../reports/2026-09/app_flash_by_function_20260928.md)。
最新证据在 `outputs/capacity-fit/sine-half-table/` 与
`outputs/capacity-fit/functional-map/`；验证清单为
`outputs/runs/flash-function-map-final-20260928/summary.json`。

## 实施批次：残留参数、数值编码与逐模块空间优化

用户同意按功能展开后的推荐顺序执行。基准为 `f0c2517`：普通 APP ROM
102240 B / RAM 31912 B；Loader APP ROM 110504 B、跨度 110544 B / RAM
32144 B。保留用户的 Keil GUI 选项修改，仅离线构建与验证。

1. 删除无控制调用者的旧位置增益和积分限值运行字段、默认值及加载/保存逻辑。
   schema11 对应字节改名为保留槽，读取忽略、导出清零，布局及有效参数不变；
   不增加旧字段别名或迁移实现。更新版本化契约与黄金偏移/忽略输入测试。
2. 对 CAN 测量编码、参数编码的软件双精度依赖做隔离实验。先冻结原算法作为
   测试参考，覆盖舍入、整数上下界、无效值和所有缩放。只有原契约逐值一致且
   整包有净收益才采用；两个入口的边界语义分别保留，不用简单 float 替换。
3. 逐模块比较 ARMCC5 空间优化结果，保存编译参数及整包 map，避免累计收益
   重复计算。优先冷路径，不改变快环控制算法或全局浮点语义。
4. 每批执行相关原生检查；最终执行 release（包含格式、lint、PR 检查及构建），
   普通 APP 与真实 Loader APP 均重建。超限只生成 NONDEPLOYABLE 诊断链接，
   不扩大正式区域。记录 Flash/RAM、产物哈希和未通过项，不宣称实机时序通过。

保持 CAN 消息布局、现行参数值、STOP/FAULT 与取消顺序、控制输出和调度频率。
移除运行字段会改变私有结构偏移，所有对象必须重建；现场 SWD 工具须使用同版
ELF。按独立提交逆向回退。确认本批净收益后再决定后续配置/状态适配和 Loader
契约调整的范围，不预先把标定功能移出 APP。

本批三步实测分别回收 248、2096、1736 B，合计 4080 B。普通 APP 98608 B ROM /
31896 B RAM；Loader APP 106424 B ROM、106464 B 跨度 / 32128 B RAM，仍缺
26592 B。22 项原生和集成检查通过，已修复旧协议测试接口及 Loader 内存模型
夹具问题。完整 release 仍因既有格式/lint/接口/历史文档链接及工作区非干净失败；
修改文件局部风格检查通过，无硬件操作。逐对象实验、资源和回退边界见
[第一批优化报告](../../reports/2026-09/app_flash_batch1_20260928.md)。

## 继续批次：大对象空间测量与配置校验收敛

基准 `cb21b25d`：普通 APP ROM 96808 B；Loader APP 诊断 ROM 104632 B、
跨度 104672 B、RAM 32128 B。78 KiB 分区仍缺 24800 B。保持统一 `Math_SinCos`
四分之一周期表，不恢复旧数学路径。保留工作树已有 angle_feedback 尾空格和
Keil GUI 选项修改，所有操作在当前隔离工作树执行。

先当次重建基准，再以单对象替换链接测量尚未比较的大对象及平台驱动 O2/O3 空间
选项。仅在整包净收益明确时采用；平台第三方源码保持原样。重点审查位置配置校验
重复展开及状态协调/启动接入职责，边界校验、失败响应、输出关断与取消顺序不删减。
若修改配置验证，用旧实现对照特殊浮点、上下界和随机配置，并执行位置逐拍等价
测试；不将缩短源码或单函数尺寸当作整包收益。

保持 schema11/CAN/镜像 ABI、保护限值、标定能力和调度。编译选项与校验实现分步
提交，按提交逆向回退。验收要求当次普通/真实 Loader APP 构建、不可部署测量链接、
原生及完整 PR/release 检查、产物/源码哈希；记录已有失败项。没有烧录、写参数、
复位、运动授权；MCU 周期、栈水位与实机电流仍未验收。若最终仍超限，明确剩余
缺口和需要用户决定的功能取舍，不扩大正式分区或宣称诊断镜像可发布。

阶段结果：标定/平台编译选项提交 `a41ac00` 回收 2408 B；位置配置有限值集中
扫描再回收 448 B，当前普通 APP ROM 94024 B，Loader APP ROM 101776 B、跨度
101816 B，仍缺 21944 B。配置判定 2464409 次及两组各 790068 拍等价检查通过；
23 项原生与 17 项集成通过，完整 PR 仍有既有门禁债务。详见
[继续优化报告](../../reports/2026-09/app_flash_continuation_20260928.md)。
用户已再次选择保留全部功能继续等价优化，禁止将标定移出 APP；后续只做平台、
状态协调与构建的隔离测量，保持无硬件动作边界。

用户进一步要求逐函数优化、同步熟悉代码。位置校验已提交 `43dbbf8`。
后续每个函数单独讲清职责、调用者、不变量与实测，不为了缩短源码强行改动。
第一项梳理 `RunState_BuildGuards`，保留其时效与资源释放守卫；仅精简其母线窗口
辅助函数的冗余有限值分支，整包实测再回收 8 B，当前缺口 21936 B。
下一项先读 `RunState_TryClearFault` 的三次守卫读取与恢复矩阵；是否合并必须先
区分资源快照和时间新鲜度，不默认认为同一临界区内重复查询都等价。

## 逐函数批次：运行所有者请求准入与事件提交

用户按新增内容排序指定先优化 `foc_run_state.c`。本批基准 `76c18107`，普通 APP
ROM 94016 B / RAM 31896 B，Loader APP 诊断 ROM 101768 B、跨度 101808 B /
RAM 32128 B。先当次重建与运行所有者测试，再逐项测量公共忙闲判定、控制模式映射、
事件提交中的真实重复。仅保留有整包收益且可解释的修改，不将函数缩短当作容量收益。

不变项：全部控制/标定/保存/Watch/Loader 功能、CAN/参数/邮箱 ABI、故障与 BUSY
返回优先级、短路求值顺序、临界区范围、守卫读取时点、停止关相与取消归还顺序、
原始操作/取消 epoch、派生 SAVE 不覆盖原标定终态、20 kHz/2 kHz 调度。
不缓存跨调用守卫，不引入新的可变状态或跨层接口，不修改生命周期纯核心。

验收：补充入口拒绝与错误优先级、四种控制模式、维护票据及终态的离线特征测试，
先在旧实现通过再验证候选；执行现有完整所有者测试、PR 检查、普通/真实 Loader APP
重建和不可部署诊断链接；记录源文件/产物哈希、Flash/RAM、汇编栈帧与未通过门禁。
MCU 周期、栈水位和电流没有实机证据，不宣称时序验收。只有离线操作，不烧录。
回退限于本批源码/测试/文档差异，保留已有 angle_feedback 和 Keil GUI 用户修改。
证据目录为 `outputs/capacity-fit/run-state-refactor/`。

实施结果：公共忙闲判断整包回收 48 B；三个事件入口直接初始化结构体再回收
64 B，合计 **112 B**。不采用无收益的控制模式映射提取、较差的公共 Step 包装和
事件指定成员初始化。模块代码加只读数据 8186→8078 B，两个发送函数的本地栈帧
120→96 B、120→88 B，守卫采集次数、故障恢复和会话编号均保持。
新增入口/优先级/终态特征测试在旧实现与最终实现均通过；23 项原生、17 项集成
通过，修改文件格式/lint 通过。完整 PR 仍因与上一批相同的全库门禁债务失败。
普通 APP ROM 93904 B，配套 APP 诊断 ROM 101656 B、跨度 101696 B，RAM 不变；
真实分区仍缺 **21824 B**，仍不能发布。源/产物哈希及保留用户修改的核对结果见
证据目录 `summary.json`；逐函数说明与汇编资源见
[继续优化报告](../../reports/2026-09/app_flash_continuation_20260928.md)。

## 逐函数批次：运行所有者残留重复与生命周期停止/故障入口合并

用户要求继续按交接顺序推进：先审查 `foc_run_state.c` 剩余协调逻辑，没有可证明收益
再转入 `app_lifecycle.c`。本批基准 `eef6e3ca`：普通 APP ROM 93904 B / RAM 31896 B；
Loader APP 诊断 ROM 101656 B、跨度 101696 B / RAM 32128 B，真实 78 KiB 分区仍缺
21824 B。仅离线构建与测试，保留两处用户修改；默认目录不参与。

`foc_run_state.c` 采用三个可证明的重复去除，先单独隔离测量再合并，均为整包净收益：

1. `RunState_ServiceBenchRequest` 内联的台架结果映射与文件内既有 `bench_result()` 的
   四分支完全一致，改为调用该函数，函数体减少 20 B。
2. `RunState_ServiceOperations` 的 ZERO 与 DEFAULTS 分支除目标常量外处理体相同，合并后
   传入当前 `lifecycle.snapshot.operation`；`sent`、`save_finish`、派生 SAVE 身份与
   短路顺序保持，函数体减少 12 B。
3. 提取文件内 `RunState_OwnerQuiescent()`，供 `configuration_idle` 与
   `FocRunState_BootHandoffReady` 复用同一组"模式失能 / 无占用 / 资源释放"判据，
   净减少 32 B。`configuration_idle` 中 `ErrorNow` 与 `ModeNow` 的读取次序交换不影响
   纯布尔合取结果，已在代码注释说明。对照实验：把 `ModeNow` 留在调用方或重复检查
   分别再少省 8 B，均未采用。

`app_lifecycle.c`：交接建议的 STOP/CANCEL 相同分支合并经 `-O2` 实测为 0 B，ARMCC 已
自行合并，未采用。改取提取文件内 `enter_stopped_state()`，供 `fault()` 与 `stopping()`
复用"取消操作 → 撤销作业 → 推进代际 → 置状态并清控制模式"序列，目标状态作为参数；
`advance()` 代际用尽转 FATAL 时两者都不再改状态的原有行为保持，净减少 24 B。

不得改动项：请求返回优先级、守卫短路顺序、故障恢复矩阵的多次读取、原始/取消/操作
epoch、派生 SAVE 不覆盖原标定终态、Watch/台架限时与快速保护、20 kHz/2 kHz 调度；
两模块 RAM 与 ABI 不变，未修改生命周期公开契约。按模块分别提交，可独立回退。

验收：`test_run_state.py` 14 项特征用例、23 项原生、17 项集成通过；当次重建普通 APP
与真实 Loader APP，超限时用 NONDEPLOYABLE 诊断链接。普通 APP ROM **93872 B**；Loader
APP 诊断 ROM **101568 B**、跨度 **101608 B**，RAM 分别 31896 / 32128 B 不变。Loader
整包减少 **88 B**（普通 APP 32 B），真实分区仍缺 **21736 B**。完整 PR 仍 FAIL：
format/lint/interfaces/docs 日志与上一批逐字一致，1 项 Python 单元仍为 10 个历史产物
断链，无新增门禁问题。证据见 `outputs/capacity-fit/deepseek/`（`summary.json`、
候选隔离尺寸与映射差）和构建目录 `outputs/capacity-fit/batch1/deepseek-01-baseline/`、
`deepseek-02-final/`；完整 PR 清单为
`outputs/runs/deepseek-runstate-lifecycle-20260928/summary.json`。没有硬件操作。

## 逐函数批次：参数读写/保存请求身份冲突合同

交接顺序 C：`motor_state.c` 与 `parameter_value_service.c` 的请求来源、序号冲突与重试。
本批只处理 `motor_state.c`，基准为上一批提交 `c8aca9f1`（Loader APP 诊断跨度 101608 B）。

`MotorControl_ReadParameter`、`MotorControl_WriteParameter`、`MotorControl_RequestSave`
各自用来源+序号判据识别"命中已缓存票据"的身份冲突，但三处重复展开同一合同、且各自
包含的缓存集合不同。提取文件内 `matches_cached_request(source_node, sequence)`，返回是否
命中 `last_read`/`last_write`/`save_job` 任一缓存；三个入口在各自的主缓存重放分支之后
调用它。由于调用点均处于"主缓存已判定未命中"的 `else` 分支，判据集合加入主缓存在逻辑
上无变化；返回优先级、原结果重放、配置修订号与持久化修订号、`stale_sequence` 顺序均保持。
`parameter_value_service.c` 本批未改。

`MotorControl_ReadParameter` 段 276→240 B、`WriteParameter` 388→332 B、`RequestSave`
276→228 B，新增辅助 72 B，函数段合计减少 68 B；整包普通与 Loader APP 各净减 **64 B**。

新增原生特征测试 `tests/unit/native/motor_state_request_test.c`（runner
`test_motor_state_request.py`，从生产源码切片真实函数体）：读身份重放与换参数冲突、
写身份重放与不同值冲突且不重复应用、保存受理后读/写冲突、同身份保存重放原结果与票据、
不同身份保存 BUSY 且无票据。该测试先在旧实现通过，再对最终实现通过。

验收：新测试、`test_run_state.py` 14 项、23 项原生、17 项集成与架构/布局/卫生通过。
当次重建后普通 APP ROM **93808 B** / RAM 31896 B；Loader APP 诊断 ROM **101504 B**、
跨度 **101544 B** / RAM 32128 B 不变，真实 78 KiB 分区仍缺 **21672 B**。完整 PR 仍 FAIL，
format/lint/interfaces/docs 日志与上一批逐字一致，1 项 Python 单元仍为历史产物断链。
证据：`outputs/capacity-fit/deepseek/summary-motor-state.json`、构建目录
`deepseek-02-final/`、`deepseek-03-final/`；PR 清单
`outputs/runs/deepseek-motor-state-20260928/summary.json`。没有硬件操作。

## 功能取舍批次：退役 SWD 台架邮箱

用户要求删除固件内的台架/调试代码以回收 Flash。原 `foc_bench_speed.h` 定义的
`FocBenchSpeed` SWD 邮箱（版本 3、92 B）及其在运行所有者内的请求受理、序号回放、
状态发布、快环期限/相电流/超速截止，以及各入口钩子全部删除；`foc_bench_speed.h`
文件删除。公开请求入口（使能、目标、维护、清空、升级交接）与生命周期核心保持不变，
维护/标定/保存改由 CAN 适配器与前台调用。这是功能取舍而非等价重构：速度台架
START/UPDATE/STOP、command 4..12 直达、独立试验期限与采样式相电流截止不再可用；
产品相电流/过流保护不依赖本邮箱。

`tests/unit/native/run_state_owner_test.c` 同步改为经公开 API 驱动会话（新增
`begin_maintenance`），保留控制受理、同模式目标、维护完成/派生 SAVE、借用取消、
故障锁存与 CLEAR、编码器离线快检等行为；台架专属用例（期限截止、采样式相电流
截止、RAM 限值切换、applied_seq）随功能移除。定向 `test_run_state.py` 13/13 通过。

文档：`docs/guides/watch_motor_commands.md` 改为退役记录；`docs/README.md`、
`docs/architecture/code_structure.md`、`docs/guides/system_refactor_candidate_20260928.md`、
`docs/guides/cogging_calibration.md`、`docs/plans/active/README.md` 与 E2/E3 计划同步更新。

验收边界：本批只做代码/测试/文档的离线修改，未进行 Keil 构建、未烧录、未操作硬件；
实际 Flash 收益需集成方用当前 normal 工程重建后以 map 核对，本计划不预填字节数。

集成结果（本机重建，`deepseek-04/05-bench-*`）：删除后台架退役残留的死变量
`completed_mode`（原仅供已删邮箱发布读取），消除唯一编译警告；普通 APP 0 错误/0 警告。
普通 APP ROM 93808→**91816 B**、RAM 31896→31752 B；Loader 配套 APP 诊断 ROM
101504→**99512 B**、加载跨度 101544→**99552 B**、RAM 32128→31984 B。两个镜像各回收
**1992 B Flash / 144 B RAM**；78 KiB 分区仍缺 **19680 B**。`test_run_state.py` 13/13 通过。
这是功能取舍批次，不并入等价优化口径。证据在 `outputs/capacity-fit/batch1/deepseek-05-bench-clean/`。

## 逐函数批次：协议链路业务入口守卫

交接顺序 D。基准上一批提交 `f3915b1`（Loader 诊断跨度 99552 B）。`yg_protocol_link.c`
的六个业务入口（device_status、motor_action、motor_control、parameter、job、boot）重复
展开同一入口守卫：link/request/result 指针非空且线格式有效。提取文件内
`request_context_valid()`，判据与短路顺序保持；`handle_motor_action` 的附加非空负载
检查仍在其后。该文件按 `-O2`（工程 Optim=3）编译；`request_is_valid` 并入新辅助函数
（40→46 B），六个入口各缩小，函数段合计减少 40 B，整包普通 APP 减少 40 B、Loader 诊断
跨度减少 32 B。这是等价去重。

验收：`test_yg_protocol_link.py` 与既有协议/所有者测试通过；当次重建普通/真实 Loader
APP，普通 0 错误/0 警告。普通 APP ROM 91816→91776 B；Loader 诊断 ROM 99512→99480 B、
跨度 99552→99520 B，RAM 不变；78 KiB 分区仍缺 19648 B。完整 PR 仍 FAIL，format/lint/
interfaces/docs 日志与上一批逐字一致。证据 `outputs/capacity-fit/deepseek/cand-link-guard/`、
构建目录 `deepseek-06-link-guard/`。

## 功能取舍批次：退役 USART1 外设

用户要求删除 UART 功能。`main.c` 的 `MX_USART1_UART_Init()` 是固件内唯一引用，其余
代码无串口收发。按 CubeMX 一致方式退役：`Vector_Mini_ST.ioc` 关闭 USART1（移除
`Mcu.IP12`、`USART1.*`、PB6/PB7 信号、`RCC.USART1Freq_Value` 与 function list 条目，
`Mcu.IPsNb/IPNb` 递减）；删除 `usart.c`/`usart.h`；`main.c` 移除 include 与初始化调用；
`stm32g4xx_hal_conf.h` 注释 `HAL_UART_MODULE_ENABLED`；普通与生成 Loader 工程的
`uvprojx` 同步移除 `usart.c`。这是功能取舍：USART1 不再初始化；UART 升级链路仍属
未实现的规划能力。README 与 `usb_removal.md` 同步更新。

验收：当次重建普通/真实 Loader APP，普通 0 错误/0 警告。普通 APP ROM 91776→**89808 B**、
RAM 31752→31600 B；Loader 诊断 ROM 99520→**97512 B**、跨度 99552→**97552 B**、
RAM 31984→31840 B。两个镜像各回收 **1968 B Flash / 144–152 B RAM**；78 KiB 分区仍缺
**17680 B**。构建输入变化需 CubeMX 复核与 release profile。证据
`outputs/capacity-fit/batch1/deepseek-08-uart-final/`。

## 布局批次：Loader 分区 28K→20K 与 Loader 寄存器级 Flash

用户要求把 Loader 分区改为 20K，并将 Loader 代码优化到 <19KB。布局唯一来源
`shared/targets/vector_mini_st_g431.json` 改为：Loader `0x08000000/0x5000`（20 KiB）、
Boot `0x08005000`、Record `0x08006000`、APP `0x08006800/0x15800`（86 KiB）、Param 不变；
用 `loader/tools/generate_target.py` 与 `tools/build/generate_loader_app_project.py`
重新生成 `.h`、`loader.sct`、`app.sct` 及两个 Keil 工程。

Loader 原 20036 B。全局改 `-O2` 实测 20060 B（反而 +24 B，已回退 O3）；源码去重无
可观项。改 `loader/ports/stm32g4/platform.c` 的擦/写为寄存器级（G431 单 bank、2 KiB 页、
双字写、写前后关/复位数据缓存），并注释 Loader 的 `HAL_FLASH_MODULE_ENABLED`，丢掉
HAL Flash（`hal_flash.o` 400 + `hal_flash_ex.o` 372）。

结果：独立 Loader ROM 20036→**19396 B（18.94 KiB）**，0 错误/0 警告，装入 20 KiB 分区
（余 1084 B）。Loader 配套 APP 槽 78K→**86K（88064 B）**；正式链接仍超限，聚合缺口
17680→**约 9468 B**。**注意**：寄存器级 Flash 擦写属安全相关，本机仅离线验证，
未烧录、未做硬件验证；新布局对所有镜像生效，需发布方按新地址重烧 Loader/Record/APP。
证据 `outputs/capacity-fit/deepseek/loader-build.log`、
`outputs/build/loader/vector_mini_st_g431/`。

## 等价批次：APP 侧改为寄存器级 Flash

APP 内 HAL Flash 只用于 `bsp/flash.c`（参数存储）与 `ports/boot/boot_hw_stm32g4.c`（记录页）。
两处擦/写改为寄存器级（单 bank、2 KiB 页、双字、写前后关/复位数据缓存），但**保留
`HAL_FLASH_MODULE_ENABLED`**：关闭模块会使 `FLASH_LATENCY_4`/`FLASH_PAGE_SIZE` 及 rcc 的
latency 宏在 vendor/CubeMX 代码中失效（实测报错）。保留使能后 HAL Flash 函数无人调用，
由链接器回收 `hal_flash.o`/`hal_flash_ex.o`。

结果：普通 APP ROM 89808→**89156 B**（−652）；Loader 配套 APP 诊断跨度 97552→**97004 B**
（−548），0 错误/0 警告。新布局下 APP 缺口 9488→**约 8940 B**。寄存器级 Flash 属安全相关，
本机仅离线验证，未烧录。证据 `outputs/capacity-fit/batch1/deepseek-10b-appflash-format/`、
`outputs/runs/deepseek-appflash-20260928/summary.json`。
