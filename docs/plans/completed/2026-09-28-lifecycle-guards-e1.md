> 历史阶段记录，2026-09-29 归档。下文是当时的设计、状态和证据，不代表当前待办或当前版本通过。现状见[框架收尾](2026-09-29-framework-closure-v1.md)与[历史清理](../../archive/plans/2026-09-29-legacy-cleanup-v1.md)。

# 生命周期真实条件接线 E1 v1

日期：2026-09-28。状态：本批代码已形成离线候选，完整框架和实机验收未完成。

## 目标、基线与范围

基线：`9226df154c1f6bc8eef9ecdd2be920d5a7d0c847`。独立工作树实施，不修改现场探针。
本批收口数字关相回读、基础采样进展与自检、清错先验条件及待执行目标撤销；按后续
删减裁决删除重复功率记录、无调用故障 getter、功率包装和旧差分矩阵。
不改变控制数学、保护阈值、schema11、调度频率、Keil 输入或线上 ABI；不接远程使能。
真实条件会改变此前占位条件下的准入，不声称为逐路径行为不变重构。

## 接线原则与验收

- 关相使用 `power_stage_hw_phases_disabled()`，只证明六路数字使能位关闭。
- 通用自检不要求编码器标定；采样尚未就绪时等待，不把第一次温度采样等待变成故障。
- 明确故障清除申请与已恢复判据，只有核心接受后才更新兼容故障值。
- STOP/FAULT 撤销尚未执行的目标；取消资源与 worker 来源代际没有证据时列为剩余项。
- 使用直接相关 native 夹具，执行适用 PR 检查和唯一 normal 工程离线编译。
  不进行硬件访问、模式矩阵扩展或其他工程构建。

## 回退

撤回本批提交恢复基线接口；不清理、重置或切换现场工作区。

## 条件来源与所有权

| 条件 | 本批来源、更新与所有者 | 限度 |
| --- | --- | --- |
| phases_disabled | 平台 `power_stage_hw_phases_disabled()`，判定时回读六路数字使能 | 不证明实际栅极电压，不证明启相六路全部成功 |
| samples_fresh | ADC2 JEOS 唯一快环末尾推进私有计数；每个 2 kHz 拍比较进展、有限电流/电压值与温度 valid/missed_ticks | 不提供 ADC 硬件帧编号；编码器新帧时间戳未统一，仍使用已有在线判据 |
| self_test_passed | 轴配置有效、上述基础健康、无兼容故障；第一次温度数据未到时留 SELF_TEST 等待 | 不要求编码器标定；未实现完整参数和所有外设上电诊断 |
| start_permitted | 轴有效；启相阶段复核既有电压窗口；依模式核对编码器在线与必要标定 | 运行阶段不套用启动电压窗口，原保护阈值保持；无新角色系统 |
| control_idle | 目标邮箱与 worker 结果邮箱均无待处理项；外环目前在同一监督上下文同步返回 | 不宣称未来后台作业已经释放 |
| startup_confirmed | 速度/位置复用外环 ready；START_DONE 另由核心核对数字关相回读为非关闭 | 部分输出使能不等于完整启动硬件证据 |
| fault_sources_clear | 普通事件仍使用现有单故障投影；显式 CLEAR 独立评估恢复/确认策略后向核心提出，不先写零 | 没有多活动故障快照；配置/操作类保留原操作员确认策略，不宣称它们已经实际修复 |
| operation_idle / maintenance_released | **未完成**，保留既有兼容值 | 不能当成实际工作区或 Flash 资源释放证据，下一批需模块查询与保存认领握手 |
| operation_permitted / request_fresh | **未完成统一策略**，保留单 normal APP 的原路径 | 没有角色/租约、新请求原始身份；事件不能只因当前 epoch 正确就被认为来源新鲜 |

生命周期和目标邮箱仍由 `foc_run_state.c` 私有对象拥有；20 kHz 只上报处理进展、结果与
快速故障关断，2 kHz 串行执行核心 Step 和模式提交，前台只提交目标/STOP/保存结果。
核心修改、启相复核和 CLEAR 判定到提交处使用短临界区，不新增 HAL、循环依赖或 extern 状态。
本批未完成 `MotorControl` 全局对象封装；这是已有跨层依赖，不作为本批新接口导出。

## 删除、替代与调用者

| 删除或收回 | 替代 | 调用者状态 |
| --- | --- | --- |
| power_on、fast_power_stopped 双份功率事实 | 平台数字关相回读与原快速关断入口 | 运行适配、目标准入、START_DONE 与快速故障均迁移 |
| Start_PWM_Generate / Stop_PWM_Generate 包装及重复声明 | 直接使用已有 power_stage_hw 契约 | APP 和齿槽标定真实调用者均迁移，native 替身对应更新 |
| Get_ErroNow | 删除无调用接口 | 搜索固件、工具和测试无实际调用 |
| Param_Return_Default 直接写 ModeNow | worker 返回 SWITCH_MODE(Save_Param)，运行所有者提交 | 默认参数实际应用仍在参数服务，启动装载不再附带模式写入 |
| 先清 ErrorNow，再检查 CLEAR 的分支，以及兼容零故障触发自动 CLEAR | 同一临界区保留原故障、评估策略、核心受理后提交 | CLEAR 后关闭输出并撤销目标，不自动启动 |
| 默认参数“下一拍即完成”猜测 | 等待默认参数 worker 请求保存的结果 | DEFAULTS → SAVE 链重新接通 |
| 46,464 格旧行为差分驱动 | 现有维护/恢复/目标用例及本批实际风险反例 | 基线差分已运行通过，仅保留为历史证据；不要求新安全准入复刻旧缺陷 |

`ModeSwitch_Handle` 暂留调试兼容准入；`Set_ModeNow` 尚有齿槽成表与前台保存握手调用。
二者不得作为新业务接线入口，后续需要有身份的请求/保存认领后再删除，不能直接删掉有效功能。

## 当前验证与剩余项

已运行基线运行状态夹具（46,464 tick）；修改后维护、标定、恢复矩阵、远程目标和
真实条件反例通过，包括首次温度等待、不把标定作为通用自检条件、ADC 无进展故障保持、
真实关相前 CLEAR 拒绝、STOP 丢弃待执行模式结果。

实际执行：`verify --profile pr --run-id e1-final-20260928`，doctor、工程布局、架构及
仓库卫生通过；19 组原生检查全部通过，Python 集成通过，单元 39 项中 38 项通过。
PR 整体 FAIL：格式 18 项、lint 925 项、接口契约 37 项均在未修改的基线文件；
文档 10 项为历史报告指向隔离树不存在的 ignored outputs，亦导致上述唯一单元失败。
没有修补或扩大豁免，也没有把这些失败记作通过。日志和机器结果位于
`outputs/runs/e1-final-20260928/summary.json`。

首次 normal 编译发现本模块引用温度失效宏却没有显式包含现有 `mcu_temperature.h`；
补齐 include 后 normal 全量编译 0 错误、0 警告。没有修改 100 ms 超时或新增板级依赖。
最终 map 实际 ROM 96728 B，静态 RAM 32512 B；Keil 报告 Code 91500、RO 5212、
RW 372、ZI 32140 B，ROM 按 map 中压缩后结果记录。没有同输入基线构建，不报资源差值。
构建日志在 `outputs/build/logs/Vector_Mini_ST.log`，首轮失败保留于
`outputs/e1/build-first-attempt.log`。只构建 normal，不操作硬件，不作为 release PASS。
补齐 include 后定向重跑运行状态夹具通过；最终 7 个修改 C/H 的 clang-format 检查和
3 个修改 Python 的 Ruff 检查通过。编译后仅调整本 C 文件换行，LF 规范化内容哈希保持
`d1db213407db668d6ecc53c6b342c630461796aa8e33ad6d9cbd980b0efd30ac`。
原 PR 记录发生在 include 修复前，保留原记录；不重复无关套件，也不声称该记录来自干净提交。
最终仅对本批修改文件使用仓库同一检查实现：format、lint、公共接口、文档及架构新增
问题均为 0，结果在 `outputs/e1/final-scope-checks.json`；这些定向结果不替代全量 PR FAIL。

明确剩余：维护取消释放仍为兼容路线；worker 和保存结果没有原始会话身份，仍会在消费时
使用当前代际，本批不声称旧结果隔离完成；CLEAR 配置/操作类仍是操作员确认策略；
部分启动仅有数字输出状态证据；内部 guard 故障可以锁入 APP_FAULT 而兼容 ErrorNow 仍为零，
此时输出关闭、目标被撤销且不能自动 CLEAR，但完整故障原因投影仍待接入。

快环增加一次私有计数递增和已存在故障入口的目标撤销；监督增加有限健康检查、平台回读与
一次运行状态轮询，无新循环、堆分配或 I/O 等待。实际 WCET/抖动与物理输出未经实机测量。
软件路径的 Flash/RAM 由上述本批 normal 构建记录，不复用旧镜像尺寸。

## 本轮 Mode2 整机接入事实

现场给定架空、33 V、最高 300 rpm、母线 3 A、相电流 6 A。这里只核对代码，不执行运动。

- `ModeSwitch_Handle(Speed_Mode)` 是尚存内部历史准入函数，会检查故障、轴配置、
  电压、编码器在线和标定，再登记模式；当前固件没有生产调用者，也没有可由 CAN
  调用该函数的启动入口。101 ENABLE 仍返回 UNSUPPORTED；116 只能更新已经
  APP_RUNNING 且同为 Speed_Mode 的目标。不得把写 `ModeNow` 或调用低层启相当成
  本批已经提供的受控外部启动接口。外部可用的 Mode2 启动接线是本轮运动的具体缺口。
- 内部速度启动由 `FocRunState_Tick` 进入 STARTING，待 `MotorOuterLoop_IsReady`
  和当前基础 guards 通过后启相；停止可用 `FocRunState_RequestRemoteStop` 及已接入的
  110/101 DISABLE 服务，关相回读使用平台接口。受理请求不等于停止确认。
- `MotorControl.current_limit` 为 A，速度 PI 归一化输出限制在 ±1，乘该值生成
  `iqRef`；`idRef=0`。`FOC.Iq` 为测量相电流经 Clarke/Park 变换的 q 轴分量，
  与 `iqRef` 同单位；在平衡正弦、Id=0 时该幅值对应相电流峰值，不是相 RMS 或母线电流。
  6 A 参考限制不能证明实际三相采样永不超过 6 A。纠正：normal Keil 工程显式定义
  `CURRENT_SENSE_SHUNT_MILLIOHM=6U`，本镜像的独立相电流跳闸为 18 A、连续
  5 个快拍；40 A 仅为未选中的头文件 2 mΩ 默认路径。没有本批新增的 3 A Ibus 控制限制。
- `speedRef`、`speed_limit` 为 rad/s；300 rpm 为约 31.416 rad/s。已有节点/轴限制
  更低时继续取低值，节点 1/2 的默认上限 20 rad/s 不能因本次运动上限自动升高。
- 维护资源释放和保存/worker 原始身份欠项，针对从无维护占用的 READY 进入并退出
  Mode2 的本轮路径，不要求开发维护功能；但其既有事实必须由集成方核实，不能在标定/
  保存并发或残留会话时外推本批结果。内部 guard fault 的完整原因投影和真实启相/
  关相、控制时序仍需单独验证；本批不声称全功能运动已安全。
