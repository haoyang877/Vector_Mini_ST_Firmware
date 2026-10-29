# 齿槽标定终止路径简化与离线验收

日期：2026-10-08。分支：`codex/cogging-core-simplify`。本轮只改齿槽适配源/头文件、
已有齿槽夹具以及本模块职责/计划/报告；未接触硬件，未集成到主线。

## 输入与待审范围

集成 base：`b0077b60b3eb84d8d72b1ec42c62141573b3245d`。
继承现场冻结 base：`7031f5cfac8c315205792c51427f75de48076cf1`。
后者是独立测量/测试基线，包含以下 20 项现场输入，不能当作本轮收益或待合入变更。
逐项原始 SHA-256、来源 worktree 与原现场状态保存在
`outputs/cogging-core-v1/inherited-inputs.json`、`integration-status.txt` 和
`integration-firmware-tests.patch`；原集成树和默认项目目录未被写入。
uvprojx 的现场差异只有换行，Git 内容一致；当前用户逐文件优化设置保持。
主源码 Git 内容与集成 HEAD 一致，现场 LF SHA-256 为
`194d13f0357db3014f013af672130025e163dd6923cfa314de7ba0da7b098f02`，
独立树检出 CRLF 不改变编译内容。

源改动提交：`f97db64f3019e75f945b36832b7357114ab034d5`。
验证还单独继承未跟踪的 `docs/protocols/yg_protocol_ordinary_start_proposal_v0_1.md`，
提交 `fa876d996a4a15b84b7c81031c0a96b06fbf830c`，现场 SHA-256 为
`b4aae662b162d62d6422434485b6107a76711aa7be724f915e05c9f5b52cc6db`。
该文档已被原架构文档引用，缺失会同时使 docs 门和 Python 单元文档门失败；
只复制现场原文，没有修改其内容。新增验证输入见 `inherited-doc-input.json`，
不影响固件容量、不属于待集成提交。本轮最终完整 base/head 与提交分类保存在
`outputs/cogging-core-v1/review-bundle.json`。

| 继承路径 | 现场状态 |
| --- | --- |
| `firmware/app/composition/yg_protocol_owner_adapter.c` | 修改 |
| `firmware/app/motor_state_machine/motor_state_machine.c` | 修改 |
| `firmware/app/runtime/motor_axis.c` | 修改 |
| `firmware/communication/can/yg_protocol_link.c` | 修改 |
| `firmware/communication/protocol/yg_protocol_job.c` | 修改 |
| `firmware/communication/protocol/yg_protocol_job.h` | 修改 |
| `firmware/communication/protocol/yg_protocol_motor.c` | 修改 |
| `firmware/motor/feedback/angle_feedback.h` | 修改 |
| `firmware/motor/foc/foc_sensing.h` | 修改 |
| `projects/stm32g4/MDK-ARM/YG_ESC_APP.uvoptx` | 修改 |
| `tests/run.py` | 修改 |
| `tests/unit/native/execution_dispatch_test.c` | 修改 |
| `tests/unit/native/run_state_owner_test.c` | 修改 |
| `tests/unit/native/test_current_precision.py` | 修改 |
| `tests/unit/native/test_execution_dispatch.py` | 修改 |
| `tests/unit/test_speed_loop_bandwidth.py` | 修改 |
| `tests/unit/yg_protocol_contract_test.c` | 修改 |
| `tests/unit/yg_protocol_link_test.c` | 修改 |
| `tests/unit/native/test_startup_current_offsets.py` | 新文件 |
| `tests/unit/startup_current_offsets_test.c` | 新文件 |

## 核心梳理与窄改动

分阶段数据流见[齿槽标定边界 v1](../../archive/architecture/cogging_calibration_v1.md)。
真正核心是双向点采样与候选表去均值/身份/CRC；位置保持、会话安全门、台架 guard、
补偿准入和前台提交是现有必要边界，全部保留。

原启动失败、安全失败、采样失败、采样完成分别清理本地目标/控制器并拼装结果。
现在 `FocCogging_Task` 统一终止清理，先清零四个控制目标，复位 PI/电流控制器并退出
本地会话，再置成表挂起/返回 FINALIZE 或返回 FAILED。运行和 PREPARED 路径原样返回，
失败报告仍在清理之前，真实关相仍由主状态机执行。生产 `.c` 为 +27/-36 行。
`.h` 仅澄清取消认领、CRC 上下文、COMPLETE 与真实保存提交的含义，公开声明无变化。
未改变数学、单位、阈值、字段布局、签名、参数记录、Watch/Keil 入口或所有者模块。

## 同输入整包与对象容量

基线及新实现各执行一次 normal 重建，ARMCC 5.06 update 7 build 960；均为 0 错误、
0 警告。同一冻结源码、工程与优化设置，仅齿槽 `.c` 的有效编译内容不同；
构建后的头文件改动仅为注释，复用本次构建，不重复构建。未放宽链接边界。
历史 3214 B 与本次对象基线相符，但容量依据是本次两份 map，不引用旧输出作证明。

| 项目 / B | 基线 | 简化后 | 差值 |
| --- | ---: | ---: | ---: |
| 模块 Code | 3212 | 3144 | -68 |
| 模块 RO | 2 | 10 | +8 |
| 模块 Code+RO | 3214 | 3154 | -60 |
| 模块 RW / ZI | 60 / 4256 | 60 / 4256 | 0 / 0 |
| 整包 Code | 90228 | 90160 | -68 |
| 整包 RO | 3120 | 3124 | +4 |
| 整包 RW / ZI | 304 / 26600 | 304 / 26600 | 0 / 0 |
| map ROM / 实际 HEX 数据 | 93360 | 93296 | -64 |
| 整包 RW+ZI（含既有 3072 B 栈保留） | 26904 | 26904 | 0 |

map ROM 使用链接器实际压缩后的初始化数据；不能把未压缩 RW 直接相加当占用。
HEX 数据区由 `[0x08000000,0x08016cb0)` 变为 `[0x08000000,0x08016c70)`。
对象大小表只有本模块变化；整包与对象收益差 4 B 来自链接后 RO 布局差异。

反汇编检查：Task 自身栈帧 40→48 B，2 kHz 服务自身栈帧 32→24 B；
旧 local_stop 为 8 B 独立帧，新实现内联到统一终止路径。静态 RAM 与栈保留不变，
这不是运行时最大栈或 WCET 测量；未宣称耗时、抖动或全调用链峰值栈完全相同。

原始 build/log/map/AXF/HEX/object 与反汇编分别保存在
`outputs/cogging-core-v1/baseline-build/`、`head-build/`；机器对比见 `measurement.json`。

| 产物 | SHA-256 |
| --- | --- |
| baseline/YG_ESC_APP.axf | `b16a5c41c30f1f08682b80bef172379c3db8223cdbb29a82678af9e6b7d541cd` |
| baseline/YG_ESC_APP.hex | `20489a47fdd360b375fd75c46c61f1d1dbe842bcc9c41815f1acdf084f5f579b` |
| baseline/YG_ESC_APP.map | `b43c7e2cdaac391a2198878fbd20b088056350dc3b5cb8585e54d18d86969940` |
| baseline/foc_cogging_calibration.o | `8a0babc1bcf91d4e78046e232752985c721084a6e65fa23f4031f1f65589e896` |
| head/YG_ESC_APP.axf | `c1543658f0c72cc378241ef7bc379057acc027d4ac9f5a43edc5854c7d3fc889` |
| head/YG_ESC_APP.hex | `52694ea94605544b5128f0d29f8fa65ec7dc2df3141a1efeb57747ee89e9eb6d` |
| head/YG_ESC_APP.map | `9b04bfd25023aa12ff1f894b1d2589026291019575229547aa3a4ee3b398061c` |
| head/foc_cogging_calibration.o | `d01b23230bd489ce650034596d869363b350ae4cae4116a5111bc585c105e414` |

## 已执行、复用与未测

- `uv sync --locked --extra dev` 成功，依赖版本按锁文件。
- 原已有齿槽夹具在基线通过；增强的同一夹具在基线与简化后通过。
  复用双满量程、全 1024 点双向覆盖、拒绝样本、超时、饱和、签名/CRC、
  补偿插值/渐变/限流、租约/测速跳闸与同拍遥测检查。
- 增加真实任务采样超时/饱和终止检查，确保目标/PI 归零、恰好一次本地复位、
  终止后不再执行 FOC_Current、旧活动表保持；认领取消复用已有 cogging_finalize 夹具。
- 15,030,360 个合成快环拍的六个控制/PI 浮点值位模式及六个结果/状态字段摘要一致：
  `37c45ddf`；候选完整记录 CRC `72892f5a`。两份夹具日志逐字节相同，SHA-256 为
  `acdfd5c3f15ec710cb96f2e701975ecdc2f1c33c63d1fc29686b835d61bb3833`。
  摘要比较是离线回归证据，不是实物动态或安全认证。
- 已独立执行 format --check、lint，通过；完整 PR profile 已通过。
  初次 `20261008T043432278265Z-f97db64f` 的 FAIL 保留：只有既有文档断链及其
  Python 单元门失败，齿槽与所有原生项通过。单独冻结上述原提案后重新验证，
  `20261008T043649780274Z-fa876d99` 全部 PASS，Git clean，
  `hardware_contacted=false`；源码仍是 f97db64，未改代码或放宽任何门。
  两次完整 `summary.json` 与各项日志均保存在 `outputs/runs/` 对应目录。
- 真实存储提交/取消资格以现有 APP 生产调用链静态核对和既有离线夹具验证；
  不把候选发布或模拟成功当成实际 Flash 保存成功。
- 未测实物采样、运动、真实关相回读、断电持久化、ISR WCET/抖动与峰值栈；
  未运行 release profile、多工程重复构建或任何硬件 bench，不作发布/部署结论。

## 未实施事项与回退

未改变采样/成表算法、浮点阈值、量程、ABI、补偿状态框架或真实力矩守卫。
没有为容量关闭功能，没有修改 EncoderCalibrationTask/MotorAxis/主状态机/共享工作区
或通信（它们仅在独立冻结基线中继承现场版本）。
回退只撤销本轮模块提交；ROOT 审查后只集成本轮源改动与证据文档提交，
不合入 7031f5c 或 fa876d9 两份继承输入冻结提交。
最终文档收尾只更新计划归档、报告证据和链接，固件/测试输入未变，复用已通过的 PR，
并单独核对最终文档链接。计划见
[等价简化 v1](../../plans/completed/2026-10-08-cogging-core-simplification-v1.md)。
