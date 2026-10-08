# 编码器标定核心抽离 J 验收 v2

日期：2026-10-08。状态：模块实现与离线验收完成，待 ROOT 审查/工程接线/集成。
一页[核心/外围数据流](../../architecture/encoder_calibration_core_v2.md)给出输入、单位、
状态写者、结果和发布/取消/保存链；[计划](../../plans/active/2026-10-08-encoder-calibration-core-v2.md)给出回退。

## 完整基点与源码

本轮 base `9b6e761f0567157232643de69c3cfcc2f24dace1`。
已验证 source head `724abb643abe9fdce6ae97f005af89d34522482b`；最终报告提交 HEAD 在交付消息另列。
上一批实现 `ddd37716bb275b6cb6776884d2bdc424d2544a46` 和报告已被 ROOT 接受、尚未集成。
原集成起点 `b0077b60b3eb84d8d72b1ec42c62141573b3245d`，继承快照仍为
`10ef59b174d4765776d13aff79c7c1ead1258a23` 与 `7c3725b7678d19fb3949b74ea54d9b130a292769`。
不回灌继承快照；本轮不复制新的集成 dirty 改动。冻结时相关集成文件哈希、全测试源码 SHA、
本轮实际 LF/原字节 SHA、证据哈希在 `outputs/encoder-calibration-v2/evidence.json`。

| 实际本轮源码 | 原字节 SHA256 |
| --- | --- |
| `firmware/motor/calibration/encoder_calibration_core.c` | `393d065fbdb98c29cc485867314fca051329a08574dfbcda6218e6f2bc813dee` |
| `firmware/motor/calibration/encoder_calibration_core.h` | `a57f349349639842fb8f649921743a843b88c34dac60cb2b073d235660625c02` |
| `firmware/motor/calibration/encoder_calibration_task.c` | `1e0c2afe05a7cbdf5a826a85038e14286f56f220ccac90ecf0021416ed790ce2` |
| `firmware/motor/calibration/encoder_calibration_task.h` | `069dc6c6bc29062d8e7e48e029646e54e87880d81b8a68489165469f3b320b93` |

基点 task.c 为 `d5da247cfd2f59fa38991c34eaa745897a40b7d11fbb44e4a9e9deb25993e295`；
基点 task.h 原 CRLF 为 `cf6e0df548da175776df6b4fc14065a24db17c411d43d440aa1383711a587450`。
集成树 task.c/task.h/angle_feedback.h 原字节为
`a5e8d511ddd41517be7bdf12882c0b2c1903a9ab8195bcf02bd2c729723e9bee` /
`18a70bbfa022dfae107cf4c9002d4608a3d5191013d661ec0ff38af95b0ad0b7` /
`b65c25d1e6211fde951541911fa4241d2632c7f771c3946bbda9cd17e9d6acf6`。
原树、用户 Watch/项目配置和实验产物没有修改。

## 实际依赖、所有权与删除证据

核心只 include 自己的头和标准 bool/int/limits，没有 FOC、MotorControlState、
EncoderState、平台头、可变全局、硬件/Flash 调用或保留的借用指针。
它处理明确采样输入与 `sample_window/build_state/verify_state`，不决定电机阶段、准入或输出。
这些状态和原点/零位标量由 20 kHz 任务持有；初始化/窗口切换由任务做，计算更新通过核心 API。
APP 仍独占/借出固定工作区，主状态机独占功率关断，后台独占独立参数 SAVE。

| 从旧任务移除的实现/表示 | 替代和真实生产调用 |
| --- | --- |
| `encoder_calib_q15_difference` 私有实现 | Core_Q15Difference：任务找原点、核心原点/采样/验证 |
| 隐式读全局工作区的候选插值 helper | Core_ApplyLut：核心验证与任务发布当前角 |
| 原点/电零位内联累加与两份均值 | Core_AddOriginSample/AddZeroSample/MeanQ15：两个真实模式采样与完成 |
| 采样单 bin 归约、四遍成表、残差及 RMS/峰值内联数学 | Core_SampleLut/BuildLutStep/VerifyLut/ValidateLut：对应生产阶段 |
| 构表、残差与行程的散落内部标量 | 单份具名计算状态，保留其计算，不保留第二套缓存 |
| 两段合格正行程加法 | 核心私有 add_positive_travel，SampleLut 与 VerifyLut 同契约复用 |
| 任务不再使用的 limits include、失效历史模块注释/旧结果类型名 | 限值头归核心，任务契约直接描述当前职责 |

`rg` 搜索 firmware/tests：旧两 helper 没有生产或 C 夹具调用；仅剩历史
`tests/fixtures/app_naming_v1.json` 的改名映射，该历史记录保留。
九个核心公开函数都有当前生产调用，没有回调框架、注册表或兼容旧执行路径。
反馈层私有插值不在本轮改动范围，未复制为新的公共数学库；齿槽算法单位/语义不同，未强行共用。

Task API、阶段枚举、现用 Watch、原点和 `s_zero_*`/任务计时符号保留。
未被现用 Watch 或登记诊断引用的计算标量改为具名状态成员，详见一页数据流。
Workspace 字段、偏移和6144 B布局不变，并静态核对 Q15/表几何；未改变线路/参数/Flash ABI。
所有边界、稳定/跟踪/超时、每拍8 bin、候选验证、发布后零位失效/rebase/测速重置、
引用释放、低速卸流、故障/取消与 owner 实际关相/保存次序保留。

## 真实接线与已执行证据

已有登记入口 `test_maintenance_workspace.py` 直接链接新 core.c 与完整 task.c；
独立核心可只用 calibration include 目录和标准头编译运行，验证了对 FOC/平台头的独立性。
直接检查：262144 个环形差、跨零/空均值、末 bin 插值、无效帧间隙、负增量、计数饱和、
四遍512拍/每拍8 bin、覆盖不足、整圈平移后范围失败、空验证及峰值/RMS含等号边界，全部 PASS。
完整任务保留上一批10类失败/低速完成/幂等取消和真实工作区互斥/复用检查。

同一夹具把基点完整 task.c 与当前完整 task.c 编译后施加同输入，逐拍归约工作区、阶段、
结果/电流/速度指令、零位/标志及零位累加状态：Mode13 34320拍（含不稳定采样帧）、
Mode15 30000拍跨零平均，均得 zero_q15=1，摘要 `58d6cc0c6e6e1ca2` 相同。
宿主夹具为原多圈 float→uint16 表达式关闭 float-cast-overflow 检查；生产表达式/选项未改，
该同编译器比较不能外推为 ARM 浮点转换或实机证明。

`format --check`、lint、干净实现提交的 PR 全 PASS：59 unit、22 integration、
全部原生入口与架构/接口/文档/卫生。Python3.13.15、Zig0.16.0。
证据：`outputs/runs/encoder-calibration-core-pr-v2/summary.json`，
`outputs/encoder-calibration-v2/equivalence-final/test.log` 和 `evidence.json`。
复用上一批已匹配的基点/继承输入记录；不复用旧对象/HEX作为本轮通过或容量证明。

共享工程未修改。可应用候选接线补丁为
`outputs/encoder-calibration-v2/keil-core-wiring.patch`；在独立树和当前集成树只读
`git apply --check` 均通过。ROOT 复核当前共享输入后应用：

- normal `YG_ESC_APP.uvprojx` 和配套 `Vector_Mini_ST_LoaderApp.uvprojx` 的 Motor 组，
  在现有 task.c 前新增 core.c，FileType=1，路径 `..\..\..\..\motor\calibration\encoder_calibration_core.c`。
- 新节点复制各工程现有 task 的 FileOption（Optim=4、oTime=0等）到 core，不改原任务节点、
  用户逐文件选项、uvoptx或Watch；不新增 include 路径、不登记头文件。
- 集成新源文件后由 ROOT 做共享组合链接；PR 的项目布局检查只证明路径/include，
  不能证明尚未应用的 Keil 链接接线。

## 未完成项与回退

待 ROOT 审查并应用共享 Keil 接线、组合构建/容量/静态栈核对。没有本轮整包重链接、
ROM/RAM净变化结论；上一批 -32 B 不能外推或相加。本轮未测 ISR时间/抖动、动态栈、
物理标定/输出关断/Flash保存；未接触硬件、烧录/复位/运动、认证或发布。
没有算法/单位/阈值调整或有效功能关闭。只回退本轮 base 之后的实现与报告，保留上一批和继承输入。
模块工作到此停止，待 ROOT 审查，不自动集成或扩大范围。
