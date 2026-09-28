# 快环调度与控制执行边界收口 v1

日期：2026-09-29。状态：本批源码与离线验收完成，完整门禁仍有既有失败。
基线：`fc6c876ce84188d61f21be140ebdfbc8c6bb1205`。

## 目标与范围

把控制执行模块反查生命周期、选择维护任务和空闲 PWM 预载的行为归还给
`app/motor_state.c`。`motor_execution.c` 只执行显式传入的控制对象，不读取状态机，
不调度维护。移除状态机动作执行中已被忽略的布尔参数及沿调用链传递的空参数。
沿用真实硬件能力、纯生命周期核心和功能结果，不增加管理器、回调或常驻状态。

本批不迁移公开 SWD 齿槽邮箱，也不同时清理其他模块的五项架构债务；它们分别需要
迁移真实调试调用者和配置契约，不能与快环调度变更混为一批。
用户随后明确要求继续收回旧全局变量，作为下一批实施，不混入本批行为差分。

## 基线与不变行为

基线 PR：25 组 native、19 项 integration 通过；unit 42/43，唯一失败为历史报告断链。
format 17、lint 601、接口 34 项未豁免问题及 10 条文档断链仍阻止完整门禁通过。
基线来源见 `outputs/runs/refactor-readiness-20260928/summary.json`（system-refactor 工作树）。

合并证据的 386 份源码及 14 个产物哈希已核对，全部匹配。LoaderApp 升级包 90,008 B，
APP 槽 94,208 B；静态 RAM 31,616 B / 32,736 B。Loader 与其布局不在本批改动范围。
本地基线源码保存在 `outputs/execution-boundary-20260929/baseline/`。

保持：schema11、升级 wire v2、Flash/RAM 布局、控制数学、保护阈值、20 kHz/2 kHz
频率和监督顺序。故障先于功能执行；可执行维护先于控制；仅 STARTING/RUNNING
执行控制；其他无故障状态预载 0.5 三相占空比且不启相。任务身份在执行前捕获，
结果入箱及配置 revision 推进仍在原位置。STOP/FAULT、旧结果拒绝、真实释放与
保存交接、CLEAR 不启动、Type56 精确发送完成复核均保持。

## 子任务与验收

1. 迁移真实调用，删除旧 FastStep 和执行模块的状态机/维护依赖，不留兼容包装。
2. 删除无效动作参数；实际动作位、临界区及转移次序不变。
3. 为真实调度体建立离线特征矩阵，对照基线，涵盖各生命周期、模式、维护、故障及
   worker 终态；比较调用轨迹、PWM 预载与返回事实。
4. 更新所有权文档，格式化仅本批文件；PR 检查并与基线失败逐项比较。
5. 普通 APP 和 LoaderApp 重新构建，核对 map、RAM、升级包及哈希；不使用扩大分区
   的诊断链接。独立 Loader 输入未变时复用匹配哈希的基线产物。

不烧录、不复位、不写设备参数或命令电机。native 验证不声称实机周期、最坏栈或掉电通过。

## 回退与交付

在隔离分支按代码批次提交，保留基线和所有忽略数据。通过审阅后的 commit revert
回退，不 reset/clean 用户工作区。交付删除/替代清单、差分结果、全局门禁失败和
新构建资源；若行为或容量不符合验收，则修复或撤回候选，不能扩大豁免。

## 实际结果

- 删除 `MotorExecution_FastStep`，由组合层私有 `run_fast_task` 选择维护、控制或
  空闲预载；控制入口 `MotorExecution_ControlStep` 删除两个无用对象参数和三条上层依赖。
- 删除 `RunState_ApplyActions` 被忽略的参数，以及 Send/RequestStop 沿调用链传递的空参数。
  动作位执行、临界区、STOP/故障优先和延后启相语义不变。
- 诊断开/关各 19,440 组调度特征，基线与候选轨迹及结果完全一致；新增检查加入 native 入口。
- PR：26 组 native、19 项 integration PASS；unit 42/43。format17、lint601、接口34、
  docs10 与基线一致，本批文件没有新增失败；architecture5known/0new、布局与hygiene PASS。
- 两种 APP 全量构建 0 Error / 0 Warning。普通 ROM 88,284 B（−32 B），RAM 31,568 B；
  LoaderApp ROM 89,940 B（−24 B），RAM 31,616 B，均未增加。
- 当前 LoaderApp 包 89,984 B，槽余 4,224 B；真实包经 Loader runtime 原生模型检查通过。
  该模型不证明实机交接、物理输出或掉电验证；开发版本0不标为正式发布。
- 证据：`outputs/runs/execution-boundary-candidate-20260929/summary.json` 与
  `outputs/execution-boundary-20260929/`。没有硬件动作，未修改 Flash 布局或参数ABI。
