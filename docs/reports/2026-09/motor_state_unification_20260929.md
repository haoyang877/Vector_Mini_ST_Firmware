# 电机主状态机模块收敛验收

日期：2026-09-29。功能基线 `983abeaf`，最终代码提交 `be4b63f`。
本轮全部离线；未连接探针、未烧录、未写设备参数、未发送电机命令。

## 最终结构

`firmware/app/motor_state_machine/` 集中所有者、公共入口、内部转换规则和私有头。
删除 services 的 app_lifecycle.c/.h，无旧头兼容、模式转换或第二条运行路径。
跨层只读状态值在 motor/motor_state_types.h；控制/维护服务不包含私有规则头。
原生测试可以引用私有头作为接缝，生产只有一个规则调用方和一个可变上下文。

普通、维护和 CLEAR 收口到唯一 state_commit_event。删除重复模式、分类状态、
未使用的读取 API、返回快照和 last_operation；事件元数据只在提交入口构造一次。
保留原请求时机、自检期限、故障/取消优先级、最终启相复核、保存提交与资源归还语义。
最近具体维护身份仍用于既有故障定向清理，活动占用由独立有效标志表达。

## 行为证据

- 迁移前后 262144 组独立规则输入的状态、代际、操作结果、效果、故障锁存、
  自检标志、动作和拒绝结果摘要相同：f98662bc6ed8b902。
- 已删除的规则侧模式镜像与 last_operation 无生产读取者，不作为保留的对外契约。
- 同一真实所有者夹具同时编译结构迁移前的 533edca 和最终所有者；断言及 stdout 相同。
  533edca 的规则行为又经上述摘要与 983abeaf 原实现对齐。
- 回归包含配置/标定不足的 READY、连续自检/超时/计时回绕、旧结果、代际耗尽、
  STOP 与失败优先、目标撤销、原始任务与派生 SAVE 身份、取消归还和 COMMITTED。
- 增加“已完成任务不占用配置资格，但后续故障仍按最近种类清理”的边界回归。
- 架构债务保持零；通信与参数黄金向量通过；目标布局和共享镜像契约未改。
- 最终 release 所含 format、lint、Python/原生测试、架构、接口、文档、布局、
  hygiene、Keil 与 manifest 全部 PASS，source 干净，产物有当前哈希。
- 普通 APP 与 LoaderApp 均为 0 Error、0 Warning；实际打包镜像通过真实 Loader
  内存模型的冷启动、ENTER、看门狗救援、损坏镜像和 APP 健康/Flash 等待验证。

## 容量

静态 ROM 为 Code + RO + RW；实际链接/升级镜像还受 RW 压缩与对齐影响，不能混用。

| 目标 | 静态 ROM 基线 → 当前 | RAM 基线 → 当前 |
| --- | --- | --- |
| YG_ESC_APP | 90772 → 90652 B（-120 B） | 31592 → 31600 B（+8 B） |
| LoaderApp | 92400 → 92272 B（-128 B） | 31632 → 31640 B（+8 B） |

LoaderApp 打包后 91976 B，94208 B 槽余 2232 B。私有上下文合并的对齐使 RAM 增加 8 B；
删除快照与包装减少重复开销。本轮主要改善模块归属和所有权，不宣称显著 Flash 节省。

HEX/AXF/MAP 哈希因模块路径、符号、布局及实际指令/数据变化而改变，不宣称逐字节等价。
参数 schema12、CAN 状态/模式与目标布局均保持；Loader 产品源码没有改动。

## 证据与限制

- `outputs/runs/state-unification-baseline-20260929/summary.json`：迁移前 release。
- `outputs/runs/state-unification-stage1-20260929/summary.json`：模块边界阶段 PR。
- `outputs/runs/state-unification-accepted-20260929/summary.json`：最终代码 release。
- `outputs/state-machine-unification/summary.json`：容量、规则、所有者和镜像哈希。
- `outputs/state-machine-unification/reference-owner/`：同夹具前后日志。
- `outputs/state-machine-unification/baseline/`：当前基线完整产物，非历史缓存。
- `outputs/state-machine-unification/real-image-runtime/test.log`：实际镜像内存模型。

20 kHz/2 kHz/前台执行边界未改；原生测试验证动作顺序与逻辑，不证明实机时延、
中断抖动、栈峰值或物理功率关断。试验包版本为 0、integrity_only、release_approved=false，
不作为正式发布或实机联调完成声明。失败的中间验证保留日志；集成测试扫描范围修正为
仓库自有业务源码，避免读取非 UTF-8 的 vendor 文件；最终完整验证覆盖修正结果。
