> 历史阶段记录，2026-09-29 归档。下文是当时的设计、状态和证据，不代表当前待办或当前版本通过。现状见[框架收尾](2026-09-29-framework-closure-v1.md)与[历史清理](../active/2026-09-29-legacy-cleanup-v1.md)。

# APP 收口与资源约束 E5 v1

日期：2026-09-28。状态：源码调整完成，ROOT 集成与统一测试待收口。
基线：`9581374fdc39909f05ea10cc5920637b4e70adaf`。

用户要求按下一步建议执行，当前批次先完成工程接入、容量约束和业务边界收口。
沿用 E/H/D/ROOT 文件归属；E 只在本隔离分支修改 APP、服务及本批说明，
ROOT 继续拥有产品工程输入与最终交付。现场与其他工作树不改动。

## 目标、基线与不变项

解除保存取消结果对主机继续轮询旧任务的依赖；让纯生命周期核心在真实故障
快照下处理资源归还，不由 APP 适配器把活动故障改成零。

不改变控制数学、模式编号、保护阈值、20 kHz/2 kHz 调度、schema 11、16 KiB
原始参数区或线上布局。STOP 仍关闭输出和撤销目标；CLEAR 不启动；
FAILED 与 CANCELLED 仍是不同终态。取消不能承诺撤回已经发生的 Flash 写入。

旧路径有两处依赖：

1. `save_job.result` 的取消收敛只有 PollSave/SaveIdle 触发。普通 APP 不运行
   Boot 服务；主机 STOP 后放弃旧 SAVE 轮询，新 WRITE/SAVE 可能一直被旧
   ACCEPTED 缓存挡住，尽管运行所有者已经确认取消和归还。
2. `RunState_SendOperation` 为 FAULT 的 CANCEL_DONE 把 `guards.active_faults`
   临时清零，以绕过核心的故障优先分支。核心看不到真实活动故障，本批移除该特例。

## 子任务与实际调用

| 子任务 | 修改范围 | 实际行为 |
| --- | --- | --- |
| E5-1 保存终态收敛 | `app/motor_state.c/.h` | 文件私有 `refresh_save_result` 查询原 ticket；WRITE、新 SAVE、PollSave、SaveIdle 在临界区内调用 |
| E5-2 真实故障下归还 | `services/lifecycle/app_lifecycle.c/.h`、`app/foc_run_state.c` | 已处 FAULT 且本拍仅 CANCEL_DONE 时先累计全部故障，再核对取消代号、操作、效果和静默资源；保持 FAULT，不清锁存，不受理新请求 |
| E5-3 资源输入清单 | 本计划 | 区分源码可知的固定占用、历史链接值和未测候选；不凭估算调整已用分区或删除功能 |
| ROOT 工程接入 | ROOT 集成分支 | 普通 APP 仅不可用 Boot 端口；Loader 配套 APP 使用真实端口、交接服务、描述符、共享镜像代码和专用 scatter |

保存仍在 CANCEL_REQUESTED 时保持 ACCEPTED；新的修改仍由运行所有者的
READY、关相、无操作和资源释放守卫决定。重复原请求沿用原 ticket/内容校验，
新请求不会重用旧 SAVE 的身份。成功仍只能由前台真实写入、回读后的结果发布。
新增 helper 无持久可变状态，不增加完整参数副本，不分配、不写 Flash、不做 CRC。

FAULT 归还特例只开放单独的 CANCEL_DONE，FATAL、首次进入 FAULT、混合事件、
其他完成或新命令仍走既有故障优先处理。确认归还后 operation 为 NONE，
FAILED 终态继续为 FAILED；取消终态由 CANCEL_REQUESTED 变为 CANCELLED。
资源未静默、旧代号或错误操作均不能释放占用。此处理不表示故障可以清除。

## 容量清单与决策

当前目标输入为 `shared/targets/vector_mini_st_g431.json`。Flash 128 KiB、
候选 Loader 28 KiB、启动记录 4 KiB、镜像记录 2 KiB、原始参数区 16 KiB，
剩余 APP 78 KiB；RAM 顶部 32 B 为不初始化的交接邮箱。

历史 APP ROM 98,980 B、RAM 32,608 B 属于 a174 基线，不是 E5 产物。
历史 ROM 比候选 APP 槽多 19,108 B；历史 RAM 余量仅 160 B，扣除邮箱后
仅 128 B。新代码尚未链接，不能据此宣称已经溢出多少或优化后能够安装。

源码可直接确认的大块占用：

| 资源 | 来源 | 约束 |
| --- | --- | --- |
| 固定软件堆 7,168 B | `common/heap.h/.c` | 编码器工作区、参数记录、齿槽候选借用；不能在没有证明全部峰值和释放顺序时缩小 |
| 编码器标定工作区 6,144 B | `motor/identification/foc_encoder_calibration.c` | 1,024 个 i32 累积值与复用的 1,024 个 u16/i16，动态占用上述堆，不另加到常驻堆预算 |
| 编码器运行 LUT 2,048 B | `motor/position/angle_feedback.h` | 现有 1,024 点标定能力，不能靠缩点数压缩 |
| 齿槽发布表 2,064 B | `motor/identification/cogging_map.h` | 包含 16 B 头和 1,024 个 i16；保存候选从堆借用，不另建静态副本 |
| RTT 通道 1 缓冲 2,048 B | 既有 SEGGER 文件中的 `bRttBuf` | 不改第三方源；如后续调整，应由产品配置明确采样与丢帧影响 |
| CAN RX/TX 队列 8/40 帧、分片区 1,767 B、控制历史 256 项 | `communication/can/yg_protocol_link.c` | 归 H；队列背压和重放契约必须先明确，不能只为减 RAM 静默缩容量 |

工程源选择先去掉普通 APP 不使用的真实 Boot/镜像库；Loader 配套 APP
保持真实校验/交接链。以上清单不替代 map，结构对齐、常量位置、栈峰值和
实际被保留的函数仍由统一构建确认。当前不改硬件容量、不删电机功能、不缩参数区。

## 后续统一验收与回退

保存缓存修复独立提交为 `5d2b3deb0f3c8d989d13937e6851643cdabd88d2`。
故障真实快照归还另行提交，只更改核心与适配边界，没有增加命令或硬件动作。
ROOT 工程入口已登记在其集成提交 `8a813808`；本分支不重复复制工程生成器。

本阶段不运行测试、检查或编译，不连接硬件。源码审阅记录不能标为 PASS。
统一测试阶段补齐以下用例，当前未执行：

- SAVE 在前台认领前 STOP，确认取消后直接新 WRITE/SAVE，不轮询旧任务。
- SAVE 在前台借用中取消；CANCEL_REQUESTED 期间新修改仍拒绝；真实归还后可重新请求。
- 原 SAVE 同身份重试、不同内容冲突、陈旧序号、错误 ticket 与 revision 冲突。
- 写入/回读失败不能返回 OK；已经提交后 STOP 不能把持久成功说成撤销。
- FAULT 中单独归还：真实故障不清零，新故障累积；FAILED/CANCELLED 各自保持。
- 错误代号、操作、效果、资源未释放、混合请求以及 FATAL 均不通过归还门禁。
- 后续 PR/release 检查、普通 APP/Loader 配套 APP/Loader 独立构建、map 与产物哈希；
  必须另有明确实机范围后才能进行硬件交接与失败场景验证。

按本批提交回退可恢复 E4 源码；不 reset/clean 现场工程，不改变其他工作树引用。
回退不代表撤回设备上已经发生的写入；本批没有进行任何设备写入。
