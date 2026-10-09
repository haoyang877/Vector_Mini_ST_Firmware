# YG_MC_HOST 真实线路只读边界 v1

日期：2026-10-08；独立分支 `codex/host-lifecycle`；基线
`7f44bd083902081b00d1761648c158c34350d098`。承接整机生命周期 v1 与
电机业务契约 v1.3；这两份文档目前在
`codex/motor-lifecycle-flow` 候选树，待 ROOT 集成，不在本批复制或修改共享 ABI。

## 目标和边界

保留现有 Qt 七工作区、演示会话和三种升级计划；给真实 CAN FD 读回预留独立的
服务所有者。仅消费按当前统一帧、CAN ID 和 Type 1（页 0..2）/2/108/110 契约通过的完整回复，
呈现身份、能力、八态状态、故障、位置 rad、速度 rad/s、Iq A 及瞬时 STOP 确认。
所有字段必须保留来源和过期状态；即便回读帧合法，未打开实际端口时也不能称为
实机连接、持续关相或完成运动。服务不构造连接、设备句柄、参数写入或功率动作。

ROOT/H 尚未冻结新启动、模式切换、租约及整计划的 CAN 线路。本批只在页面说明
六类业务视图与旧八态不同，并列出整计划父/子 ticket、完成位及 Flash COMMITTED
须分别读回的要求；不猜字节布局，不注册这些功率或维护操作。现有演示的 CURRENT
继续用 A。电感真实算法缺失，因此页面选项禁用；磁链、Nm 换算和型号自动映射
明确标为不可用，不借演示任务伪造结果。

## 验收与回退

- 共享纯 C 统一帧编码器生成的回复进入主机服务；错节点、序号、DLC、补零、CRC、
  状态值或测量有效位均拒绝，拒绝不覆盖已确认快照。请求和视图有明确单调期限。
- GET_INFO 页 0/1/2 与 GET_CAPS 不合并成无来源的身份；参数 schema 只取页 2，
  字段过期消失。
  Type108 是按需真实回复，不误认为周期流或新六类状态。
- Type110 result=1 只显示受理；result=0 的关相确认仅是最近瞬时证据，
  不能延伸为转子静止或一直安全。Qt 在无真实端口时不开放停止、使能、运动或写参数。
- 页面测试以离线黄金帧注入，确认线路读回标签、SI 单位、错误隔离与功率按钮关闭；
  既有工作台和升级离线回归仍通过。运行 host CTest、格式、lint、根 PR profile。

回退只撤销本批 `host_app` 与专属计划，不影响固件、共享协议、Loader、历史
`dist_v2` 或其他任务的工作树。当前没有物理 SDK 连接、探针或 CAN 测试；
ROOT 后续接入真实端口时，应先证明 EFF/FD/BRS/错误标志、独占、读回来源及断线语义，
再考虑任何写操作。

## 本批结果

主机 Qt/MinGW 构建成功，host CTest **2/2 PASS**，当前 JUnit 位于
`outputs/host_app/build/host-lifecycle-junit.xml`。仓库 PR profile **PASS**，记录在
`outputs/runs/host-readback-accepted-20261008/summary.json`；该验证包含本批未提交的
源码快照，未接触硬件。Windows 平台预览模式退出码 0，八个页面截图位于
`outputs/host_app/preview-lifecycle/`。计划追加后单独复核文档链接。

当前 `CAN FD` 下拉项仍不打开端口，也不自动发送 Type 1/2/108；旧演示样本不能注入
真实读回服务。`Window_BeginDeviceRead` 和 `Window_ObserveDeviceFrame` 是未来端口接线
边界，端口还必须检查物理帧 EFF/FD/BRS 和错误标志。STOP 的真实发送没有 GUI 入口；
Type 110 回读服务只区分回复语义，不代替 STOP 的现场安全确认。H 的新业务线路仅为
提案，未获 ROOT 登记，因此本批没有启动、切换、租约或整计划命令。
