# APP 业务边界接线 E4 v1

日期：2026-09-28。状态：实施中。基线：
`22fc4ab8d47db6d8ef66c7e5f0e46ecc2ea5ae80`，分支 `codex/internal-decoupling-e3`。
用户将终点扩展为通信、主状态机和 Loader 代码完成，并明确后期统一测试。
从此不新增测试、不运行回归或构建，不接触硬件；既有结果不能充当 E4 验收。

## 目标与不变项

把 Watch 独有的受理流程提炼为 CAN/Watch 共用的明确业务 API；状态与功率提交仍归
唯一 APP 所有者。增加单值参数读取、停机原子写入与异步持久保存；不导出 schema
对象或电机地址。升级交接从请求、目标撤销、真实关相、资源归还、ACK 确切发送完成，
再写 D 提供的共享值与受控复位。normal 独立 APP 保留，Loader 使用独立链接目标。

保持控制数学、Mode13/15 标定、schema11、原有保护阈值、20kHz/2kHz 调度；
101 未定的低侧制动、mode4/5 算法不擅自开放。H 拥有 communication，D 拥有
loader/shared，E 拥有 APP/services 与必要平台端口。ROOT 负责转发约定与集成。

## 接口收口

- `FocRunState_RequestEnable(MotorControlMode)`：启动请求，零目标/当前位置起步，
  只在 READY 与真实守卫满足时受理；受理不表示已输出。
- `FocRunState_RequestMaintenance(MotorMaintenanceKind, uint32_t *ticket)`：
  显式原有维护任务，返回创建时身份；不接收线上 mode 编号。
- `FocRunState_RequestIdentifiedMaintenance(MotorMaintenanceRequest *, uint32_t *ticket)`：
  来源、16位序号、任务值幂等受理，复用上述准入；同身份返回原ticket，冲突或陈旧序号拒绝。
  同来源按半范围窗口递进；只缓存最近成功受理，初始化清空，不声明跨复位去重。
- `FocRunState_CancelMaintenance(ticket)`：临界区核对原身份，取消受理不表示已归还；
  最近终态重复取消无副作用，迟到ticket绝不取消其他任务。派生SAVE不覆盖原请求终态。
- `FocRunState_RequestClear()`：同步清当前可恢复锁存源，不恢复目标。
  ROOT 冻结 107 为空 payload、4B result/detail，编码归 H。
  成功可返回wire OK，但故障态清除后进入SELF_TEST，不表示READY或已允许启动。
- `FocRunState_ReadMaintenance(ticket, MotorMaintenanceProgress *)`：按原始身份
  读取结果/持久效果；陈旧身份拒绝，不用当前 epoch 重建 ticket。
- `MotorControl_ReadParameter(ParameterReadRequest *, ParameterValue *)`：一次临界区读取单值、类型、
  权限与活动 revision，不分配完整参数记录。
- `MotorControl_WriteParameter(ParameterWriteRequest *, ParameterValue *)`：
  READY 且无资源借用，核对 revision、类型、权限、范围后原子应用；不自动保存。
- `MotorControl_RequestSave(ParameterSaveRequest *, uint32_t *ticket)` 和
  `MotorControl_PollSave(ticket, uint32_t *persisted_revision)`：持久保存排入原
  维护流程，Flash 回读校验与原身份提交前不得返回成功。
- `FocRunState_RequestBootStop(uint64_t)` 与 `FocRunState_BootHandoffReady(uint64_t)`：
  升级停机独占票据，同票据幂等；目标撤销走既有STOP，等待数字关相、维护与借用归还。
  失败后保留禁止启动、维护、配置写入；不伪造故障，不自动恢复运动。
  `MotorControl_SaveIdle()` 收敛原ticket取消终态，另行确认保存业务已空闲。

单值目录沿用参数目录 v0.1。未定义产品枚举、制造来源或持久字段的项明确不支持；
校准位与校准结果只读。耦合的电机模型改写不能静默沿用旧 PI；没有完整事务与产品
边界时只读。已明确硬件/轴限制的限流、限速与现有位置增益边界优先开放停机写。
不以 i32 最大值替代产品上限。

## 接线、风险与交付

H 用注入服务绑定上述值接口，不读取 Watch 邮箱或 ModeNow；H 冻结120/121/122
及107的 wire，SAVE 回复必须等待持久作业。D 提供最终共享交接契约、原始参数区
保护与独立 Loader 布局；历史128KiB/78KiB槽与 APP 历史98980B存在真实冲突，
E 不缩参数或伪造可安装容量，不写死旧APP入口。

160/161/162已接入专用job结果，kind1/2/3显式映射LUT/电角零位/齿槽；
START同来源序号内容重放原ticket，冲突/陈旧分别映射detail4/5；QUERY成功不等于标定成功。
CANCEL在同临界区核对种类和原ticket，不能借标定命令取消参数SAVE或其他维护；
取消受理仍需真实释放后的CANCELLED。终态保留到下一成功的带来源START或初始化。
STOP/CANCEL先锁存当前同代worker故障再撤销结果，避免停机递增代号后丢弃故障源。

新增可变状态只放实际所有者；无大副本，认领资源必须归还。RAM历史基线32608B，
新增静态量和D保留邮箱将由后期统一链接核对；未链接前不声明资源满足。
目录变化与源码输入同步进入normal工程，Loader目标由最终契约另行接入。

H共享纯codec、设备描述注入和D邮箱/56值契约已按提交接入；ROOT工程路径补丁保留。
组合层通过平台读取真实96位芯片UID；无权威来源的产品、板卡和发布版本显式为0未知。
能力仅报告已实现的CAN FD/分片/固定单位反馈，远程启动模式与看门狗不谎报。

交付记录完整SHA、API/真实调用、删除路径、未决业务约束和未测项。E3的最终normal
因用户新要求未启动；E3后续volatile保留提交亦未经重新验证。E4同样未测试/构建。
按提交逆序撤回E4可回到E3，不reset/clean用户现场或ROOT Keil工作树。
