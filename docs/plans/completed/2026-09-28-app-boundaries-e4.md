> 历史阶段记录，2026-09-29 归档。下文是当时的设计、状态和证据，不代表当前待办或当前版本通过。现状见[框架收尾](2026-09-29-framework-closure-v1.md)与[历史清理](../../archive/plans/2026-09-29-legacy-cleanup-v1.md)。

# APP 业务边界接线 E4 v1

日期：2026-09-28。状态：APP接线代码完成，工程集成与统一验证待ROOT收口。基线：
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

升级交接服务使用D唯一BootEntryService。可信本次boot_epoch作为本boot单事务u64票据；
1000ms总截止，确认精确物理回复后二次检查所有者停机/保存空闲，再由BootHw_EnterLoader复核并复位。
失败/超时撤销回复权限并保持停机独占，不重启或释放旧目标。主循环每100ms比较真实监督新进展，
结合感测/端点初始化和CAN实际HAL/CCCR/PSR健康，才喂已确认APP的继承看门狗。
试启动确认只在READY、编码器在线、无故障/维护/借用、输出关闭、保存空闲时发生；
完整标定位仍由各运动模式准入检查。确认页失败只尝试一次，保持停机等待Loader救援。
未确认trial在所有者初始化时进入启动/维护/配置禁止；确认前只观察基础自检与编码器在线。
持久确认成功后再核对本次boot_epoch/image_token与平台confirmed事实，由owner解除禁止，
不自动启动。不能靠Watch/CAN在试启动窗口先标定或运动，也不能靠虚构确认继续喂狗。
静态路径审阅补齐FAILED维护资源释放：失败终态仍需归还确认，核心保留FAILED并清操作占用，
避免SAVE失败后operation永久占用导致明确CLEAR和升级停机无法收口。该路径尚未测试。
Type3稳定值快照从BootHwInfo一次性启动观察取完整u64身份；角色固定为真实APP角色1，
无交接为state0/epoch0/无有效位，未确认trial为state2，正常/已确认为state1。
快照更新与H前台Service串行，不从GET_INFO.u32、UID、主机输入或旧record补造启动代号。
启动顺序已采用H的Init/Start拆分：软件Init、全部owner与快照/Boot绑定，再Start硬件RX；
组合层保留真实communication_ready结果，失败不启动RX、不允许trial确认，不把link初始化当链路健康。

## E4代码与工程交付边界

首批APP生产提交`d505acdc508f28a0bf372bb4200bdfd5893d55fa`已包含参数/标定接线、
真实UID和升级停机所有者门禁。后续Boot交接批次仅提交APP/services/不可用平台及USER CODE，
所需H/D固定依赖已在隔离树读取接入；ROOT已有相同依赖，不能把E父提交当重复集成清单。

工程输入由ROOT统一添加，不在本批制造另一套链接布局：

| 目标 | 实际源与输入 |
| --- | --- |
| 普通独立APP | 添加`firmware/platform/stm32g4/ports/boot/boot_hw_unavailable.c`；不定义`YG_APP_LOADER_IMAGE`；输出不可用身份，不触邮箱/IWDG/镜像记录 |
| Loader感知APP | 添加`firmware/services/boot_handoff/boot_handoff.c`、D的`boot_hw_stm32g4.c`、`shared/boot/image_mailbox.c`、`shared/image/{image_transfer,nv_record,nv_sha256}.c`、`shared/targets/vector_mini_st_g431_app_descriptor.c`；使用D的app.sct与保留32B邮箱的RAM布局 |
| 两目标公共 | `yg_protocol_boot.c`及既有参数/job/readonly源码；包含`shared/boot`、`shared/image`、`shared/protocol`；Loader目标另包含`shared/targets`，交接目标包含`services/boot_handoff` |

Loader感知目标必须显式定义`YG_APP_LOADER_IMAGE=1`、`YG_APP_IMAGE_VERSION`、
`YG_APP_IMAGE_ROLE`；版本/角色来自构建输入，不伪装未知版本为已发布版本。
只在composition的board_config选择产品能力，算法不散布硬件宏；普通APP不链接完整启动镜像库。
活动revision由MotorControl唯一持有并保留ISR共享的volatile访问；耗尽后WRITE和会修改配置的
维护都拒绝，读取及保存当前版本仍允许，不以饱和值冒充新版本。
两份BootHw实现互斥，不能同时登记；普通APP保持默认未安装Loader事实。
实际维护镜像role2由owner限制为维护流程，明确拒绝普通控制启动/目标；确认后仍不解除该角色限制。

`BootHw_Init`位于main最早USER CODE 1；其后Board_Init先装载APP状态与安全板级采样，
软件协议Init并绑定所有服务/值快照，最后Start RX。主循环依次维护/刷新/协议Service/Board_ServiceBoot。
Boot服务每100ms看到真实快慢监督新进展且控制器健康才确认或喂已确认看门狗；
不要求主机往返，不宣称已经验证端到端通信。ENTER只接受本次真实启动身份，精确TX失败/超时保持停机。

源码完成不等于可安装：历史APP ROM超候选78KiB槽、RAM余量仅历史160B，
Boot/维护缓存和新链接输入尚未用当前map确认。ROOT须给出实际Loader感知工程并保留容量冲突，
统一验证阶段再运行PR/release和实机场景；本批无测试、编译、固件/参数写入或复位结果。

交付记录完整SHA、API/真实调用、删除路径、未决业务约束和未测项。E3的最终normal
因用户新要求未启动；E3后续volatile保留提交亦未经重新验证。E4同样未测试/构建。
按提交逆序撤回E4可回到E3，不reset/clean用户现场或ROOT Keil工作树。

用户后续要求按建议继续收口；保存取消缓存与真实故障归还的调整、资源占用清单
及待统一验收项见 [E5 收口计划](2026-09-28-app-closeout-e5.md)。该增量仍未测试或构建。
