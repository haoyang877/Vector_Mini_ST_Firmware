# 平台主／子流程交付 v7

日期：2026-10-08。基线 `7f44bd083902081b00d1761648c158c34350d098`，
独立分支 `codex/motor-lifecycle-flow`；ROOT 基线树保持干净且未修改。
源码范围、真实调用图、接口清单和未实现表见[平台流程](../../architecture/motor_platform_lifecycle_v1.md)。

## 已交付

在原八状态执行边界上增加六类业务视图，不缓存第二份主状态。
已有标定步骤按真实依赖串联，最终 SAVE 独立身份且回读成功才完成整计划；
失败／STOP 撤销后续，取消等待原后台认领归还。运行模式切换先关断，再确认两次
实际快环中性 PWM 预载及准备／许可／输出开启；关断失败 FATAL，不自动续试。
全模式带身份业务启动原子登记明确控制租约；过期经既有快速关断及故障锁存链处理，
恢复须原身份新鲜证明与明确 CLEAR，不重放旧目标。旧 SPEED 入口及单项维护保持。

schema12、45B 反馈、原状态值、控制数学、保护阈值、单份维护区和调试设置保持。
Loader 跳转、参数事务、身份装载、上电零点继续复用既有真实调用。
协议／GUI 尚未接新业务接口；电感／磁链自动辨识、Nm 力矩、身份到 profile 的自动选择
及相电阻诊断值到只读模型 R 的应用政策未实现，不宣称整个平台已实机闭环。

## 当前验证

- 完整 PR：`motor-platform-lifecycle-v7-accepted-20261008`，所有门禁 PASS。
- 所有者新增定向检查：依赖顺序、独立票据、保存确认／失败、原认领取消、故障抢占、
  模式准备／真实快环进展／STOP、四模式身份与序号屏障、租约回绕和恢复后无自动启动。
- 原规则 262144 摘要保持 `9dd9307dcc7bb53f`，72000 维护失败摘要保持 `afc991123a3208fd`。
  已有叶算法检查通过；未扩展五工程矩阵或重证控制数学。
- normal：当前源码编译 0 Error／0 Warning，仅构建，无硬件连接、下载、复位或运动。

PR 摘要在 `outputs/runs/motor-platform-lifecycle-v7-accepted-20261008/summary.json`，
normal 日志／map 在 `outputs/build/`；源码及产物哈希、base/head 和窄补丁在
`outputs/motor-lifecycle/v7/delivery.json` 及同目录证据中。
这不是新发布包或 LoaderApp 正式槽验收；原 v1.1.0 包未覆盖。

## 容量与接线限制

| normal 程序统计 | ROOT 基线 | 当前 | 差值 B |
| --- | ---: | ---: | ---: |
| Code | 89896 | 91064 | +1168 |
| RO-data | 3260 | 3300 | +40 |
| RW-data | 256 | 256 | 0 |
| ZI-data | 26640 | 26688 | +48 |
| ROM 总计 | 93412 | 94620 | +1208 |
| RAM 总计 | 26896 | 26944 | +48 |

当前未接线的新 RequestCalibration、IdentifiedRun、GetFlowState、ReadCalibration、
CancelCalibration 等入口被链接器剔除；H/F接线后的候选须重新核算，不能将此表当完整
新功能接入的最终容量。map 显示上述入口至少 700 B 被剔除。
LoaderApp 正式槽基线原已超 1620 B，本批不改变槽、不删模式，也未用 normal 差值
冒充新 LoaderApp 的精确超限值。该容量问题仍由 ROOT 发布集成阶段处理。

本批源码提交和图文可独立审阅，完整 base/head 写入交付元数据。
ROOT 审查后再接下一候选；不回灌隔离树到用户树，不代替 H/F 实现协议／GUI。
