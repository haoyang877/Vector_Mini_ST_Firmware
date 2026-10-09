# Loader/APP CAN FD 联合升级台架记录（2026-09-30）

## 结论

在无电机动力、有人在场可断电的条件下，冻结的旧配套包 `d33732809bdd3f9add9a1ae7cc897ade4f01da09` 完成了 Loader 安装、CAN FD 下载、镜像校验、激活、新 APP 身份核对及健康确认。主机升级报告 `success=true`，镜像为 93,608 B、版本 1.0.0、CRC32 `19a45953`。未发送电机使能或运动命令。

测试结束后使用测试前备份恢复全片 Flash；恢复后及一次受控复位后的 128 KiB SHA-256 均为 `6e0e33b61ac19bac214b7ea3864c9045cd566bd221a83aedff9d686d2564a1d1`，与测试前相同。参数区 SHA-256 为 `9c5a6f02da39618b8f1b30d0e6cce0dbc8dcec99047222cdf0db1ec675885ef0`。CPU 运行，三相使能掩码为零，J-Link 已释放。

本次不宣称当前主线配套包或发布验收通过：当前主线 Loader 可构建，但配套 APP 链接超过 92 KiB 槽 1,780 B。恢复后只读 CAN 查询未得到回复；随后适配器打开失败，且其他实机会话已活跃，因此没有取得可靠的恢复后 CAN 功能结论。没有进行真实掉电、故障 APP 或电机运行测试。

## 证据与边界

- 准入：用户确认电机功率隔离、设备停机、MCU/CAN 供电、现场可断电；测试前完整 Flash 与参数读回，UID `5700460010504e4831303320`、CAN ID 1。`outputs/loader-app-bench-20260930-preflight/backup-20260930T064056972376Z/summary.json`。
- 旧包来源：`outputs/loader-v3-candidate/20260929-d337328/summary.json`（独立工作树）；13 件产物哈希均核对，Loader HEX SHA-256 `948d2023addcb0cf3364d6bbb4d00c8218f943e59aa28957e5daabd3326b2888`。
- 安装：Loader 限定 20 KiB 分区，分区外读回与备份一致，参数不变，复位后输出关闭。`outputs/loader-app-bench-20260930-preflight/install-20260930T064528059040Z/summary.json`。
- CAN 发现：首次 Type53 超时，未擦 APP；J-Link 只读诊断见 Loader 与 FDCAN 已运行、无总线错误。再次查询成功，确认 UID、Loader 角色、目标布局。`outputs/loader-app-bench-20260930-preflight/can-discovery-20260930T065200699265Z/summary.json`。首次超时原因未定。
- 完整升级：`outputs/loader-app-bench-20260930-preflight/upgrade-old-pair-20260930/summary.json`，834 条升级往返记录，含 BEGIN、WRITE、FINISH、STATUS、ACTIVATE、APP CONFIRM。包镜像 SHA-256 `591d0c273eab2d80e7d68718bf3ebbd1cfc26e814e74cd6ee59e3d84e1a7e98c`。
- 恢复：`outputs/loader-app-bench-20260930-preflight/restore-20260930T065251272584Z/summary.json` 和 `outputs/loader-app-bench-20260930-preflight/reset-check-20260930T065902925123Z/summary.json`。恢复后无动力输出核对通过；CAN 功能核对未完成，不将其等同于 APP 功能复测。

## 后续

先收敛主线配套 APP 的 1,780 B 超限并重新生成同源 Loader/APP 包。下一次占用独占 CAN 台架时，补查首次发现超时及恢复后 CAN 通信；独立安排断点恢复与故障 APP 救援验证。任一 STOP 或功率输出关闭无法核实时，立即停测并交由现场物理处理。
