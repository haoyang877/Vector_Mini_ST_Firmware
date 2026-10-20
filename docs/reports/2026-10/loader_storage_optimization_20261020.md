# Loader 与参数存储优化记录

日期：2026-10-20。代码冻结：`b7067cd8641990605fd45d8fd001440969e5de79`。
配套候选：APP v1.5.0、Loader v2.1.0、schema14。参数保持单份。

## 结果

| 项目 | 优化前 | 本次候选 | 变化 |
| --- | ---: | ---: | --- |
| 参数记录 | 4,352 B | 2,288 B | 删除退役齿槽表，减少 2,064 B |
| 每次参数保存擦除 | 3 页 | 2 页 | 少擦除 1 页，分区地址不变 |
| Loader Flash | 15,108 B | 14,492 B | 减少 616 B，16 KiB 分区还余 1,892 B |
| Loader RAM | 4,952 B | 4,344 B | 减少 608 B，栈预算保持不变 |
| APP Flash | 80,360 B | 80,448 B | 增加 88 B，含旧参数兼容及本轮主线默认值收口 |
| APP RAM | 18,576 B | 18,576 B | 编码器采样仍决定共享维护区大小 |

Loader 使用 ARM MicroLIB，移除未用的完整 C 运行库开销。没有删除镜像 CRC、
Flash 读回、安全关相、升级超时或失败回退。APP/Loader/CAN 的物理布局未改变。

参数保留 schema12/13 的只读兼容，检查原始整条 CRC 后按旧偏移装载；
显式保存才使用新格式。schema10 通过离线工具迁移，必须提供快照哈希和已核对的旧布局。
工具保留有效标定，但清除配置确认；清标定工具仍明确清除零位、表和标定位。
详细规则见[schema14存储与迁移](../../protocols/parameter_schema14_storage_v1.md)。

## 验证证据

当前代码的完整 release 验证通过，包含全部单元/本地集成测试、架构/接口/文档检查及
Keil 构建，构建零错误、零警告。证据位于隔离工作树的：

- `outputs/runs/20261020T143611925830Z-b7067cd8/summary.json`
- `outputs/delivery/schema14-app150-clean-20261020/summary.json`

配套包由干净源码生成；清单逐项记录 AXF、HEX、BIN、MAP 与 CAN 包的 SHA256。
合并包名为 `YG2026-ESC-FULL-STM32G431CB-debug-LOADER-v2.1.0-APP-v1.5.0-20261020.hex`。
它仍是 debug 候选，`release_approved=false`。

额外对本板此前保存的 schema10 参数快照执行离线迁移，再用真实 APP 参数装载代码做
本地检查：模型、节点、方向、零位与 LUT 保留，配置确认清除，装载未回退默认值。
原始快照和输出、报告、C 检查记录在 `outputs/optimization/schema14-board-migration/`。
该迁移没有写回设备。

## 实机边界与升级耗时

使用原指定探针安装了新配套固件，Flash 读回匹配，APP 启动及关相检查通过。
随后 USB-CAN 适配器打开失败，尚未发送升级命令；本轮成功升级次数为零。
测试立即停止，并恢复原始固件、参数和身份，完整 Flash 两次读回一致。
恢复文件 SHA256 为 `1efdab53b6cf7587ed47babeb41353c3da5b416d421baa9c1e57ecb9cb70b1aa`。
断开探针前最后观察为 CPU 暂停、六路门极输入低；这不承诺断开后永久保持暂停。
证据：`outputs/bench/loader-schema14-smoke-20261020/summary.json` 与 `restore.json`。

客户端已经分别统计各命令的发送耗时、回复等待和重试，最终超时请求也计入。
时钟注入与丢包模型验证通过；实际新版本的 CAN 耗时仍待适配器恢复后测量。
此前 20 次成功属于 APP v1.4.2 / Loader v2.0.0，每轮平均 37.335 秒，包含大量逐帧 J-Link
检查，不能视为纯 CAN 速度，也不能用作新版本验收。

真实断电、故障 APP 救援、CAN 中断和运行栈峰值仍需分别做实机验证。
不因为本次存储优化而增加参数双备份或宣称这些试验已通过。
