# YG_MC_HOST GUI 实机验证 v1

验收结论（2026-11-10）：AT32F456 PITCH 上的连接、趋势、速度命令、STOP 和 CAN 升级通过；实测数据与未接入项目见文末。

历史基线（2026-10-21）：host_app/rust源码为v2.1.2，原计划未记录GUI实机终态。
下述APP1.5.1是最初冻结的历史候选，当前shared目标已声明APP1.5.7；
后续执行前须重新核对实际包、UID和基线，不能按旧版本声明当前固件已通过。

用户本轮要求实机测试。候选基于已验收固件源码 `328dda299df0faf0b16d4eb1e183391959178aee`，
APP 1.5.1 / Loader 2.1.0 / layout6 / schema14；复用 Rust v2.1.1 界面，
不修改固件、协议、参数或标定记录。版本适配仅通过当前共享目标契约。

## 准备与边界

CAN 与 J-Link 必须先由 T 明确交接，未交接前只构建和离线检查。
连接页显式选择 CAN 设备索引，以支持台架已有两个适配器，禁止自动逐个打开尝试。
开启 eframe 原生辅助功能接口，让真实控件支持键盘与辅助功能树定位。
连接后按实际 UID、节点、版本、schema/ABI 绑定单台设备；错误目标禁止运动。
序号起点来自交接记录，不猜测旧主机的历史。

## 本轮场景

1. 通过实际原生 GUI 连接、核对身份/零故障/静止状态。
2. GUI 开始观察，切页后记录保留，验证三图及 CSV 导出。
3. GUI STOP 获得最终 0/0、IDLE/近零速；停止异常视为硬失败，须现场介入。
4. 在既有架空台架、母线≤33V/3A、相电流≤6A和更低固件限制内，
   从零速启动，分别设置 1、2 rad/s，回零并 STOP；不扩大已有测试包络。
5. 结束后只读核对输出关断并关闭句柄。若只完成只读场景，应如实标为部分完成。

本轮不下载固件、不写参数、不重新标定，不用脚本动作代替 GUI 点击的验收。
界面操作记录、截图、CSV、版本/EXE 哈希及结果写入 outputs/host-live-test。
不把 T 的既有台架结果写成 GUI PASS；未执行项目明确标为 NOT_RUN。

## 离线与回退

Rust fmt/clippy/test、配套包离线 inspect、根 PR 检查；CAN索引接口有范围检查。
本轮不修改台架工具或固件发布输入。出现 GUI 缺陷先停机、断开再修复。
旧 v2.1.1 工作树与产物保留；回退只撤销新主机候选，不改板上固件。

## 2026-11-10 AT32 PITCH 实测进度

使用重构分支的 Rust 工作台 v2.1.2 和节点 6。当前配套固件为 APP v1.6.0、Loader v2.2.0、layout 7。调试和优化构建均成功。格式及 Clippy 检查、17 项 Rust 单测、7 项 Python 读回客户端测试和仓库 PR 回归通过。优化版本离线识别 77,240 字节升级包，CRC32 为 `8f918e7d`。16 个离线界面预览已生成。

真实 GUI 打开 USB-CAN 设备索引 0，发起 Type 1 身份查询后超时，显示硬停提示。独立客户端向同一节点查询也超时。J-Link 枚举成功，两次读取目标参考电压均为 0 V，SWD 报目标未供电。尚未取得此次在线身份、实时曲线或 STOP 回读，未发启动或速度命令。恢复目标供电与参考电压后继续。

用上次 PITCH 实机停机快照的有效位 58 离线回放启动条件。固件在 IDLE 时将实际 Iq 标为无效，主机将其解析为未知。即使给定匹配身份、速度能力和范围内的母线电流，`Snapshot::safe_idle` 仍返回 false。此条件同时用于 GUI 零速启动和升级前检查。联调时须核对停机测量契约及实际关相状态。

运行证据位于 `outputs/host-live-test/pitch-20261110`，包括 `test-final.log`、`reader-tests.log`、`release-inspect.log`、`preview/preview-summary.json` 和 `idle-replay.log`。本次 release EXE 的 SHA-256 为 `dcaf8d6323c076b635c1979ddac41c9deea30a8ff1318f7c2329378b61301190`。

## 恢复供电后的真实 GUI 结果（2026-11-10）

J-Link 读到 3.238 V，CAN 与 SWD 核对同一 PITCH 节点 6。原 GUI 成功读取身份、APP 1.6.0、Loader 2.2.0、schema 14、实时反馈，跨页观察及 CSV 导出正常。即使 STOP 返回 0/0、108 回读 IDLE，原版升级页仍显示等待停机。实机 108 有效位为 0x003a：电压、电源电流、位置和速度有效，空闲态 Iq 无效。

候选版只在同一会话已得到 STOP 0/0、IDLE 回读后允许空闲态缺少 Iq；零故障、新鲜反馈、母线和近零速度限制仍有效。运行期间失去 Iq 仍要求停机。使用实机帧的回归、17 项 Rust 测试、fmt、Clippy 和优化构建通过。实测 EXE 为 `pitch-verified-host.exe`，SHA-256 为 `ce751fbd5794383eb81c46c07bd4574a45fcdf17fc946bfdb7f0a41ed5eaf8d5`。

通过 GUI 零速启动，依次下发 1、2、0 rad/s，最后 STOP 得到 0/0、IDLE。GUI 记录 877 点、约 4.90 Hz，无间隔中断或容量淘汰。每档去掉首秒，1 和 2 rad/s 的速度反馈均值为 0.997 和 2.003 rad/s，位置变化除以时间为 0.998 和 1.996 rad/s。瞬时反馈范围分别为 0.181–2.429 和 0.783–3.213 rad/s，均值跟踪不能替代低速平稳性验收。回零段均值 0.001 rad/s。

按追加的固件升级测试范围，GUI 校验 77,240 B 的 APP 包，执行一次同版本 CAN 下载、激活与持久确认。独立 CAN 复核 CRC32 8f918e7d、IDLE、零故障及 0.000 rad/s；J-Link 双读六路输出关闭、APP 槽外 163,840 B 完全相同。GUI 已正常断开。角度标定页的任务和位置模式控制尚未接入，此处不作 GUI 验收。

原始数据：`outputs/host-captures/capture-1794281855540.csv`、`outputs/host-rust-upgrade-1794281922521/summary.json`；同批 `outputs/host-live-test/pitch-20261110` 下有 `live-speed-stats.json`、`live-postcheck.json` 及升级前后 Flash 快照。
