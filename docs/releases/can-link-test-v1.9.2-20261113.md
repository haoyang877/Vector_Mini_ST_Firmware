# CAN-Link-Test v1.9.2 标定曲线与原始数据包

2026-11-13，分支`the-system-refactor`，冻结源码`9713d1eb0141cca81d63e0b9e33227510ddbc243`。
工具v1.9.2，配套固件仍为APP v1.8.3 / Loader v2.3.4；本次不改固件或参数ABI。

新增电机/Hall补偿图、电零位、Hall对齐角、两路极值、Hall原始时域与XY图。
原始RTT、逐帧CSV、LUT Q15、完整参数BIN、配置JSON及校验清单一起保存。
样机序列号和测试序列号独立，按样机/测试/时间归档。

`outputs/deliveries/CAN-Link-Test-v1.9.2-win64-20261113.zip`：44,172,406字节，13文件。

- 包SHA-256：`8b32d373e96b7ca5f9d92d2d2936db04319d01997219b7878567a8f6a25cf111`。
- EXE SHA-256：`2e66bbb524cbe171684e19e8ab0241160da0fe39725009d152e7c772a9879325`。
- FULL HEX：`578caceb0d7b68e6449c660f4209380eabd093d0b2dfd58d1d2d44b32b73c602`。
- CAN image：`3c2e99705aefc81dc9510c45befbe550a2b4871e9179fc4bc1ec59c9b509f4a5`。
- CAN manifest：`a7d22d2a3a6615818500601e00a9c63a4656efab5e619d92a85dd36dee357069`。
- 单独示例包`outputs/deliveries/P轴电机-DEMO0001-测试0004-20261113.zip`：`4b2144b8782afc3f6291690e3d4405e2bd250ec137c8d4e484a5ff1b39efa669`。

Release PASS：`outputs/runs/canfd-curves-v1.9.2-final-20261113/summary.json`，源码干净。
ZIP CRC及逐文件SHA-256复核通过。节点6两轮原码采集及保存/关相通过，最终63,394点；
原生Tk自动创建图窗与四个数据按钮通过。最终EXE的外部自动点选受窗口绑定错误限制，
不宣称完整桌面端到端验收。其余四类电机本轮未实测，原Hall个别角区问题保留，
固件发布类别仍为debug。RTT不带帧序号或设备时间，不能用本记录宣称无丢帧。

详见[实测报告](../reports/2026-11/canfd_calibration_curves_20261113.md)和
[使用指南](../guides/canfd_motor_operations.md)。
