# RTT 六路 int16 波形 v1

## 目标与基线

当前 AT32 波形通道使用 `JScope_f4f4f4f4f4f4`，每帧 24 字节六路 float32，
20 kHz 写入约 480 kB/s，主机实际读取约 18 kfps 并允许丢整帧。改为六路 int16、
每帧 12 字节，把带宽降为一半，并按物理量选择整数刻度与饱和范围。

基线提交 `9d9b2eb5`（`app_rtt.c` 独立、第六路为 q 轴电流）。台架为 AT32F456、
5708 电机和 8:1 减速器；RTT 通道和快环 20 kHz 已在 2026-11-04 实测通过。

## 实施与边界

- 通道名改为 `JScope_i2i2i2i2i2i2`，J-Scope 据此解析六路有符号 16 位整数，单帧 12 字节。
- 刻度：电流 mA（±32.767 A）、电角度 mrad（0..6.283 rad，1 mrad 分辨率）、
  机械转速 0.01 rad/s（±327.67 rad/s）、q 轴电流 mA（±32.767 A）；越界饱和，截断取整。
- 启动初期零偏未就绪的 NaN 记为 0，不再向主机传 NaN；不改变采样时刻、通道次序、
  单写者和满缓冲丢整帧的行为。
- 不变项：20 kHz 快环节拍、控制/标定/协议/参数 ABI、启动与故障策略、
  `RTT_Init()`/`RTT_Sampling()` 的调用位置与次序。

## 验收

- 原生测试：编译真实 `app_rtt.c`，核对 12 字节帧长、六路次序、刻度换算和满缓冲不重试。
- `format --check`、`lint` 与 PR profile 通过；AT32 Keil 编译 0 错误、0 警告。
- 样机关相读取：通道名正确、12 字节帧、快环约 20 kHz、六路数值在台架合理范围内。
- 证据保存在 `outputs/at32-rtt-int16-20261104/`，不把离线结论当作实机波形验收。

## 实施与离线验证（2026-11-04）

- `app_rtt.c` 使用 `RTT_Data_struct` 六路 `int16_t`（data0..data5），通道名
  `JScope_i2i2i2i2i2i2`；编译期检查结构体恰为 12 字节，防止通道名与帧长脱节。
- 原生 `test_interrupt_routing.py` 的 `at32_rtt_sampling` 夹具编译真实 RTT 模块并通过：
  A=1.0 A→1000 mA、B=−2.0 A→−2000 mA、C=3.0 A→3000 mA、电角 4.0 rad→4000 mrad、
  机械转速 5.0 rad/s→500（0.01 rad/s）、iq 6.0 A→6000 mA；满缓冲第二次写入不重试。
- `format --check`、`lint` 通过；AT32 Keil 编译 0 错误、0 警告
  （Code=50780、RO=2592、RW=168、ZI=26784，镜像在 `outputs/build/keil/ESC-APP-AT32F456/`）。
- `pr` profile 未整体通过，原因与本变更无关：`firmware/third_party/segger_rtt` 工作区
  尚未提交的 RTT 库与 `manifest.json` 钉住的哈希不一致；`docs/` 中指向已不存在的
  `outputs/` 证据的链接共 100 处。两项均为既有工作树状态，需在提交 RTT 库/清理证据链接时处理。
- 实机：2026-11-04 20:30 起 J-Link 探针未连接（`connected_emulators()` 返回空），
  本轮未下载、未做样机 RTT 读取；烧录与读取脚本已备在
  `outputs/at32-rtt-int16-20261104/`，台架恢复后执行。

## 回退

把 `app_rtt.c`、`app.h` 文档和本组测试恢复到 `9d9b2eb5` 即可回到六路 float32；
不涉及存储、协议或控制参数回退。
