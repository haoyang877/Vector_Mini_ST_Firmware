# v0.6.3 经典 CAN 准入与离线验收

## 交付状态

工作分支 `bugfix/roll-classic-reply-v061`，基础提交 `ef6bb874`。
在原未提交 v0.6.2 补丁上继续完成，本轮仍未提交、未推送、未访问硬件。
本文件记录候选补丁，不是实机通过或正式发布声明。

- APP：v0.6.3/build0，查询值 `00060300`。
- 默认部署：`shared/can_deployment.h` 中 `VECTOR_CAN_CLASSIC_ONLY=1`。
- 替代部署：编译器定义同名宏为 0，完整 Rebuild 得到 FD 兼容版。
- 控制算法、7 A 参数、节点身份、LUT、电零位、机械零位、参数 Flash 布局均未修改。
- 原本地热调试可见性、Keil 用户选项、LKS 配置差异与工作前备份一致。

## 修改内容

`interface_can.c` 使用明确读写白名单，校验完整节点、标准数据帧格式、精确线长、
有限值/枚举/范围及实际业务返回值。接收调试镜像、`can_rx_en`、心跳清零和断连清除统一移到成功之后。
电流/速度越限、位置目标越界或模式拒绝不会先切模式或写目标。
查询保留占位数据语义，包括 NaN；`6B/6D` 写、`6C/6E` 读及其他偶数查询都有覆盖。
`37D/37E/3FD/3FE` 即使落入 Roll 硬件过滤范围也不能作为合法指令喂狗。

全经典模式从协议、状态流、前台重试和 HAL 发送出口拒绝 FD/BRS/超长载荷。
状态流旧启用位无效；旧 FD 待回复被丢弃，不改写成截断经典帧。
初始化和波特率重配置显式恢复所选帧格式、节点过滤器及通知。
BOOT 请求、交接服务和复位邮箱均有部署门禁；经典 Loader 编译为不链接 CAN HAL 的静默端口。
FD 兼容版保持原维护协议、48 字节遥测及跟随请求格式的回复。

协议与兼容边界见 [部署说明](../protocols/classic_can_deployment_v063.md)，
计划见 [实施计划](../plans/2026-10-21-classic-can-v063.md)。

## 离线结果

1. `python tests/run.py --cc <native compiler> --out outputs/classic_only_v063/full`：
   31 个验证组全部通过，其中 Python 单元 56 项、集成 36 项；包括位置历史算法等价性、
   参数存储、温度保护/配置、Loader 原维护协议及收发回归。
2. 最终聚焦回归：`run_can_status_tests.py` 和 `test_can_deployment.py` 通过，
   两种编译策略分别执行真实 RX/分派/队列/看门狗/发送函数；14 条新增黄金向量通过。
   覆盖不合法流量持续 500 tick 后断连、合法查询恢复、被拒绝写入保留目标与 RX 镜像、
   回复忙重试、8 节点初始化、改速、旧状态流位、直接发送、BOOT/邮箱和旧交接 pending。
3. 经典 Loader 整个端口无需链接任何 HAL 函数，初始化、重初始化及重复空闲轮询均静默。
   Loader 主循环的有效 APP 跳转、无效 APP/邮箱停留是代码审查边界，未模拟执行整个 Cortex-M 启动。
4. 9 个受影响 Python 文件 Ruff lint 与 format 检查通过；`git diff --check` 通过。

该历史 worktree 没有根 `.editorconfig/.clang-format/pyproject.toml`，也没有 `tools/run.py`
的 `verify/format/lint` 命令；直接尝试返回 Unknown command。
因此使用既有 `tests/run.py` 及主仓配置的 Ruff 检查，不能声称新版 harness 的 PR/release profile 通过。

## 构建

ARMCC 5.06u7：以下四种构建均为 **0 错误、0 警告**。

| 产物 | 策略 | Code / RO / RW / ZI，字节 |
|---|---|---|
| APP | 经典 | 88724 / 3116 / 428 / 23844 |
| APP | FD 兼容 | 90524 / 3116 / 444 / 23844 |
| Loader | 静默 | 7988 / 552 / 48 / 3848 |
| Loader | FD 兼容 | 11624 / 584 / 56 / 4488 |

FD Loader 的加载内容合计 12264 字节，距 12 KiB 区域仅余 24 字节；后续改动必须继续检查链接大小。
默认 Keil 工程另行重建通过，生成的经典 BIN 与矩阵构建一致：

- BIN：`604dc19bfee1ada73813f154691e0e5ff1ac3058e437e6691348e47ce05561c6`
- 默认工程 AXF：`379e3f2f81d02433389753ee18f4512ef9b788520c51c9cb2f8dea5882583d2a`
- APP 长度 98304 字节，镜像 CRC32 `6EDC613C`。
- 静默 Loader HEX：`05293cf9656fe3ccca9e1890327ee8ba9b643eecea660a22f33d06a29de41c42`。

构建矩阵、哈希与日志：`outputs/classic_only_v063/builds/summary.json`。
最终汇总及审查补丁：`outputs/classic_only_v063/summary.json`、`review.patch`。
默认 APP：`outputs/build/keil/Vector_Mini_ST/`；矩阵静默 Loader：
`outputs/classic_only_v063/builds/loader_1/Vector_Mini_ST_Loader.hex`。
矩阵生成器在 `outputs/classic_only_v063/build_matrix.py`，只调用 Keil 构建，不下载。

## 部署边界

本任务没有连接 J-Link、1126B、USB-CAN，没有读写 Flash，没有发送运动命令。
另一任务报告的 v0.6.2 三轴通信/小范围位置测试不能作为本补丁的实机验收。
旧板上 Loader 未经本任务更新或核验：仅下载 APP 后，启动窗口仍可能被旧 FD Loader 响应。
后续必须独立授权配套部署经典 APP 与静默 Loader，再核验上电、复位及回退期间的实际总线格式。
旧 FD 维护仅能在物理隔离且确认设备兼容的环境进行。

## 补充：50 Hz 轨迹及 250 查询/秒

按后续主控请求，增加离散时间压力用例，只修改测试，不修改固件；上面产物哈希保持有效。
使用真实 RX、业务分派、单回复槽及 STM32 发送端口；单 pending，前台每模拟 1 ms 推进。
每 20 ms 写入一次位置目标，并串行读取位置、速度、Iq、模式、错误。
位置/速度/Iq 从模拟反馈字段取不同于命令的值，逐帧核对 ID、长度、大端编码及数值。
查询期间混入 `1FD/1FE/2FD/2FE/27D/27E` 帧，不得喂 Roll 看门狗或覆盖回复。

| 发送队列每回复额外忙时 | 每轮五项耗时 | 模拟时长 | 查询/回复 | 每项回复 | 位置写入 |
|---|---|---|---|---|---|
| 0 ms | 5 ms | 120 s | 30000 / 30000 | 6000 | 6000 |
| 1 ms | 10 ms | 120 s | 30000 / 30000 | 6000 | 6000 |
| 2 ms | 15 ms | 120 s | 30000 / 30000 | 6000 | 6000 |
| 3 ms | 20 ms | 120 s | 30000 / 30000 | 6000 | 6000 |

两种部署分别运行上述四种场景，总计 240000 个查询回复，无队列覆盖、缺回复或心跳故障。
这里的 120 s 是离散模拟时间，不能解释为实际 MCU/总线持续测试。
模型每回复的 1 ms 包含一次前台推进及模拟交付；真实 CAN 仲裁、错误重发、IRQ 阻塞、
主循环最坏时延和网络任务抖动未测量。3 ms 额外忙时已经用尽周期，不能再保证现场 50 Hz。
现场需要分别统计五项接收频率、请求到回复延迟、超时数、整轮耗时；均值不能替代最大时延检查。
串行的五项也不是同一时刻的原子快照，应分别标记主控接收时间。

证据：`outputs/classic_only_v063/polling250-console.log`、`polling250/` 与 `polling250-summary.json`。
