# Roll 经典 CAN 参数回复修复 v0.6.2

日期：2026-10-20。状态：源码修复、离线回归和普通 ARM 构建通过；已烧录、复位回读并验证停机，CAN 共线及运动验收尚未完成。

## 基线与范围

分支：`bugfix/roll-classic-reply-v061`。
基线为 ovp36 的 `ef6bb874bc62f3a960584e8512c8b4101f26ee2e`（v0.6.1），保留建树时已有的热保护开关、Keil 和现场配置改动；本次候选为 **v0.6.2，build 0**。
这是从 v0.6.1 基线修复通信回复的候选，不沿用其他工作树 v0.7.x 的功能或实机结论。

问题是普通参数回复原先固定使用 FD/BRS，即使请求为经典 CAN，也可能产生不适合源擎共线环境的 FD 回复。本次只修正回复帧格式，不修改源擎协议或设备配置。

## 实现行为

| 位置 | 本次行为 |
|---|---|
| `firmware/platform/api/comm_hw.h` | 接收帧显式携带 `fd/brs`；增加按指定格式提交回复的接口 |
| `firmware/platform/stm32g4/ports/comm/comm_status_stm32g4.c` | 从 HAL 接收头提取格式；经典回复使用经典 CAN、关闭 BRS；FD 回复保留请求的 BRS |
| `firmware/communication/can/interface_can.c/.h` | 每个已接受请求的回复 ID、载荷、长度、FD/BRS 一起冻结在单槽中；槽忙及 HAL 重试期间不得被后续请求覆盖 |
| 版本与测试 | 软件版本和黄金向量更新到 `0x00060200`；增加经典/FD/BRS、忙重试与不支持路径的回归 |

经典请求不能隐式开启 FD 维护：拒绝经典 UID 查询、BOOT 入口及非零 48 字节状态流配置。经典请求可关闭状态流；CRC 等可装入经典帧的既有回复按请求格式发送。历史 FD/BRS 维护接口保留。

普通经典查询**不会自动关闭已经存在的 FD 状态流**，也不会撤回已在硬件队列中的 FD 帧。因此本次不能单独证明从上电到维护全生命周期均为经典 CAN；共线前仍须核验初始状态流配置及所有发送源。

控制算法、位置/速度目标、参数编码与存储布局、节点配置、热保护和既有喂狗规则沿用该基线。离线记录对 `CAN_ReceiveMessage_Update`、`CAN_SetNode`、`CAN_CommandEncoding`、`CAN_ReplyEncoding`、`CAN_DisConnect_Handle`、`CAN_BaudRateSwitching` 做了函数内容哈希一致性检查。它不代表旧协议看门狗串扰问题已经解决。

## 已执行验证

| 验证 | 结果与证据 |
|---|---|
| Python 单元测试 | 56 项通过：[unit.log](../../outputs/roll_classic_tests/python/unit.log) |
| Python 集成测试 | 36 项通过：[integration.log](../../outputs/roll_classic_tests/python/integration.log)；其中故障注入文本不是本次硬件故障 |
| 原生 CAN 回归 | 通过：[tests.log](../../outputs/roll_classic_tests/can/tests.log)，覆盖请求格式、冻结/重试、经典维护拒绝及原控制/回复行为 |
| 热参数与存储 | 通过：[summary.json](../../outputs/roll_classic_tests/thermal_config/summary.json) |
| 温度保护开关 | 通过：[summary.json](../../outputs/roll_classic_tests/thermal_switch/summary.json)，含生命周期、降额、异常输入及实际 CAN 90/91 入口 |
| 普通/HIL 工程布局、差异空白检查 | 通过；见[离线汇总](../../outputs/roll_classic_tests/summary.json) |
| 普通 ARM APP 完整构建 | 0 错误、0 警告：[Vector_Mini_ST.log](../../outputs/build/logs/Vector_Mini_ST.log) |

Python 共 92 项；原生 CAN、热参数和温度保护回归单独计入，未混成一个测试数量。离线汇总生成于 ARM 构建前，其 `ready_for_arm_build` 状态不能代替随后的构建日志。本轮未执行 HIL ARM 构建或 HIL 实机测试。

构建命令（仓库根目录）：

```powershell
python tools/build/build_firmware.py --target normal
```

Keil After Build 已输出 `APP record synchronized`：APP 分区 98,304 字节，CRC32 `0x18E4746F`，版本 `0x00060200`。链接统计为 Code 89,732 / RO 3,116 / RW 440 / ZI 23,848 字节。

## 冻结构建工件

下表 SHA256 与实际文件和[构建清单](../../outputs/build/keil/Vector_Mini_ST/Vector_Mini_ST.boot.json)一致。`outputs/` 不纳入 Git，交付或安装时须单独保存这些文件和清单。

| 工件 | 字节数 | SHA256 |
|---|---:|---|
| [Vector_Mini_ST.axf](../../outputs/build/keil/Vector_Mini_ST/Vector_Mini_ST.axf) | 1,928,488 | `00dedb7e267d14d66806cbf9e2e076a77a2fef73f0cec17aaa60249b455e5bb8` |
| [Vector_Mini_ST.hex](../../outputs/build/keil/Vector_Mini_ST/Vector_Mini_ST.hex) | 270,512 | `417cff8e40421a399acf4c585cbcfbfe0f0da08580748378aa2ea517c686fa38` |
| [Vector_Mini_ST.bin](../../outputs/build/keil/Vector_Mini_ST/Vector_Mini_ST.bin) | 98,304 | `97c61a609aca5702e126a139ffcd4bf8a2f066a1c59a2cf6772e7eef0ceb93c4` |

BIN 是 `0x08004000..0x0801BFFF` 的完整 APP 分区；HEX/AXF 另含 `0x08003000` 起的有效记录，不含 Loader 或参数区。安装不得全片擦除。

## 尚未完成与安装前置条件

用户确认释放 LKSscope/Keil 后，已取得完整 Flash 双读一致备份。现场 APP 与本任务此前冻结的 v0.6.1 镜像逐字节相等，不是其他工作树的 v0.7.x。

安装已完成，结果为`UPDATED_VERIFIED_STOPPED`：只更新 APP 和最后提交的有效记录，
Loader、保留页和整个16 KiB参数区逐字节不变。复位后版本为v0.6.2，节点3、mode0、错误0、
三相高阻；电零位9151、机械零位7761、标定标志7保留，未发运动命令。
参数区SHA256为`364df8d8bb7d42d9e12d2de1bd5f402e019ae8d420ca836f8de4f247de2f3653`。
证据位于`outputs/classic_reply_install_20261020/capture_current.json`、`prepare.json`和`install.json`。
安装器对每步写入均完整回读，不以EndDownload返回数值代替逐字节校验；离线替身覆盖成功返回0及部分写入拒绝。

随后按用户指令通知上位机任务接手 CAN2 三节点测试，本任务不同时占用USB-CAN。
经典请求/回复、源擎共线稳定性与运动验收仍待实测，安装成功不能代替这些结论。

后续按以下顺序执行并另记结果：

1. 释放 J-Link 及其他控制来源，核对实际探针、MCU UID、节点 3、停机及关相状态；完整 Flash 连读两次一致后保存新备份。不能用旧参数备份覆盖现有标定。
2. 冻结本次候选哈希，仅更新 APP 与有效记录，记录最后提交；逐字节核对 Loader、保留页及 `0x0801C000..0x0801FFFF` 参数区不变。
3. 使用新 AXF 核对启动路径、`v0.6.2`/`0x00060200`、节点、参数和标定读回。候选沿用 v0.6.1 的完整标定门槛；现有仅电零位标志不能直接视作已通过闭环准入，禁止为绕过准入而改标志。
4. 用经典查询实测经典回复，并覆盖 FD/BRS 请求、队列忙重试、状态流配置及异常恢复；再验证与源擎实际流量共线时的帧格式、错误计数和控制反馈。
5. 运动试验单独验证 stop/release、限位与超时行为。新镜像烧录、共线兼容和运动性能均须取得各自证据后才能宣称完成。
