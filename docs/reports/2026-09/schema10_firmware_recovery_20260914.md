# Schema 10 固件恢复记录（2026-09-14）

## 问题与修复

电机4的 Flash 参数为 schema 10，但安装的旧固件仅接受 schema 4～9。
启动时参数加载退回默认值，CAN 节点因此变成0，pitch轴身份及已有增益未正常加载。
恢复 schema 10 参数加载和对应位置控制实现后，pitch重新使用节点4。

本次从 `bda6ace861cbb5a83f7299493a8f4adf8c7cea5e` 建立
`codex/restore-schema10`，选择性恢复已验证源码。参数和控制代码来源为 stash 的
index tree `29188dc5d3e0ec98a5f5898fae35affd91df9698`；CAN回复头初始化和
协议修订号查询来自 `outputs/can_tx_header_fix_20260911/source` 的冻结源码。
没有整体应用包含 Loader / 上位机改动的 stash。

schema 10 与更新的位置控制算法一同恢复，避免只修改版本号而沿用旧控制逻辑。
schema 10 参数直接加载，不触发迁移写入；有效旧参数使用已有迁移路径。
CAN GET 0x67 返回协议修订号2，普通回复完整初始化发送头。

## 参数 ABI 与镜像边界

- `InterfaceParam_TypeDef` 为2240字节，offset 2208处为32字节轴记录。
- 本次 pitch 为 AXS1，身份 pitch，运行节点4；不能与其他分支同名schema 10但采用不同布局的参数互换。
- APP范围为 `0x08000000..0x0801BFFF`，保留原工程布局。
- 完整参数/校准保护区为 `0x0801C000..0x0801FFFF`（16 KiB）。
- 现有旧架构中的全局控制对象、直接HAL调用和动态参数缓冲仍属后续平台迁移工作；此次恢复未增加新的跨层接口或重构这些路径。

## 验证结果

普通版与HIL版均由Keil全量重建，均为0错误、0警告。
两份HEX文件分别与此前实机验证的冻结镜像完全一致：

| 镜像 | SHA-256 |
| --- | --- |
| 普通版 | `a9545036d9ae319946cbf57648e57a4b3c20d08363bb86c80a322e46ba9693b1` |
| HIL版 | `a171ead3157d622697fe723aa8077579baba225e622c588ea7213d126bbb6992` |

`tools/run_can_status_tests.py` 的8组本机测试及
`tools/run_position_servo_tests.py` 的11组本机测试通过。
测试编译显式启用断言（`-UNDEBUG`），CAN测试支持通过Zig运行。
覆盖协议编码/长度/路由、状态反馈、优先级、轨迹、编码器、ADC调度、HIL保护和轴身份。
本次未重新进行运动试验，也未对旧schema迁移执行实机写入试验。

## pitch 下载和停机验收

使用J-Link `601012403`，先核对旧镜像和pitch轴记录，备份完整128 KiB Flash。
确认停机、相位输出关闭后，再关闭主输出并读回核验，随后暂停CPU并下载HIL版。
下载后复位运行，全部109284个镜像有效字节回读一致。

初始化定时器的MOE可以为1；停机状态的相位使能位（CCER mask 0x555）为0。
烧录前会将MOE及相位使能位同时清零并核验。
最初预检将停机误判为必须MOE=0而退出，未写Flash；修正预检后才执行下载。

16 KiB保护区前后逐字节一致，SHA-256：
`8246b1a61b950f4665f8059d79b4963927df2a10abc97e60a3913bdfcfac3b4c`。

USBCAN以1M/1M CAN FD执行只读查询：

| 检查项 | 结果 |
| --- | --- |
| pitch节点 / 协议修订号 | 4 / 2 |
| 模式 / 故障码 | 0（停机）/ 0 |
| 编码器在线 | 1 |
| 最大电流 | 6 A |
| 位置 Kp / Kd | 8 / 2 |
| 速度 Kp / Ki | 0.5 / 2 |
| 查询重试次数 | 0 |
| roll参考节点查询 | 3正常回复 |

roll固件未在此次下载。以上结果说明静态通信和参数恢复通过，不能替代后续运动性能验收。

本地证据目录：`outputs/schema10_restore_20260914/`，包括构建日志、
回归日志、源码恢复清单、`pitch_install_verified/report.json`、
前后Flash备份和 `can_verification.json`。
