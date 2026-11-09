# AT32 Loader 首次实机预检

日期：2026-11-02。目标：YG_ESC_V1.0 / AT32F456CCU7。
用户明确选择先测AT32，并确认电机动力隔离、仅逻辑供电、USB-CAN和SWD可独占。

结论：**台架预检未通过，未开始迁移、烧录或CAN升级。**
J-Link可以连接，但关闭缓存后的Flash大范围回读出现差异或读取异常；
安装入口在再次备份阶段拒绝继续。不得将“探针连接成功”当作镜像或升级验收通过。

## 实测情况

| 项目 | 结果 |
| --- | --- |
| J-Link连接、实际UID和容量读取 | PASS，实际Flash为256 KiB |
| 逻辑参考电压 | 约3.23～3.24 V |
| 六路输出关闭核对 | PASS，测试结束再次确认 |
| USB-CAN设备0、通道0、1M/5M初始化/关闭 | PASS，未发送目标命令 |
| 完整256 KiB双次备份 | 重接后取得过一次关闭缓存、两份一致的快照；安装前重验失败 |
| 旧参数离线校验 | schema14、记录CRC和节点1有效；已生成绑定UID/源快照的原样16 KiB迁移包 |
| 参数迁移、Loader/APP初装 | NOT_RUN；安装前读0x0801E000失败，尚未调用Flash下载 |
| Loader启动、CAN升级、Type56/57、保护区下载后比较 | NOT_RUN |
| Keil直接下载、连续100轮、断电验证 | NOT_RUN |

关闭缓存后，PyLink和独立J-Link Commander均复现回读不一致。
100 kHz、每笔256字节的诊断也出现读取异常。用户检查接线并重新插拔后，
一次完整回读通过，但随后安装前复核再次失败，说明尚不具备稳定的读写验收条件。
当前不能仅凭这些结果区分探针、DLL、USB/SWD连接或目标侧原因，不能直接归因于接线。
后续[11月3日复测](dual_board_loader_stress_20261103.md)记录独立连接及暂停CPU对照，仍未进入烧录。

## 工具修正

- 新增明确的`build.jlink_device`，AT32使用完整`AT32F456CCU7`，不使用发布文件名中的缩写。
- 将一次256 KiB读取改为每笔最多4 KiB，仍读取两份完整快照；短读或差异直接失败，不挑选重试结果。
- 维护CLI关闭Flash与内存缓存，保留APP的DWT时基和CPU原运行/暂停状态。
  命令依据见[SEGGER命令说明](https://kb.segger.com/J-Link_Command_Strings#SetAllowFlashCache)。
- 初装在暂停后延长并回读已有WDT配置，避免RAM烧录算法运行时受到原APP短WDT影响。
  配置更新超时/回读错误拒绝写Flash；本轮未进入该步骤，实机写入仍待验证。
- 维护工具专项13项测试通过，包含分块双次读取、后块差异/短读拒绝、暂停/WDT失败零Flash写入。
  [修正后的PR检查](../../../outputs/runs/at32-loader-bench-tool-fixes-final/summary.json)全部通过。

上述修正保持通信协议和MCU固件代码不变，不把工具修正视为Loader已通过实机测试。

## 最终状态与证据

测试中只对WDT调试冻结位和CPU暂停状态做过临时操作，**Flash写入调用为0**。
已恢复测试前CPU运行状态、解除临时WDT冻结；原WDT分频/重载仍为0/511，六路输出关闭。
未启动电机、未修改Flash参数区或选项字节；所有连接均已关闭。

[本轮实机汇总](../../../outputs/bench/dual-board-loader-at32-20261102T200523/summary.json)
记录每项状态；同目录包含失败日志、原始回读、一次有效快照、离线迁移包和退出状态。
[证据哈希](../../../outputs/bench/dual-board-loader-at32-20261102T200523/evidence-sha256.json)用于核对文件完整性。
此前的[离线开发记录](dual_board_loader_development_20261102.md)仅表示当时冻结源码的离线结果。
