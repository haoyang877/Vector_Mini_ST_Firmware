# MCU USB 服务移除说明

2026-09-17：当前 APP 已移除 USB CDC 命令、VOFA 周期打印、USB Device 中间件、
HAL PCD/LL USB 输入、USB 专用环形缓冲与应用 IRQ 实现。普通版和 HIL 版均适用。

启动代码不再初始化 USB，2 kHz 监督任务不再发送 USB 遥测，HSI48 不再开启，
PA11/PA12 配置为模拟输入且无上下拉。芯片启动文件保留 USB 固定向量槽位及弱默认处理器，
避免后续中断编号变化。CubeMX `.ioc` 与 `.mxproject` 同步清理。

PC 的 USB-CAN 分析仪及其驱动保留。现有 FDCAN、编码器、校准、保护、Flash 参数 schema、
RTT 观测和 HIL 测试支持继续保留（HIL 后于 2026-09-19 退役）。
当前 UART 仅保留已有外设初始化，不代表 Loader UART 已实现。

## 当前业务入口与缺口

| 原 USB 能力 | 当前状态 |
| --- | --- |
| 启停、模式、目标电流/速度/位置 | 使用现有 CAN 参数命令 |
| 节点、极对数、编码器方向、电流校准、限流限速、速度与位置增益 | 使用现有 CAN 参数命令；单位按 CAN 协议，不能照搬 USB 的 r/s 数值 |
| 母线、相电流、dq 电流、速度、位置、温度、Rs/Ld/Lq/磁链、故障 | 使用现有 CAN 读取命令/状态流 |
| 参数保存、恢复默认值、校准与辨识 | 保留对应模式与内部算法，通过现有 CAN 模式入口操作 |
| 摩擦候选系数、拟合误差、状态、失败原因、候选/模型有效性、应用模型 | 保留现有 CAN 摩擦命令 |
| LUT 全表与摩擦原始采样导出 | USB 导出入口移除；CAN 分块导出尚未实现 |
| 摩擦过程的 point/progress 聚合字段、独立读取已生效模型各系数 | 旧 USB 查询退出；现有 CAN API 不提供完全等价的全部字段 |
| VOFA 打印布局、USB 收发错误反馈 | 随 USB 服务退出 |

新 PC 上位机如需上述未覆盖的导出/诊断能力，应设计有版本、索引和分块规则的 CAN 业务接口，
配套协议向量与测试后实现。不能用 Loader 的 Flash PROGRAM 命令替代业务数据服务。

`tools/bench/usb_position_test.py` 已删除。
摩擦辨识主机脚本的旧 USB 采集/控制代码已删除，离线拟合保留在
`tools/analysis/friction_identification.py`：

```powershell
python tools/run.py friction_identification capture.json
```

输入为包含 `index`、`target_rps`、`speed_rps`、`iq_a`、`sample_count` 的记录数组，
也接受包含 `samples` 数组的历史报告。此工具只分析已有数据，不控制电机。

历史实验报告和原始硬件手册保留 USB 使用记录；其中 USB 指令不适用于当前固件。

## 容量与验证

| 目标 | 含 USB 的 ROM | 当前 ROM | 含 USB 的静态 RAM | 当前静态 RAM |
| --- | ---: | ---: | ---: | ---: |
| 普通 APP | 107968 B | 83224 B | 30992 B | 25488 B |
| HIL APP | 109716 B | 84976 B | 31256 B | 25752 B |

ROM 取 Keil map 的 Total ROM Size，包含链接器处理后的初始化数据；
静态 RAM 为 RW+ZI，不等于实测的运行时栈余量。两版 ROM 均减少 24744 B，静态 RAM 减少 5504 B。
两套工程全量编译均为 0 错误、0 警告。控制、保护、CAN 等离线验证见
[目录重构记录](../architecture/code_structure.md)。随后已完成普通 APP 的 J-Link 烧录、参数保留、
CAN 查询与 20/100/200 Hz 遥测实机回归，详见
[实机测试报告](../reports/2026-09/usb_removal_hardware_20260917.md)。未执行闭环运动；
电角度零点缺少校准、温度 ADC 为零等原有问题见报告。

当前镜像入口仍为 `0x08000000`，参数区仍为 `0x0801C000`；
虽然体积已低于未来 96 KiB APP 分区预算，但 Loader 链接布局、邮箱、VTOR 和交接服务尚未接入。
当前 HEX 不能作为 Loader 配套 APP 使用。
