# YG 电机工装 v1.4.0（Windows x64）

解压整个 ZIP 到可写目录，双击 `YG-Motor-Commission-v1.4.0.exe`。随包提供 AT32F456 的 Loader + APP、参数生成器、USB-CAN FD 接口库和运行库。工位需要 SEGGER J-Link 软件及 USB 驱动、USBCANFD-200U 适配器驱动。退出占用同一适配器或探针的调试软件，连接 SWD 和 CAN FD（默认物理通道 1，1 Mbit/s / 5 Mbit/s）。

## 交付文件

| 项目 | 文件名或位置 |
|---|---|
| Windows 烧录工具 | `YG-Motor-Commission-v1.4.0.exe`，文件属性和 `--version` 均显示工具版本 |
| 固件大包 | `firmware/YG2026-ESC-FULL-AT32F456CC-debug-LOADER-v2.2.0-APP-v1.6.5-20261111.hex` |
| 六轴同向初装预设 | `parameters/` 下每轴一份，文件名含轴名、型号、节点和参数版本，如 `YG-Params-PITCH-YQ-5740-0x06-v1.2.0-preset.bin` |

参数 v1.2.0 新增 Lift（0x08），其运行、标定和行程参数与 PITCH 相同。原整车地址表也将 0x08 分配给 BMS，两者共用 CAN 总线时需使用不同地址。PITCH 和 Lift 的默认运行限速为输出轴 30 rad/s，Flash 记录格式为 schema14。默认设置直接使用随包预设，自定义限速时，工装按同一版本规则生成本机记录。PITCH、Lift 和 YAW 随包预设使用 3 A 台架热参数。标定前备份和标定后读回文件名中的版本标识工装采用的参数规则，具体样机和本次内容由 UID 目录及运行记录标识。

## 初装一台电机

1. 将可连续旋转的电机接到对应的 AT32F456 空板，接通工装供电。选择 **1 烧录 + 配参 + 标定**。
2. 选择实际轴。左轮、右轮、ROLL 对应 HT8115-4；PITCH 和 Lift（0x08）对应 YQ-5740，YAW 对应 YQ-5708，三个减速轴均为 40:1。工装要求电机正转时原始编码器增加（同向），编码器反向位固定为 0。输入输出轴限速（rad/s）。PITCH 和 Lift 默认运行上限为输出轴 30 rad/s，对应 40:1 电机轴 1200 rad/s。六轴标定期间均按输出轴双向 40 rad/s 检查编码器测速，超限立即关相并报告本次标定失败。扫圈目标为左轮、右轮、ROLL 各 15 rad/s，PITCH 和 Lift 各 0.6 rad/s，YAW 0.75 rad/s（均为输出轴）。
3. 工装自动采用所选轴的热参数。PITCH、Lift 和 YAW 使用随包的 3 A 台架预设，HT 使用其型号预设。调整输出轴限速时同样自动配参。
4. 输入 Y 后开始。工装核对大包、绑定板卡 UID、烧录并回读 Flash、核对 APP 的活动型号和参数，再通过 CAN 发起角度标定。PITCH、Lift 和 YAW 运行编码器与输出 Hall 联合标定，直驱轴依次执行编码器 LUT 和电气零位。看到标定结果、曲线文件和记录路径后可换下一台。

选择 **2 仅烧录配参** 可以分工位操作。选择 **3 已配板重新标定** 先检查板上已确认的型号和编码器方向，反向记录在发起标定前被拒绝，适用于同一装配、同一版本的 APP。初装对空板及同一中断安装计划开放；已有其他固件或参数时保留原板内容。机械零位需有已知的机械参考位置，工装只标定电气零位。

Windows 多层解压目录支持长路径。若需沿用其他目录的安装备份，可用 `--out` 指向原来的 `records` 目录。

每次完成后，窗口显示板卡 UID、电气零位、方向、参数 CRC32、停机状态和补偿曲线峰峰值。`records/<UID>/<时间>/` 下保存结果 `result.json`、标定前后两份带版本的参数 BIN、角度曲线 `angle-calibration.png` 和 1,024 点数据 `angle-calibration.csv`。曲线取自本次 Flash 回读，纵轴为原始角减校正角（°）。直驱轴绘制电机编码器曲线，PITCH、Lift 和 YAW 另绘输出轴 Hall 曲线。初装中断后保留整个 records 目录，以原包、原配置重试。初装已成功、CAN 标定失败时，排除故障后选择 3 即可。结果须同时满足本次标定票据已保存、回读的标定标志有效、控制器空闲无故障和六路关断。

## 命令行

在解压目录的 PowerShell 中运行：

```powershell
.\YG-Motor-Commission-v1.4.0.exe --check
.\YG-Motor-Commission-v1.4.0.exe --operation full --axis pitch --speed 30
.\YG-Motor-Commission-v1.4.0.exe --operation calibrate --axis pitch
.\YG-Motor-Commission-v1.4.0.exe --export-curves "上一次标定目录\result.json"
```

第二条示例适用于同向的 PITCH 台架样机。`--channel 2` 切换到 CAN 第二物理通道，`--probe` 可选择探针序号。`--check` 完全离线，校验大包与 AXF 并实际生成六轴的同向记录并与包内参数逐字节比较。其余选项见 `--help`。

此版本随附 Loader v2.2.0 和 APP v1.6.5 的 debug 工装镜像；固件来源及构建状态见 firmware/summary.json。manifest.json 列出交付文件的 SHA-256。
