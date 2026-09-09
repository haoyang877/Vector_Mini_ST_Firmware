# roll / pitch 电机配置与持久化

用户于2026-09-09确认最大巡航速度为 **8 s/rev，每圈8秒，45°/s**，不是8 rev/s。

| 项目 | roll | pitch |
| --- | --- | --- |
| 电机与硬件 | 原已验证直驱电机/驱动 | 用户确认与roll型号、硬件一致 |
| 允许角度范围 | −90°～+90° | −0.3～+0.9 rad，约 −17.19°～+51.57°（用户最新修正） |
| 最大巡航速度 | 45°/s | 45°/s，沿用此次确认的速度 |
| 本次状态 | 已写入Flash并复位读回 | 独立电机已校零；最新轴范围已写入Flash并回读，控制参数实测整定中 |
| 配置文件 | [roll.json](../tests/mode3/roll.json) | [pitch.json](../tests/mode3/pitch.json) |

角度沿用各电机自身编码器机械零点。型号与硬件相同不表示编码器零点、偏心补偿或装配摩擦相同；pitch不会自动复制roll的校准区。pitch.json保存当前Flash基线，RAM候选在pitch_candidate_20260909.json；最新范围、短时相电流保护及实测过程见[pitch续测记录](pitch_mode3_burst_20260909.md)，早期过程保留在[pitch验证记录](pitch_mode3_validation_20260909.md)。

最新补充：[30 秒保护和重复性排查](pitch_mode3_repeat_20260909.md)，包含 HOLD 退出后恢复逻辑修正、12 轮实测及最终四轮回归结果。

## Flash格式与控制生效范围

新增 [MotorAxisProfile](../software/config/motor_axis_profile.h) 为纯C配置模块。现有参数结构尾部追加32字节AXS1记录：magic、独立版本1、8字节名称、最小/最大角度、最大巡航速度和CRC32。名称存储为ASCII `roll` 或 `pitch`，角度/速度使用rad、rad/s；CRC覆盖前28字节，采用CRC-32/ISO-HDLC的反射多项式0xEDB88320。

本目标由匹配AXF的DWARF确认，扩展偏移为参数区起点+2208，即地址0x0801C8A0；既有pos_maxspeed偏移2132。记录位于原参数页57内部，没有另占页，也没有移动原有字段、校准表或schema/magic。模块不操作Flash，仍由既有参数存储路径负责保存和恢复。

AXS1采用独立版本，不改变外层schema9/10的控制参数解释。全零/全FF尾部兼容未配置轴的旧参数；非空但CRC、名称或范围无效时，模式3拒绝使能。加载时计算CRC，20 kHz运行路径仅使用缓存有效性及边界比较，不在ISR中计算CRC。

模式3使能前检查当前位置严格位于配置范围内部；运行时检查实际位置和目标范围，越界通过既有故障管理入口报告MotorParam_Error。规划巡航速度被AXS1最大速度进一步限制。它是软件停止边界，不保证惯性机构不会发生物理越界，也不改变其他模式的运动限制。45°/s限制的是轨迹巡航，原有限速度反馈修正仍保留。

旧固件不认识AXS1，保存旧参数可能擦除尾部；全空尾部视为兼容旧参数，不能把CRC机制当作不可绕过的机械安全装置。恢复默认参数会清除轴配置。重新部署或恢复参数后，必须核对名称、范围与有效标志。

## 本次roll写入证据

开始时识别到板卡运行正常工程固件并处于mode3保持。用户确认仍为roll并停止使能后，已核验mode0/error0、三相输出关闭，备份完整128 KiB Flash。

下载工作区HIL镜像 `image34_axis_profile`，验证镜像数据和参数保留；随后只修改参数页57中的AXS1记录及已有pos_maxspeed字段。pos_maxspeed原本已是45°/s，其他控制参数、编码器零点、偏心表和电流校准保留。复位后检查完整Flash及实际运行上下文：

- 名称roll、范围约±90.0000025°、最大速度约45.0000013°/s；误差来自float32表示。
- AXS1有效标志为true，CRC32=3400232026。
- `pos_maxspeed=0.785398185 rad/s`。
- mode0/error0、HIL未使能、三相输出关闭。
- 完整Flash SHA-256：`be2c496d41cd5884daecc0666431268ce810c16a3531349086cc64a3b62fa91e`。
- 实机HIL AXF SHA-256：`a5a74a06ac02e21a9438ff6d79d914791d688fdb8b0bf34ee1f14a9c84b17ac1`。

台架备份、DWARF偏移、准备清单与读回报告在本地 `outputs/axis_profiles_20260909/`，其中 `roll_saved.json` 为最终读回结果。数据未随源码分发。本次没有运行新轨迹，新增边界检查后的最坏IRQ时间和运动效果尚未实机复测。

## 工具与回归

[motor_axis_record.py](../tools/motor_axis_record.py) 只在离线生成参数副本，不连接J-Link、不写Flash。输入必须是当前参数备份、匹配固件导出的偏移和所选轴配置。工具保留所有非目标字段，拒绝覆盖无法解释的非空AXS1记录。

```text
python tools/motor_axis_record.py --parameters <参数区备份.bin> --offsets <匹配AXF导出的参数偏移.json> --profile tests/mode3/roll.json --out <新输出目录>
python tools/run_mode3_validation.py host --cc <C99编译器或zig路径>
```

新增C回归覆盖空白旧参数、roll/pitch边界、无效值、逐字节损坏及Python/C一致的CRC样例；Python回归覆盖编码、解码和参数/校准保留。既有模式3回归继续运行。

本次从Git暂存区导出的独立源码快照验证通过：23组C伺服场景（含72组组合）、独立轨迹/编码器/HIL服务/AXS1测试、全部27项Python检查。正常及HIL工程均0错误0警告；正常Code=94760 B、ZI=31688 B，HIL Code=95740 B、ZI=31704 B。该快照仅构建测试，未覆盖下载到板卡。

实机测试后端支持 `--axis-profile tests/mode3/roll.json` 或pitch配置：先核对运行上下文的AXS1名称/CRC/范围/速度、实际电流上限与加减速度，再使能。pitch必须指定独立 `--session-dir outputs/pitch_validation_20260909/session`。按最新范围采用5°目标留量时，目标区间约−12.19°～+46.57°，主机停止边界约−15.19°～+49.57°。HIL原±90°保护仍作为外层限制，不能代替pitch自身范围。

## 提交与后续pitch测试

提交仅包含轴配置、模式3适配、参数扩展和前一轮整理的测试/文档。已有mode18控制器、schema10迁移和GUI改动保留在工作区。实机镜像来自包含这些工作区改动的构建，外层schema10；独立提交快照仍为schema9。两者都支持AXS1尾部，但整个固件不是同一二进制，不能直接把当前schema10参数交给schema9版本解释。

roll提交后已切换到整套pitch硬件，保留其独立校准。用户先后修正过机械范围，当前以−0.3～+0.9 rad为准；旧±30°和−9°～+30°试验配置仅供历史回放，不能用于后续下载或运动。
