# V0.7.x 固件维护方法

更新日期：2026-11-09。适用于 `feature/roll-motion-telemetry` 分支的 STM32G431CB 固件。
维护顺序为：确认源码和构建类型 → 备份目标板 → 更新程序 → 配置并保存参数 → 标定 → 通信和运动验收。
日常更换左右轮或带阻尼关节，优先修改电机参数；只有控制功能或通信部署方式变化时才重新构建程序。

## 源码和版本

| 项目 | 当前基线 |
| --- | --- |
| 分支 | `feature/roll-motion-telemetry` |
| 原 v0.7.1 基线提交 | `21b7038d4939a8f825b49ed142c064c80e60a1e0` |
| 当前 APP | V0.7.2，build 0，编码 `0x00070200`；包含启动零偏修复 |
| Loader | V1.0.0，独立构建 |
| 本机工作树 | `D:/Work/Code/Vector_Mini_ST_Firmware_roll_telemetry` |

在该工作树同步代码，不在其他分支目录直接覆盖文件：

```powershell
Set-Location 'D:/Work/Code/Vector_Mini_ST_Firmware_roll_telemetry'
git status --short --branch
git pull --ff-only origin feature/roll-motion-telemetry
git log -1 --oneline
```

同步前保留未提交工作；发生冲突时先处理冲突，不用强制重置清掉本地修改。
版本号在 [firmware_version.h](../../firmware/app/firmware_version.h) 中配置，修改后必须重新构建。
维护记录同时写明提交号、构建类型和文件 SHA256，仅有“V0.7.1”不足以确定实际镜像。

## 程序和参数分开维护

| 区域 | 地址范围，末地址包含 | 更新原则 |
| --- | --- | --- |
| Loader | `0x08000000`～`0x08002FFF` | 首次安装或明确更新 Loader 时写入 |
| APP 有效记录 | `0x08003000`～`0x080037FF` | 与 APP 的版本、长度、CRC 配套更新 |
| 保留页 | `0x08003800`～`0x08003FFF` | 保持原数据 |
| APP | `0x08004000`～`0x0801BFFF` | 日常程序更新区域 |
| 参数与标定 | `0x0801C000`～`0x0801FFFF` | 单板配置，程序更新时保留 |

APP 工程为 [Vector_Mini_ST.uvprojx](../../firmware/platform/stm32g4/cubemx/MDK-ARM/Vector_Mini_ST.uvprojx)，
Loader 工程为 [Vector_Mini_ST_Loader.uvprojx](../../loader/mdk/Vector_Mini_ST_Loader.uvprojx)。两者分别编译。

普通 APP 工程输出的 HEX 包含 APP 和有效记录，**不包含 Loader**。空板或旧版直接启动 APP 的板使用完整合包；
已经有匹配 Loader 的板，日常只更新 APP。完整合包也不包含任何电机的标定数据。

## 构建类型

| 配置 | 用途 |
| --- | --- |
| `POSITION_EXTERNAL_REFERENCE_ENABLE=0` | 普通点到点位置控制；本轮完整合包使用此配置 |
| `POSITION_EXTERNAL_REFERENCE_ENABLE=1` | 主控连续下发位置参考，使用外部参考控制路径 |
| `VECTOR_CAN_CLASSIC_ONLY=1` | 默认经典 CAN，共线运行；Loader 不初始化 CAN，在线升级关闭 |
| `VECTOR_CAN_CLASSIC_ONLY=0` | FD 兼容维护版本，保留原 FD 升级协议；须确认总线上其他设备兼容 |

CAN 部署宏必须对 APP 和 Loader 使用相同值，并完整 Rebuild。
默认经典 CAN 包不能通过“SSH 登录主控后走 CAN Loader”升级；SSH 只是主控访问方式，不能代替板端升级协议。
详见 [经典 CAN 部署](../protocols/classic_can_deployment_v063.md)。

## 构建和合包

### 构建 APP

在上述工作树根目录运行。每次使用新的输出目录，保留原有交付文件：

```powershell
$pythonExe = 'D:/Work/Code/Vector_Mini_ST_Firmware/.venv/Scripts/python.exe'
$releaseDir = 'outputs/releases/v071-classic-20261106-rebuild01'
if (Test-Path -LiteralPath $releaseDir) { throw '请更换为新的输出目录' }
& $pythonExe tools/build/build_position_candidate.py `
  --uv4 'C:/Keil_v5/UV4/UV4.exe' `
  --reference point-to-point --can classic `
  --out "$releaseDir/build-app"
```

该脚本生成隔离工程、完整构建，并生成有效记录、AXF、HEX、纯 APP BIN 和摘要。
外部参考版本将 `--reference` 改为 `external-200hz`，同时使用不同输出目录和文件名。
要求构建为 0 错误、0 警告，并出现 `APP record synchronized`；后处理失败时不使用残留产物。

Keil 后处理使用当前工作树的 `.venv/Scripts/python.exe`。`.venv` 不随 Git 同步，新建工作树后若报
`After Build ... CreateProcess failed`，先建立本地 Python 环境，不关闭后处理：

```powershell
& 'D:/Work/Code/Vector_Mini_ST_Firmware/.venv/Scripts/python.exe' -m venv --without-pip .venv
& '.venv/Scripts/python.exe' -c 'import sys; print(sys.executable)'
```

上述首个 Python 路径是本机已有解释器；其他电脑使用实际安装的 Python。有效记录工具只需要标准库。
随后重新构建，确认有效记录生成成功，再下载。后续出现的“cannot open AXF”应先排查构建是否完整成功。

### 构建 Loader 并生成完整合包

1. 打开独立 Loader Keil 工程，确认本次 `VECTOR_CAN_CLASSIC_ONLY=1`，执行 Rebuild。
2. 检查 Loader 不超过 12 KiB、APP 不超过 96 KiB。
3. 使用 `fromelf` 导出 Loader BIN，再使用现有合包工具。以下是 V0.7.1 build 0 的示例：

```powershell
& 'C:/Keil_v5/ARM/ARMCC/bin/fromelf.exe' --bin `
  --output "$releaseDir/loader.bin" `
  'outputs/build/keil/Vector_Mini_ST_Loader/Vector_Mini_ST_Loader.axf'

& $pythonExe loader/tools/loader_factory.py pack `
  --loader-bin "$releaseDir/loader.bin" `
  --app-bin "$releaseDir/build-app/Vector_Mini_ST.bin" `
  --app-version 0x00070100 `
  --out "$releaseDir/factory"
```

`factory` 目录必须尚不存在。后续版本要同时更改版本头文件、输出目录和 `--app-version`，不能只改文件名。
合包包含 `combined.hex`、`loader.bin`、`app.bin`、`boot_record.bin` 和 `manifest.json`。
发布时保留完整目录，给合并 HEX 另取含版本与构建类型的名称，内容保持不变。

以下已生成完整合包仍为 **build 0**，不包含 build 1 的启动零偏修改：

- [Loader V1.0.0 与 APP V0.7.1 经典 CAN 完整合包](../../outputs/releases/v071-classic-20261106/Loader_v1.0.0_APP_v0.7.1_classic_20261106.hex)
- [构建与合包摘要](../../outputs/releases/v071-classic-20261106/release_summary.json)
- 合并 HEX SHA256：`7269f64ad2ece7d53eafa53198d1b859a8b5a4165e14ce070a2f09886abb6f1e`
- APP CRC32：`7A59F6D7`；APP 长度：98,304 字节；Loader 有效长度：8,588 字节。

`outputs/` 不随 Git 推送。交付源码后仍需另行交付这些二进制和清单。

2026-11-09 实测的 v0.7.1 build 1 仅更新 APP 与有效记录，沿用板上 Loader：
[APP HEX](../../outputs/current_offset_boot_20261109/build/Vector_Mini_ST.hex)，
[版本与哈希清单](../../outputs/current_offset_boot_20261109/build/Vector_Mini_ST.boot.json)。
APP CRC32 为 `D0B65355`，长度 98,304 字节；此 HEX 不用于空板初装。
该改动现以 v0.7.2 build 0 归档；新版本构建及验证范围见
[v0.7.2 版本记录](../reports/2026-11-09-startup-current-offset-v072.md)。不要将旧 build 1 文件重命名为 v0.7.2。

## 烧录和确认

完整合包使用 `VectorMiniFlasher.exe` 的“初装烧录”页面。工具的版本与所选固件版本相互独立，
必须检查所选 HEX 的实际版本，不能依据工具所在文件夹的版本名判断。

1. 电机停机，暂停其他运动指令并释放目标板的 Keil Debug、Scope 等调试占用。
2. 选择本次完整 HEX，刷新探针，明确选择 J-Link 序列号。
3. 读取目标并记录 MCU UID、型号与容量，防止连错板。
4. 确认功率输出关闭后开始烧录。保留两次一致的完整 Flash 备份和参数备份。
5. 完成后核对报告 `PASS`、完整回读、APP 有效记录，以及参数区和保留页逐字节不变。
6. 区分“烧录后保持暂停”与“复位后运行”。正常启动应由 Loader 跳到 APP，`VTOR=0x08004000`，最终 mode 0、输出关闭。

用 Keil 日常更新 APP 时，选 `Erase Sectors`、`Program`、`Verify`，不使用全片擦除。
地址由 HEX/AXF 提供，不把 APP BIN 当成从 `0x08000000` 开始的完整镜像。
完整步骤见 [初装合包](loader_factory_package_v1.md)、[维护工具](loader_flasher_exe_v1.md)、[Keil APP 调试](keil_app_debug_v1.md)。

### v0.7.2 上电电流零偏校准

有效电机配置启动时自动进入 `mode=11`，三相功率输出关闭，六路相输出引脚保持高阻；
仅保留 TIM1 CH4 触发 ADC 注入采样。按 20 kHz 累积 20,000 次原始采样，约 1 秒后得到三相 ADC 均值。
三相均处于 1948～2148 counts 才一起更新 RAM 的 `A_Offset/B_Offset/C_Offset`，随后进入 `mode=0`。
如果均值不合格，保留原偏置并报 `CurrentOffset_Error`，维持停机。未配置的无效电机档位仍保持停机并报参数错误。

启动校准不会进入 `mode=9`，不写 Flash，也不会自动启动电机。每次复位重新测量；
需要保存 RAM 偏置时，必须另行显式执行参数保存。启动校准期间不要主动发送 STOP，STOP 会中止本轮采样。
完整实机证据见 [启动零偏修改与验证记录](../reports/2026-11-09-startup-current-offset.md)。

## 电机参数维护

电机角色和运动设置以 [五台电机总体参数表](five_motor_parameter_table.md) 为目标；
实际生效值以所连接板的 RAM 和复位后 Flash 回读为准。修改参数不需要更换五套源代码。

当前右轮的目标配置：

| 项目 | 值 |
| --- | --- |
| CAN ID | 2 |
| 极对数与采样电阻 | 21，6 mΩ |
| 运行与标定电流 | 7 A，3 A |
| 正反向最高速度 | 20 rad/s，即约 1145.9 °/s、190.99 RPM |
| 纯速度模式 Kp、Ki | 0.06、0.6 |
| 阻尼与摩擦前馈 | 关闭，`load_profile_flags=0`，补偿电流为 0 |
| 位置范围 | 连续旋转；不沿用 Roll/Pitch 的位置限位记录 |

本版本右轮使用合法节点 2 与空轴记录表示角色，不把不受支持的 AXS2 身份记录强行写入。
LUT、电零位、机械零位、编码器方向属于单台电机；更换电机后不得复制另一台的标定数据。

停机后修改 RAM，再执行 `mode=9` 保存，等待返回 `mode=0`。
检查 `last_save_result`：1 为写入成功，2 为数据相同且无需擦写；随后复位并重新核对字段和扩展记录 CRC。
J-Link 地址必须从当前板匹配的 AXF/DWARF 获取，不复用旧版本 Watch 地址。
详见 [参数保存](parameter_flash_save_v1.md)、[阻尼配置](roll_wheel_runtime_profile.md)、[两套速度 PI](separate_speed_pi.md)。

## 标定和运行验收

1. 确认实际机械范围、架空状态、电流和速度边界以及现场可断电条件。角度标定需要连续旋转多圈。
2. 在相输出关闭时确认编码器在线、原始角度合理、电流采样零偏合理。
3. `mode=13` 执行角度传感器 LUT 标定。轮档预定位电流 2 A，目标速度 15 rad/s；预定位与开环拖动阶段不用速度 PI。
4. 进入磁链观测器速度闭环后，使用 `MotorControl.speed_Kp/speed_Ki`。日常值为 0.06/0.6；本轮减小 5 倍的试验值为 0.012/0.12。
5. LUT 成功后核对标志位、保存结果、曲线和复位读回，再执行 `mode=15` 电零位标定。其电流受 `calib_current` 和运行电流上限共同约束。
6. 标定有效后才执行低速正反转、逐级提速以及目标范围内的运动验收；本次请求只做角度标定时，不自动扩展到速度或位置试验。

`calib_flag` 的 bit 0 表示 LUT，bit 1 表示电零位，bit 2 表示机械零位。
因此 1 仅代表 LUT，3 代表 LUT 与电零位，7 再包含机械零位；标志不代替实际运动验证。

本轮右轮标定边界为相电流停止门限 3 A、最高 30 rad/s、单次通电不超过 10 秒。
为留余量，运行电流指令临时限制为 2.8 A。主机采样停止门限不是硬件峰值钳位，不能保证采样间隔内没有越限。
时间到仍未完成就停机并记录，不以降低判据或伪造标志完成标定。

mode 13/15 标定成功可能自动触发参数保存，因此试验参数也可能随之落盘；mode 11 电流零偏校准不自动保存。
结束后要恢复日常增益和 7 A 运行限流，再保存并复位回读；不能仅凭“开始时只改 RAM”判断 Flash 没变。

### J-Link 曲线采集

`FOC.Ia/Ib/Ic` 在中断内先写入原始采样值，再进行两相重构。J-Link 顺序读取这些地址可能跨越中间状态，
不能保证三相与 Id/Iq 属于同一快拍。三相和明显不为零、而 Id/Iq 稳定的孤立样本，应保留为异常记录并核查采集一致性。
优先使用控制中断完成后发布的遥测帧；不要直接把单次 Watch 数值认定为真实峰值，也不要自动删除异常点后继续运行。

至少记录时间、模式、故障、标定步骤、原始角度、速度、Id/Iq 参考与反馈、母线电压、标定标志，以及停止原因。
STOP 后确认 mode 0 和三相输出关闭；停止或调试链路失效时立即结束试验并要求现场断电处理。

## 经典 CAN 通信验收

右轮节点 2，仲裁速率 1 Mbit/s，标准经典数据帧，BRS 关闭。以下请求均为 DLC 4、数据 `00 00 00 00`：

| 检查项 | 请求与回复 ID | 期望回复 |
| --- | --- | --- |
| 节点号 | `0x209` | `40 00 00 00`，大端 float32 的 2 |
| 软件版本 | `0x26F` | v0.7.2 为 `00 07 02 00`，四字节版本号，不是 float32；旧 v0.7.1 build 1 为 `00 07 01 01` |
| 运行模式 | `0x201` | 停机时 `00 00 00 00` |
| 故障状态 | `0x24D` | 无故障时 `00 00 00 00` |

先检查单次收发，再做固定频率持续查询，记录有效回复率、超时和错误计数。
合法只读查询可以刷新已启用的通信看门狗；`can_hb_set=500` 是超时时间毫秒数，不是自动周期反馈间隔。
一次回复只证明基本通路，不代表电机已标定或运动控制通过。

## 变更归档和回退

常用代码入口如下。修改程序前先区分控制逻辑、参数默认值和当前板已保存的值；改变默认值不会直接替换有效 Flash 参数。

| 维护内容 | 代码入口 |
| --- | --- |
| 控制模式与切换 | `firmware/app/foc_task.c`、`firmware/motor/data_type.h` |
| 速度和位置控制 | `firmware/app/foc_run.c`、`firmware/motor/position/` |
| 标定与阻尼档位 | `firmware/motor/identification/foc_calibration.c`、`firmware/motor/motor_load_profile.c` |
| 参数默认值与序列化 | `firmware/services/parameters/foc_param_profile.h`、`foc_param.c/.h` |
| 参数保存 | `firmware/app/parameter_persistence.c` |
| CAN 协议与收发 | `firmware/communication/can/interface_can.c/.h` |
| 电流采样与重构 | `firmware/motor/foc/foc_sensing.c` |
| APP 与 Loader 的公共契约 | `shared/boot_contract.h`、`shared/can_deployment.h` |

修改控制、参数存储或通信后，先跑离线回归，再构建相关 APP/Loader 变体：

```powershell
& $pythonExe tests/run.py `
  --cc 'D:/Work/Code/Vector_Mini_ST_Firmware/outputs/protocol_sync_20260911/.build_deps/ziglang/zig.exe' `
  --out "$releaseDir/tests"
```

编译器和 Python 路径是本机环境，新电脑须替换为已安装工具的路径。
只有通过的软件检查才进入实机步骤；离线通过不代表电流、温升、标定或运动性能通过。
改变 Flash 布局、记录结构或升级协议时，必须同时处理 APP、Loader 和维护工具的兼容性，不能只修改一端。

程序变更保存提交号、源码差异、构建日志、AXF、HEX、BIN、版本/CRC/SHA256，以及构建宏。
单板操作保存探针号、UID、参数修改前后值、完整 Flash 备份、采样曲线和停止结果。
将“构建通过”“烧录通过”“标定通过”“通信通过”“运动通过”分别记录，避免互相替代。

回退优先使用本板备份，或使用确认兼容同一 Loader/参数布局的旧 APP 加有效记录。
不能恢复其他电机的全片 Flash，也不能将 `0x08000000` 入口的旧 APP 直接放入新 APP 分区。
完成回退后重新核对版本、参数、标定标志和停机状态。

## 当前板维护结果

2026-11-06，J-Link `601012403`，MCU UID `3700350017504b5332393120`：

| 项目 | 实际结果 |
| --- | --- |
| 完整程序安装 | Loader V1.0.0 与经典 CAN APP V0.7.1 已写入并完整回读通过 |
| 右轮配置 | 已按上表保存，复位后 RAM/Flash 一致 |
| 个体标定 | LUT 已完成并保存，`calib_flag=1`；电零位和机械零位尚未标定 |
| 首次减小 5 倍增益的 mode 13 | 约 0.75 秒在预定位阶段停止，未进入速度闭环 |
| 首次停止原因 | 监测到单点 Ia=7.951 A，触发 3 A 判据；该点三相和约 5.954 A，Id 约 2.001 A，真实过流尚未确认 |
| RTT 采集复试 | 保持 Kp=0.012、Ki=0.12、2.8 A 指令上限，改用中断完成后发布的 Id/Iq 帧监测；8.672 秒完成 mode 13 |
| 复试曲线 | 17,131 个 RTT 电流帧，电流矢量幅值峰值 2.014 A；主机采样最高速度 18.264 rad/s，均未触及本轮停止门限 |
| 最终状态 | mode 0、故障 0、三相关闭；日常增益与 7 A 限流已恢复；LUT 保存及复位后完整 Flash 一致，编码器方向仍为 0 |

本板已完成本轮角度传感器标定，后续仍需电零位标定和运行验收。复试使用 Id/Iq 合成幅值作为相电流幅值的保守判据，保留首次原始相电流异常记录；没有把该异常直接认定为真实过流或直接删除后沿用原判据。
2026-10-27 的节点 2 通信通过记录属于此前设备/试验，不能替代本板的新验收。

本机记录：

- [烧录报告](../../outputs/flash_601012403_v071_20261106_run2/summary.json)
- [右轮参数保存与复位核验](../../outputs/right_wheel_config_601012403_20261106/result.json)
- [本次标定记录](../../outputs/right_wheel_angle_calibration_601012403_20261106/attempt_low_gain/result.json)
- [采样一致性分析](../../outputs/right_wheel_angle_calibration_601012403_20261106/attempt_low_gain/measurement_analysis.json)
- [标定曲线 CSV](../../outputs/right_wheel_angle_calibration_601012403_20261106/attempt_low_gain/samples.csv)
- [RTT 复试与标定结果](../../outputs/right_wheel_angle_calibration_601012403_20261106/attempt_low_gain_rtt/result.json)
- [LUT 保存和复位核验](../../outputs/right_wheel_angle_calibration_601012403_20261106/attempt_low_gain_rtt/verification.json)
- [复试 RTT 电流曲线 CSV](../../outputs/right_wheel_angle_calibration_601012403_20261106/attempt_low_gain_rtt/rtt_currents.csv)

这些 `outputs/` 链接只在保留了本机交付目录和测试记录的电脑上有效。
