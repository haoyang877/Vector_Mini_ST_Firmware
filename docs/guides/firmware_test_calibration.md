# AT32F456 固件烧录、测试与标定使用指南

适用于 APP v1.8.2、Loader v2.3.4、CAN-Link-Test v1.8.1 和电机工装 v1.4.3。
本包为 debug 开发验证包。当前连接的 PITCH 台架用于验证，其他轴没有本轮实机结论；
动态角度检测的已知限制与本次结果以随包测试报告为准。
v1.8.0 的[历史复测](../reports/2026-11/hardware_check_v1_8_0_20261113.md)保留单拍判据的失败结论。
v1.8.1 按整体趋势验收，当前结果以匹配版本的随包报告为准。
本轮[首答修复与复测](../reports/2026-11/can_first_answer_v1_8_2_20261113.md)电机命令与
编码器方向通过，CAN 50 ms首答问题在当前PITCH修复；剩余Hall原角区尖峰，交付保持debug。
此前[GUI实测](../reports/2026-11/hardware_check_v1_8_1_20261113.md)的超时作为历史保留。

## 连接和节点

安装 SEGGER J-Link 软件及驱动、USB-CAN FD 适配器驱动。退出占用同一探针或 CAN
适配器的程序，连接 SWD、CAN_H/CAN_L 和地，确认板卡与电机接线。默认 CAN 为
ISO FD+BRS、仲裁 1 Mbit/s、数据 5 Mbit/s，工具的“通道 1”对应 SDK channel_index=0。

| 节点 | 轴 | 电机 |
| --- | --- | --- |
| 3 / 4 | 左轮 / 右轮 | HT8115-4 |
| 5 | ROLL | HT8115-4 |
| 6 / 8 | PITCH / Lift | YQ-5740，40:1 |
| 7 | YAW | YQ-5708，40:1 |

所有手动开环拖动统一使用模式 10：电流闭环、角度开环，相电流 3 A。
供电、电源限流和可用行程应按实际台架填写。当前样机条件为 28 V、电源限流 5 A、
空载，两个方向可用行程不限；不能将此行程条件套用到装有限位机构的其他轴。

## 文件与烧录方式

| 产物 | 用途 |
| --- | --- |
| `firmware/*FULL*.hex` | J-Link 大包，包含 Loader 和配套 APP，HEX 自带地址 |
| `firmware/*APP*.hex` | 只更新 APP，要求板上已有匹配 Loader |
| `firmware/app-package/` | CAN 升级包，包含 image.bin、manifest.bin、package.json，保持三者配套 |
| `firmware/*.axf`、`*.map` | 与镜像匹配的调试符号和资源信息 |
| `manifest.json`、`firmware/summary.json` | 文件 SHA-256、版本、目标、布局和构建来源 |
| `CAN-Link-Test-v1.8.1.exe` | SWD烧录、CAN升级、硬件检测、独立角度标定和速度控制页 |
| `Hardware-Check-v1.0.1.exe` | 独立命令行硬件检测，不依赖本机 Python |
| `YG-Motor-Commission-v1.4.3.exe` | 空板配参、标定和标定结果导出 |

大包仅包含 Loader 和 APP 的实际加载段，**不包含参数、LUT 或生产身份**。
现有已配板使用 CAN-Link-Test 的“J-Link 烧录”：先识别单台板卡，核对 UID/节点，
选择本包 FULL HEX，再开始写入。工具先完整备份 Flash，写入后逐字节回读镜像及
保护区，复位后核对 APP、参数和 CAN 回复。保留本轮记录目录，换板前重新识别。
使用 Keil 时选择扇区擦除和下载后校验，禁止整片擦除，具体设置见[Keil 调试](keil_loader_app_debug.md)。

空白板使用电机工装的“仅烧录配参”或“烧录 + 配参 + 标定”，选择真实轴及型号。
该流程会为新板生成初装参数；已有板重新标定使用“已配板重新标定”，不要用新板
参数覆盖现有标定。初装、断点恢复和工装命令见[电机工装](motor_commission.md)。

CAN 更新只写 APP 槽：在“CAN 升级与参数”中扫描在线板卡，核对节点与 UID，选择
`firmware/app-package/`。升级期间保持供电，结束应看到新版本和已确认状态。
Loader 更新使用 SWD 大包。具体分区及恢复步骤见[双板 Loader](dual_board_loader.md)。

## 硬件功能检测

CAN直接标定和速度控制分别使用“角度标定”“速度控制”页，操作见
[该页指南](canfd_motor_operations.md)，不需要为此重新烧录当前固件。

1. 在“通信压测”中选择实际节点，先完成只读查询，核对 APP 版本及故障状态。
2. 打开“硬件功能检测”。v1.7.4默认节点6和“静态和双向驱动”，一次执行八项静态与
   两项动态；需要单独关相采样时切到“关相静态”，两个动态项会显示“未测”。
3. 当前PITCH台架默认室温20 °C、相电流 **3 A**、电频3 Hz、单程1500 ms、两侧行程360°。
   行程指从当前位置向两侧分别可用的输出轴角度，按实际台架修改。清空必填值时会在
   打开适配器前提示对应字段。温度可按实测修改或清空。
4. 续租计数和目标序号默认0。已保存UID继续沿用本机计数数据库，填0不会清零已保存值。
   首次使用且确认刚复位、尚无运动目标时可用0；已有运动历史的新工位填写真实高水位。
   后续由工具按UID递增保存。
5. 每方向至少预转 500 ms，且连续两个 50 ms 平均转速窗满足稳定条件后才验证轨迹。
   v1.8.1 取 50 ms 角度均值观察趋势，至少三个窗口确认位移和方向，单拍波动只作诊断。
   净位移与至少 2/3 可辨窗口同向；趋势无主方向、停滞及去回同向仍判异常。中间零频保持并确认停稳，再反转。
6. 结束确认 STOP、轴空闲、无故障和实际关相，自动比较正命令角度递增、反命令角度递减。霍尔仅核对自身趋势，无需现场旋向。
   未稳定、行程不足或中止的项目保留“未测”，不能算通过。

母线电流以原始 ADC 数据计算去均值 RMS，限值为 **<50 cnt**；三相和霍尔仍为
**<10 cnt**。四路电流的偏置界限仍为距 2048 **<300 cnt**，电压仍为 27～29 V。
NTC 为 0～60 °C，电机端静态峰峰角 <0.01°。软件显示滤波不参与原码验收。
SN/motor_type 未填写不阻止本项测试，节点/UID、APP 角色、能力和运动保护仍核对。

从源码运行的等价入口：

```powershell
uv sync --locked --extra dev --extra maintenance
uv run --extra maintenance python tools/run.py hardware_check --dll <ControlCANFD.dll> --node 6 --ambient-c 20 --out outputs/static.json
uv run --extra maintenance python tools/run.py hardware_check --dll <ControlCANFD.dll> --node 6 --drive --current-a 3 --frequency-hz 3 --leg-ms 1500 --travel-deg <两侧可用角度> --last-lease 0 --last-target 0 --out outputs/drive.json
```

默认自动比较命令与编码器方向；`--observe` 仅供需要实物附加核对时选用。命令参数中的温度、
行程和计数必须按本次现场填写。原始 CAN 帧、板端统计和 STOP 结果一起留档。
交付 EXE 的同一入口是 `Hardware-Check-v1.0.1.exe`，将上述命令的
`uv run --extra maintenance python tools/run.py hardware_check` 替换为 EXE 路径，
`--dll` 指向随包 `bin/ControlCANFD.dll`。

`--travel-deg` 是行程保护值，不是旋转目标。目标由电频和时间决定：
单程输出轴角度约为 `360 × 电频Hz × 单程秒 / (极对数 × 减速比)`。
当前 14 极对、40:1 的台架，3 Hz、1500 ms 对应约 2.89°/程；实际提前中止时达不到
该角度。改变电频、时长或电流用于对照，不代表可以改动检测限值；推荐至少留出
1500 ms 单程时间；工具允许 700～3000 ms，不能用不足 500 ms 的短段宣称完成验收。

## 标定

标定前保存原参数备份，固定电机并确认工装允许连续旋转。LUT 会连续扫圈，ROLL、
PITCH、YAW、Lift 若连接限位或阻尼机构，须先按[五轴配置](at32f456_five_axes.md)
核对工装条件；有限行程内的短时硬件检测不能替代 LUT 标定所需的连续行程。

已有配套固件及有效参数的板卡，用工装“已配板重新标定”或：

```powershell
.\YG-Motor-Commission-v1.4.3.exe --operation calibrate --axis pitch
```

PITCH、YAW、Lift 的 LUT 请求联合电机端和输出 Hall 标定及电角零位；直驱轴按工装
流程依次执行 LUT、电角零位。固件标定由自身状态机控制，不通过手动电压开环代替。
机械零位必须将机构放到已知机械参考位置后单独设置，LUT 完成不能代替机械零位。

CAN 底层入口是 Type160：kind=1 LUT、kind=2 电角零位、kind=6 机械零位；Type161
查询返回的 ticket，Type162 或 STOP 取消。受理不等于完成，只有票据显示保存完成、
回到空闲且 Flash 回读的标定标志/LUT 有效，才能交付。不得运行中强写 Flash。
标定前后参数 BIN、票据、结果 JSON、1024 点 CSV 和曲线按 UID 保存。
详细原理和发布语义见[标定流程](calibration_process.md)。

## 从代码构建与复测

构建目标来源为 `shared/targets/yg_esc_v1_at32f456.json`；先生成目标输入和配套工程：

```powershell
uv run python loader/tools/generate_target.py --target yg_esc_v1_at32f456
uv run python tools/build/generate_loader_app_project.py --target yg_esc_v1_at32f456
uv run python tools/run.py build_loader_bundle --target yg_esc_v1_at32f456 --uv4 <UV4.exe> --fromelf <fromelf.exe> --token <本次非零事务号> --output outputs/<新包目录>
$env:KEIL_UV4 = '<UV4.exe>'
uv run python tools/run.py verify --profile release
```

Windows 建议设置 `PYTHONUTF8=1`，避免中文日志被默认 GBK 解码干扰。
release 检查要求干净源码；失败、未测范围和构建来源写进交付清单。
没有完整功能验收的镜像保持 debug 类别，不能仅改文件名标成量产 release。
