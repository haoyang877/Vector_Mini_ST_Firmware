# Windows设备维护工具

双击`VectorMiniFlasher.exe`即可运行，不需要安装Python。
维护软件当前版本为 **v1.1.0**，标题、侧栏和 Windows 文件属性显示同一版本。
该版本独立于板卡 APP/Loader 版本；配套合包为 Loader v1.0.0 + APP v0.5.2。
发布包包含完整合并 HEX 与 `can_app/manifest.json`、`can_app/app.bin`；后者仅升级 APP。
当前发布EXE内置64位`JLink_x64.dll`和`ControlCANFD.dll`，启动后从自身资源目录自动加载，无需选择DLL。
首次在新电脑使用时，仍需安装对应USB设备驱动；内置DLL不等于安装Windows内核驱动。
本工具用于STM32G431CB的当前APP+Loader初装布局，GUI默认使用SWD 1000 kHz，不写参数或标定区。

## 四个页面

窗口左侧为四页导航，右侧共享连接卡片和分步骤操作区。每次操作由按钮触发，空闲时不自动访问设备。
操作日志默认折叠，通过“查看操作记录”展开；忙时显示进度，连接与固件输入保持锁定。

| 页面 | 操作与数据 |
| --- | --- |
| 初装烧录 | J-Link写入合并HEX，保留完整备份、参数和标定区保护 |
| CAN固件升级 | 经USB-CAN更新APP，核对同板UID、布局、版本、STOP和全参数CRC |
| 版本与状态 | CAN查询当前APP/Loader；J-Link只读Flash APP版本、CPU、向量表及故障状态 |
| 标定参数 | J-Link读取持久化标定，分组表格、1024点线性化曲线和逐点明细 |

当前内置CAN库为厂商64位`ControlCANFD.dll`，导入依赖是Windows系统库，不需另选SDK目录。
选择设备索引、通道0/1、节点0～7后，点击“读取CAN设备”显示适配器序列号和MCU UID。
接口为当前USBCANFD_200U的1M/1M ISO CAN-FD，不支持任意USB-CAN厂商驱动。
修改连接设置撤销此前绑定；多探针情况下必须明确选择，不能依赖枚举顺序。

### CAN升级

选择完整合并HEX，或含`app.bin`的升级包`manifest.json`；工具提取并校验APP，不经CAN写Loader。
先读取CAN设备，再点击“开始升级”。停机准入、UID或适配器身份不匹配时拒绝升级。
APP通过原升级交接服务关相后进入Loader；当前Loader存在会话时不抢占、不自动重擦。
升级显示传输进度，激活后核对版本、UID、STOP和全参数CRC；任何检查失败保留失败记录，不自动重复整轮。
CAN的参数保护检查是整个16 KiB的CRC核对，不等同于J-Link初装的逐字节备份/验证。

### 状态与标定读取

版本页选择J-Link或CAN后读取。J-Link显示的是通过有效提交记录及APP CRC验证的Flash版本，
不能将其等同于当前执行中的版本；CAN显示当前运行APP或Loader的版本，不为查询切换运行环境。
APP运行时暂不能只读查询Loader版本；运行在Loader时APP实时状态不可用。
CAN参数查询不是原子同时采样，页面显示读取时间；协议修订非2时不猜测旧版速度/位置编码。

标定页保存完整Flash和16 KiB参数两次一致的快照，不复位、不暂停/启动CPU，不修改参数。
当前只解释真实头文件偏移已核对的schema10；空白参数显示未配置，未知/损坏格式保留原始文件并明确提示。
电流零偏、电机电气参数、电零位/机械零位、方向、三项标志、LUT、标定电流、摩擦模型分别列出。
独立AXS/DMP/PVF/PSP记录核对版本及CRC；基础schema10记录本身没有整体CRC，不能宣称所有值已做CRC验证。
电零位Q15保存的是机械单圈参考位置，度数按65536计数/圈换算；不额外乘极对数。
界面角度和速度使用度/度每秒，粘性摩擦系数转换为A/(°/s)。
保存标志只表示Flash记录内容，不代表角度、电零位或摩擦模型已通过实机有效性测试。
“LUT明细”仅查看已读取快照，不再次连接探针。

## 使用

1. 电机停机并暂停其他下载/调试及运动命令。
2. 在“初装烧录”页点击“选择文件…”打开`Loader_vX.Y.Z_APP_vA.B.C_YYYYMMDD.hex`完整合并文件，等待HEX校验通过。
3. 点击“刷新探针”，选择对应J-Link编号；有多个探针时程序不会按枚举顺序选择。
4. 点击“读取目标”，检查芯片、UID和参考电压；新板的UID自动读取，无需手填。
5. 默认SWD速度为1000 kHz，可选100、1000、4000 kHz；按需要选择“烧录校验成功后启动MCU”，点击“开始烧录”。
6. 显示“烧录成功”后，点击“打开记录目录”查看备份和`summary.json`。

烧录时重新校验文件哈希、实际MCU UID、芯片容量及关相状态，拒绝读取目标后换板或改变所选文件。
先备份完整128 KiB Flash与16 KiB参数，再撤销记录、写Loader和APP、验证全Flash、最后提交有效记录。
前后参数及保留页必须逐字节一致，失败不自动重试或复位；失败后需要重新读取目标。
运行选项不勾选时保留MCU暂停，勾选时仅在烧录及启动检查通过后保持运行。
改变SWD速度会清除目标绑定，需重新读取目标；连接或烧录失败不自动降速重试。
独立Python/PowerShell脚本的默认速度仍为100 kHz，Python入口可通过`--speed-khz`显式指定。

日志保存在`%LOCALAPPDATA%/VectorMiniFlasher/outputs/`的独立时间/UID目录，包含：

- `flash_before.bin`：完整Flash备份。
- `parameters_before.bin`：电机参数及标定备份。
- `flash_before_record.bin`：提交有效记录前的全Flash回读。
- `flash_after.bin`：最终完整回读。
- `summary.json`：结果、固件哈希、实际UID、SWD速度、烧录耗时与保护区验证结果。

只接受当前布局的完整合并HEX：Loader在0x08000000，记录页在0x08003000，APP在0x08004000。
保护区地址、旧APP、单Loader、截断或损坏的HEX在连接探针前即拒绝；不用连续BIN或单APP CAN包做初装。
新板原本没有标定数据时，工具不会复制另一台电机的参数，仍需配置身份并标定。

## 可复现构建与离线检查

在Windows x64的Python环境安装锁定的直接依赖，再构建到新outputs目录：

```powershell
python -m pip install -r loader/tools/flasher_build_requirements.txt
python loader/tools/build_flasher_exe.py --jlink-dll <JLink_x64.dll> --can-dll <ControlCANFD.dll> --out outputs/flasher_exe_build
```

构建保留PyInstaller日志、依赖完整版本、源码快照、EXE和原始厂商DLL的SHA256。
已有独立构建的初装包时，生成完整下载ZIP：

```powershell
python loader/tools/package_maintenance.py --flasher-build outputs/flasher_exe_build --factory-package outputs/factory_package --loader-version 0x01000000 --out outputs/maintenance_package
```

输出包括EXE、按Loader/APP版本与日期命名的合并HEX、初装包、CAN包及备选烧录脚本。
先校验初装输入、EXE哈希和源码快照；此命令不访问硬件、不代表实机验收通过。

两个DLL验证Windows x64 PE架构后原样内置，许可文本和厂商说明同时保留；不修改厂商二进制。
J-Link使用范围与DLL分发说明参见[SEGGER官方SDK说明](https://www.segger.com/products/debug-probes/j-link/tools/j-link-sdk/)。
PyInstaller使用官方单文件窗口模式，运行时在临时目录展开依赖，不需外置Python脚本。
参考[PyInstaller官方文档](https://pyinstaller.org/en/stable/usage.html)。

GUI和冻结EXE的离线检查不访问硬件：

```powershell
python tests/loader/test_factory_hex.py
VectorMiniFlasher.exe --check-hex candidate.hex --report hex_check.json
VectorMiniFlasher.exe --check-hex candidate.hex --smoke-ui ui_check.json
VectorMiniFlasher.exe --check-drivers drivers_check.json
VectorMiniFlasher.exe --version-report software_version.json
```

离线入口输出不作为实机烧录证据。2026-10-14在独立板上完成新版冻结EXE的实际按钮烧录，
1 MHz下用时9.323 s，两次脚本复烧分别为9.416 s和9.358 s；均保留参数及保留页。
断开烧录连接后重新读取，两次完整Flash与GUI回读一致，MCU运行APP且无故障状态。
这些是同一固件的复烧结果，不能作为所有新板首次擦写的耗时保证；备份及完整回读步骤未减少。
证据位于`outputs/flasher_standalone_20261014_gui_v2/`及`outputs/loader_flasher_exe_20261014_v2/`。
这些实机计时对应之前的外置DLL版本；内置库和视觉改版的离线验证归档在`outputs/loader_flasher_exe_20261014_v4/`，
不能将旧版硬件烧录结果视为本次内置库EXE的新实机测试。
