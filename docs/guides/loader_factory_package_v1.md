# APP + Loader 合包与J-Link初装

使用`loader/tools/loader_factory.py`合包，使用其`flash`子命令或
`loader/tools/flash_factory.ps1`烧录。合包不连接硬件。
Windows用户也可使用[单文件图形烧录工具](loader_flasher_exe_v1.md)，直接选择合并HEX并自动读取目标UID。

## 包内容与地址

| 区域 | 地址范围（末地址包含） | 内容 |
| --- | --- | --- |
| Loader | 0x08000000～0x08002FFF | 独立Loader，尾部填充FF |
| APP有效记录 | 0x08003000～0x080037FF | APP大小、版本、CRC及提交标记 |
| 保留页 | 0x08003800～0x08003FFF | HEX不包含，脚本保持原字节 |
| APP | 0x08004000～0x0801BFFF | 重定位APP，尾部填充FF |
| 参数/标定 | 0x0801C000～0x0801FFFF | HEX不包含，脚本保持原字节 |

`combined.hex`是带地址的单个初装镜像，不需要人为指定APP偏移。
目录中的`manifest.json`、`loader.bin`、`app.bin`、`boot_record.bin`供脚本严格预检。
不能把旧0x08000000入口APP直接平移到0x08004000，必须使用当前重定位Keil工程构建。
不使用全片擦除，也不把合包作为CAN升级输入；后续CAN升级仍使用APP包。

对外下载的合并HEX按`Loader_vX.Y.Z_APP_vA.B.C_YYYYMMDD.hex`命名，日期使用北京时间。
例如`Loader_v1.0.0_APP_v0.5.0_20261014.hex`。仅复制并改名，内容及SHA256与包内`combined.hex`相同；
原包保留固定文件名用于脚本严格校验，下载副本的版本、日期和来源另记清单，不能因改名改变发布类别。

## 1. 构建并合包

APP工程为`firmware/platform/stm32g4/cubemx/MDK-ARM/Vector_Mini_ST.uvprojx`，
Loader工程为`loader/mdk/Vector_Mini_ST_Loader.uvprojx`。分别Rebuild，要求0错误0警告。
用ARMCC的`fromelf --bin --output <输出.bin> <输入.axf>`导出BIN。
产物放在`outputs/`，检查Loader不超过12 KiB、APP不超过96 KiB。

已有`app.bin`和`manifest.json`的CAN APP包时：

```powershell
python loader/tools/loader_factory.py pack `
  --loader-bin outputs/build/loader.bin `
  --app-package outputs/app_package `
  --out outputs/factory_package
```

只有两份BIN时，提供与实际APP一致的版本。当前合并候选v0.5.1、build0编码为0x00050100：

```powershell
python loader/tools/loader_factory.py pack `
  --loader-bin outputs/build/loader.bin `
  --app-bin outputs/build/app.bin --app-version 0x00050100 `
  --out outputs/factory_package
```

版本必须与本次构建的`firmware/app/firmware_version.h`一致。原始BIN本身不证明构建来源，
需保留独立构建的日志、AXF和源码信息；清单中的`packaging_source_head`仅标识合包工具源码。
输出目录必须不存在，避免覆盖已有包。文件哈希和HEX内容在烧录前重新检查。

## 2. 烧录

Python环境需要`pylink-square`；可用已有环境或运行`python -m pip install pylink-square`。
选择正确的探针序列号和已只读确认的96位MCU UID（24个十六进制字符）。
目标板已停机、功率输出关闭，暂停其他调试器及运动命令。
脚本允许空板GPIO输入、APP STOP模拟态、Loader低电平输出；其他状态拒绝烧录。

```powershell
python loader/tools/loader_factory.py flash `
  --package outputs/factory_package `
  --jlink-serial $probeSerial --uid $mcuUid `
  --out outputs/factory_flash_run1
```

默认完成后保持MCU暂停，便于检查。需要验证成功后复位启动时增加`--run`。
`--run`仅在启动和关相检查都通过后设置探针关闭时保持运行；默认入口及失败路径不启用该策略。
J-Link DLL无法自动发现时加`--jlink-dll $jlinkDll`；依赖在单独目录时加`--pylink-dir $pylinkDir`。
SWD默认100 kHz，可以用`--speed-khz`明确指定。每次烧录用新的输出目录。

Windows包装脚本等价入口：

```powershell
./loader/tools/flash_factory.ps1 `
  -Package outputs/factory_package -JLinkSerial $probeSerial -Uid $mcuUid `
  -Out outputs/factory_flash_run1 -Python $pythonExe
```

包装脚本的`-Run`、`-JLinkDll`、`-PylinkDir`对应同名Python参数。

## 3. 判定与保护

烧录前备份两次一致的128 KiB Flash，另存16 KiB参数。
先撤销旧APP有效记录，依次写Loader和APP，验证完整Flash含保护区后再提交新记录。
最后两次完整回读必须一致，并与“原备份仅替换三个允许写入分区”的期望逐字节一致。
输出`summary.json`为PASS才表示本次烧录及保护区验证成功；关闭探针失败也不算PASS。

失败不自动复位或重试整次烧录；保留输出目录定位问题。`--run`后关相检查失败时
执行紧急关相并暂停，失败状态不自动恢复，关相无法确认须现场断电介入。

包不含任何电机的参数/角度LUT/电零位/机械零位。同一包可以初装多块同硬件驱动板，
已有板的个体标定保持；空板的身份与标定需后续单独配置，不能复制另一台电机的参数。

## 离线检查

```powershell
python tests/loader/run_tests.py --cc $nativeCompiler --out outputs/tests/loader_factory
```

检查使用Flash/探针替身及真实共享C记录解码，不连接硬件。
