# YG-ESC

<img src="./docs/assets/project/logo.png" width="600"/>

## 选择板卡与固件工程

Vector Mini 是面向永磁同步电机的单轴 FOC 驱动固件。独立 APP 从 Flash 起始地址直接启动。LoaderApp 是由 Loader 校验并跳转启动的电机 APP。Loader 自身是独立的启动与升级镜像。表中的基址取工程链接布局。

| 板型 | 镜像 | Keil 工程 | 链接基址 | 升级命令 |
| --- | --- | --- | --- | --- |
| STM32G431 | 独立 APP | [ESC-APP-STM32G431](projects/stm32g4/MDK-ARM/ESC-APP-STM32G431.uvprojx) | `0x08000000` | 56/57 未绑定 |
| STM32G431 | LoaderApp | [ESC-APP-STM32G431-LoaderApp](projects/stm32g4/MDK-ARM/ESC-APP-STM32G431-LoaderApp.uvprojx) | `0x08004000` | 56 停机交接、57 镜像确认 |
| STM32G431 | Loader | [YG2026_ESC_LOADER](loader/targets/vector_mini_st_g431/YG2026_ESC_LOADER.uvprojx) | `0x08000000` | 接收升级命令 50–55 |
| AT32F456 | 独立 APP | [ESC-APP-AT32F456-Standalone](projects/at32f456/application/ESC-APP-AT32F456-Standalone.uvprojx) | `0x08000000` | 56/57 未绑定 |
| AT32F456 | LoaderApp | [ESC-APP-AT32F456](projects/at32f456/application/ESC-APP-AT32F456.uvprojx) | `0x08004000` | 56 停机交接、57 镜像确认 |
| AT32F456 | Loader | [ESC-LOADER-AT32F456](loader/targets/yg_esc_v1_at32f456/ESC-LOADER-AT32F456.uvprojx) | `0x08000000` | 接收升级命令 50–55 |

配套 APP 使用带连字符的工作区，默认选中 AT32F456 LoaderApp。独立 APP 使用带下划线的工作区。Loader 工程从表中单独打开。[配套工作区 YG-ESC](YG-ESC.uvmpw) · [独立工作区 YG_ESC](YG_ESC.uvmpw) · [Keil 使用说明](docs/guides/keil_workspace.md)

## 阅读固件代码

固件设计按四个主题维护：[架构设计](docs/architecture/software_architecture.md)、[状态机设计](docs/architecture/motor_state_machine_v1.md)、[通信协议设计](docs/protocols/README.md)、[参数管理设计](docs/architecture/parameter_ownership_v1.md)。查找源码、Keil 工程及离线测试时使用[文件导航](docs/architecture/code_structure.md)。

人工调试的观测对象、变量清单、单位和枚举值见
[Keil Watch 只读观测指南](docs/guides/watch_motor_commands.md)。
Watch 只用于观察；符号必须匹配板上固件，不能直接修改内部模式、目标或 PWM 来启动电机。

```powershell
python -m pip install uv==0.12.16
uv sync --locked --extra dev
uv run python tools/run.py doctor --profile pr
uv run python tools/run.py verify --profile pr

# 兼容的底层入口仍然可用
python tools/run.py --list
python tools/run.py check_project_layout
python tools/run.py build_firmware
python tests/run.py
```

统一验证会在 `outputs/runs/<run-id>/summary.json` 留下与 Git 提交、工具版本和产物哈希
绑定的证据。`quick` 与 `pr` 完全离线；`release` 只构建、不烧录；台架命令必须显式提供
工作台配置与场景。项目协作约定见 [AGENTS.md](AGENTS.md)，架构地图见
[ARCHITECTURE.md](ARCHITECTURE.md)，文档入口见 [docs/README.md](docs/README.md)。

代码风格和可读性规则见 [STYLE.md](STYLE.md)。新 C 模块和 Python 命令应从持续接受
CI 检查的 [templates](templates/README.md) 示例开始，再按模块职责进行重命名和裁剪。

## 电机控制与配置

两板共用有感电机控制、编码器标定和 CAN FD 启停、目标及反馈接口。速度和位置模式通过外环产生电流参考。在线参数命令 120–122 当前返回不支持；角度标定完成后的参数持久化由 APP 前台执行。[控制执行](docs/architecture/motor_execution_core_v3.md) · [标定流程](docs/guides/calibration_process.md) · [参数记录](firmware/parameters/README.md#storage)

STM32G431 原板使用板载 SPI 编码器。AT32F456 根据有效轴记录的节点和 AXS2 身份装配关节或轮轴，型号、减速比和限值见[五轴配置](docs/guides/at32f456_five_axes.md)。各板外设接线与原理图见[硬件资料](docs/hardware/README.md)。

## 注意事项

> STM32G431 原板的 PB8 引脚为BOOT0引脚，在设计时由于引脚紧张复用成FDCAN1_RX，让程序正常启动需要使用STM32CubeProgrammer将BOOT0软件下拉。

> 当前固件默认按 **8 串常规三元锂电池（满充 33.6 V）**配置母线保护：欠压 24.0 V、过压 34.0 V、快速过压 34.5 V；启动/故障清除后重新使能要求 25.6–33.8 V。保护配置与验证见 [8S 母线电压保护](docs/guides/bus_voltage_8s.md)。这是固件配置，不代表板级耐压认证；原硬件超过 35 V 可能损坏的约束仍保留。

> 原始手册见 [Vector_User Manual.pdf](<docs/hardware/Vector_User Manual.pdf>)。其中 USB/VOFA 章节属于历史功能；当前接口见[设备发现与支持范围](docs/protocols/README.md#device)。

## 参考项目

**ODRIVE**:https://github.com/odriverobotics/ODrive

**VESC**:https://github.com/vedderb/bldc

**MIT**:https://github.com/bgkatz/3phase_integrated

**AXDR**:https://oshwhub.com/lylssy/foc_driver

**dgm**:https://github.com/codenocold/dgm
