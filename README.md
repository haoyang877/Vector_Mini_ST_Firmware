<img src="./docs/assets/project/logo.png" width="600"/>

## 代码目录

```text
firmware/                         电机 APP
  app/                            启动组装、公共请求接口、主状态机与任务调度
  motor/                          FOC、位置控制、编码器标定与保护
  parameters/                     配置默认值、标量读写和参数记录转换
  communication/                  CAN 接入与协议
  platform/                       硬件接口、STM32G4 BSP/端口/CubeMX
loader/                           独立 G431 Loader、升级事务与 CAN FD 端口
host_app/                         Qt6 维护页面、离线升级规划与参数模型（设备执行后端待接线）
shared/                           APP/Loader 共用协议、镜像及交接契约
tools/                           build、bench、analysis
tests/                           unit、integration
docs/                            architecture、protocols、guides、hardware、reports
outputs/                         构建、测试与分析产物（忽略提交）
```

Keil 主工程为
[`YG_ESC_APP.uvprojx`](firmware/platform/stm32g4/cubemx/MDK-ARM/YG_ESC_APP.uvprojx)，
目标和输出名统一为 `YG_ESC_APP`；构建产物输出到 `outputs/build/keil/YG_ESC_APP/`。

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

详细目录与验证结果见 [代码框架与目录规范](docs/architecture/code_structure.md)，
Loader 与后续 PC 边界见 [Loader](loader/README.md)、[上位机](host_app/README.md)、
[共享契约](shared/README.md)。

## 功能介绍

​        Vector 驱动器是一款中小型尺寸，基于矢量控制的永磁同步电机驱动，适用于轮毂电机，关节电机等。24V母线电压下，实测最高支持约400W的持续功率输入（需散热片）。内部算法可实现基础的电流、速度、位置闭环，以及编码器LUT和零位标定。目前仅支持有感运行，板载了一块TLE5012B绝对式编码器，同时支持外部SPI编码器信号输入。当前应用通过 FDCAN 接收控制命令和回传状态；USART1 外设已随容量优化退役，不再初始化。MCU USB CDC 命令、遥测和协议栈已移除，PC 端 USB-CAN 分析仪仍可使用。FDCAN数据段最高波特率5Mbps，可通过外部主控向总线发送报文以实现一对多控制。驱动器具有母线过压和欠压，相线过流，过温等保护，最大程度地保障驱动能够稳定运行。

## 软件已有功能

模式3轴配置：[roll / pitch名称、范围与Flash持久化](docs/guides/motor_axis_profiles.md)。

底盘轮子限速：[节点 1/2、10 cm 轮径、1 m/s 对应 190.986 rpm](docs/guides/chassis_wheel_speed_limit.md)。

* 基础有感FOC算法 电流 速度 位置 可控
* 直接位置→速度→电流串级，保留原增益、限幅和抗饱和
* 磁编码器偏心补偿
* FDCAN基本启停、目标、反馈、参数读写与Loader交接
* 绝对式 SPI 编码器：板载 TLE5012B（传感器通道已解耦，其他型号按 driver 接入）
* 过压、欠压、过流、过温保护
* 编码器断连识别

当前以[核心功能删减v3](docs/plans/active/2026-10-08-core-only-v3.md)为准。
轨迹、摩擦/相电阻辨识、齿槽标定/补偿、运行租约、持久启动计数和自动RTT已退役。
历史报告与归档镜像保留，但不能作为当前功能或实机通过证据。

## 注意事项

> MCU的PB8引脚为BOOT0引脚，在设计时由于引脚紧张复用成FDCAN1_RX，让程序正常启动需要使用STM32CubeProgrammer将BOOT0软件下拉。

> 当前固件默认按 **8 串常规三元锂电池（满充 33.6 V）**配置母线保护：欠压 24.0 V、过压 34.0 V、快速过压 34.5 V；启动/故障清除后重新使能要求 25.6–33.8 V。保护配置与验证见 [8S 母线电压保护](docs/guides/bus_voltage_8s.md)。这是固件配置，不代表板级耐压认证；原硬件超过 35 V 可能损坏的约束仍保留。

> 原始手册见 [Vector_User Manual.pdf](<docs/hardware/Vector_User Manual.pdf>)。其中 USB/VOFA 章节属于历史功能，不适用于当前固件；当前接口与功能缺口见 [USB 移除说明](docs/guides/usb_removal.md)。

## 参考项目

**ODRIVE**:https://github.com/odriverobotics/ODrive

**VESC**:https://github.com/vedderb/bldc

**MIT**:https://github.com/bgkatz/3phase_integrated

**AXDR**:https://oshwhub.com/lylssy/foc_driver

**dgm**:https://github.com/codenocold/dgm
