<img src="./docs/assets/project/logo.png" width="600"/>

## 代码目录

```text
firmware/                         电机 APP
  app/                            启动组装、模式与任务调度
  motor/                          FOC、位置控制、轨迹、辨识、保护
  services/                       参数与遥测
  communication/                  CAN 接入与协议
  platform/                       硬件接口、STM32G4 BSP/端口/CubeMX
  common/                         通用数学与内存工具
  third_party/                    SEGGER RTT
loader/                           独立 Loader 工程入口（后续实现）
host_app/                         PC 上位机工程入口（后续实现）
shared/                           APP/Loader/PC 契约规划
tools/                           build、flash、bench、analysis
tests/                           unit、integration、hil
docs/                            architecture、protocols、guides、hardware、reports
outputs/                         构建、测试与分析产物（忽略提交）
```

Keil 工程位于 `firmware/platform/stm32g4/cubemx/MDK-ARM/`，
普通/HIL 构建产物输出到 `outputs/build/keil/`。

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
绑定的证据。`quick` 与 `pr` 完全离线；`release` 只构建、不烧录；HIL 必须显式选择工作台、
电机配置、镜像哈希和场景。项目协作约定见 [AGENTS.md](AGENTS.md)，架构地图见
[ARCHITECTURE.md](ARCHITECTURE.md)，文档入口见 [docs/README.md](docs/README.md)。

代码风格和可读性规则见 [STYLE.md](STYLE.md)。新 C 模块和 Python 命令应从持续接受
CI 检查的 [templates](templates/README.md) 示例开始，再按模块职责进行重命名和裁剪。

详细目录与验证结果见 [代码框架与目录规范](docs/architecture/code_structure.md)，
后续 Loader 与 PC 边界见 [Loader](loader/README.md)、[上位机](host_app/README.md)、
[共享契约](shared/README.md)。

## 功能介绍

​        Vector 驱动器是一款中小型尺寸，基于矢量控制的永磁同步电机驱动，适用于轮毂电机，关节电机等。24V母线电压下，实测最高支持约400W的持续功率输入（需散热片）。内部算法可实现基础的电流，速度，位置闭环，同时具有一定的参数辨识能力。目前仅支持有感运行，板载了一块TLE5012B绝对式编码器，同时支持外部SPI编码器信号输入。当前应用通过 FDCAN 接收控制命令和回传状态，保留 UART 外设供后续开发。MCU USB CDC 命令、遥测和协议栈已移除，PC 端 USB-CAN 分析仪仍可使用。FDCAN数据段最高波特率5Mbps，可通过外部主控向总线发送报文以实现一对多控制。驱动器具有母线过压和欠压，相线过流，过温等保护，最大程度地保障驱动能够稳定运行。

## 软件已有功能

模式3轴配置：[roll / pitch名称、范围与Flash持久化](docs/guides/motor_axis_profiles.md)。

底盘轮子限速：[节点 1/2、10 cm 轮径、1 m/s 对应 190.986 rpm](docs/guides/chassis_wheel_speed_limit.md)。

* 基础有感FOC算法 电流 速度 位置 可控
* 模式3连续加速度位置伺服与模式18电流域位置阻抗控制（[优化过程](docs/reports/2026-09/mode3_optimization_20260908.md)、[参数适配](docs/guides/mode3_parameter_guide.md)、[执行测试](docs/guides/mode3_test_plan.md)）
* 磁编码器偏心补偿
* 电机相电阻+dq轴电感+永磁体磁链辨识
* 空载正反向库仑/粘性摩擦辨识（见 [docs/friction_identification.md](docs/guides/friction_identification.md)）
* FDCAN通信控制+超时保护
* 支持绝对式SPI编码器 TLE5012B，MT6816, MT6701
* 过压、欠压、过流、过温保护
* 编码器断连识别

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
