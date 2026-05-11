<img src="./images/logo.png" width="600"/>

## 功能介绍

​        Vector 驱动器是一款中小型尺寸，基于矢量控制的永磁同步电机驱动，适用于轮毂电机，关节电机等。24V母线电压下，实测最高支持约400W的持续功率输入（需散热片）。内部算法可实现基础的电流，速度，位置闭环，同时具有一定的参数辨识能力。目前仅支持有感运行，板载了一块TLE5012B绝对式编码器，同时支持外部SPI编码器信号输入。通信外设提供了USB，UART，FDCAN三种，USB和FDCAN具备自主控制协议，USB FS 波特率12Mbps，可以通过VOFA+上位机以字符串的形式向下位机发送指令，即插即用。FDCAN数据段最高波特率5Mbps，可通过外部主控向总线发送报文以实现一对多控制。驱动器具有母线过压和欠压，相线过流，过温等保护，最大程度地保障驱动能够稳定运行。

## 软件已有功能

* 基础有感FOC算法 电流 速度 位置 可控
* 位置控制梯形速度轨迹规划
* 磁编码器偏心补偿
* 电机相电阻+dq轴电感+永磁体磁链辨识
* FDCAN通信控制+超时保护
* USB通信控制
* 支持绝对式SPI编码器 TLE5012B，MT6816, MT6701
* 过压、欠压、过流、过温保护
* 编码器断连识别

## 注意事项

> MCU的PB8引脚为BOOT0引脚，在设计时由于引脚紧张复用成FDCAN1_RX，让程序正常启动需要使用STM32CubeProgrammer将BOOT0软件下拉。

> 驱动供电电压为**13V-30V**，外部电源超过35V运行可能会引发器件过压损坏。

> 请按照**Vector_User Manual.pdf**完成各项参数的配置及校准，错误设置将会导致电机不正常运行。

## 参考项目

**ODRIVE**:https://github.com/odriverobotics/ODrive

**VESC**:https://github.com/vedderb/bldc

**MIT**:https://github.com/bgkatz/3phase_integrated

**AXDR**:https://oshwhub.com/lylssy/foc_driver

**dgm**:https://github.com/codenocold/dgm

