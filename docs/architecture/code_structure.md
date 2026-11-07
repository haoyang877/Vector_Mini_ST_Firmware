# 代码框架与目录职责

更新：2026-11-06。当前运行链路见[软件架构](software_architecture.md)，
工程约束见根目录 [ARCHITECTURE.md](../../ARCHITECTURE.md)。

## 产品边界

| 目录 | 职责 |
| --- | --- |
| [firmware](../../firmware/README.md) | 两板共用单轴电机 APP |
| [loader](../../loader/README.md) | 独立的升级和启动产品 |
| [host_app](../../host_app/README.md) | 独立主机应用 |
| [shared](../../shared/README.md) | 真实跨产品共用的协议、CRC、镜像与启动契约 |
| projects | 板级 main、启动汇编、Keil/scatter 和 CubeMX 输入 |
| tests / tools | 离线验证、构建和显式硬件台架工具 |
| outputs | 忽略的生成产物与证据，不作为源码或当前验收结论 |

## 固件文件归属

| 路径 | 责任 |
| --- | --- |
| app/app.c | 应用启动、前台服务、快慢调度、看门狗和 Loader 交接 |
| app/app_protocol.c | 服务绑定、协议与领域请求映射、最小状态刷新 |
| app/composition/app_rtt.c | 可选RTT波形的静态缓冲、初始化和监督拍读取最近的外环输出 |
| app/motor_axis.c | 单份配置、运行对象、采样/控制/标定调度 |
| app/motor_lifecycle.c | 状态、启停、三类标定、会话代际、故障和功率许可 |
| app/motor_calibration.c | 标定快拍、关相发布、停稳计时和保存 |
| app/app_loader.c | 进入 Loader 的停机票据、超时和镜像确认 |
| motor/control | 位置/速度外环、控制模式、PI、FOC 与无感启动 |
| motor/sensing | 电流零偏、物理量读取、母线滤波和 MCU 温度 |
| motor/feedback | 编码器方向/LUT/零位、位置速度估计和磁链观测 |
| motor/calibration | 单一实现文件内的LUT/电零位任务与私有计算，公共头只暴露任务入口 |
| motor/protection | 故障发布与恢复；保留热模型，当前 APP 不调度 |
| motor/math | 有实际调用的确定性数学函数和坐标类型 |
| communication | yg_protocol.c 负责收发和普通业务，yg_protocol_boot.c 负责 Loader 通信，共用 yg_protocol.h |
| parameters | 默认值、值校验、轴记录与 schema14 转换 |
| config | config.h、board_config.h、motor_profile_config.h、application_config.h |
| platform/bsp/include | 八个公共功能头，只使用项目自有类型 |
| platform/bsp/bsp_internal.h | 初始化、采样启停和台架专用声明 |
| platform/bsp/mcu/{stm32g4,at32f456} | 对应 MCU 的初始化、硬件读写、换算和 IRQ |
| platform/bsp/mcu/stm32g4/generated | CubeMX 生成区，手改限 USER CODE |
| platform/sdk / third_party | 厂商和第三方源码，不重排或格式化 |

BSP 根目录没有公共转发源文件，也没有独立 devices 层。
两板通过同一组 BSP 接口调用各自实现，硬件资源映射只供对应 MCU 使用。
ADC、监督和SysTick向量统一放在各板projects目录的main.c，由各BSP MCU目录的interrupt_map.h映射名称；
事件确认、清标志、HAL调用和时基仍由BSP负责，CAN及故障异常入口保持原位置。
文件与同职责头文件相邻；不为未来功能创建空目录或注册框架。

## 数据和执行

MotorAxis 持有 MotorConfiguration、MotorControlState、FocState、编码器及观测器。
配置分 model/application/device；LUT 和零位只存一份，位于 device.encoder。
FocState 直接持有电流、电压、温度及控制量，测量没有第二份所有者。
生命周期属于 g_app.axis.lifecycle。五态迁移只读取状态和事件，在 motor_lifecycle.c 内执行。

观测器状态与接口归 [flux_observer.h](../../firmware/motor/feedback/flux_observer.h)，
无感启动阶段与状态归 [sensorless_startup.h](../../firmware/motor/control/sensorless_startup.h)；
编码器标定直接包含所使用类型的所属头文件。两模块不另设隐藏的可变状态副本，
首次初始化、重入及复位保留项以各自公共头的契约为准。

ADC 快环 20 kHz，监督 2 kHz；前台处理协议与 LUT 发布。
启动零偏在关相 IDLE 累计 2048 拍，成功后一次发布，不产生参数修订号。
编码器标定使用独占的 6144 字节静态工作区，记录装载使用另外的启动静态缓冲。
没有堆分配，也没有在线保存任务与工作区复用协议。

构建和验证入口为 `tools/run.py`。`check_project_layout --all` 核对全部 APP、Loader
和台架工程；PR 验证包括 Python/原生测试、分层、接口和文档检查，均不接触硬件。
