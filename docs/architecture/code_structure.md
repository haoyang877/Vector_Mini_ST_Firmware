# 代码框架与目录职责

核对基线：176f3c7e。当前运行链路见[软件架构](software_architecture.md)，
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
| app/app.c | 应用启动与前台服务，按镜像能力装配 Loader 交接 |
| app/app_protocol.c | 服务绑定、协议与领域请求映射、按请求生成反馈 |
| app/app_motor_configuration.c | 型号候选、完整 UID 提交、配套参数保存与回读、显式重启；不回写当前活动配置 |
| app/composition/app_rtt.c | 可选RTT波形的静态缓冲、初始化和监督拍读取最近的外环输出 |
| app/motor_axis.c | 单份配置、运行对象、采样/控制/标定调度 |
| app/motor_state_machine.c | 本轴五态、会话与功率许可、目标写入、外环输入与结果、反馈快照 |
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

## 修改行为时从哪里进入

| 要定位的行为 | 所属源码 | 对应离线验证 |
| --- | --- | --- |
| 启动请求、ADC 启相、STOP 与标定收尾 | [APP 协议适配](../../firmware/app/app_protocol.c) → [轴状态机](../../firmware/app/motor_state_machine.c) | [两板状态与标定用例](../../tests/unit/native/test_run_state.py) |
| 56 停机交接、57 确认、健康窗口 | [交接回包](../../firmware/communication/yg_protocol_boot.c) → [APP 交接事务](../../firmware/app/app_loader.c) → 对应 MCU 的 BootHw | [四种 APP 编排](../../tests/unit/native/test_app_orchestration.py) |
| 50–55 升级事务、租约、激活票据 | [Loader 命令分发](../../loader/runtime/main.c) → [事务核心](../../loader/core/upgrade.c) → [镜像擦写与标记](../../shared/image/image_transfer.c) | [升级内存模型](../../tests/unit/loader_upgrade_test.c) |
| 复位、CAN 完成事件、受保护的 Flash 边界 | [板级引脚与安全输出](../../loader/boards/) 和 [MCU 端口](../../loader/ports/) | [G431 运行时](../../tests/unit/native/test_loader_runtime.py)、[AT32 端口](../../tests/unit/native/test_at32_loader.py) |
| 工程基址及 GET_INFO 布局页 | [目标清单](../../shared/targets/)、[Loader 工程及 scatter](../../loader/targets/)、[APP 工程](../../projects/)、[APP 身份页](../../firmware/app/app.c) | [目标清单测试](../../tests/unit/test_loader_targets.py)、工程路径检查 |
| 标定与 Flash 记录 | [标定发布](../../firmware/app/motor_calibration.c) → [记录编码](../../firmware/parameters/parameter_record.c) → 本板 BSP Flash | [保存用例](../../tests/unit/native/test_calibration_save.py) |

启动装载和停机保存串行复用 `MotorAxis.record`。标定算法的工作区和会话另由同一轴持有。对象字段与执行上下文见[APP 目录](../../firmware/app/README.md)。

构建和验证入口为 `tools/run.py`。`check_project_layout --all` 核对全部 APP、Loader
和台架工程；PR 验证包括 Python/原生测试、分层、接口和文档检查，均不接触硬件。
