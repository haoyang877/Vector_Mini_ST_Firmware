# 代码框架与目录职责

更新：2026-09-29。此页描述当前目录，不用历史报告代替当前实现和验证结果。
产品依赖规则见根目录 [ARCHITECTURE.md](../../ARCHITECTURE.md)。

## 独立产品与共享契约

| 目录 | 职责 |
| --- | --- |
| [firmware](../../firmware/README.md) | 电机驱动 APP，负责控制、运行管理、通信业务与硬件适配 |
| [loader](../../loader/README.md) | 独立升级与启动产品，不把核心实现链接进 APP |
| [host_app](../../host_app/README.md) | 独立主机应用边界，外部主机项目不复制进 APP |
| [shared](../../shared/README.md) | APP/Loader 真正共用的协议 codec、CRC、镜像、启动邮箱和目标契约 |
| tests / tools | 离线夹具、构建与实机工具；硬件操作不进入默认验证 |
| outputs | 忽略的生成产物和证据，旧记录不能作为本轮 PASS |

## APP 实际文件归属

| 目录 | 内容与边界 |
| --- | --- |
| app/composition | board_config：启动/端口绑定/启动交接；bsp_task、foc_task：任务入口；yg_protocol_*_adapter：业务所有者与协议之间的组装 |
| app/runtime | motor_axis：唯一电机对象组装及调度；motor_execution：控制执行与外环协调；motor_maintenance：维护协调；maintenance_workspace：唯一固定维护工作区；rtt_telemetry：当前 RTT 输出适配 |
| app/motor_state_machine | 唯一主状态所有者、实际动作与内部转换规则；内部头不作为其他模块接口 |
| motor/foc | 电流、速度、无感控制计算、采样换算与补偿应用 |
| motor/feedback | angle_feedback：编码器采样、角度校正、多圈与速度估计；所有模式共同使用 |
| motor/position | position_cascade 与其配置：位置伺服控制 |
| motor/trajectory | 平滑轨迹计算 |
| motor/math | control_math：既有数学、角度与滤波计算，无硬件或 APP 依赖 |
| motor/calibration | 编码器与齿槽标定、标定任务适配及补偿表记录计算 |
| motor/identification | 相电阻、摩擦辨识的纯计算及任务适配 |
| motor/protection | 电机故障与母线保护策略 |
| motor 根目录头文件 | 电机域值类型、模式、状态、任务结果及现有模型/启动 profile，不放应用运行实例 |
| services/lifecycle | 控制/维护请求、readiness 值与停机服务，不再实现另一套状态机 |
| services/parameters | schema12 装载/导出、纯标量校验、轴解析及应用/设备默认值；不拥有通信节点变量 |
| services/telemetry | 只读电机状态值契约 |
| communication/can | CAN 接入、队列、中断接入、链路及端点调度 |
| communication/protocol | APP 业务 Type 的解析、回复与服务适配；纯帧 codec 归 shared/protocol |
| platform/api | 无 STM32 类型的硬件能力、单位、失败及执行上下文契约 |
| platform/stm32g4/bsp | 芯片/板卡原语与驱动 |
| platform/stm32g4/ports | 板级、CAN、编码器、电机、时间及启动能力的实现 |
| platform/stm32g4/cubemx | 生成入口、Core、HAL/CMSIS、IOC 和 Keil 工程；不搬业务到生成区 |
| third_party | 保持原样的 SEGGER RTT 等外部源码 |

源码与对应头文件相邻。目录按职责分组，不为单个函数新建目录，不创建空的未来配置目录。
RTT 目前读取外环就绪事实并调用 SEGGER，因此仍归 app/runtime；本轮不通过搬进 services 引入反向依赖。
数据流和所有权见 [APP 命名与所有权](app_naming_v1.md)、
[主状态机契约](motor_state_machine_v1.md)。

## 参数与运行状态

硬件默认值归 platform/api，电机模型默认值归 motor，应用/设备默认值归 services/parameters。
MotorAxis 私有运行对象内的 model/application/device 子对象持有单份活动参数，
命令目标、模式、故障与测量留在易失状态中。
schema12 保持现有混合存储布局，由装载/导出边界映射，不能当作控制对象使用。
参数所有者给启动组合层传节点值，通信不再作为参数节点的存储所有者。
参数记录、齿槽候选和编码器标定复用一份 6 KiB 固定联合工作区，APP 管理占用与归还。
编码器任务显式借用采样对象，取消/完成清除引用；实际归还后 APP 才允许复用。
没有 heap 分配器、空闲链表或三份常驻大数组。
具体读写规则与持久层尚未拆开的兼容限制见[参数所有权](parameter_ownership_v1.md)。

## 工程与工具接线

主工程为 firmware/platform/stm32g4/cubemx/MDK-ARM/YG_ESC_APP.uvprojx。
Loader 配套 APP 为同目录 Vector_Mini_ST_LoaderApp.uvprojx；两者复用同一业务源码。
普通 APP 独立从 Flash 起始地址链接；只有配套工程使用 Loader APP 槽，不能混称两者布局。

Keil 保持七组：App、Motor、Services、Communication、Platform、Shared、Vendor。
物理子目录不扩展为更多 Keil 分组，工程树只登记源文件，不登记 .h。
用户逐文件优化、源码顺序与调试配置保留。CubeMX Core/Drivers 内部相对位置不变。
VS Code 按实际目录浏览；原生测试 include 路径统一由 tools/project_paths.py 提供。

目录迁移映射见 [path_migration.json](path_migration.json)，
本轮目标、回退和证据见[目录职责收口记录](../plans/completed/2026-09-29-directory-ownership-v1.md)。
历史路径转换夹具和 git show 引用保持其原版本路径，当前测试输入更新为实际目录。

## 校验与历史信息

目录整理核对源文件内容不变、实际工程路径、架构方向与 normal HEX；不替代实机功能验收。
历史目录迁移、USB 移除、全局状态收口和去重证据分别保存在 docs/plans 与 docs/reports。
当前默认验证不会烧录或操作电机；实机测试由专用会话在明确台架场景下执行。
