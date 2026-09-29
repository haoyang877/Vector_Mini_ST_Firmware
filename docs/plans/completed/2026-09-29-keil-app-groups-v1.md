# Keil APP 职责分组 v1

> 历史分组记录。当前按用户要求改为八组并隐藏全部头文件，见
> [精简分组 v2](2026-09-29-keil-app-groups-v2.md)。下文保留 v1 的原始结果。

日期：2026-09-29。基线：`7df4dd360956eca59bee8237fa8cafbcbea884ef`。
状态：工程导航整理；不作为功能、容量或发布验收。

## 目标与不变边界

将普通 APP 与 Loader 配套 APP 的混合大组改为职责分组，APP 所有者优先显示，
关键模块的头文件与实现相邻，硬件实现与厂商文件集中在下部。
保留原编译源集合、相对路径、目标配置、逐文件优化/编译选项和调试连接设置。
只增加已有头文件的导航项，不新增编译单元，不移动实际源码目录。

现有工程逐文件节点整体搬移；主工程 `.uvoptx` 仅同步树分组及文件索引，
目标调试设置与已有文件记录保留。修改前输入备份在忽略的
`outputs/keil_grouping_20260929/before/`。

## 分组与归属

| 分组 | 职责 |
| --- | --- |
| `00_App` | 启动组合、motor_axis、状态机、执行、维护、调度与业务适配 |
| `10_Motor_FOC` | FOC、采样、速度环、观测器与补偿 |
| `11_Motor_Position` | 角度反馈、位置级联、轨迹 |
| `12_Motor_Calibration` | 标定与辨识 |
| `13_Motor_Protection` | 故障与恢复规则 |
| `14_Motor_Contracts` | 电机类型、模式、任务结果及硬件/启动 profile 等头文件 |
| `20_Service_Lifecycle` | 纯生命周期规则、停止服务与命令值契约 |
| `21_Service_Parameters` | 活动值、默认参数、schema 与轴 profile |
| `22_Service_Telemetry` | 状态快照 |
| `30_Comm_CAN` | 运行时、收发、中断接入、队列与端点 |
| `31_Comm_Protocol` | 路由、业务载荷与适配器 |
| `40_Platform_Ports` | STM32 硬件能力实现与 platform API 头文件 |
| `41_Platform_BSP` | 板级底层驱动 |
| `50_Shared_Protocol` | APP/Loader 共用纯编解码 |
| `51_Shared_Boot` | 仅配套 APP：RAM 邮箱与 APP 镜像描述符 |
| `60_Common` | 内存池、CRC、基础计算 |
| `90_CubeMX_Core` | main、中断、外设初始化、启动汇编 |
| `91_STM32_HAL` | HAL/LL 厂商源码 |
| `92_CMSIS` | 系统初始化与 CMSIS |
| `93_SEGGER_RTT` | 第三方 RTT |

`00_App` 按 `board_config → motor_axis → motor_state_machine → motor_execution →
motor_maintenance` 排列，再放调度、遥测与业务适配。Keil 树默认展开 APP，
其余组默认折叠；不把文件夹整齐当作内部解耦完成的证明。

Loader 配套工程继续从普通工程生成，继承其分组；生成器将启动共享组插入编号顺序，
避免后续生成恢复旧 `APP Boot Entry` 大组。现有配套工程只改分组，不重建其目标设置。

## 接受与回退

编辑时对比原节点与目标设置，保留原编译输入及逐文件选项，记录在
`outputs/keil_grouping_20260929/edit_manifest.json`。这是工程输入核对，不是编译证明。
分组改变会改变工程遍历顺序，不承诺二进制地址、字节或资源占用完全相同。
本轮不运行测试、构建、lint/format 或硬件操作，统一验证后续进行。

实际整理结果：普通 APP 为 19 组，配套 APP 为 20 组，差异仅为共享启动组。
原工程 113/115 个文件节点及其逐文件设置保留，新增 87/88 个已有头文件导航项；
没有新增编译源。普通工程树缓存同步这套分组，目标调试配置保持修改前内容。

通过回退本次独立提交恢复导航；用户后续编辑不得被覆盖。Keil 已打开旧树时，
须重新加载工程，避免旧窗口保存时把外部整理覆盖。CubeMX 再生成后也须保留本分组。
