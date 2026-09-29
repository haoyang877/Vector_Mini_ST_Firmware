# 文件目录职责收口 v1

## 目标

按用户要求先完成实际文件归属与目录导航，再进行参数解耦。
保留 firmware/app、motor、services、communication、platform、common 的层级和独立产品边界。

## 调整

| 当前归属 | 新归属 | 原因 |
| --- | --- | --- |
| app 根目录的 board_config、bsp_task、foc_task、yg_protocol_owner/status_adapter | app/composition | 启动、任务入口与业务端口绑定属于组装 |
| app 根目录的 motor_axis、motor_execution、motor_maintenance、rtt_telemetry | app/runtime | 单轴调度、运行与维护协调、当前 RTT 适配属于执行组装 |
| app/motor_state_machine | 原位置 | 唯一主状态所有者，已有内部模块边界 |
| motor/position/angle_feedback | motor/feedback | 编码器反馈被所有控制模式使用，不属于位置控制器 |
| motor/identification 的 encoder_calibration_task、cogging_calibration、cogging_map、foc_cogging_calibration | motor/calibration | 装配标定与补偿表生成，区别于电机模型/摩擦辨识 |
| motor/identification 的 phase_resistance、friction_identification 及各自 task | 原位置 | 电机及摩擦辨识 |

源码与头文件相邻；Keil 保持原八组和仅列源码的规则，不把物理子目录展开成更多工程分组。
只迁移已有文件，不预建硬件/电机/应用/生产配置的空目录。四类参数拆分留下一阶段。

## 基线与不变行为

- 源码基线 ccd8d8a8f9778f526fa397207e4aab6b232268b6。
- 28 个 C/H 文件迁移，27 个内容逐字节不变；board_config 的既有初始化项仅由格式工具调整缩进。
  控制数学、状态机、协议、参数 schema12 与标定区均不变；另补齐既有接口/测试的格式差异，不改表达式。
- normal HEX 基线 SHA256：76f4153eda4f1be02d999634f9f103daa457d830abe6ba49c14ff0b2df2f0178。
- 用户 YG_ESC_APP.uvoptx 已有修改，保持其全部内容；其他工作树及现场参数不操作。
- 专用 T 测试会话先结束当前只读阶段，使用自己的冻结证据；本轮目录整理不接触硬件。

## 接线与验收

同步普通 APP、Loader 配套 APP 的源码路径与 include 路径，保持文件顺序、逐文件编译选项及分组。
同步离线路径表、当前源码夹具、当前目录导航；历史 git show 路径、历史报告和实验输出保持历史事实。
验收包括文件内容/清单一致、两个 APP 工程路径可解析、头文件不登记、架构方向不扩张、PR 离线检查，
以及 normal 主工程编译与基线 HEX 比较。不重复五工程构建，不下载、不启动运动。

## 回退与证据

迁移清单及原文件哈希保存在 outputs/directory-ownership-20260929；相反方向移动清单中的文件即可回退。
工程与工具恢复本批之前内容，不 reset/clean，不覆盖用户 uvoptx 或实验输出。
完成后记录实际检查结果、构建哈希及未完成项，再移入 completed。

## 完成记录

28 个文件已按清单归位；27 个字节一致，board_config 仅格式缩进变化，全部生产源码 token 一致。
normal / LoaderApp 工程分别为 111 / 113 个源码、8 个分组、0 个登记头文件；
源码顺序、逐文件选项、优化和链接设置相同，用户 uvoptx 哈希保持。
两工程路径检查均 0 错误，架构债保持 0。

PR 离线检查通过：格式、lint、路径、Python/原生测试、架构、接口、文档及仓库卫生。
证据：outputs/runs/directory-ownership-pr-20260929/summary.json。
normal 主工程编译 0 错误、0 警告，ROM/RAM 用量保持；HEX 与基线逐字节一致，
SHA256 仍为 76f4153eda4f1be02d999634f9f103daa457d830abe6ba49c14ff0b2df2f0178。
工程和源码清单比较见 outputs/directory-ownership-20260929/equivalence.json。

本轮没有硬件操作、ABI/参数配置重构、Loader 配套布局编译或新的实机 PASS。
下一步才按四类配置实施参数解耦，先定义所有者与权限再调整存储契约。
