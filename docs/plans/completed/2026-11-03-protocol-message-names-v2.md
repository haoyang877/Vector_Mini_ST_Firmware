# 协议消息宏名简化 v2

## 目标与不变项

简化其余 19 个线上消息宏，同步所有源码和测试引用。
沿用用户已修改的电机消息名；Boot、升级消息去掉 `TYPE`。
参数和任务的简写名与内部枚举冲突，消息使用动词在前的名称，保留内部枚举原值。
已有五个只读消息名保持不变；参数、任务内部操作枚举和载荷字段常量保持不变。
本次只改变 C 标识符及必要排版，不改变消息编号、报文、分派顺序、故障处理或接口布局。
不新增旧名兼容别名。

| 业务 | 编号 | 新宏名 |
| --- | --- | --- |
| 升级 | 50 | `YG_PROTOCOL_UPGRADE_BEGIN` |
| 升级 | 51 | `YG_PROTOCOL_UPGRADE_WRITE` |
| 升级 | 52 | `YG_PROTOCOL_UPGRADE_FINISH` |
| 升级 | 53 | `YG_PROTOCOL_UPGRADE_STATUS` |
| 升级 | 54 | `YG_PROTOCOL_UPGRADE_ACTIVATE` |
| 升级 | 55 | `YG_PROTOCOL_UPGRADE_ABORT` |
| Boot | 56 | `YG_PROTOCOL_BOOT_ENTER` |
| Boot | 57 | `YG_PROTOCOL_BOOT_CONFIRM` |
| 电机 | 101 | `YG_PROTOCOL_MOTOR_ENABLE` |
| 电机 | 107 | `YG_PROTOCOL_MOTOR_CLEAR_FAULT` |
| 电机 | 110 | `YG_PROTOCOL_MOTOR_STOP` |
| 电机 | 116 | `YG_PROTOCOL_MOTOR_CONTROL` |
| 电机 | 117 | `YG_PROTOCOL_MOTOR_START` |
| 参数 | 120 | `YG_PROTOCOL_READ_PARAMETER` |
| 参数 | 121 | `YG_PROTOCOL_WRITE_PARAMETER` |
| 参数 | 122 | `YG_PROTOCOL_SAVE_PARAMETER` |
| 任务 | 160 | `YG_PROTOCOL_START_JOB` |
| 任务 | 161 | `YG_PROTOCOL_GET_JOB_STATUS` |
| 任务 | 162 | `YG_PROTOCOL_CANCEL_JOB` |

## 基线、兼容与回退

使用本次替换前工作树作为基线，保留用户已有的通信层等改动。
电机头文件已采用用户的新名，引用尚未同步，因此先保存源码基线，不将旧构建当作本次证据。
修改前文件、映射、消息编号与构建证据保存在 `outputs/protocol-message-names-v2-20261103/`。
源码调用者须同步使用新宏名，线上协议及主机工具的数值常量保持兼容。
回退仅逆向替换本次宏名并撤销相应排版，不覆盖用户其他修改。

## 验收

- 全仓核对旧标识符引用和新名冲突，确认 19 个消息编号及内部操作枚举不变。
- 前后源码对照仅包含宏名与空白变化。
- 完整 PR 验证覆盖参数、任务、电机、Boot、升级及固定报文向量。
- 双板 APP、STM32 LoaderApp、独立 Loader 构建，记录资源用量和产物哈希。
- 所有验证离线执行，不连接硬件或烧录。

## 结果

已同步 14 个文件中的 66 处宏定义或引用，旧消息宏名无残留。
沿用用户已修改的五个电机宏名，电机头文件保持原样。
全部 19 个消息编号逐项核对一致；源码对照仅有标识符和排版变化，内部操作枚举未变。
核对证据见 `outputs/protocol-message-names-v2-20261103/source-check.json` 及同目录编号清单。

完整 PR 通过：单元、集成、原生回归，以及格式、lint、布局、架构、接口、文档和卫生检查。
证据：`outputs/runs/protocol-message-names-v2-20261103/summary.json`。
双板 APP、STM32 LoaderApp 和独立 Loader 四个目标均为 0 错误、0 警告；
资源用量、日志和产物哈希保存在 `outputs/protocol-message-names-v2-20261103/builds/summary.json`。
没有可直接编译的旧名称基线，因此不作 HEX 前后相同声明；编号及源代码逻辑已单独核对。
未连接硬件或烧录。
