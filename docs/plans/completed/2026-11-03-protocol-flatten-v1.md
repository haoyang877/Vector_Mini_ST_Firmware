# 通信调用链压平 v1

## 目标与边界

按用户“最简单、太多层级”的要求，删除没有独立职责的协议框架层。
线上版本、命令编号、字段、CRC、地址和回复语义保持不变；不烧录或驱动电机。

基线为 `the-system-refactor` 的工作区（含已有未提交修改），开始时 HEAD 为
`6c355b96`。本地源码副本和哈希保存在 `outputs/protocol-simplification/baseline/`。
不回滚或覆盖其他工作；此前台架移除、消息宏命名及 BSP 改动继续保留。

## 实施

- 删除通用 Endpoint、命令注册表和动态路由；Link 直接按命令分发、构造并排队回复。
- Link 统一初始化、启动、中断与前台服务，删除 Runtime 的重复公开入口。
- 删除独立 motor adapter，应用协议适配直接把 STOP/启动/目标交给既有业务所有者。
- CAN FD 编解码及有界队列保留为具体工具，不作为可配置传输框架。
- APP 只发送现有单帧回复，删除无真实调用方的任意长消息发送接口。
  APP 的合法请求均可单帧，删除 GET_INFO 无合法载荷可达的重组路径；
  Loader 自己的升级分片、CRC 和坏帧拒绝保留。

## 验收与回退

黄金向量与现有命令/参数/标定测试保持通过；补充直接链路的分片、坏帧、
拥塞下不重复执行业务和升级 READY 实际发送确认验证。
运行 PR 验证及双板编译；比较 Flash/RAM 与编译告警。
不宣称硬件时序或实机验证通过。

回退仅恢复本计划列出的改动，依据开始时工作区副本，不对整个仓库执行 reset。

## 完成与证据

通信目录从 22 个 C/头文件、4101 行变为 18 个文件、3247 行，净减 854 行。
通用 Endpoint、Runtime 包装、动态注册、motor adapter、只读 provider 和 APP 分片缓存已删除。
新增消息宏名原样保留；内部电机操作使用 OP 前缀，避免与消息宏重名。

| 工程 | Flash 前→后（B） | RAM 前→后（B） |
| --- | ---: | ---: |
| STM32G431 APP | 61364→58936 | 19488→17544 |
| AT32F456 APP | 59572→56744 | 23648→21704 |
| STM32G431 LoaderApp | 63476→61024 | 19528→17584 |

三个工程均以当前 Keil 重建前后源码，0 错误、0 警告。
基线为开始时工作区副本，不是其他分支或过期产物。
资源日志、镜像哈希及对比见 `outputs/protocol-simplification/summary.json`。
四组 APP（两板×普通/Loader）启动、失败、看门狗及交接事件轨迹与基线逐字一致。

修改前 PR：`outputs/runs/protocol-simplification-baseline-venv-20261103/summary.json`。
修改后完整 PR：`outputs/runs/protocol-flat-v2-20261103/summary.json`，全部 PASS。
黄金向量、坏帧/分片拒绝、拥塞不重复执行、STOP 真实确认、READY 超时和发送完成均覆盖。
当前未提交工作区没有生成正式发布包；已执行的 release 检查及其限制单独保留在
`outputs/runs/protocol-flat-v2-release-final-20261103/summary.json`。
发布检查中的 doctor、格式、lint、工程路径、全部测试、分层、接口、文档及卫生检查均 PASS；
仅“源码必须已提交”的发布门禁不满足，因此没有执行该发布流程的自动构建或生成发布清单。
上述三个工程已单独完成当前源码构建，不把它们冒充正式 release PASS。
没有连接硬件或烧录。
