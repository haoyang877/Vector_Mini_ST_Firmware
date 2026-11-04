# 通信命令直接调用 v1

## 目标与不变项

删除 `call_motor` 与 APP 的二次操作分发 `handle_lifecycle`，按启动、目标、停机和清错
直接绑定业务函数。报文函数命名为 `handle_start_request`，APP 执行函数为 `start_motor`。
顺带删除 `call_job` 和状态回复转发；不增加生产文件或目录。

按后续明确要求，将固定的电机/标定/Loader 回调、状态来源地址、停机端口、收发队列
全部静态初始化，删除 `Protocol_Bind*`、状态来源绑定、`AppProtocol_Init`、
`ProtocolDevice_Init` 和 `queue_init`。节点号和 UID 等确实来自运行时的数据仍需赋值。

保留权限、载荷、模式校验，保持 STOP 与 ENABLE=0 的停机语义、ENABLE=1 不支持、
目标序号去重、结果映射、标定和 Loader 行为。保留队列、ISR/前台边界和发送预算。
通信不直接操作硬件或绕过 MotorLifecycle 的准入检查。

## 基线和验收

通信与适配器基线为本轮开始时的相关文件，副本及哈希见 `outputs/protocol-direct/baseline/`。
后续扩大范围的 APP 文件另外保存了本轮变换前版本；其他并行修改原样保留。
基线已通过上一轮完整 PR 和三个 APP 构建；本轮另存当前的定向测试与构建结果作比较。
验收包含真实 APP 启停/目标/标定结果、协议黄金向量、拥塞与升级发送、完整 PR 和三个 APP
重编译，记录资源变化及镜像哈希。不连接硬件，不声称完成硬件时序验收。
回退只恢复上述相关副本，保留其他未提交工作。

## 结果

协议静态化及直接调用已完成，仍为三个实现文件和一个头文件。
通信四个文件由 2327 行降至 2125 行，净减 202 行。

启动链现在是 `handle_start_request → Protocol_Motor.start（start_motor）→
MotorLifecycle_RequestEnable`。目标、停机、清错同样直接进入对应 APP 函数。
`Protocol_Motor`、`Protocol_Jobs`、`Protocol_Boot` 和两个快照地址都是只读静态绑定；
队列存储及默认状态由静态初始化完成。没有运行时注册或二次操作分发，
四个电机回调也不再检查空绑定；独立 APP 的 Loader 能力仍由静态空表区分。

已删除 `call_motor`、`call_job`、`handle_lifecycle`、`handle_status`、`Protocol_Bind*`、
`Protocol_SetMotorStatus`、`Protocol_SetDeviceStatus`、`AppProtocol_Init`、
`ProtocolDevice_Init`、`Protocol_IsIdle` 和 `queue_init`；电机请求的冗余操作枚举也已删除。
快照仍由 APP 更新，协议只读；实际节点号、UID、Loader 可用性仍按运行时值设置。

## 验证结果及限制

- 128,592 组请求的实际 CAN 输出、业务请求值及队列余量前后逐项一致，
  见 `outputs/protocol-direct/comparison.json`。比较适配只消除了旧分发接口的操作标签，
  保留请求和回复字节；ENABLE=1 按实际产品规则返回不支持。
- 真实 APP 控制/标定及两板生命周期测试的前后输出哈希一致；协议核心、黄金向量、
  拥塞不重执行、STOP 和升级确认/超时回归通过。
- 四组 APP 健康与 Loader 交接轨迹一致。对照夹具去掉了已不存在的“注册失败”注入点，
  不把删除的初始化调用计入事件轨迹；运行期故障、健康、关相和交接检查仍保留。
- 三个 APP 全量构建均为 0 错误、0 警告。

| 工程 | Flash 前→后（B） | RAM 前→后（B） |
| --- | ---: | ---: |
| AT32F456 | 55468→54760 | 21704→21600 |
| STM32G431 | 57956→57300 | 17544→17440 |
| STM32G431 LoaderApp | 60092→59244 | 17584→17480 |

构建期间其他上下文同时修改了 ADC 和 main；资源表是两次完整工作区的实测值，
不能把全部差额单独归因于本次协议改动。未进行实机时序测试。

整仓 PR 检查已执行，结果为 FAIL：ADC 文件格式、双板 main 模板不一致、Keil 通信文件
排列顺序不一致。未改动这些并行变更，也未放宽检查。全部原生测试、集成测试及
lint/工程路径/架构/接口/文档/卫生检查通过。
完整日志见 `outputs/runs/protocol-direct-static-20261103/summary.json`。
源码与镜像哈希、测试对照见 `outputs/protocol-direct/summary.json`。未提交、未烧录。
