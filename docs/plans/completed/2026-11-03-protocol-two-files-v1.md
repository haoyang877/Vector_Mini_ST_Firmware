# 通信收敛为两个文件 v1

## 目标与基线

按用户要求继续合并并删除无用代码：`firmware/communication/` 只保留
`protocol.c` 和 `protocol.h`，没有子目录或转发头。
以本轮开始的重构工作区为基线，保存于 `outputs/protocol-two-files/baseline/`。
原工作区改动保留；回退按该副本逐文件恢复，不执行仓库整体 reset。

## 改动

- 合并 CAN、命令及服务定义，内部编解码和状态不再作为模块间接口暴露。
- 删除没有任何生产调用方的 APP RTT 遥测实现；台架仍使用的 RTT 配置移入 config。
- 删除从未绑定到 APP 的参数后端、可写结果模型和参数服务接口。
  保持当前参数请求的字段验证、主站权限及 UNSUPPORTED 回复。
- 按用户追加要求缩短文件名、类型、宏和接口名：公开接口用 Protocol_*，内部用 decode_*/handle_*/send_*。
- 删除重复结果码包装和内部发送回调，固定消息直接处理。
- 更新真实调用方、所有工程输入、测试、架构约束和文档，不保留旧路径兼容层。

## 保持与验收

线上报文字段、命令编号、CRC、有效请求回复、STOP 确认及升级发送确认保持不变。
APP 仍无分片，独立 Loader 仍使用共享分片。中断及前台有界预算不变。
不接触设备、不烧录。

修改前三个 APP 工程已重新构建，日志为 `outputs/protocol-two-files/before-*.log`。
验收要求：完整 PR、协议黄金向量、权限/坏帧/队列/停止/升级回归、
两个 MCU 和 LoaderApp 构建通过，并记录资源与镜像哈希。
发布验证不擅自提交既有工作区；正式发布要求按实际门禁结果报告。

## 完成记录

通信目录实际只保留 protocol.c / protocol.h，无子目录；18→2 个 C/头文件，3247→2209 行，
净减 1038 行。无生产调用方的 APP RTT 源码/头文件删除，仍有台架调用的 RTT 配置迁入 config。
未绑定参数后端删除后，参数字段检查、权限与拒绝报文字节保持原样。

169 个文件内或应用接口标识符缩短，公开入口为 Protocol_Init/Start/Process、
Protocol_RxInterrupt；请求/回复为 ProtocolCommand/ProtocolReply，消息常量使用 PROTOCOL_*。
内部函数均为 static，不保留旧名字转发；公共共享协议的 ABI 名称和线上字节不变。

完整 PR：`outputs/runs/protocol-two-files-20261103/summary.json`，全部 PASS。
四组 APP（两板×普通/Loader）启动、故障和升级交接轨迹与基线逐字一致。
主站/非主站、参数错误字段/长度、CRC、队列拥塞、STOP 关相和 READY 发送确认均通过原生测试。

| 工程 | Flash 前→后（B） | RAM 前→后（B） |
| --- | ---: | ---: |
| STM32G431 APP | 58936→58096 | 17544→17536 |
| AT32F456 APP | 56744→55576 | 21704→21696 |
| STM32G431 LoaderApp | 61024→60200 | 17584→17576 |

三个 APP 均 0 错误、0 警告；基线及当前构建日志、镜像哈希与资源计算保存在
`outputs/protocol-two-files/summary.json`。本轮没有提交、烧录或宣称正式发布。
