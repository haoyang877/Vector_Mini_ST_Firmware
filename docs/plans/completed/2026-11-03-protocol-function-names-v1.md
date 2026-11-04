# 通信函数按用途命名 v1

将含糊的 metadata、pack/unpack、process、flush 和未说明对象的编解码函数，
改为能表达 CAN 帧格式、命令载荷、回复或队列动作的名称。不新增包装或实现文件。
同步真实调用方、测试夹具及唯一协议说明。

本次只改标识符及对应说明：线上字节、CRC、权限、队列、状态、回调顺序和时间预算不变。
基线、完整改名表在 `outputs/protocol-names/`。验收反向归一化后的 C 词法一致、
协议与真实 APP 原生回归，以及三个 Keil 工程前后构建；不连接硬件。
回退按基线逐文件恢复，保留其他未提交改动。

## 结果

已完成 46 个函数标识符的改名，维持三个实现文件和一个头文件。
跨文件接口保持 `Module_Action`，私有函数保持 `snake_case`，不添加兼容转发层。

主要对应关系：

| 原名 | 现名 | 含义 |
| --- | --- | --- |
| valid_metadata | is_valid_can_format | 检查 CAN 标志、长度和 ID |
| padded_length | align_can_frame_length | 补齐 CAN FD 物理帧长 |
| pack_frame / unpack_frame | encode_can_frame / decode_can_frame | 整帧编码 / 解码 |
| process_frame | handle_can_request | 解码、分发请求并准备回复 |
| ProtocolMotor_Flush | ProtocolMotor_QueueFeedback | 将待发反馈放入发送队列 |
| Protocol_Send | Protocol_QueueMessage | 将协议消息编码后入发送队列 |
| send_one | send_next_can_frame | 向硬件提交下一帧 |

完整映射和相关文件的原始哈希见 `outputs/protocol-names/renames.json` 与 `manifest.json`。
还原标识符后，全部相关 C/H 的有效词法及 Python AST 一致；说明性注释和排版不参与比较。
两个真实 APP 控制/生命周期测试的前后输出哈希一致。
核心协议、黄金向量、链路、完整 PR 检查全部通过，见
`outputs/runs/protocol-function-names-20261103/summary.json`。

三个 APP 前后全量编译均为 0 错误、0 警告；总 Flash/RAM 不变：

| 工程 | Flash（B） | RAM（B） |
| --- | ---: | ---: |
| AT32F456 | 54792 | 21600 |
| STM32G431 | 57340 | 17440 |
| STM32G431 LoaderApp | 59284 | 17480 |

镜像哈希并不相同：map 显示改名后函数段排序变化；两个 STM32 目标的 Code 少 4 B、
RO-data 多 4 B，总量相同。LoaderApp 构建还重新生成了镜像事务 token。
不宣称逐指令或实机周期相同；报文和业务语义由词法对照及现有回归验证。
详细哈希、资源与对照见 `outputs/protocol-names/summary.json`。未提交、未烧录。
