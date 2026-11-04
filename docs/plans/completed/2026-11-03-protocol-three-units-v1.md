# 协议按内容拆分 v1

按用户指定，通信目录保留同级的 `yg_protocol.c`、`yg_protocol_motor.c`、
`yg_protocol_device.c` 和唯一公共头 `yg_protocol.h`。

收发单元拥有 CAN 帧、队列、调度和具体帧发送确认；电机单元拥有控制、反馈、标定和参数拒绝；
设备单元拥有设备描述、查询、Loader 请求和业务票据。
单元之间直接调用少量函数并传递请求/回复值，不恢复 Endpoint、注册表或 adapter 层。
现有 `Protocol_*` API 和全部线上字节、功能、序号、错误与时限语义保持不变。

基线为开始时完整工作区，副本和哈希位于 `outputs/protocol-split/baseline/`。
仅改拆分相关文件、工程引用、测试和唯一协议说明，不覆盖其他未提交工作。
回退逐文件依据上述副本；不对整个仓库 reset。

验收：协议黄金向量、拥塞不重复执行、STOP 确认、Loader 精确发送/超时、APP 四配置轨迹、
完整 PR 检查和三个 APP 工程编译通过，记录资源与镜像哈希。不接触硬件或烧录。

## 完成

目录现为四个同级文件：yg_protocol.c（598 行）、yg_protocol_motor.c（737 行）、
yg_protocol_device.c（477 行）和 yg_protocol.h（487 行）。旧 protocol.c/.h 已删除，无转发文件。
业务静态状态各自私有；升级业务票据与具体 CAN 发送关联通过显式值传递，未暴露队列。

完整 PR：outputs/runs/protocol-three-units-20261103/summary.json，所有检查 PASS。
四组 APP（两板×普通/Loader）启动、失败和升级交接轨迹与基线逐字一致。
现有协议黄金向量、权限、拥塞、STOP、标定和精确发送确认/100ms 超时回归通过。
三个 APP 工程均 0 错误、0 警告。

| 工程 | Flash 前→后（B） | RAM 前→后（B） |
| --- | ---: | ---: |
| STM32G431 | 58096→58308 | 17536→17544 |
| AT32F456 | 55576→56132 | 21696→21704 |
| STM32G431 LoaderApp | 60200→60436 | 17576→17584 |

独立编译单元改变了编译器优化，Flash 增加 212～556 B；私有状态划分后 RAM 增加 8 B。
线上 ABI、字节、业务时序和发送预算不变，未做硬件时序验收。
基线、编译日志、源码与镜像哈希见 outputs/protocol-split/summary.json。
未提交、未连接设备、未烧录。
