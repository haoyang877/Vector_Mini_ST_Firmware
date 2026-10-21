> 历史归档（2026-10-21）：旧提案、阶段快照或退役能力；现行入口按产品静态注册表和设备发现契约确定。 本文的“当前”指原始基线；现行入口见[当前说明](../../protocols/yg_protocol_device_discovery_v1.md)。归档不新增执行授权。

# 公司统一帧公共编解码迁移 v1

日期：2026-09-28。前置通信提交：`4cd70e6c`。本轮仅重构代码，按用户要求不构建、不测试。

## 目标与不变项

公司统一帧、CRC、CAN ID 与分片由 APP、Loader 编译同一份实现，避免独立产品维护两套帧规则。
原有九个文件通过 Git 移动到 `shared/protocol/`，文件内容不变。
不修改 ABI、端序、CRC 参数、分片时限、状态/故障语义、报文字节或资源容量。
既有核心与分片测试仍是真实调用者，最终统一测试前需更新其源码路径。

## 文件映射与依赖

旧目录：`firmware/communication/protocol/`；新目录：`shared/protocol/`。

| 文件 | APP / Loader 需要编译或包含 |
|---|---|
| `yg_protocol_wire_types.h` | 公共头 |
| `yg_protocol_crc.c/.h` | 编译 C，包含头 |
| `yg_protocol_frame_codec.c/.h` | 编译 C，包含头 |
| `yg_protocol_can_id.c/.h` | 编译 C，包含头 |
| `yg_protocol_fragment.c/.h` | 编译 C，包含头 |

公共目录只依赖 C 标准头。原文件名与 API 不变，所有 `#include` 保持原写法，新增公共包含目录。
APP 的 CAN FD 承载、收发队列、消息目录、路由与业务服务留在 `firmware/communication/`。
Loader 使用自己的小队列和固定命令分支，不引入 APP 的运行期框架，也不保留旧 YI 协议。

## 工程交接

ROOT 更新 APP 构建及测试工具输入，D 更新独立 Loader 工程；本次迁移提交不代改其工程文件。

1. `firmware/platform/stm32g4/cubemx/MDK-ARM/YG_ESC_APP.uvprojx`：
   四个 C 文件改为 `..\..\..\..\..\shared\protocol\<文件名>`；新增
   `..\..\..\..\..\shared\protocol` 包含目录。不要把新旧实现同时加入工程。
2. `tools/project_paths.py`：`NATIVE_INCLUDE_DIRS` 加 `shared/protocol`。
3. `tests/unit/native/test_yg_protocol_core.py` 与 `test_yg_protocol_link.py`：
   四个公共 C 源码的路径由旧目录替换为新目录。
4. `tests/unit/native/test_yg_protocol_fragment.py`：替换分片 C 源码路径。
5. Loader 工程使用相同四个 C 源码和公共包含目录，不访问 APP 私有目录。

迁移与上述工程输入应一起集成。只合入文件移动尚不能宣称工程可构建。

## 信息流程与所有权

```mermaid
flowchart LR
    HW[各产品 CAN FD 平台端口] --> RX[产品拥有的接收缓冲]
    RX --> CRC[shared 帧长度与 CRC 校验]
    CRC --> FRAG[shared 有界分片重组]
    FRAG --> ROUTE[产品命令分支]
    ROUTE --> APP[APP 状态机与参数所有者]
    ROUTE --> LOADER[Loader 镜像事务所有者]
    APP --> ENCODE[shared 帧编码]
    LOADER --> ENCODE
    ENCODE --> TX[产品发送队列与物理完成回执]
```

各产品先校验 FD/BRS、物理 DLC、ID 与公司头的地址一致性、零填充，再交付业务。
分片函数仅接收已经完成帧校验的消息，不替代这些承载检查。
分片输出借用调用方缓冲；业务异步持有数据时须由该业务所有者复制或明确转移所有权。
公共编解码不把“入队”或“控制器接收”当成物理发送完成，升级激活必须关联具体回复的完成回执。

## 验收、回退与限制

这批是内容不变的路径重构，没有当前构建、测试、RAM/Flash 或硬件通过证据。
后续统一检查复用原核心、CAN FD 和分片测试，核对正常/非法字段向量、超时、地址、填充，
再分别构建 APP 与 Loader，确认公共源码各编译一次及最终资源余量。
回退时同时撤回公共路径迁移和对应工程输入，恢复原目录；不要保留两份实现。
