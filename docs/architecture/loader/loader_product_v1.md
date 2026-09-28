# Loader 独立产品设计与实现 v2（简化候选）

版本日期：2026-09-28。本文按 codex/capacity-fit、基线 abc3b45a 上尚未提交的简化源码更新。
文件名保留为现有文档入口。当前为**审查未通过的候选实现**，不代表可发布固件。
发现、修复顺序与离线证据见[简化审查记录 v1](../../plans/active/2026-09-28-loader-simplification-review-v1.md)。

## 设计目标与本次取舍

目标是独立 Loader、APP、参数三段布局：通过 CAN FD 接收 APP，限制 Flash 写入范围，
跳转前检查完整镜像，APP 在安全停机和精确回复发送完成后交出控制权。
硬件细节位于端口与板级，升级核心不依赖电机运行态、HAL 或 MCU 寄存器。

本次删除 boot 双页日志、镜像记录页、SHA256、试运行确认及首次 boot 供应工具，
改为 RAM ENTER 邮箱与 CRC32 校验。相对上一版 20KiB Loader + 86KiB APP，为 APP 释放 6KiB，
形成 92KiB APP 槽；这是分区变化，不等于已测得同等代码缩减。
原先就没有旧 APP 自动回滚；本次进一步取消未确认 APP 的持久救援判定与跨复位启动号隔离。
这属于功能取舍，不能称为行为等价简化。

## 软件框图与所有权

```mermaid
flowchart TB
    Host[主机：公司协议50到55] --> Wire[CAN FD端口与共享帧编解码]
    Wire --> Runtime[Loader runtime：路由与启动调度]
    Runtime --> Upgrade[升级核心：会话、租约、激活TX票据]
    Upgrade --> Image[镜像核心：连续写入、描述符、CRC32]
    Image --> Flash[G431端口：固定Flash写域、读回、ECC]
    Runtime --> Platform[平台与板级：安全输出、时钟、复位、跳转]
    App[APP组合层：交接事务] --> Owner[运行所有者：停机、资源归还]
    App --> BootHw[APP boot_hw：ENTER邮箱与复位]
    BootHw --> Mailbox[32B RAM邮箱]
    Mailbox --> Runtime
```

- loader/runtime/main.c 独占产品调度；loader/core/upgrade.c 独占升级事务与租约。
- shared/image/image_transfer.c 通过 Flash 回调擦写 APP、校验向量和描述符，当前不再由 APP 调用。
- shared/protocol 保留单份纯编解码；Loader 不链接 APP 路由或电机运行态。
- loader/ports/stm32g4 拥有 CAN、Flash/ECC、时钟和跳转；板级拥有引脚与安全初始化。
- APP 交接事务从独立 boot_handoff 服务移入 firmware/app/board_config.c，
  通过 BootEntryService 绑定通信层、通过运行所有者核对停机。
- 无 USB 或 UART 升级实现。UART 仍为后续传输适配，不能登记为已支持能力。

## Flash 与 RAM

唯一几何输入为 shared/targets/vector_mini_st_g431.json，当前目标 STM32G431CBUx。

| 区域 | 起始地址 | 大小 | 权限与用途 |
| --- | --- | --- | --- |
| Loader | 0x08000000 | 20KiB / 20480B | 离线安装；线上不可擦写 |
| APP 单槽 | 0x08005000 | 92KiB / 94208B | Loader 整槽擦除、连续 8B 单元写入 |
| raw11 参数区 | 0x0801C000 | 16KiB / 16384B | Loader 只读，APP 参数服务负责写入 |
| 普通 RAM | 0x20000000 | 32736B 上限 | 各镜像的数据、栈、堆合计 |
| ENTER 邮箱 | 0x20007FE0 | 32B | APP 写、Loader 一次性读并清除 |

APP 向量在槽首，描述符位于 0x08005200、固定 64B，代码区从 0x08005240 开始。
旧 0x08006800 或 0x08008800 布局镜像不可直接使用；standalone APP 是另一独立构建目标。
旧 boot/record 地址已归 APP，部署时必须按配套布局安装 Loader 与打包后的 APP。

Loader 栈预留 3072B、堆 512B，当前不动态分配；CAN 软件环 8 槽（可用 7），
TX 票据 3 槽，协议重组缓冲 146B。完整 ROM/RAM、加载跨度和栈峰值须以新 map 验收；
本次未构建，不能用历史产物证明候选装入 20KiB/92KiB。

TARGET_HARDWARE=1 是历史 identity.hardware=1 沿用的镜像硬件兼容 profile，
来源为提交 0f617a5af4e2043793e2f724778e6ca8fb6443b2 的历史目标配置。
它不是实测 PCB revision；设备信息缺少权威板卡数据时应报告未知。

## 启动与升级流程

拆为三个短图，分别说明启动、下载与 APP 交接。

### 启动判定

```mermaid
flowchart TB
    A[安全输出、参数节点识别、启动CAN和IWDG] --> B{软件复位且ENTER有效?}
    B -- 是 --> R[驻留Loader]
    B -- 否 --> C{约50ms内收到有效BEGIN首片?}
    C -- 是 --> R
    C -- 否 --> D{描述符、整包CRC32、向量合法?}
    D -- 否 --> R
    D -- 是 --> E[恢复时钟并跳APP]
```

邮箱仅在符合软件复位来源时强制驻留，掉电后不依赖 RAM 邮箱。
参数空白或不可信时采用救援节点 1；当前不以参数识别结果限制跳 APP，
完整参数合法性与电机准入仍由 APP 自身负责。
IWDG 复位没有单独驻留策略，CRC 正确但运行故障的 APP 可能反复被启动。

### 下载与激活

```mermaid
flowchart TB
    A[BEGIN校验清单并建会话] --> B[整槽擦除APP]
    B --> C[WRITE连续写入并读回]
    C --> D[FINISH分步CRC32校验]
    D --> E{镜像检查通过?}
    E -- 否 --> R[留Loader重新下载]
    E -- 是 --> F[ACTIVATE回复绑定TX票据]
    F --> G{对应帧发送成功且未超时?}
    G -- 否 --> R
    G -- 是 --> H[完整复核、复位、重新判定启动]
```

WRITE 接受 8..128B、8B 对齐连续偏移；相同已写内容只比较，不重复编程。
失败后不得向已编程或不确定 ECC 单元补写，应重新整槽下载。
租约 5s、激活回复期限 500ms；仅对应物理 TX 成功事件能触发复位，入队或 FIFO 空不能替代。

当前没有持久激活标记：**完整镜像在 FINISH/ACTIVATE 前就可能满足冷启动检查**。
完整写入后掉电再上电可能运行该镜像，ACTIVATE 不再是跨复位唯一启动门槛。
若保留这一门槛，可将一个 8B 启动有效单元留至激活时最后写入；需另行定义版本、CRC、
ECC 写入及掉电规则，不能原位改写已编程单元。此方案当前未实现。

### APP 安全交接

```mermaid
flowchart TB
    A[Type 56请求] --> B[撤销目标并申请停机]
    B --> C{关相、资源归还、保存空闲?}
    C -- 否 --> W[等待或超时失败]
    C -- 是 --> D[READY回复绑定TX票据]
    D --> E{精确发送成功并再次复核?}
    E -- 否 --> W
    E -- 是 --> F[写ENTER邮箱并复位]
```

期限 1s；READY 表示具备交接条件，不表示已复位。失败后继续保持停机所有权，
不自动恢复旧运动目标。当前内部票据直接采用主机提供的非零 boot_epoch，
已失去旧版设备可信启动身份的含义。

BootHw_Init 按实际链接向量地址识别偏移 APP 并设置 VTOR，不再核对邮箱、镜像或持久记录；
它只能判断布局，不能证明由某个 Loader 版本实际启动。
BootHw_Watchdog 保留为端口函数，但 APP 当前没有调用者，正常 APP 也会因继承 IWDG 超时复位。
须恢复由监督进展和健康条件驱动的有界喂狗，不能无条件喂狗掩盖故障。

## 邮箱、镜像与协议兼容性

RAM 邮箱当前为 v4、32B、小端：magic、version、ENTER、node 各 4B，12B 保留零，
末 4B 为前 28B 的 CRC32。不携带 token/epoch/TRIAL/NORMAL。APP 最后发布 magic 并读回，
Loader 读后立即清零。APP 当前将主机来源节点写入邮箱，Loader 仍从参数读取本机节点，
尚不消费邮箱 node；该字段含义需同步收口。

镜像仍为 96B 清单和 64B 描述符，大端字段，当前候选规则如下。

| 对象 | 字节偏移 | 含义 |
| --- | --- | --- |
| 清单 | 48..51 | 镜像 CRC32 |
| 清单 | 52..95 | 全零 |
| 描述符 | 44..47 | 含 8B 补齐的镜像长度 |
| 描述符 | 48..51 | 镜像 CRC32 |
| 描述符 | 52..63 | 全零 |

CRC32 使用反射多项式 0xEDB88320、初值与末异或 0xFFFFFFFF。
覆盖完整补齐镜像，描述符 CRC 字段按零参与，长度字段参与计算。
它检查偶发损坏，不提供签名、发布者认证或恶意篡改防护。

package_image.py 现在补齐并回填长度/CRC，输出 image.bin 和 manifest.bin。
用于升级或偏移 APP 安装的是**打包后的 image.bin**；原始链接 BIN/HEX 中占位字段为零，
不满足启动判据，不能直接作为部署包。

当前 YGI_PROFILE、描述符版本 1、TARGET_LAYOUT=3 和包格式名 loader-image-v1 仍沿用旧值，
但摘要字段与 APP 基址已变化；旧 SHA256 fixture、协议文档和主机调用约定未同步。
须定义新 profile/布局版本、兼容拒绝策略及三端黄金向量，不能隐式混用。

Type 50..55 字节布局仍沿用公司载荷 v1，但启动身份语义改变：
空 STATUS 返回 boot_epoch=0，BEGIN 要求主机提供非零值并回显，APP 也不再发布有效启动身份。
旧 v1 主机“先发现设备启动号，再 BEGIN/56”的流程不兼容。
活动 BEGIN 当前不比较请求 boot 字段，跨复位 session/ticket 计数从零开始；
不得宣称旧启动报文隔离仍存在。协议修复须覆盖发现、BEGIN 幂等与 APP 入口。

## 掉电、故障 APP 与参数权限

| 情况 | 当前行为 | 边界 |
| --- | --- | --- |
| BEGIN 受理但尚未擦除时掉电 | 旧 APP 若仍完整可再次启动 | 无持久事务标志 |
| 擦除或部分写入后掉电 | 描述符/CRC/向量不合法则留 Loader | 重新整包下载，无跨复位续传 |
| 写完整但未 FINISH/ACTIVATE 就掉电 | 新镜像通过检查即可启动 | 无持久激活门槛 |
| CRC 正确但 APP 崩溃或不喂狗 | 看门狗复位后可能重跳同一 APP | 无试运行失败驻留 |
| CRC 正确但 APP 无法通信 | 复位后利用约 50ms BEGIN 窗口救援 | 主机链路与窗口可靠性待实测 |
| Loader 被离线误刷或启动入口破坏 | 线上升级不能恢复 | 需 SWD 等离线恢复 |

建议增加“看门狗/异常复位驻留 Loader”的最小策略，无需恢复两个持久分区；
该策略当前**尚未实现**，复位归属与救援矩阵待验收。

参数保留现有 4304B raw11 记录与完整 16KiB 区域；Loader 只识别外形和节点，
不提供参数或任意地址擦写。SN、类型字段和权限服务未在此批新增，不能宣称已实现。
参数合法性、校准和电机使能由 APP 所有者负责。
role=2 仍被镜像解码接受，但 APP 的维护禁止运动标志固定为 false；
恢复角色限制或明确拒绝 role=2 前，不能发布维护镜像能力。

## 更换 MCU 与后续验收

新 MCU 增加目标配置、端口与板级安全实现，内核不添加型号分支。
核对 Flash/RAM、擦除页、编程/ECC 单位、异常与复位来源、CAN 精确 TX 完成及时基。
当前逻辑仅支持 8B 写单元，16B 硬件单元不能伪装成可独立 8B 补写。
UART 后续适配应复用会话和镜像层，并定义准确发送完成与链路错误契约。

本轮 Loader 纯内存用例通过，补充复现确认未激活镜像可冷启动、不同 boot BEGIN 被接受、
复位模型会重用会话身份。quick 验证 FAIL，详细证据见审查记录。
未运行 Keil 构建、烧录、CAN/J-Link 或电机操作。

发布前修复看门狗与维护角色，闭合协议/profile/黄金向量，核对寄存器 Flash 和生成输入，
完成 PR/release 与明确授权的掉电、故障 APP、ECC、CAN FD 测试。
按用户选择，后续硬件测试结束恢复测试前固件和**完整原参数**，保留校准和空 SN；
读回比对备份，不能确认输出关闭或恢复状态时停止测试。
