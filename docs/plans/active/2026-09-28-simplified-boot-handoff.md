# 最简启动/升级方案（去 Boot/Record）v1

日期：2026-09-28。状态：**设计，待批准**。基线：`c495cf1`。
关联：[Loader 产品与内存](../../architecture/loader/loader_product_v1.md)、
[容量计划](2026-09-28-capacity-fit-v1.md)。

## 目标与范围

把当前 Loader↔APP 交接（Flash **Boot 4K + Record 2K**、32B 邮箱、`boot_epoch`
高水位、试启动确认、镜像描述符）简化为**最小可救砖方案**：

**Loader 独立分区 + 跳 APP 前校验 + 进 Loader 入口。**

释放 Boot(4K)+Record(2K) 给 APP，并移除 APP 侧交接代码，
使窄口真正装得下。代价：从"自动救援 + 可回滚 + 防重放"降级为
"校验保护 + 人工重刷"。不改变电机控制、参数 schema11、CAN 协议与标定。

## 布局（三段）

| 区域 | 起始 | 大小 | 说明 |
| --- | --- | --- | --- |
| Loader | `0x08000000` | 20 KiB（0x5000） | 独立、APP 升级不覆盖 |
| APP | `0x08005000` | **92 KiB（0x17000）** | 原 86K + 释放 6K |
| Param | `0x0801C000` | 16 KiB | 不变 |

RAM 顶部 32 B 邮箱保留（不占 Flash）。

## 启动决策（替代 Record）

Loader 上电：
1. 读 RAM 邮箱：若为"请求进 Loader"(`ENTER` + magic/CRC) → 留在 Loader。
2. 否则校验 APP：栈指针/复位向量合法 + 声明的长度合法 + **整包 CRC32 或 SHA-256**
   与头部声明一致 → 通过则跳 APP（`VTOR` 设 APP base）；**不通过留在 Loader**。
3. 可选：上电握手窗口超时未见 APP 存活证据 → 回 Loader。

不使用持久"已提交"记录；"是否可用"由每次启动现场校验判定。

## 升级流程与掉电行为

| 掉电点 | 后果 | 结果 |
| --- | --- | --- |
| 擦/写 APP 中途 | APP 区半写 | 上电校验失败 → 留 Loader → 重刷（可救） |
| 写完、未校验 | 未定 | 校验决定 |
| 校验通过 | 可用 | 启动 |

进入 Loader：APP 收升级命令 → 写 RAM 邮箱 → `NVIC_SystemReset()`；Loader 接管。

## 取舍与风险

- **失去**：自动回滚、原子提交记录、单调 epoch 防重放、UID 防换板、双份容错。
- **新增**：每次启动一次整包 CRC/SHA（几十 ms 级）。
- **极端砖**：仅当 Loader 自身被写坏或校验/入口缺失；此类需 SWD/BOOT0 恢复。
- 属功能取舍，**非等价优化**；寄存器级 Flash 与校验路径均需实机验证。

## 代码改动点（预估）

1. `shared/targets/vector_mini_st_g431.json`：删 Boot/Record，APP=0x08005000/0x17000；
   用 `loader/tools/generate_target.py` 与 `tools/build/generate_loader_app_project.py`
   重新生成 `.h`/两 scatter/两工程。
2. Loader：`runtime/main.c` 的 `try_boot` 改为"邮箱入口 + APP 校验"；
   停用 `boot_epoch` 双页分配与 `LoaderImage_PrepareBoot/Confirm/ReadRecord` 的持久记录依赖。
3. APP：停用 `boot_handoff`、`image_transfer` 确认路径、`nv_record`、`nv_sha256`；
   仅保留 RAM 邮箱 + 进 Loader + 一个版本/角色字（描述符可简化）。
4. `loader/targets/vector_mini_st_g431/loader.sct`、`app.sct` 随布局再生。

## 验收与回退

- 验收：三工程（Loader/APP/配套 APP）按新布局成功链接；普通 APP 0 错误 0 警告；
  Loader 装入 20K；配套 APP 装入 92K；原生/集成/PR 门禁与既有债务对比；
  离线记录"未做烧录与硬件验证"。
- 回退：按本批提交逆向恢复旧布局、Loader 决策与 APP 交接代码。
- 证据放 `outputs/`，不追踪绝对路径，不宣称实机通过。
