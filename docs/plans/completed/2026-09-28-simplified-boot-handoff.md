# 最简启动/升级方案（去 Boot/Record）v1

日期：2026-09-28。状态：**已完成（B：彻底删除）**。设计基线：`c495cf1`；执行起点：`abc3b45a`。
关联：[Loader 产品与内存](../../archive/architecture/loader/loader_product_v1.md)。

2026-09-28补充审查：本页保留实施与构建历史，“已完成”仅表示简化源码落地，
不表示运行验收通过。看门狗、维护角色、故障救援和协议兼容问题见
[简化审查与修复计划](2026-09-28-loader-simplification-review-v1.md)。

## 目标与范围

把原 Loader↔APP 交接（Flash **Boot 4K + Record 2K**、32B 邮箱、`boot_epoch` 高水位、
试启动确认、SHA 镜像描述符）简化为**最小可救砖方案**：**Loader 独立分区 + 跳 APP 前校验 + 进 Loader 入口。**

释放 Boot(4K)+Record(2K) 给 APP，并移除 APP 侧交接代码，使窄口真正装得下。代价：从
"未确认镜像驻留救援 + 持久启动号隔离"降级为"校验保护 + 人工重刷"。
原版也未提供旧镜像回滚。参数 schema11 与标定区保持；CAN帧布局沿用，
但boot身份和镜像profile语义已经变化，不能称为完全协议兼容。

## 布局（三段）

| 区域 | 起始 | 大小 | 说明 |
| --- | --- | --- | --- |
| Loader | `0x08000000` | 20 KiB（0x5000） | 独立、APP 升级不覆盖 |
| APP | `0x08005000` | **92 KiB（0x17000）** | 原 86K + 释放 6K |
| Param | `0x0801C000` | 16 KiB | 不变 |

RAM 顶部 32 B 邮箱保留（不占 Flash）。

## 启动决策（替代 Record）

Loader 上电：
1. 读 RAM 邮箱：若为"请求进 Loader"（`ENTER` + magic/CRC32）→ 留在 Loader。
2. 否则校验 APP：栈指针/复位向量合法 + 声明的长度合法 + **整包 CRC32** 与描述符声明一致
   → 通过则跳 APP（`VTOR` 设 APP base）；**不通过留在 Loader**。
3. 保留 50 ms 启动窗口，防止重组清单期间跳 APP。

不使用持久"已提交"记录；"是否可用"由每次启动现场校验判定。

## 升级流程与掉电行为

| 掉电点 | 后果 | 结果 |
| --- | --- | --- |
| 擦/写 APP 中途 | APP 区半写 | 上电校验失败 → 留 Loader → 重刷（可救） |
| 写完、未校验 | 未定 | 校验决定 |
| 校验通过 | 可用 | 启动 |

进入 Loader：APP 收升级命令 → 写 RAM 邮箱 → `NVIC_SystemReset()`；Loader 接管。

## 取舍与风险

- **失去**：试运行确认与失败驻留、持久激活记录、单调epoch隔离、启动日志UID绑定和双页见证。
- **新增**：每次启动一次整包 CRC32（几十 ms 级）。
- **极端砖**：仅当 Loader 自身被写坏或校验/入口缺失；此类需 SWD/BOOT0 恢复。
- 属功能取舍，**非等价优化**；寄存器级 Flash 与校验路径均需实机验证。

## 实际代码改动点

1. `shared/targets/vector_mini_st_g431.json`：删 Boot/Record；APP=0x08005000/0x17000；
   由 `loader/tools/generate_target.py` 与 `tools/build/generate_loader_app_project.py`
   重新生成 `.h`/两 scatter/两工程。
2. Loader：`runtime/main.c` 的 `try_boot` 改为"邮箱入口 + `LoaderImage_CheckImage` 跳前校验"；
   移除 `boot_epoch` 双页分配与 `LoaderImage_PrepareBoot/Confirm/ReadRecord` 持久记录依赖；
   `publish_start` 端口改为 `boot` 复位。
3. APP：删除 `boot_handoff`；`boot_hw` 精简为 Init/EnterLoader/Watchdog；56 进 Loader 服务内联进
   `board_config.c`；`foc_run_state` 去试启动确认；`yg_protocol_status_adapter` 恒报无 epoch 身份。
4. **彻底删除**：`loader/core/boot_epoch.*`、`shared/image/nv_record.*`、`shared/image/nv_sha256.*`、
   `firmware/services/boot_handoff/*`、`loader/tools/provision_boot.py`。

## 验收与回退

- 验收：三工程按新布局成功链接；普通 APP 0 错误 0 警告；Loader 装入 20K；配套 APP 装入 92K；
  原生/集成/PR 门禁与既有债务对比；离线记录"未做烧录与硬件验证"。
- 回退：按本批提交逆向恢复旧布局、Loader 决策与 APP 交接代码。

## 执行结果与证据

- 落地方式：**B（彻底删除）**。
- 构建（离线，未烧录/未接探针/未操作硬件），三工程 **0 Error / 0 Warning**：
  - Loader `YG2026_ESC_LOADER`：Code 12780 + RO 532 + RW 60 = **13372 B**（20 KiB 内）。
  - 配套 APP `Vector_Mini_ST_LoaderApp`：Code 87460 + RO 2836 + RW 448 = **90744 B**（92 KiB 内）。
  - 普通 APP `Vector_Mini_ST`：Code 86156 + RO 2732 + RW 428。
- 测试：native `test_loader_upgrade` PASS、`test_run_state` PASS、integration PASS、其余原生 PASS。
- 打包闭环：`fromelf` → `package_image.py` 生成的镜像，其声明长度 90352 与整包 CRC32
  `0x96779539` 由跳前校验复算一致，SP/复位向量合法（`outputs/package/loaderapp_pkg/`）。
- 门禁对比：`check_project_layout`/`check_architecture`/`check_hygiene` PASS；
  `format`/`lint`/`check_docs`/`check_interfaces` 仅剩本次之前既有的 stale 债务（如指向被忽略
  `outputs/` 的文档断链、过期的 style/interface 基线），本次未新增（lint −2）。
- 未做烧录与硬件验证；掉电矩阵、CAN 1M/5M 实机帧/ACK、IWDG 链路与产物哈希待后续 release 验证。
