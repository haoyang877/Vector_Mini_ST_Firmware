# Refactor V2 Validation

## 目标与适用范围

本验证骨架用于在架构迁移期间同时守住两类约束：

1. 已有控制、标定、参数和产品变体行为可以在主机上回归；
2. 新代码只能向目标架构收敛，不能继续扩大旧目录或软硬件耦合。

目标固件树固定为：

```text
Firmware/
├── Core/
│   ├── Application/
│   ├── Services/
│   ├── Communication/
│   ├── Config/
│   └── Infrastructure/
├── Drivers/
├── Bsp/
│   ├── Api/
│   └── Boards/
│       └── <board>/Bootstrap/  # 每个板卡目标一个 CompositionRoot
└── Platform/
```

迁移期的 `Firmware/Application`、`Communication`、`Composition`、
`Domain`、`Ports`、`Product` 和 `Runtime` 是显式技术债，不属于目标树。
`Composition` 暂时保留为唯一组装根；最终应迁入
`Bsp/Boards/<board>/Bootstrap`。该目录作为独立 `CompositionRoot` 层，不能
通过放宽整个 `BspBoards` 层来复制组装逻辑。同一仓库可保存多个板卡的根，
但每个具体构建目标必须且只能选择一个。具体决策见
[`ADR 0001`](adr_0001_composition_root_and_forward_migration.md)。

## 主机测试入口

运行：

```powershell
pwsh -NoProfile -File tests/host/build_and_run_host_tests.ps1
```

指定编译器或仅构建：

```powershell
pwsh -NoProfile -File tests/host/build_and_run_host_tests.ps1 `
    -Compiler C:\path\to\clang.exe
pwsh -NoProfile -File tests/host/build_and_run_host_tests.ps1 -BuildOnly
```

脚本不使用递归通配符。测试源、生产源和 include 目录均为显式清单，
新增依赖必须经过代码审查。支持 Clang、GCC、MSVC、clang-cl 和 Zig，
并尽可能启用 C11、完整警告和 warnings-as-errors。统一 runner 逐套调用
`*_RunHostTests()`，输出每套 PASS/FAIL 和最终汇总。

退出码契约：

| 退出码 | 含义 |
| ---: | --- |
| `0` | 构建和全部测试通过，或 `-BuildOnly` 构建通过 |
| `1` | 测试失败；也可能是编译器本身返回的 `1` |
| `2` | `NOT_RUN`：没有找到受支持的主机 C 编译器 |
| `3` | 显式清单中的输入文件或目录缺失 |
| 其他 | 原样传播编译器或测试进程退出码 |

未安装主机编译器不是“通过”。脚本会明确输出
`HOST_TEST_RESULT=NOT_RUN` 并返回 `2`，CI 不得把它降级为成功。

## 架构门禁

运行迁移期门禁：

```powershell
pwsh -NoProfile -File tools/verify_architecture_v2.ps1
```

发布前严格模式：

```powershell
pwsh -NoProfile -File tools/verify_architecture_v2.ps1 `
    -FailOnLegacyExceptions
```

默认模式允许已登记且数量被冻结的迁移例外，但任何新增例外、例外次数
增长或非目标目录都会失败。严格模式只要还存在一个例外就失败，用于判断
是否真正完成 V2 重构。

当前检查包括：

- 目标目录存在性和 `Firmware` 一级目录白名单；
- `Core/Application`、`Services`、`Communication`、`Infrastructure` 不得
  出现 STM32、HAL、CMSIS、CubeMX 生成头、寄存器或具体器件名；
- `Core/Config` 可以显式命名产品、板卡、电机和传感器型号，但不得包含
  MCU 实现依赖，也不得 include Driver、Platform 或 BspBoards；
- `Bsp/Api` 不得出现 MCU、厂商 SDK、板级或具体器件依赖；
- `Drivers` 可以描述器件协议，但不得包含 MCU SDK、HAL、生成外设头或
  寄存器访问，也不得依赖 `Core/Config`；
- `Bsp/Boards/<board>/Bootstrap` 被识别为独立的 `CompositionRoot`；只有
  当前构建目标所选的一个根可以依赖全部目标层，普通 BspBoards 保持窄依赖；
- 解析固件内部 `#include`，按唯一头文件解析实际依赖层，并检查依赖方向；
- 未限定且同名的内部头会失败，要求调用方使用可唯一解析的 include 路径；
- 每个 legacy 依赖边和内容泄漏都有原因及最大出现次数，不能静默扩散。

为支持小步迁移，legacy 层可以指向依赖的最终目标层，这类单向边不计为新增
技术债。任何目标层回指 legacy 仍立即失败；已有 legacy-to-legacy 例外继续按
冻结次数审计。

无环核心规则是 `Core/Communication -> Core/Application` 的 use-case/command
契约。`Application` 不依赖协议或 Communication 实现；调度由唯一
CompositionRoot 同级调用。`Core/Services` 和 `Core/Config` 是纯核心，
`Bsp/Api` 是硬件语义边界，`Platform` 才能包含厂商实现。

## Keil Flash 余量门禁

Keil 全量构建完成后，对已经生成的 map 执行只读余量检查：

```powershell
pwsh -NoProfile -File tools/verify_flash_budget.ps1
```

默认解析 `MDK-ARM/Vector_Mini_ST/Vector_Mini_ST.map` 的 `LR_IROM1`，要求至少
保留 2048 bytes。也可以显式指定输入、region 和阈值：

```powershell
pwsh -NoProfile -File tools/verify_flash_budget.ps1 `
    -MapPath artifacts/Vector_Mini_ST.map `
    -Region LR_IROM1 `
    -MinimumRemainingBytes 4096
```

脚本不会调用 Keil、不会生成或修改 map，也不会改固件。对于 Keil load region，
存在 `COMPRESSED[0x...]` 时以实际压缩后的装载字节数计算 Flash 占用，否则使用
`Size`；execution region 始终使用 `Size`。输出使用稳定的
`FLASH_BUDGET_*` 键值，便于 CI 保存证据。

退出码契约：

| 退出码 | 含义 |
| ---: | --- |
| `0` | region 存在且余量达到阈值 |
| `1` | region 已溢出，或剩余字节数低于阈值 |
| `2` | 参数、文件读取或 map/region 解析错误 |

2026-09-06 对当前 map 的默认门禁实测为：

```text
FLASH_BUDGET_REGION=LR_IROM1
FLASH_BUDGET_USED_SOURCE=COMPRESSED
FLASH_BUDGET_USED_BYTES=114396
FLASH_BUDGET_MAX_BYTES=114688
FLASH_BUDGET_MINIMUM_REMAINING_BYTES=2048
FLASH_BUDGET_REMAINING_BYTES=292
FLASH_BUDGET_RESULT=FAIL
exit code: 1
```

这表示当前镜像仍能链接，但尚未满足 2 KiB 发布余量要求。将阈值设为 `0` 的
解析自检返回 `PASS`/`0`；它只验证解析路径，不可替代发布门禁。

该检查必须紧跟在 Keil 全量构建之后；旧 map 只能证明旧镜像的大小，不能作为
当前源码的发布证据。阈值是发布策略而非链接器容量，不能通过降低阈值掩盖空间
回归。

## 2026-09-06 验证记录

本机原有 PATH 未发现原生 Clang、GCC、MSVC、clang-cl 或 Zig。验证过程在
用户临时目录安装了便携 Zig 0.16.0（不写入仓库或系统工具链），并用 `zig cc`
以 C11、完整警告、warnings-as-errors 编译显式清单中的 52 个源。17 套测试均
被实际执行：

```text
HOST_TEST_SOURCE_COUNT=52
HOST_TEST_SUMMARY total=17 passed=17 failed=0
HOST_TEST_RESULT=PASSED
```

测试覆盖原有服务与领域回归、统一 ProductConfig、零/单/双角度反馈、纯无感、
温度缺失、单电阻采样前置条件、BSP 板级绑定，以及 ProductConfig 到 BSP 端点
投影。迁移一致性测试还逐项核对新旧配置中重叠的板卡、电机、负载、编码器、
CAN 和电流采样字段，并验证配置指纹、BSP 指纹和错误 endpoint 任一漂移都会被
门禁发现。测试同时发现并修复了旧 `ProductVariant_Validate` 漏检电机位置速度
设计值超限的问题。

当时的架构默认门禁结果为：

```text
files=216 dependency_edges=478 failures=0 warnings=0 legacy_exceptions=25
exit code: 0
```

严格模式预期并实际因 legacy 例外返回 `1`。这表示“新增边界与冻结基线有效”，
不表示重构已经完成；完成条件是默认和严格模式均返回 `0`。

Keil ARMCC 5.06u7 生产目标已将紧凑产品选择身份、活动 legacy 配置指纹和 BSP
身份接入真实启动路径；完整 ProductConfig 组合矩阵与 endpoint 逐项验证保留在
主机门禁，以适应当前 112 KiB 应用区的空间约束。目标端仍先执行完整的
`ProductVariant_Validate`，因此迁移不会绕过当前真正被控制代码消费的配置。
构建结果为：

```text
Program Size: Code=108248 RO-data=6176 RW-data=456 ZI-data=31432
0 Error(s), 0 Warning(s)
```

启动时会先校验当前实际运行的 legacy ProductVariant，再核对紧凑选择记录中的
产品/平台/板卡/配置指纹和实际 BSP 身份；失败时 `FirmwareIsInitialized` 保持
false，PWM 初始化与周期中断均不会启动。目标端紧凑记录只承担“选择与身份哨兵”
职责，不等同于完整 ProductConfig 内容校验；full validator、BSP endpoint 逐项
校验以及新旧重叠字段 parity test 由强制 host build gate 执行，不占用目标 Flash。

P1 仍是双配置模型迁移期，不能宣称 ProductConfig 已成为唯一运行时配置源。
后续阶段必须逐个迁移消费者并删除 `Firmware/Product` legacy profiles；完成前
严格架构门禁会持续失败。

## CI 建议顺序

1. 运行架构默认门禁，阻止新增耦合或 legacy 扩散；
2. 用原生主机编译器构建并执行全部 host suites；
3. 执行 Keil 全量构建；
4. 对全量构建生成的 map 执行 Flash 余量门禁；
5. 对影响 ISR、外设、持久化、标定或控制律的变更执行对应实机回归；
6. 发布候选运行 `-FailOnLegacyExceptions`，确认旧层已清零。

主机编译器补齐后，应优先增加配置矩阵测试：零/单/双角度传感器、无感、
可选温度区域、3/2/1 电阻电流采样、无效 endpoint、能力不足和标定计划裁剪。
BSP 测试还应验证安全关断、PWM/ADC 原子采样计划以及板级资源冲突。

## 已知限制

- include 图是静态源码门禁，不展开宏条件，也不替代编译器依赖文件；
- 具体器件词检查采用已知词表，新增器件时必须同步评审词表；
- 默认模式只保证技术债没有增长，严格模式才代表目录迁移完成；
- ARMCC 交叉编译不能证明测试断言已实际执行；必须补装原生编译器并取得
  `HOST_TEST_RESULT=PASSED`。
