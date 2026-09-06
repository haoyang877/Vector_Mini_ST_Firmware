# BSP 目录说明

BSP 把产品需要的硬件语义与 MCU/HAL 实现分开。公共接口不暴露 vendor handle、寄存器、引脚或中断类型；板包只声明真实物理能力，目标 Platform 完成具体外设操作。

## 目录

```text
Bsp/
├── Api/                         MCU/板卡无关的窄硬件契约
└── Boards/
    ├── bsp_board.*              endpoint 与能力查询
    ├── bsp_product_binding.*    ProductConfig 需求到板能力的校验
    └── <board>/
        ├── *_bsp.*              该板真实 endpoint/capability
        ├── *_memory_map.h       Flash 物理所有权
        └── Bootstrap/           该 target 唯一 Composition Root
```

`Api/` 当前包含 motor drive、angle、temperature、communication、system、indicator 和 synchronous serial 契约。20 kHz 使用的 motor-drive 函数必须有界、非阻塞、无动态分配、无日志；`commit_cycle()` 在同一个 PWM 更新边界提交占空比与电流采样计划。完整固件物理上只有 `Core/Bsp/Drivers/Platform` 四个顶层所有者，板级 `Bootstrap` 是唯一 Composition Root。

## endpoint 规则

endpoint 是稳定逻辑资源 ID，不是 ADC Rank、GPIO、外设实例或数组下标。ProductConfig 用 `role + endpoint` 提出需求，Board capability 判断资源是否存在，binding 再把 endpoint 解析为实际 acquisition index。缺失、重复或能力不匹配必须 fail-closed。

## VectorMiniSt 当前能力

- 一个主 motor-drive endpoint；只支持低侧三分流、A/B/C 三个电流 endpoint、固定同步采样；
- 两个 16-bit synchronous-serial 角度物理 endpoint；目标可组装 0/1 个实例或 `primary + output shaft` 两实例。`primary + redundant`、自动 fallback、双转子对齐和独立第二 LUT 不支持并拒绝；
- 一个可用的 MCU 内部温度 endpoint；功率级模拟温度 endpoint 仅预留且未装配；
- 一个 CAN endpoint 和一个全双工 byte-stream endpoint；当前 Bootstrap 要求 CAN 与 USB 两者都启用并成功绑定；
- 参数存储、单调时钟、执行周期计数、临界区、身份、复位、诊断与指示能力。

通用 Core 电流策略可表达 3/2/1-shunt，但本板只接受低侧 3-shunt。温度策略执行 `OFF / OPTIONAL / REQUIRED`；当前内部温度只监测不保护。活动 CAN 配置是 Classic CAN 1 Mbit/s、8-byte、BRS 关闭。虽然板能力能描述 CAN FD/BRS，仍需独立 target 配置与实机验证后才能作为产品能力发布。

Board capability 是物理事实，不是第二套产品默认参数。完整统一 `ProductConfig` 只由 Bootstrap 读取并投影成反馈、控制模式、温度、标定和通信窄配置；普通板描述、Platform 和 Driver 都不能选择产品。Bootstrap 按实例的精确 `design_id` 分派 Driver，未知 ID 必须 fail-closed。标定 mask 已接入运行时；全禁用时可启动但拒绝 Mode 21/单项标定，Mode 8/9 维护命令仍可用。
