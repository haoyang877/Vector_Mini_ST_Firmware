# BSP私有头合并与直接查询收敛

日期：2026-10-26。状态：代码整理、九工程构建、完整PR及最终独立release全部通过。
本轮没有访问J-Link、USB CAN或板卡，没有烧录、复位或启相。
用户随后要求进行硬件测试，后续故障、修复和实测结果见[AT32实测记录](bsp_private_at32_hil_20261026.md)；
不能把本文的离线完成状态当作后续候选的实机结论。

## 结果

按用户确认的方案，删除bsp/internal目录。原有7个通用私有头合并为
[bsp_internal.h](../../../firmware/platform/bsp/bsp_internal.h)，按ADC、温度、PWM、CAN、LED、Flash和初始化分段。
它只包含跨文件声明与数据类型，没有函数实现、可变对象定义、SDK类型或板级配置值。
单个MCU内部使用的采样状态仍留在该MCU目录，未扩大为全局状态。

原有另外2个头移到devices：encoder_protocol.h与encoder_spi.h。
编码器初始化声明从器件协议头移入bsp_internal.h；器件协议、SPI传输和应用生命周期不再混写声明。
公共功能、MCU硬件操作、器件协议仍保留真实职责，未增加转发层或注册机制。

```text
bsp/
├─ include/                    修订6公开接口，未修改
├─ bsp_internal.h              唯一通用私有声明头
├─ bsp_adc.c
├─ bsp_pwm.c
├─ bsp_encoder.c
├─ bsp_can.c
├─ bsp_led.c
├─ bsp_uart.c
├─ can_frame_codec.c/.h
├─ devices/
│  ├─ encoder_protocol.h
│  ├─ encoder_spi.h
│  ├─ encoder_mt6835.c
│  └─ encoder_tle5012b.c
└─ mcu/{at32f456,stm32g4}/      各板硬件操作及必要本地声明
```

真正删除了两条纯转发路径：

| 接口 | 原先 | 现在 |
| --- | --- | --- |
| Adc_IsHealthy | 公共包装转调McuSampling_IsHealthy | 两板采样实现直接提供Adc_IsHealthy |
| BspPwm_ArePhasesDisabled | 公共包装转调McuPwm_AreChannelsDisabled | 两板PWM实现直接提供公开查询 |

相应旧符号、重复声明和重复include已清除。公共PWM的窗口检查、故障锁、关相序号与失败收尾仍只有一份，
未复制到两板。其余有实际行为的调用保留，本轮没有重写ADC硬件或合并编码器协议与传输。

## 源码与验证

工作树基线快照：aa294fa96ff0137ff21c0bd2acd38a114c65a238。
九工程构建快照：fa2aa7e33532f142794947442c05ad6033f46183。
最终独立验收快照：6aa5f920d074bd9fdc4732cf6afa22ef6466cc04。
后两者的firmware、projects、loader、shared、tests和tools逐字节相同，差异仅为Git属性与计划说明。

证据位于outputs/bsp-v1-implementation-20261026：

| 证据 | 内容 |
| --- | --- |
| builds-internal-final/builds.json | 九工程均为零错误零警告，含AXF/HEX/MAP哈希 |
| internal-build-equivalence.json | 构建与最终验收源码一致；全新检出SDK哈希无差异 |
| internal-audit.json | 公开头/SDK/生成代码不变；运行代码除指定转发外无额外标记变化；资源与热区核对 |
| private-header-checks/summary.json | 两板C11/C++17独立、重复包含及器件头组合编译通过 |
| 根outputs/runs/bsp-private-headers-pr-20261026/summary.json | 完整PR通过，122单元、14集成、39组原生回归 |
| internal-verified/outputs/runs/bsp-private-verified-20261026/summary.json | 干净快照release全部通过，hardware_contacted=false |

AT32两个完整APP和五个台架的Code各减少8字节，RO/RW/ZI不变。
STM32普通APP的Code减少4字节、RO增加4字节，合计不变；LoaderApp资源不变。两板RAM均未增加。
七个检查的关键查询、采样、编码器和PWM函数仍位于原SRAM热区；这不是新的物理时序实测结论。

源码审查按标记比较C实现：忽略include和格式、归一化上述两次更名并去掉原纯转发后，与基线一致。
该静态证据与寄存器/状态回归共同使用，不单独代替行为验证。
修订6的10个功能头和57个函数未改变；能力使能、保护数值、参数和协议ABI均保持。

## 验证中发现并处理的问题

器件初始化声明迁移后，编码器测试夹具缺少新的私有头引用，已补齐后重跑通过，没有恢复协议头的职责混用。
全新Git检出还暴露原有SDK换行保护路径过期：属性只覆盖迁移前目录，导致AT32 CAN厂商头哈希不同。
已为firmware/platform/sdk补充原字节保护，未修改SDK内容或manifest要求；修复后全新检出的SDK全部哈希一致。
此前失败日志保留，最终验收不能引用失败运行替代。

## 后续

本报告仅证明私有接口收敛阶段的离线结果。随后用户授权下载并进行AT32关相实测，
发现并修复重复ADC读取和Flash执行导致的时序问题；最终镜像、九工程及实机结果见
[AT32实测报告](bsp_private_at32_hil_20261026.md)。中间候选不能替代最终硬件证据。
