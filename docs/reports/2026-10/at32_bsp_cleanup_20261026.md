# AT32 BSP命名与实现收敛

日期：2026-10-26。按用户要求统一命名、删除实现期测试代码，不增加目录或转发层。
本轮只进行源码整理和离线验收，没有烧录、复位、启相或硬件测试。

## 结果结构

AT32目录从21个文件收敛到11个：9个C实现、厂商配置头和README。

```text
firmware/platform/bsp/mcu/at32f456/
├─ system.c
├─ adc.c
├─ pwm.c
├─ encoder_spi.c
├─ can.c
├─ flash.c
├─ led.c
├─ temperature.c
├─ boot.c
├─ at32f45x_conf.h
└─ README.md
```

STM32的ADC、PWM、CAN和温度实现同时采用adc.c、pwm.c、can.c、temperature.c命名。
厂商生成代码仍保留原结构。函数遵循公开Bsp模块_动作、内部Mcu模块_动作、文件内static小写下划线；
既有BootHw/EncoderSpi契约和厂商中断入口保持接口名称，不能为形式统一破坏调用约定。

## 实际删除和职责调整

- ADC配置、采集、IRQ、故障和Flash暂停/恢复归同一个adc.c。删除adc_sampling_internal.h、
  adc_sampling_setup.c、adc_sampling_storage.c及存储转发，状态不再跨文件传递。
- system_setup合并到system.c；门极GPIO准备与复用归pwm.c。原先先置低、再配置时钟/定时器、
  核对关闭、最后切复用的顺序保留，没有新增转发链。
- MCU温度和NTC换算归temperature.c。删除测试温度快照、未校准典型值显示、NTC专用诊断读取接口。
  正常BspAdc温度通道及校准失败语义保留。
- 删除专用LED诊断命令、SPI调试统计、ADC存储计数/故障现场数组、IRQ最大值测试快照。
  ADC错误锁存、PWM故障关断和9000周期快中断保护仍在。
- 删除临时sin/cos试验采集及辅助诊断头。该功能原来只有台架快照，没有产品读取接口，
  现在能力明确为未支持；不把历史试验接口作为BSP正式功能。
- Flash布局归板配置，删除r40_storage_layout.h。公开接口头、厂商SDK与STM32生成代码逐字节不变。
- SPI事务期限等构建差异改由配置提供，MCU传输实现不识别具体APP测试目标名称。

独立tests目录继续负责回归；测试数据结构不再被产品BSP包含。此处“删除测试代码”不等于删除故障检查。

## 验证

最终源码快照：`58d830beea8c82164b3732c62dbe1588b1dba72b`。
九工程重建全部零错误、零警告；完整PR回归通过。独立release通过，覆盖123项单元、14项集成及39组原生回归。

原始证据位于`outputs/bsp-v1-implementation-20261026`：

- `builds-at32-cleanup-final3/builds.json`：双板九工程。
- `cleanup-targeted-final.log`：ADC、温度/存储、编码器、启动/PWM及能力组合回归。
- `cleanup-final-audit.json`：180个受保护文件不变、目录层级、已删除符号和执行地址。
- `at32-cleanup-release3/outputs/runs/bsp-cleanup-release3-20261026/summary.json`：独立release。

链接核对：ADC快拍、PWM提交、CAN提交仍在SRAM；快代码区53368/57344字节，
数据区34888/36864字节，均未越界，Flash末16KiB参数范围不变。

本次没有产生新的实机通过结论。此前关相实测仅适用于其原始镜像，
见[上一轮实测](bsp_private_at32_hil_20261026.md)。当前源码需重新构建后再做硬件验证。

后续：2026-10-27已对本页最终快照完成烧录和关相验证，
见[当前整理镜像实测](bsp_cleanup_at32_hil_20261027.md)。该记录不扩大本页原始离线验收范围。
