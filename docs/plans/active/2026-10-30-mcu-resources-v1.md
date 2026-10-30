# MCU资源配置收拢 v1

## 目标和范围

2026-10-30用户要求AT32、STM32各有一个资源头文件，集中描述可选的PWM、TIM、ADC输入
及转换结果、GPIO、SPI、CAN、DMA、UART和中断。分别放在各自MCU目录的mcu_resources.h，
只由对应MCU实现包含。板级能力、模拟比例、时钟数值和Flash布局继续引用已有配置。

## 不变行为

保持当前引脚、电气模式、ADC顺序/触发/累加、PWM极性/死区/关相顺序、SPI事务、
CAN协议、DMA缓冲、IRQ优先级、ABI及故障行为。没有新增UART、LED或外部编码器驱动。
STM32生成区继续由CubeMX维护；头文件区分手写代码配置与需要同步.ioc/生成代码的映射。

## 基线、验收与回退

- 基线：e2248c2c2eaeea852b93c71154d6a4045d469916，工作树原先干净。
- PR基线：outputs/runs/20261030-mcu-resources-baseline/summary.json；已有格式、工程路径
  及旧接口测试问题，不能作为通过结果。
- 修改前12个MCU源的预处理结果、ARM汇编和备份：outputs/mcu-resources-20261030/before/。
- 验收：修改文件格式、架构检查；对比前后ARM编译与快速路径；运行PR验证并区分既有失败。
- 回退：仅撤销本计划对应的两个资源头、驱动引用、配置迁移及说明；修改前备份在上述目录。
- 不烧录、不连接硬件；离线结果不代表板上时序或实机验收。

## 证据

- 两个资源头及11个实际MCU源已接入；公开接口、生成区、工程输入和存储布局未修改。
  编码器引脚从board_config.h迁入本MCU资源头。AT32 PWM/ADC输入通道按显式表初始化，
  避免假设通道连续；GPIO换端口后所需时钟仍由初始化代码保证。
- 使用ARMCLANG、Cortex-M4/hard-float、C99、O2、关闭调试信息进行离线编译。
  ADC/CAN/Encoder/PWM/System/Flash共12个源的规范化汇编与基线逐个相同，见
  outputs/mcu-resources-20261030/after.json。规范化只忽略空白，不改指令或数据。
  APP/Loader交接端口的关相检查也引用同一资源表，单独比较汇编相同，见loader-equivalence.json。
  此结果不等同于全部工程、实际优化选项或整机镜像逐字节验证。
- 资源宏替代CAN初始化函数名后，同步调整现有USB移除检查，让它沿资源绑定确认FDCAN初始化。
- STM32/AT32正式APP的Keil整机构建均被未修改的motor_axis_profile.c中13个缺失
  PARAM_APP_JOINT_V1_*宏阻止；AT32原有内部参考值为0还产生一处不可达语句警告。
  没有产出可交付的新镜像；构建日志在outputs/build/logs/。
- 最终PR记录：outputs/runs/20261030-mcu-resources-final/summary.json。lint、架构和接口检查
  通过；14项集成测试通过。修改文件的单独format --check与git diff --check通过。
  全仓库PR仍因基线已有问题失败：旧测试/删除文件引用、一个未修改测试的格式、工程中的
  缺失include目录、旧文档断链以及测试清单漂移。最终42组测试的通过/失败状态与基线一致，
  没有新增失败组；未扩展本次工作去迁移旧接口测试或补齐电机参数。
- 本次资源整理与离线核对已完成；没有进行烧录或实机验证。
