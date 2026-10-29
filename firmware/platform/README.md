# Platform

这里按bsp、devices、板级config和原样厂商sdk归类。

采样、PWM、编码器、CAN与LED共用bsp根目录下的功能实现；UART有一处明确的禁用实现。bsp/include是公开契约，bsp_internal.h集中必要的私有声明，器件契约在devices。
mcu/at32f456与mcu/stm32g4处理SDK/寄存器和硬件事务；devices仅实现器件协议。
板型只由config/bsp_config.h选择一次，每板一个配置头；应用使能和关相目标在firmware/app/config。
工程入口在projects，应用回调在firmware/app/composition，台架在tests/bench。厂商SDK与生成外设配置保持原实现。

详见[共用接口与职责](../../docs/architecture/platform_interface_implementations_v1.md)。

PWM初始化与运行操作归对应MCU的同一文件；单调用方许可/诊断函数归使用方私有。
STM32指示器不再经过LED/RGB转发，参数存储不再经过第二套Flash接口；未使用延时已删除。
