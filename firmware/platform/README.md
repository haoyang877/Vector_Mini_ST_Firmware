# Platform

bsp/include只放8个统一功能头；bsp/bsp_internal.h放内部初始化、采样控制及台架声明。
ADC、PWM、编码器、CAN、Flash和系统硬件操作直接在bsp/mcu/<芯片>/实现对应接口。
不保留公共转发源文件，LED、UART和快速环阶段诊断已删除。

板型和硬件参数在firmware/config；应用入口在projects，业务回调在firmware/app/composition。
底层台架在tests/bench，厂商SDK保持原样，生成代码手工调整仅在USER CODE区域。

详见[共用接口与职责](../../docs/architecture/platform_interface_implementations_v1.md)。
