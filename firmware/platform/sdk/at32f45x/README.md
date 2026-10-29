# AT32F45x vendor subset

来源为用户提供的F456_base，AT32F45x标准外设库2.0.1。
本目录保存原始字节，不修改厂商实现或格式；版权及使用条款保留在原文件头。
`manifest.json`逐文件记录输入相对路径和SHA-256，离线测试检查导入后未被修改。

当前链接CRM、GPIO、TMR、Flash、PWC、MISC、ADC、DMA、CAN、SPI及CMSIS启动支持。
驱动头文件保留依赖集合，CMSIS仅保留内核头，不引入SDK示例应用或DSP库。
项目配置头及适配实现位于`firmware/platform/at32f456/`。
