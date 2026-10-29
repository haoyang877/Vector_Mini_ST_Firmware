# 双板编码器链路与职责 v1

更新：2026-10-26。公开契约见[bsp_encoder.h](../../firmware/platform/bsp/include/bsp_encoder.h)。

公共bsp_encoder.c拥有采样/配置互斥、Begin→BeginReadback→Complete流程、样本序号及诊断快照。
devices内MT6835/TLE5012B实现仅解释命令、状态、校验和角度。
各板encoder_spi.c拥有SPI/DMA、片选、电气方向、超时及失败收尾。
应用角度层保留多圈、速度、零位和LUT，不读SPI寄存器。

| 项目 | AT32F456 + MT6835 | STM32G4 + TLE5012B |
| --- | --- | --- |
| 传输 | 模式3，全双工，8/16位或DMA | 模式1，16位，响应前释放MOSI，完成恢复 |
| 请求/响应 | A0 03，四字节 | 80 21，两字节 |
| 超时 | 整帧20us，APP 40us；有界轮询 | 原每步340次轮询 |
| 角度 | 21位舍低5位 | 15位左移1位 |
| 校验 | CRC-8/0x07及三状态位 | 只读角度字，crc_checked=false |

Begin取得逻辑事务；异步启动失败时Complete保留原同步回退。
重复/乱序请求明确失败；Complete(NULL)仍清理事务后返回INVALID_ARGUMENT。
失败不发布旧值为新样本。样本与诊断分开，失败尝试不伪造成功序号。
配置六槽读取须关相并独占事务；失败槽为NOT_READY，不沿用旧内容。
重复初始化不重配总线。TLE未读安全字，不宣称有CRC验证。

## 验证

实际协议和SPI/DMA内存外设测试覆盖全域角度、状态/CRC、重入/交错、配置失败、片选/MOSI恢复及角度层映射。
九工程核对链接对象及限时目标热段；AT32关相实测核对连续采样、错误计数及时间预算。
