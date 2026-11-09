# v0.7.2：上电自动校准电流偏置

日期：2026-11-09。分支：`feature/roll-motion-telemetry`。
APP 版本：v0.7.2 build 0，编码 `0x00070200`；CAN 版本查询回复 `00 07 02 00`。

## 修复内容

有效电机配置上电后保持三相关断、相引脚高阻，自动进入 mode 11，
通过 TIM1 CH4 触发 ADC，按 20 kHz 累积 20,000 次原始采样。
三相均值均在 1948～2148 ADC counts 时才更新 RAM 电流偏置，然后进入 mode 0。
校准不合格时保留原偏置并报错停机；中止时清除未完成的累积。
启动校准不会自动写 Flash，不会启动电机。

Loader、参数 ABI、CAN 编码和个体 LUT/电零位格式保持兼容。
仅需更新 APP 与有效记录；更新时保留 Loader、保留页和参数区。

## 验证范围

v0.7.1 build 1 已实测同一控制逻辑，两次复位均约 1.18 秒完成校准，
三相始终未开启，结束模式和故障均为 0，整片 Flash 在两次启动前后逐字节不变。
详细偏置数值、板 UID 和原始证据见[启动测试记录](2026-11-09-startup-current-offset.md)。

v0.7.2 仅在该控制逻辑上更新版本标识及文档、测试向量。
本轮全套离线回归通过，Keil 构建 0 错误、0 警告；没有再次下载，当前左轮板仍是实测的 v0.7.1 build 1。
附加 USB-CAN 复验未收到回复，尚未定位原因；不将该项记录为通过，也不将历史其他板的通信结果作为本版验收。

本版不宣称完成运动、温升或耐久验收。
构建产物、日志及校验清单归档于 `outputs/releases/v072-startup-offset-20261109/`，不随 Git 自动推送。

## 构建产物

Keil ARMCC 5.06 update 7，点到点参考、经典 CAN，0 错误、0 警告。
Code 90,348 B，RO-data 3,116 B，RW-data 444 B，ZI-data 24,132 B。
APP 分区 98,304 B，CRC32 `051DE7BE`；BIN SHA256：
`d649cb3a0196d97ef0385e737e3c2c4e6e9c4bfc2fc1a2aec3f4203e77f38529`。

- [APP HEX，含有效记录、不含 Loader](../../outputs/releases/v072-startup-offset-20261109/build/Vector_Mini_ST.hex)
- [镜像版本、CRC 和文件哈希](../../outputs/releases/v072-startup-offset-20261109/build/Vector_Mini_ST.boot.json)
- [构建日志](../../outputs/releases/v072-startup-offset-20261109/build/build.log)
- [离线回归日志目录](../../outputs/releases/v072-startup-offset-20261109/validation)
