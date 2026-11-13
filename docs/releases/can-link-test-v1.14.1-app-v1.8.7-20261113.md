# APP1.8.7 / CAN-Link-Test1.14.1 配套大包

合入来自“调整 roll 电机标定拖动电流”的e28ba613更新：ROLL标定Kp/Ki为0.05/0.5，
保留5 A标定与对齐、4.9 A启动Iq、0.5 A启动Id、15 rad/s及10圈。
其余轴、运行PI、限速及协议不变。旧Flash较低标定电流不自动覆盖。

## 固件与工具配对

APP1.8.7 / Loader2.3.4复用原已验证固件字节，已核对编译后ROLL参数与全部产物哈希。
firmware/shared/loader/projects与e28ba613来源一致，FULL逐字节等于Loader和APP组合，
不包含参数与身份区。工具1.14.1更新内置FULL、CAN包和目标版本，增加APP1.8.7的RTT采集兼容。
保留顶部实际版本/状态、六路实时参考/反馈曲线和所有既有操作。

当前合并源码`9be5cbc4bd7b5f7be1a36d17470b5ab6a4682eef`，干净Release全部通过：
`outputs/runs/roll-repackage-v1.14.1-app1.8.7-20261113/summary.json`。
原组件证据来自`outputs/AT32F456-FULL-v1.8.7/summary.json`及其release-summary、
compiled-roll-parameters.json。构建级通过不代表带阻尼轴实机标定通过。

## 交付

- ZIP：`outputs/deliveries/CAN-Link-Test-v1.14.1-APP-v1.8.7-win64-20261113.zip`，43,391,971字节。
- ZIP SHA-256：`278f8deaa40bdbd28449e8cd8370d94022adf3875a949feec362b51b7df79a34`。
- EXE SHA-256：`d13b2d12d92690809098f1ae6dc59ebec7324b3abf4da9aaa3a1a13ee71abae1`。
- FULL HEX SHA-256：`2eb8206028287ed822225c0ca8b712fa69536a6bf42e8b2f997d3dde0c3d2d52`。
- 包含上位机、FULL/APP/Loader HEX、CAN包、使用说明、原组件和当前合并验证证据。

ZIP CRC和成员哈希通过。未自动烧录、擦除或启动电机；实机由用户测试。
原个人Keil、J-Link、VS Code及示波设置独立备份并恢复。
