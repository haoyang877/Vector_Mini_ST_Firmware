# CAN-Link-Test v1.12.0 候选

## 内容

J-Link页可直接连接并烧录已配板；空白板显示未选中的类型选择，
明确选择后初始化并烧录。提供独立识别刷新按钮，中断恢复继续锁定原型号。
擦除流程补上8/4/1 MHz的试读提速和自动降速，保留完整备份和所有回读检查。
只读实机256 KiB双读由6.693秒降至1.375秒，实际擦除总耗时尚未测量。

配套APP1.8.5/Loader2.3.4原镜像不变；APP仍为十圈标定候选，
并未新增实机标定精度结论。详见[验证报告](../reports/2026-11/jlink_workflow_speed_20261113.md)。

## 产物与限制

交付：`outputs/deliveries/CAN-Link-Test-v1.12.0-APP-v1.8.5-win64-20261113.zip`，43,377,560字节。
工具为debug验证用途；实际擦除、烧录及断电恢复未在当前板执行。

- 工具源码：`0a0c69c2`；干净工作树Release全部通过，293项单元测试、21项集成测试及原生测试通过。
- Release：`outputs/runs/jlink-workflow-v1.12.0-20261113/summary.json`，hardware_contacted=false。
- EXE SHA-256：`1b8de35df5dabda76fc51f07e8d2fd42e148340becf964cf882d7d4aeeda6a64`。
- ZIP SHA-256：`808b2393940596d79a6e0f6c46e6e1e45c66af30f7d86a05eef5af534afb7c4c`。
- 固件复用来源：`08ae7e59`的APP1.8.5/Loader2.3.4构建，FULL HEX及CAN包与上次候选相同。
- 包含完整固件、工具、使用指南、当前Release证据、原始固件构建清单、只读速度对照及文件哈希。

ZIP CRC和包内逐文件SHA-256通过。原Keil、J-Link、VS Code和示波配置另存并按原字节恢复，
不包含在提交或交付中。
