# 硬件检测方向诊断 v1.10.2

2026-11-13，分支`the-system-refactor`，实现提交`d8f1e667`。
基线APP1.8.4/Loader2.3.4不变，未改变固件、协议、阈值、方向参数或驱动电流。

原板端已用50 ms趋势判断电机命令与编码器方向，但主控摘要只显示动态项异常。
本版沿用Type127 P20的去回方向，新增JSON字段`motor_direction`并在硬件结果和
GUI状态栏显示原因。双向反向示例：

```text
电机角动态　异常
  编码器方向反向，命令与编码器方向不一致
  去程：正向命令→编码器反转；回程：反向命令→编码器正转
```

单段反向单独标记；方向一致但动态项失败仍保留失败，不掩盖其他检测原因。
静态、未取到方向和无可辨位移均为方向数据不足，不由笼统编码器故障码推断反向。
原CLI现场参照输入仍保留，并标明与默认命令参照的区别；Hall判据未改。
启动驱动前已有故障时保留原始码，并说明尚未验证本轮旋向。

使用真实CAN编解码的内存端点验证双向反向、单段不符、数据不足、无位移、
方向一致但动态失败及既有故障阻止驱动；JSON回读与摘要一致。硬件工具16项和
GUI13项专项通过；完整单元271项通过。模拟报告保存在
`outputs/hardware-direction-20261113/reversed-example.json`及同名txt，不能当实机证据。
本轮未操作硬件、清除故障、修改方向或重新驱动，也未自动点选原生窗口。
指南见[测试与标定](../../guides/firmware_test_calibration.md)。

干净源码Release全部通过，证据为
`outputs/runs/hardware-direction-v1.10.2-20261113/summary.json`及`release-manifest.json`。
GUI构建成功并已启动；配套APP1.8.4/Loader2.3.4仍为原调试固件字节。
交付`outputs/deliveries/CAN-Link-Test-v1.10.2-APP-v1.8.4-win64-20261113.zip`，
43,375,446字节，ZIP CRC及内文件哈希已校验。

| 产物 | SHA-256 |
| --- | --- |
| ZIP | `a8558cac1e50e0c1aae5c50cfcf2c36cbee88653830e3752a85cc5d335081c2f` |
| EXE | `36854de69d70f4c515b18a71a3d438179770669743da2e9d03d056dad82bbf00` |
