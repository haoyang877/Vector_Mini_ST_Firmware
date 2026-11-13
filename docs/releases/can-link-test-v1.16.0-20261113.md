# CAN-Link-Test v1.16.0 参数页面候选

速度页的使能、停止、打开记录移到右上方，去掉首次续租/目标号输入，保留UID持久计数。
参数读写页平铺电机、电流环、速度环、位置环四组。读取当前值后可编辑可写项，
保存核对UID、修订、活动值，再写RAM与整轴Flash，实际保存值回读通过才成功。
模型、方向与零位保持只读。用法见[操作指南](../guides/canfd_motor_operations.md)。

合入已发布的工具1.15.0实测值显示和APP1.8.8更新，未重新修改固件字节。
配套为APP1.8.8/Loader2.3.4/debug，FULL不含参数和身份页。
发布源码5eade97；当前Release全部通过，315项Python测试和原生回归、Keil零错误零警告。
证据：`outputs/runs/parameter-desktop-v1.16.0-final-20261113/summary.json`。
EXE内嵌固件与目录、FULL组合、ZIP及所有成员哈希均已核对。

交付：`outputs/deliveries/CAN-Link-Test-v1.16.0-APP-v1.8.8-win64-20261113.zip`。
大小 44046258 B；SHA-256 `abe79fcbe1f50e36ad03e83a05d3ece52eb9687b488b295f96265c093ee24a67`。
EXE SHA-256 `8b4479e28e1d53015ef2310c8299e35570379334f22ef9714ed09496bbc2cebf`。

本次未烧录、未驱动电机、未进行实机参数写入。离线原生预览检查完成后，用户按Esc停止界面自动化；
没有再次控制窗口。硬件验收及高速运行仍由操作员执行。
