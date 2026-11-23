# APP / Loader / 主机共享边界

共享实现用于已经存在的多个调用方，不包含设备句柄、HAL或电机控制状态。

- `checksum/`：标准反射 CRC32 单一核心；轴记录、邮箱、设备身份和镜像校验共用。
- `protocol/`：定义公司帧、CRC、CAN ID、分片和 50–55 升级载荷。两端共享编解码，业务权限和状态迁移由各自的命令处理函数判断。
- `image/`：96B清单/96B描述符、整包CRC32及三个一次写入的激活/尝试/确认双字。
- `boot/`：32B一次性 ENTER 邮箱、进入 Loader/确认 APP 的值回调及64B只读生产身份契约；不保存设备 epoch/token。
- `targets/`：板型配置、生成的地址头与板型APP描述符；不是通用协议常量。

[Loader当前实现与内存约束](../docs/architecture/loader/loader_product_v3.md)区分源码、设计预算和待验收项。
板型、工程与基址见[镜像选择表](../README.md#选择板卡与固件工程)。
