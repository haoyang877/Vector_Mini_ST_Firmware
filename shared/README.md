# APP / Loader / 主机共享边界

共享实现用于已经存在的多个调用方，不包含设备句柄、HAL或电机控制状态。

- `protocol/`：公司帧、CRC、CAN ID、分片与50～55 payload；由H维护，APP/Loader共用单份。
- `image/`：96B清单/64B描述符、SHA和持久记录；Loader升级/启动与APP确认共用。
- `boot/`：32B一次性交接邮箱、APP进入Loader的值回调契约。
- `targets/`：板型配置、生成的地址头与板型APP描述符；不是通用协议常量。

[Loader当前实现与内存约束](../docs/architecture/loader/loader_product_v1.md)区分源码、设计预算和待验收项。
本轮未运行构建、测试或硬件，不以文件存在宣称接口已通过系统验收。
