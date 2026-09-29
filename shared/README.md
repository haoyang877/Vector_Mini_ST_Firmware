# APP / Loader / 主机共享边界

共享实现用于已经存在的多个调用方，不包含设备句柄、HAL或电机控制状态。

- `checksum/`：标准反射 CRC32 单一核心；轴记录、齿槽数据、邮箱和镜像续算共用。
- `protocol/`：公司帧、CRC、CAN ID、分片与50～55 payload；由H维护，APP/Loader共用单份；小端字节序和确认单播请求规则使用纯内联契约。
- `image/`：96B清单/64B描述符与整包 CRC32 跳前校验；无持久记录或 SHA。
- `boot/`：32B一次性 ENTER 邮箱与 APP 进入 Loader 的值回调契约；无 epoch/token。
- `targets/`：板型配置、生成的地址头与板型APP描述符；不是通用协议常量。

[Loader当前实现与内存约束](../docs/architecture/loader/loader_product_v1.md)区分源码、设计预算和待验收项。
本轮离线构建三工程 0 错误 0 警告，未烧录、未接探针、未操作硬件；不以文件存在宣称接口已通过系统验收。
