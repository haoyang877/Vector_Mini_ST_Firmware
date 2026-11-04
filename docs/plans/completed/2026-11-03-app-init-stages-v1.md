# APP通信初始化分步表达 v1

按用户要求将App_Init中的多函数短路表达式展开成逐阶段判断，并注释各阶段职责。
保持CAN运行时、业务服务、设备描述、可选Loader服务的调用顺序和失败短路，统一失败关相。
基线：`outputs/app-init-readable-20261103/app.before.c`；回退仅撤销本次初始化表达增量。
同时完成上一轮重复Vq清零删除：启动校正测试改用MotorCurrentOffsets_Init定位参数装载块，
避免继续依赖被删除的if。参数读取和默认值回退不变。
验收：完整APP四配置前后轨迹、启动校正回归、完整PR档与双板编译；不烧录或访问硬件。

## 结果

已将通信初始化展开为依次执行的CAN运行时、业务服务、设备信息和可选Loader服务，
每一步成功才执行下一步；末尾仍统一关相。四种配置的完整APP事件摘要与修改前一致。
启动校正和初始化顺序专项通过；中途发现的空白行尾空格已清理。

最终PR档通过：124项单元、16项集成、43组原生及全部格式、lint、布局、架构、接口、文档和卫生检查。
证据：`outputs/runs/app-init-stages-20261103-final/summary.json`。
双板普通APP和STM32 LoaderApp三个目标均0错误0警告，构建汇总见
`outputs/app-init-readable-20261103/builds/summary.json`。未连接硬件或烧录。
