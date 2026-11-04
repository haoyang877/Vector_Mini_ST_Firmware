# APP启动分步表达 v1

将App_Start中的短路调用改为显式if，并说明通信初始化成功后才启动硬件。
保留运行时启动、失败关相、最后启动看门狗的顺序；初始化已失败时仍跳过通信启动。
不改变任何接口、故障策略或控制算法。
基线：`outputs/app-start-readable-20261103/app.before.c`；回退仅撤销本次if及注释。
验收：现有启动顺序检查、完整APP四配置前后轨迹、完整PR与正式APP构建。不烧录。

## 结果

已改为显式if并补充注释；通信未就绪时不调用Start，启动失败仍关相，随后始终执行原看门狗启动逻辑。
双板及有无Loader四种完整APP事件轨迹与修改前一致，覆盖初始化失败、启动失败及看门狗行为。
完整PR通过：124项单元、16项集成、43组原生，以及格式、lint、布局、架构、接口、文档和卫生检查。
证据：`outputs/runs/app-start-readable-20261103/summary.json`。
双板普通APP和STM32 LoaderApp三个目标均0错误0警告，日志在
`outputs/app-start-readable-20261103/builds/`；未连接硬件或烧录。
