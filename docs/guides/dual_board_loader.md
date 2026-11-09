# 双板 Loader 开发候选与维护

核对日期2026-11-06，Loader v2.2.0 / APP v1.6.0。支持G431和AT32F456CCU7独立构建，
共用现有升级状态机和镜像核心。新候选未执行实机验收，不作为量产发布包。
本次已同步重构分支`ba800000`，APP入口以Loader设计为准；
离线结果见[同步记录](../plans/completed/2026-11-06-dual-board-loader-sync-v1.md)。

## 功能与固定边界

- Type1页0..2读取身份、版本和状态；Type50..55完成会话、分块下载、CRC检查及激活。
- Type56安全停机交接，等待真实发送完成后复位；Type57确认一次试运行的APP。
- 50 ms启动救援窗口；坏镜像、未激活镜像、已失败试运行留在Loader。
- 5 s确认期限与继承看门狗；通信或组合层健康失效后停止所有喂狗路径。
- 完整Flash双次备份、参数/LUT只读查看、UID绑定、受控初装、保护区逐字节比较。
- AT32旧参数整16 KiB原样迁移，保留schema14、节点、零位、LUT、身份及尾部字节。

通信协议、波特率1M/5M、描述符格式、命令、结果码及重试时序保持现有契约。
普通CAN更新只能写APP槽；Loader无参数/生产身份写接口。单槽升级中断后重新完整下载，
不承诺恢复旧APP，也不支持CAN更新Loader。

| 区域 | 两板地址（含末字节） | 容量 |
| --- | --- | --- |
| Loader | 0x08000000..0x08003FFF | 16 KiB |
| APP | 0x08004000..0x0801BFFF | 96 KiB |
| 参数 | 0x0801C000..0x0801F7FF | 14 KiB |
| 生产身份 | 0x0801F800..0x0801FFFF | 2 KiB |

两板所有运行分区均在Flash前128 KiB。AT32物理Flash仍为256 KiB；上半部只保留旧数据，
新Loader/APP不会使用它。RAM末32字节是一次性邮箱；AT32保留32 KiB `ER_FAST`代码执行区。
两板的分区起址、容量及逻辑擦写粒度由回归检查保持一致。LoaderApp均从`0x08004000`
启动并继承Loader看门狗，只允许健康及试运行确认路径喂狗。

| 目标 | MCU标识 | 硬件标识 | 布局 | Keil APP工程 |
| --- | --- | --- | --- | --- |
| `vector_mini_st_g431` | 0x04310080 | 1 | 6 | `projects/stm32g4/MDK-ARM/ESC-APP-STM32G431-LoaderApp.uvprojx` |
| `yg_esc_v1_at32f456` | 0x04560100 | 2 | 7 | `projects/at32f456/application/ESC-APP-AT32F456-LoaderApp.uvprojx` |

AT32身份在本仓库目标表登记，产品标识沿用0x59473236；不是新增线协议字段。
不同板卡包严格互拒。生产编号分配在正式发布前与工装核对。

## 构建和日常Keil下载

修改目标JSON后，对对应目标分别运行：

```text
python loader/tools/generate_target.py --target yg_esc_v1_at32f456
python tools/build/generate_loader_app_project.py --target yg_esc_v1_at32f456
python tools/build/build_loader_bundle.py --target yg_esc_v1_at32f456 --uv4 <UV4.exe> --fromelf <fromelf.exe> --token 0x20261102 --output outputs/<新包目录>
```

G431将目标名换成`vector_mini_st_g431`。两套工程各自生成AXF、HEX、BIN、CAN包和哈希清单。
后处理验证唯一Flash加载段在APP槽内，SRAM代码装载副本与填充计入预算。
调试AXF中的三个标记预确认，CAN包保持三个标记擦除态；二者不可互换。

日常打开根工作区`YG-ESC.uvmpw`选择对应LoaderApp，采用扇区擦除和下载后校验，
关闭整片擦除、强制设置PC/SP/VTOR和Run to main。详见[Keil步骤](keil_loader_app_debug.md)。
这个配置下只更新APP；错误的全片擦除设置仍会破坏保护区，必须用下载前后快照验证。

## 维护工具

安装依赖`uv sync --locked --extra dev --extra maintenance`。
命令入口`loader/tools/device_maintenance.py`，Rust工作台“SWD维护”页使用同一脚本。
硬件操作必须显式指定`--target`、`--probe`、24位十六进制`--uid`、新`--out`目录及
`--exclusive-bench`。指定单只探针，停止电机并关闭其他调试器和 CAN 会话。母线可以保持带电，维护工具核对六路输出已关闭后才继续。旧参数 `--exclusive-no-power` 仍可使用。
无硬件参数的`parameters`、`verify`和`prepare-migration`只处理本地文件。

1. `backup`读取真实UID、物理容量、六路输出状态，以每笔最多4 KiB的SWD事务读取两份完整物理Flash。
   两份全片逐字节一致才接受；短读或差异直接失败，不靠重试挑选一致的数据。
   维护CLI显式关闭J-Link Flash/内存缓存，退出保留CPU原状态及APP的DWT时基。
   生成`summary.json`、`snapshot/flash.bin`、`snapshot/parameters.bin`及CRC/SHA256。
   不主动halt/reset/run；对目标CPU无调试状态写入。
2. `parameters --input <parameters.bin>`显示保存值与1024点LUT；不写回，不把保存结果当作当次标定。
3. `verify --target <目标> --input <下载前flash.bin> --after <下载后flash.bin>`
   比较APP槽以外的每个字节，包括Loader、完整参数/身份和AT32上半区。
4. 只有本地保护区比较通过才报告`protected_regions_verified=true`；CAN独立成功不代表这项已验证。

## AT32首次迁移与初装

1. 用上述backup保存完整256 KiB及摘要。旧参数源为0x0803C000..0x0803FFFF。
2. 执行`prepare-migration --input <snapshot目录> --source-sha256 <flash摘要> --out <新迁移目录>`。
   工具校验schema14和记录CRC，复制全部16 KiB，不重新序列化；输出绑定UID与完整源Flash摘要。
   旧区非空却未提供迁移包、格式未知或已是layout7时，初装拒绝操作。
3. 执行`install --target yg_esc_v1_at32f456 --bundle <已校验配套包> --migration <迁移目录>`，
   同时提供前述硬件参数。再核对UID和关相；暂停前设置WDT调试冻结，不改选项字节。
   暂停后将已运行WDT延长为Loader使用的配置并回读，防止CPU执行RAM烧录算法时被原APP短WDT复位。
4. 工具先写未激活APP，再按显式迁移包写新参数，最后写Loader；全片双次回读匹配后，
   以完整首APP页提交激活标记，保留attempt/confirm擦除态。这样避免SWD算法按页擦除破坏向量。
   最后再次全片比较，仅允许计划的三块区域变化；旧参数源保持不变。
5. 成功或失败均保留CPU暂停，不自动运行。关闭探针失败也报告失败。恢复运行前检查日志和备份；
   正常从Loader入口复位后会撤销调试WDT冻结。APP只有一次未确认试运行机会，
   用现有Type57流程及时确认；超时后可用现有CAN客户端重新完整升级。

初始化工具不是日常APP下载入口。G431初装不带迁移包，保留完整现有尾区。
生成的整机SWD HEX供审查地址和恢复材料，不应替代上述受控提交步骤。
任何写入阶段断电均用匹配UID的原始完整备份恢复；不能用默认参数替代丢失数据。

## 验收与限制

`verify --profile release`构建两板普通APP、Loader及LoaderApp，默认不连接硬件。
显式台架脚本`tools/bench/loader_acceptance.py`在升级前备份后释放调试器复位捕获，复用现有CAN客户端，每轮前后备份和比较保护区；
每板100轮、掉电、Keil重复下载、WDT实测及重定位快环时序均须在明确台架上下文另行执行。
方案见[双板计划](../plans/active/2026-11-02-dual-board-loader-v1.md)。
