# YG_MC_HOST

Qt6/C++17 原生电机调试工作台，独立于 APP 和 Loader 构建。启动先显示连接页，
连接演示 APP 后进入首页；会话内有首页、调试工作台、参数配置、标定、辨识、记录与诊断、
设备维护七个工作区。[基础工作区恢复记录](../docs/plans/active/2026-09-29-host-workbench-restore-v1.md)
说明从两页维护入口恢复完整工作台的范围及验收。

升级是设备维护工作区的一项功能。当前交付本地镜像校验、升级任务导出及离线交互，
维护设计与两张流程图见
[升级与参数页面 v1](../docs/plans/active/2026-09-29-host-upgrade-parameters-v1.md)。
默认不连接设备，页面明确标记演示数据；CAN FD 可显式连接只读 GET，SWD 与功率/写操作尚未接线。

## 基础工作区

| 工作区 | 当前可操作内容 |
| --- | --- |
| 连接 | 选择节点及正常/只读/故障/Loader 演示场景，或显式打开 CAN FD 只读端口；连接不自动使能或采集 |
| 首页 | 设备身份、版本、状态和最近任务；未提供的身份/版本保持未知，本地目标配置另列 |
| 调试工作台 | 位置 rad、转速 rad/s、Iq A 三组共用时间轴的波形；显式采集、记录、冻结、时间窗；运动与常用参数同屏 |
| 参数配置 | 同一份活动快照和草稿；常用视图编辑与完整参数表同步，应用一项不丢失其他草稿 |
| 标定 | 角度传感器、电角零位、机械参考零位、齿槽演示任务；阶段、进度、取消和候选状态 |
| 辨识 | 电阻、电感、转动惯量、摩擦演示任务；结果含单位及参照说明，应用演示与设备保存分开 |
| 记录与诊断 | 有界事件、显式清除演示故障、完整 CSV 导出与本程序演示 CSV 回放；回放禁用控制 |
| 设备维护 | 保留下面三种升级模式、本地校验、任务导出和步骤演示 |

工作台不整体滚动，顶部停止并失能入口始终保留；切换工作区或冻结显示不终止采集、记录
或任务。演示 APP 操作需显式解锁；任务要求失能、无故障且互斥。停止清除旧目标，重新使能
不会重放上次运动。只读或 Loader 场景不允许运动/参数写入。

显示窗口保存最多 3000 点，完整本次记录最多 30000 点，满后停止记录；事件最多 100 条。
CSV 保留演示节点、模型时间、SI 数值、缺失参考和数据间隙，不把模型数据宣称成设备采集。
目前只回放本程序导出的演示格式。位置曲线中的虚线标为“演示目标”，不冒充设备轨迹参考。
速度环增益按当前参数目录标为内部系数；它们和电流环/电机模型字段保持只读演示权限。
只读物理 CAN FD 按当前共享公司协议查询身份、能力、启动编号与反馈；历史 RAW11/YI 接口未接回本产品。

## 功能与实际边界

| 功能 | 当前可用 | 实机接线要求 |
| --- | --- | --- |
| 单独升级 Loader | BIN 范围/向量检查，独立 Loader 安装任务 | SWD 分区安装；审核 Loader 构建及布局信息，保留 APP/参数 |
| 单独升级 APP | 当前 image.bin + manifest.bin 真实共享校验，APP 任务 | CAN FD 升级 v3：发现、nonce、下载、完整校验、激活及健康确认 |
| Loader + APP | 两个独立写域的校验与任务，不生成全片覆盖包 | SWD 分区安装；备份参数、逐域读回并重新发现 |
| 读取参数 | 离线标量模型读取，显示类型、单位、权限及 revision | Type 120，消费实际 READ access 和活动值 |
| 写入参数 | 示例模型校验、expected_revision、写后读回 | Type 121，权限/状态/范围由设备参数所有者裁决 |
| 保存参数 | 示例异步受理/完成，活动与已保存 revision 分开显示 | Type 122，实际提交完成后才能确认持久化 |

当前 Loader 仅允许 CAN 写 APP 槽，无法通过该线路覆盖 Loader 自身。
导出 `job.json` 的状态是 `prepared_only`，设备未执行；合成样本永久标记 `synthetic`，
不能用于设备安装。任务冻结已校验文件内容并记录 SHA256；Loader BIN 的向量检查
不能替代版本/布局兼容性或发布审核。APP CRC32 也不提供来源认证。

参数目录是页面字段定义，示例范围不是产品范围。READ 权限控制编辑；只读、标定所有者
字段及尚无真实来源的 SN 不开放通用写入。WRITE 只修改活动值，SAVE 的 ACCEPTED
不代表完成；保存期间仍可读值，但禁用再次写入和保存。活动快照不是完整 Flash 备份。

## 构建与运行

安装 CMake、Ninja 以及与 Qt 6.8 以上版本匹配的 C/C++ 工具链。在仓库根运行；
将 `QT_PREFIX` 指向实际 Qt 安装前缀，并将编译器、Qt bin、CMake、Ninja 加入 PATH：

```powershell
uv sync --locked --extra dev
cmake -S host_app -B outputs/host_app/build -G Ninja "-DCMAKE_PREFIX_PATH=$env:QT_PREFIX" -DCMAKE_BUILD_TYPE=Debug
cmake --build outputs/host_app/build
ctest --test-dir outputs/host_app/build --output-on-failure
uv run python tools/run.py format --check
uv run python tools/run.py lint
uv run python tools/run.py verify --profile pr
```

Windows 主程序为 `outputs/host_app/build/YG_MC_HOST.exe`。独立打包时使用匹配 Qt 的
`windeployqt` 收集插件/运行库，并加入编译器运行库；完整工作台结果放入 `outputs/host_app/dist_v2/`。
普通启动显示连接页；连接演示 APP、进入工作台、解锁后，显式使能并提交目标。
采集需单独开始；调参前停止并失能，先读取再编辑。设备维护选择“载入演示样本”可检查三种模式。
升级页可滚动，在较小窗口中查看完整日志与操作按钮。下拉弹层使用显式浅色背景与深色文字。

APP 必须使用[当前打包工具](../loader/tools/package_image.py)生成 `image.bin` 和同目录的
`manifest.bin`；不复制旧 YI/历史打包器。文件校验复用 `shared/image/image_transfer.c`，
目标地址、schema 和 ABI 来自单份 `shared/targets/vector_mini_st_g431.json`。
任务、快照和截图应导出到 `outputs/`。保存页面预览可运行：

```powershell
outputs/host_app/build/YG_MC_HOST.exe --preview-dir outputs/host_app/preview-workbench
```

`--preview-dir` 推进明确标记的演示会话、读取内置合成镜像，保存连接页与七工作区截图后退出，
不连接物理设备。演示不会执行厂家命令或烧录。
Windows 字体视觉检查使用 Windows Qt 平台；CTest 使用 offscreen 平台做交互检查。
测试不调用设备、探针、驱动或烧录命令。

## 模块依赖与后续接线

`gui/` 只拥有输入和呈现，调用 `services/`；`services/` 拥有演示会话、完整记录、冻结镜像与示例参数状态；
`protocol/` 负责显式小端标量载荷和严格回复解释。共享纯 C 核心独立链接，无 firmware
或 HAL 依赖。设备句柄/供应商 DLL 留在 Python `ports/`，不进入 GUI 或共享契约。

设备执行须消费 [shared 契约](../shared/README.md)、
[参数线路契约](../docs/protocols/yg_protocol_parameter_wire_v1.md) 和
[升级线路 v3](../docs/protocols/yg_protocol_upgrade_wire_v3.md)。当前流程图给出接线目标，
升级与参数页面演示不构造真实写事务、重试或设备身份结果；只读 GET 有独立端口路径。
现场验证需另行明确设备与 bench 场景。

既有 `examples/company_canfd_position.c` 保留为独立 C99 帧编解码示例，配套
[说明与黄金向量](../docs/protocols/canfd_application_example.md)。现场主机工具仍位于
`tools/bench/`，离线分析工具位于 `tools/analysis/`。UART、USB CDC 与旧 YI 协议不属于入口。

## CAN FD 最小升级客户端

`upgrade_client.py` 独立于 GUI，默认 `inspect` 仅完整检查 v3 包。
`upgrade` 通过 `ports/controlcanfd.py` 显式打开 Windows SDK；入口要求 12B 实际 UID、目标节点、
独占无动力台架和 outputs 证据目录。升级前检查 MCU、配套版本、布局、schema 与 CRC；
一次一个请求，至多四次保持原身份重试，激活后核对新 APP 并通过 Type 57 确认，再读持久状态。
`--rescue-window` 反复发送空 STATUS 捕获复位后的 50ms 窗口，只驻留；身份检查通过后才擦写。
无法确认停止或关相时中止交接；任何失败不恢复电机运行。

已提供真实 Loader C 内核的离线模型及丢回复、错 UID、损坏包测试；硬件 SDK 未在本轮执行。
GUI 使用相同 v3 镜像契约，导出任务仍为 `prepared_only`，不能用模拟成功作为实机证据。

## 只读线路回读边界（2026-10-08 候选）

`services/device_readback` 使用共享 CAN ID 与统一帧核心，按单节点、单请求关联
Type 1 页 0/1/2、Type 2、Type 3、Type 108 与 Type 110 回复。应用层核对状态、有效位和 SI 比例；
身份最多显示 5 秒，反馈和 STOP 瞬时确认最多显示 500 毫秒。STOP result=1 只是
受理；result=0 也不表示转子静止或此后一直关相。后续状态回读会撤销这条瞬时确认。
生产 SN 按 ASCII 显示，UID 与有效启动编号变化会清除旧会话快照。

连接页需用户显式选择节点、Python 3 可执行文件和厂家 SDK DLL，再点击 CAN FD 连接。
`device_reader.py` 与现有 `upgrade_client.py`、`ports/controlcanfd.py` 在构建目录一同复制；
Python 运行时和厂家 DLL 不随 EXE 打包。只读辅助进程独占打开现有端口，顺序查询
Type 1 页 0/1/2、Type 2、Type 3，并按需轮询 Type 108；超时、错误帧、断线时关闭端口
并使回读失效。辅助进程严格只接受这四类 GET，不发送 STOP、参数写入或功率命令。
目前只有假端口/假 DLL 和 Qt 离线测试，没有连接实机，因此不能称为硬件 PASS。

线路回读只在未连接演示的 CAN FD 页面展示，显式区分未连接的离线帧回放和已打开的只读端口；
演示波形与设备回读不混用，所有功率和写参数入口仍禁用。旧 108 八态不直接转换为
新六类业务视图。整计划父/子 ticket、完成步骤和 Flash 提交状态在 H 线路冻结前
均为未知。CURRENT 单位为 A；电感、磁链、Nm 和型号映射的缺口在页面显式说明。
[范围与验收计划](../docs/plans/active/2026-10-08-host-readback-boundary-v1.md)。
