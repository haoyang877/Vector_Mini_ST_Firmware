# YG_MC_HOST

Qt6/C++17 原生上位机维护页面，独立于 APP 和 Loader 构建。当前交付本地镜像校验、
升级任务导出及离线参数交互，设计与两张流程图见
[升级与参数页面 v1](../docs/plans/active/2026-09-29-host-upgrade-parameters-v1.md)。
默认不连接设备，页面明确标记演示数据；CAN FD 与 SWD 的设备执行尚未接线。

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
`windeployqt` 收集插件/运行库，并加入编译器运行库；结果放入 `outputs/host_app/dist_v1/`。
普通启动保持离线；选择“载入演示样本”可检查三种模式，参数页先读取再编辑。
升级页可滚动，在较小窗口中查看完整日志与操作按钮。下拉弹层使用显式浅色背景与深色文字。

APP 必须使用[当前打包工具](../loader/tools/package_image.py)生成 `image.bin` 和同目录的
`manifest.bin`；不复制旧 YI/历史打包器。文件校验复用 `shared/image/image_transfer.c`，
目标地址、schema 和 ABI 来自单份 `shared/targets/vector_mini_st_g431.json`。
任务、快照和截图应导出到 `outputs/`。保存页面预览可运行：

```powershell
outputs/host_app/build/YG_MC_HOST.exe --preview-dir outputs/host_app/preview-windows
```

`--preview-dir` 只读取内置合成样本并保存两张截图后退出，不连接设备。
Windows 字体视觉检查使用 Windows Qt 平台；CTest 使用 offscreen 平台做交互检查。
测试不调用设备、探针、驱动或烧录命令。

## 模块依赖与后续接线

`gui/` 只拥有输入和呈现，调用 `services/`；`services/` 拥有冻结镜像与示例参数状态；
`protocol/` 负责显式小端标量载荷和严格回复解释。共享纯 C 核心独立链接，无 firmware
或 HAL 依赖。设备句柄/供应商 DLL 应留在未来传输边界，不进入 GUI 或共享契约。

设备执行须消费 [shared 契约](../shared/README.md)、
[参数线路契约](../docs/protocols/yg_protocol_parameter_wire_v1.md) 和
[升级线路 v3](../docs/protocols/yg_protocol_upgrade_wire_v3.md)。当前流程图给出接线目标，
页面演示不构造真实事务、重试或设备身份结果；现场验证需另行明确设备与 bench 场景。

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
