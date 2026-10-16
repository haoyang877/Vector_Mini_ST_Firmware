# YG_MC_HOST

v1.4.3 是单页 CAN FD 台架工具。默认断开且所有设备值显示未知；没有演示设备、导航、参数写入或标定任务。程序与 APP、Loader 独立构建。

页面只提供：

1. 显式连接/断开。节点在主界面；Python 3、ControlCANFD.dll 和源节点 2 的手动控制序号起点在“高级设置”。不自动重连或重放。第一条真实 GET_INFO 固定本次 UID；完整产品/硬件/schema/ABI 回读兼容后，用户核对显示的 UID 并确认台架才可操作。会话 UID 变化会清除确认、禁用新启动/目标并请求 STOP，需人工检查后重新连接；不会把旧确认移给另一设备。
2. 真实 GET 1/2/3/108 的 UID、APP/Loader 版本、状态/模式、故障、实际 rpm、Iq 与母线读数。身份最多保留 5 秒，反馈最多保留 500 毫秒；过期、断线显示未知。兼容性来自共享目标契约，不固定单台 UID 或 APP 1.3.0。SPEED操作要求新鲜GET_CAPS明确声明速度功能位、mode2位及必需CAN FD/分片/反馈位；缺失或过期时按钮、请求排队与实际发送均禁止START/SET。旧APP仍报告模式0时保持可读取、STOP和合法配套升级，不猜测支持或降级旧协议。能力位表示已实现路径，不等于本候选实机验收。
3. SPEED 零目标启动、0..300 rpm 设置与顶部 STOP。保留架空、33 V/母线 3 A/相 6 A 保守包络；108 的 Iq 不代替相电流监视。STOP 受理 1/1 后有界查询最终 0/0，再要求新 108 IDLE；失败须现场核对关相。只读查询不消耗控制序号；本工具按 UID/source2 在用户级 QSettings INI 保存控制高水位，并预留辅助进程退出的一个 STOP。功率命令发送前必须同步保存；写入失败禁用新启动/目标，STOP 仍会尝试。序号 65535 后回到 1。高级起点 0 自动恢复本工具记录；首次接入或其他工具曾向同一设备发命令时，须根据实际历史明确填写非零起点。本机记录不能自动推断其他主机历史；不确定时不猜起点、不自动复位设备。手动起点按 16 位半范围顺序校验，不能早于本机已存高水位。
4. 同目录 `image.bin` + `manifest.bin` 的 APP 包校验与显式 v3 升级。只在配套 LoaderApp、真实身份、STOP 最终确认、IDLE/近零速与限值内回读齐备时释放读回端口，交给独立 `upgrade_client.py` 执行；普通 APP 未安装配套 Loader 时明确禁用。兼容新包不要求升级工具同步更换 APP 版本号，结果必须匹配本次所选包的实际版本、CRC32 与固定 UID；升级期间不恢复运动。`summary.json` 与现场物理状态仍须另行验收。

构建使用 Qt 6.8+、CMake 和 Ninja：

```powershell
uv sync --locked --extra dev
cmake -S host_app -B outputs/host_app/build -G Ninja "-DCMAKE_PREFIX_PATH=$env:QT_PREFIX" -DCMAKE_BUILD_TYPE=Debug
cmake --build outputs/host_app/build
ctest --test-dir outputs/host_app/build --output-on-failure
cmake --install outputs/host_app/build --prefix "$PWD/outputs/host_app/package"
uv run python tools/run.py format --check
uv run python tools/run.py lint
```

安装目录的 `bin/YG_MC_HOST.exe` 为入口；Qt 运行库、两个 Python 客户端、`ports` 和
`bin/contracts/vector_mini_st_g431.json` 由 CMake/Qt 原生安装规则一起部署。
契约直接来自 shared，不手工维护副本。部署目录可整体移出仓库；只复制 EXE 不构成可运行工具。
升级包需要同目录的 `image.bin`、`manifest.bin`、`package.json`；GUI 校验镜像后，
Python 客户端还会独立核对元数据和文件哈希，不能省略其中任何文件。

`YG_MC_HOST.exe --preview-dir <目录>` 只保存未连接页面截图并退出，不打开 SDK。
GUI 日志与升级 `summary.json` 写入程序父目录的 `outputs/host-gui-*`、`outputs/host-upgrade-*`，
与证据目录约束一致。这些日志不代替实机验收。Python 运行时和厂家 DLL 由用户显式选择，不随 EXE 分发。
