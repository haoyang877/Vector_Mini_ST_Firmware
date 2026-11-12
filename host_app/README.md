# YG_MC_HOST

双板开发候选增加目标选择与SWD维护入口，复用`loader/tools/device_maintenance.py`完成全片备份、
保存参数/LUT查看、保护区比较和AT32原样迁移/初装。Windows v3.1.2 发布包完整解压后即可使用随包 Python、脚本与 J-Link 器件库。
中控可打开整机烧录页并自动识别 USB 探针。详见[构建与发布](rust/README.md)。

Rust 候选 v2.7.1 位于 `rust/`，配套版本、layout、schema 及 ABI 读取 shared 目标契约；
AT32 PITCH 既有实测配套 APP 1.6.0 / Loader 2.2.0；v2.6.0 四模式尚未实机验证。空闲态无 Iq 时，先按 STOP，等待 0/0 和 IDLE 回读后才可零速启动或升级；运行期间仍要求有效 Iq。
中控打开「添加设备」后自动扫描可打开的 USBCANFD-200U 索引 0–15
及通道 0 的节点 1–254，使用只读 GET_INFO 识别目标和 UID，选中设备后仍须显式连接，
连接时重新核对 UID。扫描不会发现被其他进程占用的适配器，超时节点可能遗漏；
手动连接仍在高级设置。启用原生辅助功能接口，用于键盘操作和 GUI 验证。
v2.7.1 调试页使用右侧集中控制面板，模式下拉选择、使能/失能、红色 STOP 与目标就近放置。
运动控制取消台架勾选，设备身份、反馈与状态门禁保持；升级与配置写入确认独立保留。
v2.7.0 增加CAN仲裁/数据段波特率配置，以及需配套APP1.6.1的CAN型号配置事务。
支持明确指定HT8115-4、YQ-5708、YQ-5740，配套参数保存、回读与显式重启；旧标定清除后须重新标定。
v2.6.0 调试页支持 Vq（V）、力矩 Iq（A）、速度（rad/s）、位置（rad），按设备 GET_CAPS 开放。
切换模式先 STOP 并确认空闲；位置启动保持当前位置。调试限值与详细要求见[当前需求](../docs/guides/yg_mc_host_requirements_v1.md)。
v3.0.0候选采用仪器工作台布局，常驻型号/连接/固件版本，提供分类全参数表；保留原有连接、四模式、升级、标定及诊断。实机验证与限制见当前证据。
开始观察后 CAN108 轮询默认目标为 60 Hz（约 16.67 ms）；工具栏分别显示实际接收与曲线新数据更新频率。
CAN 快照直接发送给曲线线程，由它负责时间窗、范围和保留极值的抽稀，UI 线程采用不可变快照绘制。实际速率取决于设备、总线和主机，须实测确认。暂停观察后恢复 200 ms 周期。
本轮范围和验收见[GUI 实机验证记录](../docs/plans/completed/2026-10-21-host-gui-bench-v1.md)。
界面候选存在不代表实机通过；实际结果以本轮 `outputs/host-live-test/` 记录为准。

```powershell
$env:CARGO_TARGET_DIR = "$PWD/outputs/host-rust-cargo"
cargo fmt --manifest-path host_app/rust/Cargo.toml --check
cargo clippy --manifest-path host_app/rust/Cargo.toml --all-targets -- -D warnings
cargo test --manifest-path host_app/rust/Cargo.toml --locked
cargo build --manifest-path host_app/rust/Cargo.toml --release --locked
```

Windows GNU 链接和维护脚本的运行条件见 [Rust 构建说明](rust/README.md)。

v1.4.4 是单页 CAN FD 台架工具，配套共享目标契约的 schema 14 / 参数 ABI 0x000E08F0。默认断开且所有设备值显示未知；没有演示设备、导航、参数写入或标定任务。程序与 APP、Loader 独立构建。

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
`bin/target_contract.py` 与 `bin/contracts/<目标名>.json` 由 CMake/Qt 原生安装规则一起部署。
CMake 的 `YG_LOADER_TARGET_FILE` 选择构建契约，默认 G431；生成目标头后再配置主机工程。
CLI 支持 `--target` 名称或 JSON 路径，错误目标不回退；GUI 使用编译时所选目标启动 CLI。
契约直接来自 shared，不手工维护副本。部署目录可整体移出仓库；只复制 EXE 不构成可运行工具。
升级包需要同目录的 `image.bin`、`manifest.bin`、`package.json`；GUI 校验镜像后，
Python 客户端还会独立核对元数据和文件哈希，不能省略其中任何文件。

`YG_MC_HOST.exe --preview-dir <目录>` 只保存未连接页面截图并退出，不打开 SDK。
GUI 日志与升级 `summary.json` 写入程序父目录的 `outputs/host-gui-*`、`outputs/host-upgrade-*`，
与证据目录约束一致。这些日志不代替实机验收。Python 运行时和厂家 DLL 由用户显式选择，不随 EXE 分发。
