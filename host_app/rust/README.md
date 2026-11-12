# Rust 上位机构建

在仓库根目录执行 `cargo test --manifest-path host_app/rust/Cargo.toml --locked` 和
`cargo build --manifest-path host_app/rust/Cargo.toml --release --locked`。建议将
`CARGO_TARGET_DIR` 指向仓库的 `outputs/`。生成的程序默认断开设备；`--preview-dir <目录>`
只生成离线界面截图，不打开 CAN SDK。

每次发布更新 `Cargo.toml` 的版本并同步 `Cargo.lock`；窗口标题和界面从该版本自动显示，
发布目录和文件名标注同一版本号，并记录变更、校验结果及实机验证状态。
v2.7.2 修复 BUSY/单次只读丢包/操作拒绝导致的误断联；重试有次数和总时限，
持续失联和未知运动结果仍停机，不自动重发运动命令。
故障日志自动保存在 `%LOCALAPPDATA%/YG/YG_MC_HOST/logs`，数据与日志页可查看路径及导出JSONL。
每个分片约8MiB；正常报文只保留最近64帧，异常时才落盘。离线预览不创建设备日志。
连接页在用户点击扫描后使用 ControlCANFD 查询可用的 USBCANFD-200U 和设备身份；
优先查找程序目录或 `drivers` 下的 `ControlCANFD.dll`，也可在高级设置中手动选择。
扫描使用通道 0 和连接页所选的 CAN FD 速率，默认 1M/5M。AT32F456 的仲裁 500K、1M 可与数据 500K、1M、2M、3M、4M、5M 独立组合。扫描期间适配器由上位机独占，节点范围 1–254，只发送 GET_INFO。
占用中的适配器无法打开，超时设备可能遗漏。扫描后须选定设备再连接，连接时重新核对 UID。
趋势观察期间 CAN108 默认目标为 60 Hz；曲线线程按 60 Hz 周期准备不可变绘制快照，保留尖峰与缺测分段。
界面分别显示最近 2 秒的实际接收频率和曲线新数据更新频率。
这是主机接收时间，不是设备同步采样时间；实际刷新及接收频率尚须实机验证。
导出保存到 `%LOCALAPPDATA%/YG/YG_MC_HOST/captures`，同时记录曲线新数据更新的时间戳；重复绘制同一份数据不计入曲线刷新率。

v2.6.0 调试页提供 Vq 电压、Iq 力矩、速度、绝对位置四种模式。力矩输入使用 Iq（A）。
v2.7.1 将模式下拉框、使能/失能、本地 STOP 和目标统一到右侧；
运动控制不再需要台架确认勾选，身份、故障、反馈、配置和停机门禁仍生效。
失能发送 STOP 并确认空闲，
使能所选模式后才能下发同模式目标；位置使能保持当前位置。
主机调试范围 Vq ±6 V、Iq ±6 A、速度 0…300 rpm，位置每次目标相对实测偏移 ≤10°；
设备电压、电流、热和行程约束仍然生效。四模式实机控制尚未验证。

Windows GNU 工具链需要可用的 `dlltool` 和链接工具。如果默认链接生成的程序不能正常启动，
可以使用 Rust 自带的 LLD 链接器，并在构建前将兼容的 GNU `dlltool` 加入 `PATH`：

```powershell
$taskRustSysroot = (& rustc --print sysroot).Trim()
$taskRustLld = Join-Path $taskRustSysroot 'lib/rustlib/x86_64-pc-windows-gnu/bin/rust-lld.exe'
$env:CARGO_ENCODED_RUSTFLAGS = @('-C', 'linker-flavor=ld.lld', '-C', "linker=$taskRustLld", '-C', 'link-self-contained=yes') -join [char]31
cargo build --manifest-path host_app/rust/Cargo.toml --release --locked
```

此方式需在本机准备与 Rust 目标兼容的 `dlltool`；构建成功后仍需运行程序验证。
J-Link/SWD 维护入口依赖仓库中的维护脚本及 Python 环境；在仓库根目录启动上位机，
或在维护页面高级设置中填写相应路径。普通 GUI 程序不随包提供厂家 CAN SDK。

只读实机测试入口（需要明确 UID 和目标板，窗口真实运行，默认观察 60 秒后导出并断开）：

```powershell
YG_MC_HOST_v2.4.3.exe --target yg_esc_v1_at32f456 --bench-observe <输出目录> --uid <24位MCU_UID>
```

该入口复用真实扫描、UID 选择、CAN 会话与曲线线程，只请求静止观察，不请求运动或升级；
设备会话已有故障保护仍然有效。只有指定 UID 处于 IDLE、零故障且近零速时才开始观察。
`samples.csv` 保存实际反馈；`curve.csv` 保存采用新数据快照的时间戳；`run.json` 保存运行结果。

型号配置位于参数管理页。合并版 APP 通过 GET_CAPS bit17 声明 190–193 型号配置命令，bit13 声明 500 ms 运动租约。
支持 HT8115-4、YQ-5708、YQ-5740 与现有轴角色/节点配对；不修改生产身份编号。
先失能并确认空闲，指定型号、轴位置、速度限制和已核实热参数，再保存配套参数；
保存动作包含内存候选与完整UID绑定提交，Flash回读后才显示已保存。
随后显式应用并重启，按原UID及新节点验证生效。旧标定整体清除，须重新标定。
GUI 标定执行仍未接入，可使用现有标定工装；不会自动使能或重放目标。
PITCH 在默认速率下已实测配置读取、准备及错误票据拒绝。成功提交并应用新配置会清除旧标定。
