# YG_MC_HOST

v1.4.0 是单页 CAN FD 台架工具。默认断开且所有设备值显示未知；没有演示设备、导航、参数写入或标定任务。程序与 APP、Loader 独立构建。

页面只提供：

1. 显式连接/断开。节点在主界面；Python 3、ControlCANFD.dll、同一未重启设备的源节点 2 序号起点在“高级设置”。本轮已知 STOP 序号 6005，现场应手动填起点 10000，下一请求为 10001；默认 0 不适合已有更高历史序号的设备。连接中锁定设置，不自动重连或重放。
2. 真实 GET 1/2/3/108 的 UID、APP/Loader 版本、状态/模式、故障、实际 rpm、Iq 与母线读数。身份最多保留 5 秒，反馈最多保留 500 毫秒；过期、断线显示未知。
3. SPEED 零目标启动、0..300 rpm 设置与顶部 STOP。只在匹配本轮 UID `5700460010504e4831303320`、节点 1、APP 1.3.0、schema12、新鲜 IDLE/状态和台架限值时开放启动。STOP 受理 1/1 后有界查询最终 0/0，再要求新 108 IDLE；失败须现场核对关相。页面确认架空、33 V/母线 3 A/相 6 A，108 的 Iq 不代替相电流监视。
4. 同目录 `image.bin` + `manifest.bin` 的 APP 包校验与显式 v3 升级。只在配套 LoaderApp 1.0.0、真实身份、STOP 最终确认、IDLE/近零速与限值内回读齐备时释放读回端口，交给独立 `upgrade_client.py` 执行；普通 APP 未安装配套 Loader 时明确禁用。升级结果须核对 `summary.json`，现场物理状态另行验收。

构建使用 Qt 6.8+、CMake 和 Ninja：

```powershell
uv sync --locked --extra dev
cmake -S host_app -B outputs/host_app/build -G Ninja "-DCMAKE_PREFIX_PATH=$env:QT_PREFIX" -DCMAKE_BUILD_TYPE=Debug
cmake --build outputs/host_app/build
ctest --test-dir outputs/host_app/build --output-on-failure
uv run python tools/run.py format --check
uv run python tools/run.py lint
```

`YG_MC_HOST.exe --preview-dir outputs/host_app/core-preview` 只保存未连接页面截图并退出，不打开 SDK。真实按钮、命令及回复写在 `outputs/host_app/outputs/host-gui-*/events.jsonl`；APP 升级另写 `outputs/host_app/outputs/host-upgrade-*/summary.json`。这些日志不代替实机验收。Python 运行时和厂家 DLL 由用户显式选择，不随 EXE 分发。
