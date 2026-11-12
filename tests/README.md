# 离线测试

统一实机测试草案见[电机统一台架测试用例v1.1](../docs/plans/active/2026-10-31-unified-motor-test-cases-v1.md)。
该用例待评审，包含三相/sin-cos纹波、dq闭环/阶跃、速度、标定波形和协议；不接入默认离线测试执行。

- `unit/`：Python 单元测试及 C 用例；`unit/native/` 为原生 C 编译驱动。
- `integration/`：固件接口边界、工程配置和工具流程测试，使用模拟设备。

从仓库根运行：

```powershell
python tools/run.py check_project_layout
python tests/run.py
python tests/run.py --cc <zig-executable>
python tests/run.py --cc <zig-executable> --reference-source <baseline-position-cascade.c>
```

推荐先执行 `uv sync --locked --extra dev`，然后使用
`uv run python tools/run.py verify --profile pr`。Python 离线测试使用锁定版本的
`numpy`；原生 C 用例使用锁定的 Zig，
部分独立脚本也支持 GCC/Clang；外环运行测试目前按 Zig 命令调用。
测试日志默认写入 `outputs/tests/`，可以用 `--out` 指定。
统一测试入口不会烧录、连接探针或操作电机。

双板状态机与标定入口见 [test_run_state.py](unit/native/test_run_state.py)，四种 APP 镜像配置见 [test_app_orchestration.py](unit/native/test_app_orchestration.py)。Loader 会话与镜像提交见 [loader_upgrade_test.c](unit/loader_upgrade_test.c)，两板运行时及端口模型见 [G431](unit/native/test_loader_runtime.py) 和 [AT32](unit/native/test_at32_loader.py)。

当前 AT32 APP 的 RTT 通道 1 每帧包含八个 int16 槽，后三个槽是 Hall 单圈角与 sin/cos 原码。波形定义见 [app_rtt.c](../firmware/app/composition/app_rtt.c)。独立台架工程有各自的遥测格式。

历史 `3980f92` 等价性比较也有既存失败，统一入口仅在明确提供
`--reference-source` 时进行本次变更的状态等价比较。
