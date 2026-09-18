# 离线测试与 HIL

- `unit/`：Python 单元测试及 C 用例；`unit/native/` 为原生 C 编译驱动。
- `integration/`：固件接口边界、工程配置和工具流程测试，使用模拟设备。
- `hil/firmware/`：仅 HIL 固件使用的测试支持。
- `hil/profiles/`：关节和运动场景配置，JSON 本身不会写入参数或驱动电机。

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

RTT 测试覆盖统一的 4 通道帧：固件编码由 `run_position_servo_tests` 的夹具执行，
帧布局、JScope 描述符与三个波形工程的一致性由 `test_rtt_control_telemetry.py` 校验；
通用 `pro_lks.lksscope` 保留用户自定义单位，只校验通道数量。
历史 `3980f92` 等价性比较也有既存失败，统一入口仅在明确提供
`--reference-source` 时进行本次变更的状态等价比较。
