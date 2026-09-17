# 离线测试与 HIL

- `unit/`：Python 单元测试及 C 用例；`unit/native/` 为原生 C 编译驱动。
- `integration/`：固件接口边界、工程配置和工具流程测试，使用模拟设备。
- `hil/firmware/`：仅 HIL 固件使用的测试支持。
- `hil/profiles/`：关节和运动场景配置，JSON 本身不会写入参数或驱动电机。

从仓库根运行：

```powershell
python tools/run.py check_project_layout
python tests/run.py
python tests/run.py --cc 'C:/path/to/zig.exe'
python tests/run.py --cc 'C:/path/to/zig.exe' --reference-source 'C:/baseline/position_cascade.c'
```

Python 离线测试使用 `numpy`；解释器需预先具备该依赖。原生 C 用例支持 Zig，
部分独立脚本也支持 GCC/Clang；外环运行测试目前按 Zig 命令调用。
测试日志默认写入 `outputs/tests/`，可以用 `--out` 指定。
统一测试入口不会烧录、连接探针或操作电机。

RTT 测试覆盖普通固件的 8 路标定帧和 HIL 固件的 12 路伺服帧；通道单位检查
针对各自配套工程。通用 `pro_lks.lksscope` 保留用户配置，不要求使用 HIL 单位。
历史 `3980f92` 等价性比较也有既存失败，统一入口仅在明确提供
`--reference-source` 时进行本次变更的状态等价比较。
