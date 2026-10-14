# Loader 独立工程入口

本次 `codex/ovp-36v` 接入依据为
[CAN Loader 完整设计方案 v1](../docs/plans/2026-10-13-can-loader-minimal-design-v1.md)：
12 KiB Loader、单 APP、1M/1M CAN-FD、参数区保留及中断后完整重下载。
已接入最小实现、独立APP/Loader构建和离线测试；当前用户验收目标调整为100轮内最多1次失败。
合并回`codex/ovp-36v`的当前APP候选为v0.5.1/build0，保留v0.4.1控制更新；见
[整合记录](../docs/reports/2026-10-14-loader-merge-ovp36.md)。
2026-10-14完成单板100轮全部通过及结束参数区逐字节回读；详见
[测试记录](../docs/reports/2026-10-13-can-loader-bringup-and-endurance.md)。当前仍为开发候选包。

当前实现结构：

```text
loader/
  firmware/  升级核心、协议编解码、G431 Flash/CAN/启动端口
  mdk/       独立Keil工程与链接脚本
  tools/     主机升级、耐久测试与工程生成工具
```

Loader 与 `firmware/` 中的电机 APP 分别编译、分别链接。Loader 核心不依赖
APP 的 `common_inc.h`、电机全局状态、控制循环或动态内存。
协议版本、布局标识、镜像描述和启动交接契约归 `shared/boot_contract.h/.c`。
APP 的升级交接服务归 `firmware/services/boot_handoff/`，需在停机并确认功率输出关闭后再复位。

当前仅支持CAN-FD。APP已重定位到`0x08004000`，RAM首32 B保留为升级邮箱；
Loader以`0x08000000`为入口，参数区`0x0801C000`起16 KiB受独立保护。
构建产物统一放 `outputs/build/keil/`，发布按
[固件发布规范](../docs/guides/firmware_release_standard.md)执行。

当前源码入口：`firmware/`；独立Keil工程：`mdk/Vector_Mini_ST_Loader.uvprojx`。
主机工具：`tools/loader_updater.py`和`tools/loader_cycle_test.py`。
APP+Loader初装合包及指定J-Link烧录：`tools/loader_factory.py`、`tools/flash_factory.ps1`，
详见[初装烧录指南](../docs/guides/loader_factory_package_v1.md)。
Windows单文件GUI入口：`tools/loader_flasher_gui.py`，提供初装烧录、CAN升级、版本/状态、标定参数四页，
支持选择合并HEX和探针并自动读取UID，标定页只读当前schema10保存值及完整1024点LUT；
当前发布版内置J-Link和USB-CAN DLL、自动加载，使用侧边导航、任务卡片与折叠日志；
详见[图形烧录工具](../docs/guides/loader_flasher_exe_v1.md)。
离线验证：`python tests/loader/run_tests.py --cc <native compiler>`。
