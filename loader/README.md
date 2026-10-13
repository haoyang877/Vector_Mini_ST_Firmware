# Loader 独立工程入口

本次 `codex/ovp-36v` 接入依据为
[CAN Loader 完整设计方案 v1](../docs/plans/2026-10-13-can-loader-minimal-design-v1.md)：
12 KiB Loader、单 APP、1M/1M CAN-FD、参数区保留及中断后完整重下载。
已接入最小实现、独立APP/Loader构建和离线测试；在线连续1000次验收尚未完成，不能视为已发布。

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
离线验证：`python tests/loader/run_tests.py --cc <native compiler>`。
