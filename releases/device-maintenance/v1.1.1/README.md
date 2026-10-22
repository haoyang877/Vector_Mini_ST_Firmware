# Vector Mini 设备维护 v1.1.1

此目录只保存维护软件的版本说明和校验清单，EXE、ZIP 与固件包不纳入 Git。
标签 `maintenance-v1.1.1` 固定仓库中的维护工具源码，不表示当前分支电机 APP 的版本。

## 使用

本机恢复目录为 `outputs/releases/VectorMini_v1.1.1_Loader_v1.0.0_APP_v0.5.3_20261019/`，
双击其中的 `VectorMiniFlasher.exe`。该目录和 ZIP 仅在本地保存，不随 Git 推送。
J-Link 与 USB-CAN DLL 已内置；电脑仍需相应 USB 设备驱动。
其他电脑可按[维护工具说明](../../../docs/guides/loader_flasher_exe_v1.md)从源码构建。

软件提供四页：初装烧录、CAN 固件升级、版本与状态、标定参数。
初装使用合并 HEX，CAN 升级使用 `can_app/manifest.json`。烧录行为、参数区保护和
构建方法见[维护工具说明](../../../docs/guides/loader_flasher_exe_v1.md)。

## 版本与验证边界

| 组件 | 版本与来源 |
| --- | --- |
| 维护工具 | v1.1.1；源码节点 `7e01354c10caceb1eb9048172519e244e9f02f3d`，源码位于 `loader/tools/` |
| 包内 Loader | v1.0.0，12 KiB 布局 |
| 包内 APP | 历史 v0.5.3；源码节点 `abac5dec9599f15b3ee02aea4f0c19dc819343cf` |
| 原包日期 | 2026-10-19 |
| 恢复归档日期 | 2026-10-22 |

这是原包恢复，未重新编译、未升级版本。包内历史 APP 使用 `0x08004000` 入口和
原标准 CAN-FD 1M/1M 协议，不适用于新布局、新参数 ABI 或扩展帧协议。
它不是当前主分支最新 APP，也不包含任何一台电机的个体标定参数。

本次已验证 ZIP 完整性、包内清单的 22 个文件长度和 SHA-256、维护源码匹配，
并运行 EXE 版本、内置 DLL 加载、合并 HEX 和四页界面的离线检查，全部通过。
没有连接硬件或执行烧录，不将本次归档声明为新固件的实机验收或正式发布。
包、EXE、HEX、源码与验证证据的哈希记录在 `manifest.json`。

Loader 工程仍位于 [loader/mdk/Vector_Mini_ST_Loader.uvprojx](../../../loader/mdk/Vector_Mini_ST_Loader.uvprojx)，
上位机入口仍位于 [loader/tools/loader_flasher_gui.py](../../../loader/tools/loader_flasher_gui.py)。
