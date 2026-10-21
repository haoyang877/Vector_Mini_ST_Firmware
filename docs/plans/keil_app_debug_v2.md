# Keil APP 无脚本调试入口 v2

日期：2026-10-15。分支：`codex/ovp-36v`。

目标：删除 DebugApp.ini，清空普通 APP/HIL 的 DebugOpt/tIfile，
统一从 Loader 校验后进入 APP；保留 Load Application 与 Run to main。
原先绕过 Loader 的调试入口行为不再保留。

不变项：固件逻辑、记录编码、参数 ABI、Flash 分区、下载挂钩、编译优化和探针选择。
按字节备份两个 uvoptx；只替换各自一个 tIfile 字段，不重写整个 XML。
历史说明保留并标记被替代，当前操作说明同步更新。

验收：无工程脚本引用；普通/HIL 构建、目录检查、Loader 离线套件通过。
用户授权不启动电机的验证：通过 J-Link 确认现有目标 UID、暂停及关相，
双读备份 Flash；从 Loader 复位，以硬件断点停在 APP main，
不写 PC/SP/VTOR、不下载新固件、不执行电机初始化；回读全 Flash 必须不变。
若关相或暂停不能确认，停止，不自动重试。
若接入时 CPU 正在运行，只允许先确认两次关相一致再暂停，并重新核对状态。
Keil 图形界面缓存必须由用户重新打开工程加载；不将 SDK 验证冒充 GUI 自动运行验证。

证据位于 outputs/keil_without_ini_20261015/；本分支无统一 verify profile，
执行现有构建、路径检查和离线测试。回退仅恢复 tIfile 与备份脚本，保留其他修改。

结果：普通/HIL 重新构建、Loader 离线套件、路径与所选 Python Ruff 检查通过。
J-Link SDK 两次真实复位均到 main，未下载，完整 Flash 不变；未执行 Keil GUI 自动运行验证。
