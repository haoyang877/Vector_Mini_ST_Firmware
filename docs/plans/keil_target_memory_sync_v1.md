# Keil Target 内存显示同步 v1

日期：2026-10-15。基准分支：`codex/ovp-36v`。

## 目标与不变项

普通 APP、HIL 工程的 Target 页 IROM1/IRAM1 与现有独立
`Vector_Mini_ST.sct` 一致。仅修改 `.uvprojx` 中 `OCR_RVCT4`、
`OCR_RVCT9` 的起点和大小，不改变链接脚本、控制逻辑、参数 ABI、
故障行为、时序或下载擦除策略。

## 基线与验收

- 修改前重新构建普通 APP、HIL 和 Loader，导出 BIN 作为基线。
- IROM1：起点 `0x08004000`，大小 `0x18000`。
- IRAM1：起点 `0x20000020`，大小 `0x7FE0`。
- 保留 `umfTarg=0` 及独立 scatter 文件选择。
- 修改后重新构建，检查 map 地址、XML 一致性及前后 BIN 字节一致。
- 检查项目目录；不下载、不接触电机。当前分支不具备新版 verify profile，
  使用本分支现有构建与目录检查工具。

## 回退与证据

回退仅恢复本次两个工程的四项数值，不覆盖其他工作。
工程备份、构建日志、BIN、输入与产物哈希、`summary.json` 放在
`outputs/keil_target_memory_sync_20261015/`。
Keil 已打开时需重新打开工程以读取磁盘配置，避免旧窗口覆盖新配置。

## 完成结果

普通 APP、HIL、Loader 修改前后重建均为 0 错误、0 警告，三份 BIN
分别逐字节一致。XML、map 与 scatter 地址核对、项目目录检查均通过；
只有两个预期工程文件的构建输入哈希改变。未烧录或执行硬件测试。
详见上述输出目录的 `summary.json`，本分支无统一 format/lint/verify 入口。
