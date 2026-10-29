# 集中板卡与电机选择 v1

目标：新增 firmware/config/config.h，集中维护两个正式工程各自的板卡与电机选择；板卡、电机和应用实际参数及派生公式保持原文件。

约束：STM32 APP 仍使用 STM32G4/HT8115-4；AT32 OFF 仍使用 AT32F456/YQ-5708 并永久关相；不改变现有 Flash 参数优先级、采样尺度、保护参数或设备 ABI。拒绝 AT32 MCU 使用不受支持的目标。

基线：修改前配置备份于 outputs/config-selection-20261030/；同源正式工程 HEX 基线由上一轮构建记录。验收包括两个 Keil 工程、二进制比较、离线目标选择及关相/派生检查和仓库 PR 验证。不访问硬件。

回退：只恢复本次备份的配置头和说明，并删除新建的 config.h；保留其他用户修改。

验收结果：两个正式 Keil 工程均为 0 错误、0 警告；STM32 与 AT32 的 HEX 哈希分别为 DDE6256A1E48D0FE89695853282C4E9DC251FA324A5383F1A6FFA0068F7E3759 和 09105DC92DA82C173816AFC0C98C7839059CD369CC982512C2FF5E3BA7C222FE，与重构前相同。离线检查覆盖两工程选择、电机参数派生、有效记录读取和 AT32 启相拒绝。

PR 验证的 format、lint、project-layout、architecture、interfaces、docs、hygiene 通过；测试阶段失败，仍有测试依赖已删减的旧台架、可覆写配置和旧文件结构。证据见 outputs/runs/config-selection-20261030/。本次未连接硬件。
