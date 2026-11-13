# CAN-Link-Test v1.11.0

J-Link增加APP区和整片Flash两个擦除入口，确认UID/范围后备份并执行，双读校验，
保持停机。APP擦除保留Loader/参数/身份并支持重烧；整片清空256 KiB后按空板初装。
实现提交4fb328eb，干净Release通过，已打包并启动；未执行实机擦除。
APP1.8.4/Loader2.3.4未改。交付、哈希、测试范围和限制见
[报告](../reports/2026-11/jlink_flash_erase_20261113.md)。
