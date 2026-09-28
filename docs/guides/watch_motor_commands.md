# Watch 电机请求入口

当前内部调试邮箱版本为 2、大小 64 B。使用当前 ELF 的 `FocBenchSpeed` 及字段定义，
不要复用 v1 字段偏移或历史 RAM 地址。它不是线上 CAN ABI，也不存入 Flash。
只有请求区可以写；模式、故障、测量与回复均为观测值，不能直接写电机对象或启相。

## 发布和读取

单一外部生产者先把 request_seq 写为奇数，再填写请求字段，最后提交下一个偶数。
首次用 1→2，下一次 3→4，持续递增、不回绕。普通请求等 ack_seq 匹配后再发下一条；
STOP 可以优先覆盖尚未受理的请求。读取回复前后检查 status_seq 相等且为偶数。

| command | 请求字段 | 结果语义 |
| --- | --- | --- |
| 1 START | duration_ms、iq_limit_a、phase_limit_a、speed_limit_rad_s；target_speed_rad_s=0 | 零速度准入；ack_result=1 只是受理，state=2 才是内部 RUNNING |
| 3 UPDATE | target_speed_rad_s，单位 rad/s | 已 RUNNING 的同模式速度目标；其余配置不更新，不续租 |
| 2 STOP | 无其他载荷要求 | 撤销目标并请求关相；state=4 为本会话停止确认 |

START 仍限制期限 1..5000 ms、电流参考不高于既有配置且低于本次相电流截止、截止最大
6 A、速度不高于既有上限/轴限制和 300 rpm（约31.416 rad/s）中更小者。
初试可设置期限 1000 ms、Iq 1 A、相电流截止 6 A、速度上限 5 rad/s；仍须满足设备
实际已有上限和台架条件。没有请求时上电不启动，不因更改观测字段自动启动。

零速度进入 RUNNING 后，可发布 UPDATE。例如 target_speed_rad_s=2，提交 seq=4。
ack_seq=4、ack_result=1 表示进入公共目标邮箱；applied_seq=4 表示所有者在监督拍
复核并写入了目标，不表示物理达到速度。CAN 116 与 Watch 共用这一邮箱，占用时返回
BUSY，不覆盖已受理目标。期限包含 STARTING 时间，目标更新不会延期。

ack_result：1 受理、2 参数错误/越限、3 准入拒绝、4 忙、5 被撤销。
state：0 空闲、1 启动、2 运行、3 停止中、4 停止确认。
STOP/FAULT 使旧序号失效；同序号不重试，清故障不恢复旧目标。新会话应用序号归零。
101 ENABLE 未开放，STOP 仍可经 110 或 101 DISABLE 提交。

相电流截止是 20 kHz 采样式保护，不是硬件绝对钳位；母线限流仍由台架电源保证。
未确认停止时必须物理干预，不能恢复试验。本文只定义接入方式，不作为新版本实机验收。
