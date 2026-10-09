# 角度传感器标定与轮电机速度验证（2026-09-30）

## 结论

更新（北京时间 14:25）：用户要求再次驱动观察，使用同一 V4 镜像/保存参数、临时 3 A、
正向 60 rpm 运行 8 s，均值 59.062 rpm、标准差 4.867 rpm，超过 3 rpm 门槛，复测失败。
下述六档通过仅为此前单轮记录，不能据此继续宣称 3 A 配置已具备可重复的稳定性。
没有自动重试。STOP 完成、NONE 零目标、关相/资源释放、限流恢复 6 A 已确认。
末帧速度 -489 mrad/s，不由关相推断机械静止。没有保存参数或修改固件。
本次证据：`outputs/coordination/firmware_test/speed-trial-20260930T062503224240Z/report.json`。

LUT 和电角度零位均已完成、保存并通过固件复位重载核验。V4 候选在 **3 A 临时相电流
限值**下，正反 30/60/120 rpm 六档全部满足预定稳定性门槛；**6 A 配置尚未通过**。
不能用降额结果替代 6 A 验收。已确认 STOP 完成、NONE 零目标、三相关闭、资源归还，
J-Link/USB-CAN 已释放；活动及保存限流均为 6 A，3 A 没有写入 Flash。
最后状态帧仍有 -4292 mrad/s 滑行速度；未另行核实转子机械静止，不把关相等同于静止。

## 基线和环境

- 分支 `codex/system-refactor`，HEAD `a246a962c5b0686da46eec595a2eb33a1729f4ba` 加实际
  dirty 源码。758 个源文件快照及补丁见 `outputs/speed-acceptance-20260930-v4/summary.json`，
  不能只用 HEAD 识别台架镜像。
- 架空、无负载轮电机；33 V 母线、电源上限 3 A、相电流允许上限 6 A、6 mΩ 采样电阻、
  21 极对、最高允许 300 rpm。应用速度上限 20 rad/s；主机停止线 240 rpm、母线 2.8 A、
  相电流 5.7 A、母线窗口 25.6～33.8 V。出力读取预算 20 ms、采样间隔/新鲜度 30 ms。
- V4 HEX SHA256：`426a8f7faa1d883eafaee3380bd8e8741a2ad2c666668f2d4d928935b6308b32`。
- V4 AXF SHA256：`12d37c7c5e139d283ab625f42e8f19772b07a258b5ba895b91817de82cacb589`。
- 完整参数区 SHA256：`9c5a6f02da39618b8f1b30d0e6cce0dbc8dcec99047222cdf0db1ec675885ef0`。
  编码器有效标志 3，电零位 12489 Q15，记录 CRC32 为 2596425200，LUT 与活动 RAM 一致。
  下载备份、逐字回读及复位重载证据：
  `outputs/coordination/firmware_test/speed-download-20260930T045616160797Z/report.json`。
  六轮试验工具 SHA256 均为 `301bce56bbd4a7a10042b5eb4fa3ef1bea47d86d886e1dc17e427f31d14ec6fb`；
  对应 `outputs/coordination/firmware_test/speed_acceptance_20260930.py` 收尾读取完整参数区并
  比较上述 SHA，异常进入 errors 并使 success=false。六轮检查均通过，但没有独立保存末次
  参数字节或单列 hash 字段；不能将它描述为存在一份末次逐字备份证据。

## 修复与未通过项目

1. 保留之前的上电三相零偏、PI 初始化和空闲电压限幅更新修复，不覆盖其他在途修改。
2. 已修复普通启动竞态：117 已受理但 `enable_pending` 未消费时，后台补偿仍借用配置；
   监督中断据此拒绝 ENABLE。仅增加待启动互斥条件，不增加状态机或放宽保护。
   原所有者夹具在补偿函数内注入 Tick，改前失败、改后通过，并覆盖 RUNNING 补偿/STOP。
3. V3（300 Hz、6 A）30/120 rpm 通过；60 rpm 均值 60.072 rpm、标准差 3.076 rpm，
   超过 3 rpm 门槛。早期主机读取超时的不足时窗试验不计通过。
4. V4 仅将既有 PLL 自然频率调到 90 Hz，不叠加滤波器，PI 仍为 Kp=0.05、Ki=0.5。
   这是控制整定候选而非标定算法修复。离线估计延迟从约 0.75 ms 增至 2.43 ms；
   6 A 下 30 rpm 标准差 3.486 rpm，仍失败。
5. 速度 PI 限幅 ±1 后乘 `application.current_limit_a`，因此通过 CAN 121 临时把 6 A
   降为 3 A，也使未饱和区等效 Kp/Ki 减半。以下通过结果属于这套组合，不证明只降低
   PLL 带宽就能消除真实机械纹波，也不修改“6 A 未通过”的结论。

## V4/3 A 分档结果

每档至少 8 s，舍弃前 2 s。均值误差及标准差均要求不超过 `max(3 rpm, |目标|×5%)`。
每次都验证 117 原身份 STARTED、108 RUNNING/速度模式、116 目标及 124 APPLIED，
并确认编码器反馈。末尾独立验证 STOP/关相/资源释放，再经 121 回读确认恢复 6 A。

| 目标 rpm | 均值 rpm | 标准差 rpm | 稳态样本数 | 结果 |
|---:|---:|---:|---:|---|
| 30 | 29.947 | 2.036 | 287 | 通过 |
| 60 | 60.028 | 1.651 | 287 | 通过 |
| 120 | 119.887 | 1.314 | 287 | 通过 |
| -30 | -30.000 | 2.056 | 287 | 通过 |
| -60 | -60.027 | 1.663 | 287 | 通过 |
| -120 | -120.084 | 1.603 | 286 | 通过 |

六档所采相电流绝对峰值不超过 0.336 A，母线电流不超过 0.063 A；这些是离散采样值，
不代表未采到的瞬时峰值。最大读取耗时 9.311 ms、最大采样间隔 23.472 ms，无固件锁存故障。
没有降低监测预算或事后改变稳定性门槛。

机器可读汇总及每档原始报告 SHA256：`outputs/speed-acceptance-20260930-v4/bench-summary.json`。
六档报告位于 `outputs/coordination/firmware_test/`，依次为：

- `speed-trial-20260930T045834369142Z/report.json`
- `speed-trial-20260930T045906580345Z/report.json`
- `speed-trial-20260930T045933028225Z/report.json`
- `speed-trial-20260930T050000425267Z/report.json`
- `speed-trial-20260930T050026159743Z/report.json`
- `speed-trial-20260930T050044736712Z/report.json`

## 工程验证与剩余边界

- V4 PR：`outputs/runs/20260930T045439784649Z-a246a962/summary.json`，全部通过。
- Keil：0 错误、0 警告，Code 90156、RO 3120、RW 304、ZI 31272 字节；实际 APP 字节 93288。
- Release 尝试：`outputs/runs/20260930T045553777697Z-a246a962/summary.json`。普通检查通过，
  clean-release-source/release-manifest 失败：dirty 调试候选不是干净发布，不宣称 release PASS。
- 90 Hz 对所有使用此测速的模式有影响；仅本轮空载轮电机/3 A 分档验证通过，其他轴、
  带载、持续运行、频响及产品默认均未验收。没有启动 Loader 测试。
- 3 A 是否作为接受的运行配置由用户决定；未擅自保存。若要求保持 6 A，继续做受控的
  PI 整定或明确安培单位/限幅语义迁移；不可用私有 SWD 写或静默改存储参数绕过所有者。
