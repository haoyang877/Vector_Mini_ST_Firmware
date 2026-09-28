# 公司统一帧升级载荷 v2：主机 nonce 与 CRC32 镜像

日期：2026-09-28。适用：本轮简化 Loader 与配套偏移 APP。外层公司帧仍为 v1，
仲裁 1 Mbit/s、数据 5 Mbit/s、扩展 CAN FD、PF=0xEF，来源主站 2。
本页替代[升级载荷 v1](yg_protocol_upgrade_wire_v1.md)中的启动身份、镜像和角色定义；
v1 的请求长度、数值小端、分片、40B 回复偏移、结果码及精确发送完成契约继续适用。
不增加 UART/USB 能力。当前没有可发布的主机升级产品，主机实现必须按本页适配。

## 发现与身份

40B 回复 flags 偏移 17：bit0=待激活、bit1=INTEGRITY_ONLY、
**bit2=HOST_NONCE（0x04）**，其余必须为零。无活动会话的发现 STATUS 可返回 boot=0、
session=0、phase=IDLE、flags=0x06、max_block_bytes=128；boot=0 不表示设备不可升级。
只有成功受理 BEGIN 后，回复 boot 才回显被绑定的主机 nonce。
已有会话结束后仍可报告最后一次绑定值；它不能用作持久设备启动身份。

线上的字段名 boot_epoch/expected_boot 保留，但 v2 含义为**主机生成的非零随机 u64 事务 nonce**。
每次新升级、设备复位或旧会话过期后，主机产生新值；同一事务重试保持原值。
可用标准库 `secrets.randbits(64)` 并重试零值，禁止使用固定值、节点号或复位后总为 1 的计数。
nonce 防止主机误关联旧回复，不提供发布者认证或恶意重放防护；设备没有跨复位持久计数。

| 请求 | v2 身份与幂等规则 |
| --- | --- |
| 50 BEGIN | nonce:u64 + manifest[96]；同活动 source、nonce、完整清单才返回原会话，其他身份返回忙 |
| 51 WRITE | nonce:u64 + session:u32 + offset:u32 + length:u16 + data；原身份/偏移/内容重试 |
| 52 FINISH、54 ACTIVATE、55 ABORT | nonce:u64 + session:u32；不能省略任一字段 |
| 53 STATUS | 空请求只发现；12B 会话请求带 nonce 与 session，可续租 |
| 56 APP_ENTER_LOADER | nonce:u64；配套偏移 APP 以它绑定停机与精确 READY 回复，standalone APP 不提供此能力 |

主机先发现 Loader 的 HOST_NONCE 能力，再产生 nonce 并 BEGIN；不把发现回复的 0 用于 BEGIN。
如果设备仍在 APP，只有已明确安装配套 v2 偏移 APP 时，才能按 v2 发 Type 56；
不能用 APP 的旧启动身份状态推断支持 v2。进入 Loader 后重新发现并确认协议/profile 配对。
保持每次一个逻辑请求在途，不能只按公司分片 seq 判断升级块完成。

## 镜像 v2 与旧包拒绝

| 标识 | 当前值 |
| --- | --- |
| YGI_PROFILE | 0x53593449（旧 SHA profile 0x53593349 被拒绝） |
| 描述符版本 | 2（旧版本 1 被拒绝） |
| G431 TARGET_LAYOUT | 4，APP base=0x08005000（旧 layout 3 被拒绝） |
| 打包格式 | loader-image-v2-crc32 |
| image_role | 仅 1 普通 APP；2 维护及其他值在打包、清单、描述符三处拒绝 |

清单仍为 96B 大端容器：48..51=镜像 CRC32、52..95=零。
描述符仍为 64B 大端容器：44..47=补齐后镜像长度、48..51=CRC32、52..63=零。
CRC32 使用反射多项式 0xEDB88320、初值/末异或 0xFFFFFFFF，覆盖按 8B 补齐的完整镜像；
描述符 CRC 的 4B 按零计算，长度字段参与计算。清单与内嵌描述符身份/CRC/长度必须相同。
CRC 只检测偶发损坏，无签名、来源认证或防降级策略。

原始链接 BIN 的长度/CRC 占位为零，必须由 `loader/tools/package_image.py` 生成
`image.bin` 和 `manifest.bin`。只部署打包后的 image.bin；不能截断超限包或现场修改身份字段。
旧基址、旧 SHA、旧维护角色包不能线上直接迁移。更换布局必须离线配套安装新 Loader 与 v2 APP，
保留原 16KiB 参数，核对备份与恢复；旧 v1 主机不得静默回退或混发两种身份语义。

## 启动、断电与故障边界

BEGIN 后整槽擦除，WRITE 连续写入并读回；FINISH 分步校验，VERIFIED 后才受理 ACTIVATE。
租约 5s、ACTIVATE 精确发送完成期限 500ms；对应回复物理发送成功后再次核对整包及关相/链路，
通过才复位。激活后复核失败返回 ERROR/DIGEST 状态并留 Loader。

无持久 ACTIVATE 标志：完整 WRITE 后掉电，即使没有 FINISH/ACTIVATE，冷启动也可通过完整校验而运行。
部分镜像或 CRC/向量/描述符非法则驻留重刷，无跨复位续传、无旧镜像自动回滚。
IWDG/WWDG 复位来源触发驻留；APP 每 100ms 仅在监督计数有新进展、采样新鲜、未 FATAL、
通信初始化和本地 CAN 健康时喂狗，不要求主机在线。
应用业务异常但仍满足全部健康条件时，需复位后利用约 50ms BEGIN 窗口救援。
Type 56 的 READY 不证明 APP 已复位；发送或停机复核失败保持停机且不自动恢复运动。

## 黄金向量与测试

- [升级载荷 v2 向量](yg_protocol_upgrade_wire_v2_vectors.json)：空 STATUS、40B IDLE、BEGIN、Type 56 nonce 小端。
- [镜像 v2 向量](../../tests/fixtures/loader_image_v2.json)：合成源码 BIN、打包镜像与 96B 清单，不可烧录。
- `tests/unit/loader_upgrade_test.c` 使用真实 C 编解码与状态机检查 flags、旧包拒绝及 nonce 幂等。
- `tests/unit/test_loader_package.py` 使用实际打包 CLI 比较黄金字节，并交叉核对载荷清单与端序。
- `tests/unit/native/test_loader_runtime.py` 检查真实启动组合层、APP 喂狗调度和 Flash 等待失败路径；
  硬件均替换为桩，不证明实机 ECC、Flash 时序或 CAN 总线可靠性。
  可通过 `--image-bin outputs/.../image.bin --out outputs/.../runtime-real-image` 对实际打包产物执行同一检查。
