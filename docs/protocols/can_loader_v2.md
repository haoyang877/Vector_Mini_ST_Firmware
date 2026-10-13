# CAN Loader v2 上位机接入

日期：2026-10-13。状态：当前开发候选，独立构建与单次在线升级已通过，连续1000次验收尚未通过，未发布。

完整设计见[设计方案](../plans/2026-10-13-can-loader-minimal-design-v1.md)，逐字节实例见[黄金向量](../plans/2026-10-13-can-loader-minimal-v1-vectors.json)。协议不依赖Python、USB-CAN厂商或J-Link，其他主控只需能发送标准CAN-FD数据帧。

## 固定接口

仲裁和数据段均1 Mbit/s，BRS开启，标准11位ID。当前target支持APP节点0～7；部署五电机通常为1～5，node0仅作单板恢复/调试，节点必须唯一。

| 阶段 | 请求ID | 回复ID |
| --- | --- | --- |
| APP | `(node << 8) | command` | 同请求ID |
| Loader | `0x7D0 + node` | `0x7E0 + node` |

APP接口均不会主动启动电机：

| command | 请求 | 回复 |
| --- | --- | --- |
| `0x01` 模式 | 4 B零 | 4 B大端float32模式值，STOP为全0 |
| `0x63` UID | 4 B本次nonce | 16 B：nonce4＋芯片UID原始字节12 |
| `0x66` 进入Loader | ASCII `BOOT`，`42 4F 4F 54` | 4 B大端结果，0成功；4关相失败 |
| `0x6F` APP版本 | 4 B零 | 4 B `major/minor/patch/build` |
| `0x7C` 参数CRC | 4 B本次nonce | 8 B：nonce4＋大端CRC32 |

进入Loader的回复0只在APP确认STOP和六路关断后发送；随后软件复位。上位机应等待Loader INFO/UID，不能把ENTER命令提交给驱动的成功返回当作设备已进入。

## Loader帧

```text
B1 02 OP 00 SEQ_BE16 PLEN_BE16 SESSION_BE32 PAYLOAD CRC32_BE [FF padding]
```

头12 B，CRC4 B，payload最多48 B。CRC覆盖头和payload，不覆盖padding。CRC-32/ISO-HDLC与zlib.crc32一致；`123456789`→`CBF43926`。使用16/20/24/32/48/64 B中能容纳整帧的最小DLC；接收校验所有尾部为FF。

回复payload开头2 B为大端result；非0回复只有result2 B。回复必须匹配目标ID、op、seq和session；只读查询还必须匹配nonce。重试保持整帧相同，不生成新序号或新session。

| OP | 请求payload | 成功回复payload（含result） |
| --- | --- | --- |
| 01 INFO | nonce4 | 38 B，见下一表 |
| 02 STATUS | nonce4 | result2＋nonce4＋state1＋error1＋reserved2＋offset4＋size4＋image_crc4＋session4，共26 B |
| 03 UID | nonce4 | result2＋nonce4＋UID12，共18 B |
| 20 BEGIN | 清单40 B | result2＋APP基址4 |
| 21 ERASE | 空 | result2 |
| 22 PROGRAM | offset4＋data8～40 B | result2＋next_offset4 |
| 23 VERIFY | 空 | result2＋计算得到的image_crc4 |
| 24 ACTIVATE | 空 | result2 |
| 25 ABORT | 空 | result2 |

查询session为0。BEGIN指定本次新的非0session32，由设备接受并回显；后续变更请求保持相同session。seq递增、不回绕；当前最大96 KiB镜像不会耗尽16位序号。字段全大端，UID除外（原始12字节）。

INFO回复的payload偏移：

| 偏移 | 字段 |
| ---: | --- |
| 0 | result:u16 |
| 2 | nonce:u32 |
| 6 | loader_version:u32 |
| 10 | app_base:u32=`08004000` |
| 14 | capacity:u32=`00018000` |
| 18 | layout_id:u32=`4F564C31` |
| 22 | hardware_id:u32=`00000001` |
| 26 | param_abi:u32=`4F563130` |
| 30 | node:u8＋reserved:3 |
| 34 | parameter_crc32:u32 |

参数CRC覆盖 `0801C000～0801FFFF` 全16 KiB。记录开始值，升级后用APP的7C重新核对，必须一致。设备Flash写入端口独立限制APP和记录页；CRC比较不是写范围保护的替代。

BEGIN载荷偏移：size0、image_crc4、app_version8、image_type:u16在12（必须1）、reserved:u16在14（0）、layout_id16、hardware_id20、param_abi24、UID28，共40 B。

## 最小流程

1. 本机先预检发布包：MCU/target/layout/版本、重定位基址、SHA256、8 B补齐、96 KiB容量和Cortex-M向量。
2. 读取APP版本、UID、STOP和参数CRC；唯一绑定目标。
3. ENTER_BOOT，等待成功及Loader启动，再读INFO/UID。身份和参数CRC必须与预检一致。
4. BEGIN，不擦Flash；ERASE成功后开始PROGRAM。
5. 每块 `n=min(40, remaining, 2048-offset%2048)`，n为8 B倍数。等ACK后推进，ACK的next_offset必须等于本块末尾。
6. VERIFY的CRC必须等于包CRC；ACTIVATE后等待APP。
7. 读取相同UID的新APP版本、STOP和全参数CRC。全部符合后才报告升级成功；不能只看ACTIVATE ACK。

普通请求每次超时1 s，包含首发最多尝试10次，始终保持相同字节；ERASE每次初值20 s，VERIFY10 s，APP恢复5 s。Loader在新请求到来时取消已挂起超过100 ms的旧回复，该请求的回复可能尚未入队，主机应继续精确重试；取消绝不算ACTIVATE已发送。上位机不得在ERASE时持续发送数据。尝试耗尽或session不确定时停止本轮，重新读取身份/状态，不能盲目重新擦写。扩展帧级重试不能重置整圈失败记录。

升级中断后记录无效，设备驻留Loader，完整重新下载。首版没有断点续传、A/B镜像、自动回滚或认证。版本字节为2，旧v1消息明确拒绝，禁止自动降级写入。

## 其他上位机的实现检查

当前USBCANFD_200U的ControlCANFD SDK会把错误被动状态的有效FD帧标记为`can_id=0x20000000 | 实际ID`，不能直接用`can_id <= 0x7FF`丢弃。该适配器接收边界取低29位作为ID，拒绝扩展/远程位，Classic错误帧仍拒绝；Loader帧还必须通过完整CRC及ID/op/seq/session/nonce核对。此行为属于该SDK的接收元数据，不改变线上v2帧，不应直接套用到其他SDK的错误帧定义。

将本仓库黄金向量作为独立测试输入，验证BE整数、UID顺序、padding、帧CRC和镜像CRC；再验证错误result解析、旧nonce/seq丢弃、重复PROGRAM和错UID/layout无副作用。

参数区不属于升级包。升级构建启动仅在RAM解释旧参数，禁止自动迁移写入，并强制STOP；显式mode9保存属于另外的配置操作。APP二进制不得从0x08000000独立镜像简单平移；必须重新链接到0x08004000。该版本号和layout_id描述不同契约，不能仅按版本大于当前值放行写入。
