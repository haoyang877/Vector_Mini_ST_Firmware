# 核心边界续收口 v1

日期：2026-10-16。基线：`codex/system-refactor@661243b9`。

## 目标与范围

只处理本轮实查发现的三个窄边界，不增加控制模式、页面或存储/请求格式：

1. 运行期 DEFAULTS 保留当前设备 node，避免活动端点与自动 SAVE 后地址不一致。
   启动空白/损坏记录仍按已有策略用默认node1；其他恢复默认语义保持。
2. 主状态机 SAVE 认领改用现有维护释放门面，删除对具体标定任务的直接依赖。
   工作区仍被占用时不借出SAVE；这是准入收紧，不改变正常任务流或保护阈值。
3. APP 公布已有核心 SPEED 实现能力，主机仅在真实新鲜能力声明后允许START/SET。
   GET/STOP/升级不受速度能力位限制；其他候选模式继续不公布。

## 不变边界与接受标准

schema12/4304B/CRC/16KiB、控制数学/阈值、START零目标、STOP关相和CLEAR不启动保持。
不在线改地址，不迁移Flash，不新增配置框架、命令编号或协议字段。
能力页使用已定义bit8/bit2，详见[核心SPEED能力v2](../../protocols/yg_protocol_core_speed_caps_v2.md)。

使用现有夹具保留DEFAULTS地址反例及工作区反例；修复后检查真实维护、装载/导出和Loader
节点一致性，主机检查能力缺失/清零/过期时禁启但可STOP。最终冻结输入只跑一次PR、
主APP及LoaderApp各一次构建；主机独立构建定向检查，不增加五工程/运动矩阵。
产物和日志归档于 `outputs/coordination/core_followup_20261016/`。
本批不连接J-Link/CAN，不下载、复位或运动，不继承旧实机PASS。

## 回退与现场保护

按本批独立提交回退；保留用户RAM计划、VS Code与D盘现场文件，不reset/clean或改其他引用。
固件查询版本仍按当前开发线，未形成独立正式发布；用源SHA和产物哈希区分候选。


## 已完成的维护修复

提交`7b4d0d04`。原版两个反例均编译成功后运行失败：node7执行DEFAULTS变成1，
以及任务已释放但encoder工作区仍借用时SAVE被错误认领。修复后完整生产所有者、维护
与真实工作区夹具通过，实际参数导出/装载/CRC和Loader节点解析夹具也通过。
默认叶在所有者测试中为明确桩；序列化夹具使用真实Param实现，二者证据没有混称为Flash实测。
纯状态规则262144输入摘要仍为`9dd9307dcc7bb53f`。
独立只读复核核对四份输入与四份日志哈希全部匹配，未重复执行测试。
日志与边界记录在`outputs/coordination/core_followup_20261016/parameters/summary.json`。


## SPEED实现收口

APP仅增加features bit8与mode2声明，使用已有117/116/110，不新增路由或模式功能。
主机v1.4.3以现有DeviceReadbackView判断能力，按钮、请求排队和发送前共用一个数值谓词。
STOP明确豁免，GET与配套升级不依赖SPEED位。旧模式0的APP仍可读取、STOP及合法升级。
现有Qt定向检查已通过，读取v2黄金JSON并经假端口验证三种缺失声明时禁启但STOP可完成。
过期值拒绝由现有时效视图和谓词检查；“排队恰逢5s过期”的完整调度竞态未做端到端注入，
发送前守卫与升级豁免经独立源码复核；以上不冒充真实升级或实机运动验证。

上位机安装目录为`outputs/host_app/YG_MC_HOST-core-v1.4.3-20261016/`。
最终PR、APP构建、匹配产物及未测项统一见
`outputs/coordination/core_followup_20261016/summary.json`，不用历史outputs作为本批通过证明。
