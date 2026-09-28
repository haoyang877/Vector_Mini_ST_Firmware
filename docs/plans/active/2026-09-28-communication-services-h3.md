# 通信业务接入 H3 v1

日期：2026-09-28。基线：`c40431bb59f01ccd98b5f273aa30b04b47a7a06c`。
开发分支：`codex/protocol-services-h3`。本轮用户要求完成整体重构，测试留到最终统一进行。

## 目标与职责

保持公司16B头、CRC、小端、CAN FD扩展ID和1M/5M时序；保留101/108/110/116/124行为。
完成通信层既定参数目录120/121/122的字段解码、结果编码及显式服务绑定；107按本轮
ROOT裁决冻结空请求、4B result/detail回复，不引入掩码或新增控制模式。

协议仅验证线路字段与主站地址。E拥有命令应用、参数范围/版本、持久化作业、维护取消
和真实静默确认；D拥有Loader镜像事务和shared契约。通信不得直接写电机状态/Flash，
不得调用HAL或复位。Loader公司帧50～56的适配由H唯一实现，不并存旧YI线协议。

## 分批顺序

1. 107及120/121/122编解码、路由与绑定；元数据随值结果返回，不复制整套参数到link。
2. 与E对接101/CLEAR/参数所有者接口；同序号写/保存必须由所有者幂等，保存受理不是完成。
3. 依据D提供的固定镜像契约接升级公司帧，并把激活绑定到具体回复的TX完成证据。
4. 收口真实APP接线需求及剩余未实现目录；删除无调用旧路径，不保留双协议兼容。

## 验收、回退与证据

本轮不运行测试、回归、构建或硬件操作，不写PASS。静态阅读全部影响调用者并手动核对
字段、错误和资源所有权；版本化文档、黄金payload及后续统一测试清单随提交交付。
线上新增业务未绑定时明确UNSUPPORTED；不得宣称已完成APP生产接线。

已有参数schema11、控制数学/保护限值、状态反馈布局、快慢环调度和Keil用户配置不变。
不新增大参数副本或无界队列；资源实际布局与时序留最终构建及统一验收。
各提交可独立撤回到基线；不改现场板卡或还原其标定数据。

## 已完成的代码批次

- `63007db8`：107 与参数字段、错误、修订号和所有者接口契约。
- `4cd70e6c`：107/120/121/122 实际通信路由及回复编解码；APP 所有者绑定由 E 接入。
- 公共帧核心：九个文件内容不变，移到 `shared/protocol/`；APP/Loader 工程输入分别由 ROOT/D 接入。
  [迁移清单 v1](../../protocols/yg_protocol_shared_codec_migration_v1.md) 列出全部路径变化和边界。

以上均为代码进度，不代表生产接线、构建、测试或硬件验证通过。

## 真实设备描述接线

通信不再在初始化时填充占位产品号、硬件/软件版本和能力位。组合层通过
`YgProtocolLink_SetDeviceDescription(info, capabilities)` 复制真实身份与已实现能力；
两个参数同时为NULL撤销绑定，一空一非空拒绝且保持原值。初始化后与撤销后
1/2查询明确数据不可用，使用公共4B result6/detail0，不能只返回内部错误导致无业务回复。
成功时1/2既有请求和回复字节布局不变。
信息由平台UID/产品版本与业务所有者提供，通信不新增硬件访问或参数副本。
需要ROOT/E在产品初始化并完成真实服务绑定后接入；旧测试占位期望留统一测试阶段更新。

## 升级v1接线

ROOT批准50～55的最小字段及分片序号澄清；实现shared纯请求解码、40B回复编码和版本化黄金载荷。
升级seq仅关联本次回复；业务幂等使用真实source/boot/session/offset/operation及内容，
不能对不同块的相同分片尾seq全局拒绝。详情见[升级v1](../../protocols/yg_protocol_upgrade_wire_v1.md)。
D接入实际Loader命令分支、存储及精确TX票据；56等待真实APP静默与handoff后再注册。

## 标定任务v1

ROOT批准160～162单帧kind/ticket最小布局；H实现专用任务reply、4B请求codec、12B结果与link绑定/路由。
取消身份、START重放及结果均由E维护所有者决定，163与未定义任务不注册。
[任务v1](../../protocols/yg_protocol_calibration_wire_v1.md)列出内部枚举映射及统一测试前的旧contract调用适配。

## APP进入Loader

选用D固定`1109361d`的共享BootEntryService，新增56窄载荷编解码。
复用ROOT固定`6592f64f`的三个platform文件，完成56的link路由及精确TX旁记录关联，
Service先消费真实完成、匹配完整u64票据，再交E所有者；不扩普通帧队列。
E实际APP静默/启动身份/ENTER复位仍由E交付，不能把codec或回调存在当成已安装Loader。
字段和依赖见[进入Loader v1](../../protocols/yg_protocol_boot_entry_wire_v1.md)。

## 移除旧升级空接口

旧`yg_protocol_update.c/.h`只有legacy contract测试调用，没有APP生产调用或Keil源码项。
它的OPEN/MANIFEST/QUERY等泛型接口被本轮50～55共享codec与D独立Loader事务取代，删除且不保留包装。
ROOT需在统一测试前移除native core旧源码项，并将contract中的旧泛型update边界检查迁至真实共享codec；
这批没有运行旧测试，也不能把旧stub测试作为新升级验收。

## 真实启动身份发现

ROOT冻结Type3空请求/16B角色、阶段、有效位及boot64回复；H在既有readonly_payload模块实现，
link借用稳定APP快照，无新服务、无硬件访问。未绑定仍16B返回当前数据不可用，不猜补高位。
[发现v1](../../protocols/yg_protocol_device_status_wire_v1.md)连接主机Type3→56的实际流程；
E须用真实BootHwInfo来源接线，Loader可继续用53，不要求重复实现。
