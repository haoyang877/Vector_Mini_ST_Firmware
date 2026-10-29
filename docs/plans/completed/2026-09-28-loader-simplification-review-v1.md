> 历史阶段记录，2026-09-29 归档。下文是当时的设计、状态和证据，不代表当前待办或当前版本通过。现状见[框架收尾](2026-09-29-framework-closure-v1.md)与[历史清理](../../archive/plans/2026-09-29-legacy-cleanup-v1.md)。

# Loader 简化审查与修复计划 v1

日期：2026-09-28。对象：codex/capacity-fit 的简化提交 **95f1aa0a**，比较基线 abc3b45a；
审查开始时改动尚未提交，期间已提交，关键发现按提交后的源码再次核对。
状态：初审未通过；用户要求“检查并修改”后已实施源码修复，待发布与台架验收。
下文 R1～R7 保留初审发现；当前修复状态与证据见末尾执行记录。本轮不操作硬件。
关联：[候选设计](../../archive/architecture/loader/loader_product_v1.md)、
[简化实施记录](2026-09-28-simplified-boot-handoff.md)。

## 结论与保留能力

三段布局与删除持久启动日志能降低复杂度、释放6KiB APP分区，但属于能力取舍。
固定APP写域、参数保护、顺序写入读回、重复内容比较、5s租约、500ms激活期限、
精确TX回调与APP停机资源复核仍存在，适合保留。
当前正常APP周期复位、维护角色保护失效和协议不兼容必须先修复；编译通过不能替代这些行为检查。

## 发现与修复要求

### R1 / P1：正常 APP 无人喂继承看门狗

位置：loader/ports/stm32g4/platform.c 的 LoaderPlatform_Init；
firmware/app/board_config.c 的 Board_ServiceBoot；
firmware/platform/bsp/mcu/stm32g4/boot_hw_stm32g4.c 的 BootHw_Watchdog。

Loader启动IWDG后跳APP；旧BootHandoff_Service的健康喂狗调用随服务删除。
全仓APP/ports/CubeMX Core只剩BootHw_Watchdog定义，没有调用者，正常配套APP也会超时复位。
约32s是旧配置标称值，实际时间未测。修复：由唯一运行监督所有者给出新进展及健康证据，
前台有界喂狗；不得在任意循环无条件喂狗。验收正常运行、监督卡死、采样失效和通信故障。

### R2 / P1：故障 APP 复位后继续被启动

位置：loader/runtime/main.c 的 main/try_boot；
loader/ports/stm32g4/platform.c 的 LoaderPlatform_Init。

IWDG原因被排除于software_reset，随后清除；resident没有看门狗/异常复位策略。
CRC正确但运行崩溃的APP会重新通过CheckImage被启动，不能承诺自动驻留救援，并放大R1。
建议端口返回明确复位原因，启动所有者将看门狗/异常复位映射为驻留，无需恢复boot/record。
通信业务坏但仍持续喂狗的APP仍需人工复位后利用启动窗口救援。

### R3 / P1：维护镜像仍接受，但禁止运动标志被清空

位置：shared/image/image_transfer.c 的 descriptor_decode/DecodeManifest；
firmware/app/foc_run_state.c 的 FocRunState_Init、validate_control和start_permitted。

role=2仍合法，旧初始化读取BootHwInfo.role设置限制，新初始化固定boot_maintenance_only=false，
无其他赋值来源，原维护镜像禁止普通运动的保护失效。
修复：以明确版本启动身份或APP描述符恢复角色限制；若只实现普通APP，
打包和设备两端显式拒绝role=2。验收ENABLE、CONTROL、维护操作和保存权限。

### R4 / P1：启动身份语义改变，旧 v1 主机流程无法继续

位置：loader/runtime/main.c 的 status_reply/dispatch；
loader/core/upgrade.c 的 Init/Begin；
firmware/app/yg_protocol_status_adapter.c 的 refresh_boot_source。

初始化boot=0，发现STATUS回0；BEGIN和56仍要求非零boot，但现在由主机给值，
APP设备状态不再发布有效boot。现行公司升级v1要求先发现设备分配的非零启动号，
并禁止回显未校验请求值；遵守现行流程的主机不能继续升级。
修复：版本化发现、BEGIN/56、身份隔离和错误回复，同时同步APP、Loader、主机与黄金向量。

### R5 / P1：新 CRC32 镜像和基址复用旧 profile/版本

位置：shared/image/image_format.h、shared/targets/vector_mini_st_g431.json、
描述符生成源码与loader/tools/package_image.py。

清单摘要从SHA256改为CRC32+零，描述符保留区改为长度/CRC，
APP从0x08006800迁至0x08005000；profile、描述符版本1、layout=3和loader-image-v1包名未改变。
旧tests/fixtures/loader_image_v1.json仍是SHA向量。新解码会拒绝旧镜像，
但版本标识不能表达配对与迁移原因。须定义新profile/布局、兼容拒绝策略和三端向量；
增加打包正负例、单字节损坏、旧版本拒绝和端序一致性检查。

### R6 / P2：活动 BEGIN 忽略请求 boot 身份

位置：loader/core/upgrade.c 的 LoaderUpgrade_Begin活动分支。
该分支只比较source和manifest，boot从9改10仍返回YI_OK并回旧boot=9；纯内存已复现。
即使使用主机不透明事务标识，幂等匹配也应比较它。验收同身份重试及不同boot/source/清单。

### R7 / P2：寄存器 Flash 错误和等待契约未闭合

位置：loader/ports/stm32g4/platform.c 的 flash_ready/flash_program_doubleword。
这是前序容量提交e26219af引入、简化版本仍保留的问题。
BSY等待无软件上限，写前忽略flash_ready失败；错误掩码漏SIZERR/MISERR/FASTERR/RDERR，
两次32位写之间未保留本仓ST HAL的__ISB屏障。读回/ECC守卫不能替代完整错误和超时语义。
参照本仓vendor的FLASH_WaitForLastOperation、FLASH_Program_DoubleWord、
FLASH_FLAG_SR_ERRORS补全失败退出、上锁、编程序列及时限，验证缓存和中断约束。

## 须明确接受的取舍

- 无持久ACTIVATE门槛：完整WRITE后仍处RECEIVING，CheckImage已经为true；
  冷启动不要求FINISH/ACTIVATE。纯内存已复现；若要求激活后才启动，
  需设计最后写入8B有效单元并同步版本/CRC/ECC规则，该建议当前未实现。
- 复位后主机boot与session=1可复用，失去持久跨复位隔离；原版也没有旧镜像自动回滚。
- package_image回填描述符长度/CRC，原始链接BIN/HEX不能直接部署，使用打包image.bin。
- Loader不再用参数识别状态限制跳APP，APP仍须保护参数和电机准入。
- BootHw_Init仅识别链接布局，不能证明实际Loader来源；邮箱node当前不被Loader消费。

## 离线验证和证据

| 检查 | 结果 |
| --- | --- |
| test_loader_upgrade.py | PASS，真实通用升级/镜像/邮箱源码的纯内存用例 |
| review_probe | PASS，复现未激活启动、不同boot BEGIN、复位身份复用 |
| quick doctor / project-layout | PASS；工程检查仅普通APP，不代替独立Loader验收 |
| quick format / lint | FAIL，完整日志记录现有问题，本轮不扩大源码修改范围 |
| quick unit / integration | unit 38/39；唯一失败为10条旧outputs文档断链，integration PASS |
| docs检查 | 文档更新后仍为原10条旧outputs断链，无新增问题；文档diff空白检查通过 |

用工作树已有.venv直接运行harness，未安装依赖。
证据：outputs/runs/loader-simplification-review-20260928/summary.json，
更新后复验outputs/runs/loader-simplification-review-20260928-final/summary.json仍为FAIL，
outputs/reviews/loader-simplification-20260928/native-review.log、review_probe.c与源快照摘要。
本轮未做Keil构建、烧录、CAN/J-Link或电机操作；实施记录中的旧构建结果作为历史，不能替代本次验收。

## 修复顺序、验收与回退

1. 恢复健康喂狗、异常复位驻留与维护角色保护。
2. 冻结新协议/profile/布局，同步三端与向量，修复BEGIN匹配。
3. 决定ACTIVATE掉电门槛，完善Flash与生成输入一致性。
4. 对当前候选运行PR、独立Loader和配套APP release检查，按加载跨度确认槽容量。
5. 按明确台架流程测试掉电、坏APP、精确ACK及ECC，结束恢复原固件与完整参数。

接受标准：P1/P2关闭、新旧包可判别、正常APP持续运行、故障APP可救援、
维护镜像不误使能、参数原样保留、证据绑定当前源和产物哈希。
回退先保存本批补丁，再恢复旧布局及匹配Loader/APP；禁止清理其他进行中的用户改动。

## 修复执行记录（2026-09-28）

目标：消除简化引入的实际错误，保留三段分区、无USB、CAN FD优先、参数只读、
顺序写入/读回、精确ACK、安全停机的现有边界。此批为行为修复，非行为等价重构。
基线为95f1aa0a及工作树现存文档；保留angle_feedback.c与Keil用户配置的无关改动。

| 发现 | 源码修复 | 离线验收 |
| --- | --- | --- |
| R1 | 所有者提供监督完成计数和基础健康；组合层每100ms有条件喂狗 | 实际所有者与实际调度函数覆盖进展停滞、采样失效、CAN失效和计时回绕 |
| R2 | 清标志前区分IWDG/WWDG来源，启动所有者驻留 | 真实runtime覆盖冷启动、ENTER、看门狗、坏镜像、链路丢失 |
| R3 | 仅普通APP；打包/构建宏/清单/描述符拒绝role2 | 真打包CLI与设备解码负例 |
| R4 | 回复HOST_NONCE标志明确升级v2；空闲boot0、主机新随机nonce | 40B发现向量与实际C编解码；主机需按新文档适配，未交付主机产品 |
| R5 | CRCprofile=0x53593449、描述符v2、layout4、新包格式 | 镜像/清单黄金字节，旧profile/version/layout拒绝 |
| R6 | 活动BEGIN比较source、nonce、完整清单 | 同身份幂等、不同身份拒绝，不二次擦除 |
| R7 | 完整SR错误位、1s软件等待、写前失败退出、ISB及缓存/中断恢复 | 抽取实际flash_ready与vendor位定义，覆盖全部错误、BSY截止与回绕；真实写擦/ECC待台架 |

另修复ACTIVATE后整包复核失败仍显示VERIFIED的问题，现转ERROR/DIGEST且不复位；
ENTER邮箱node改为本机节点，与字段含义一致。保留无持久ACTIVATE门槛的简化取舍，
不新增日志页，不承诺掉电续传、签名认证或旧镜像自动回滚。
协议/迁移和黄金向量见[载荷v2](../../archive/protocols/yg_protocol_upgrade_wire_v2.md)。

验收命令通过工作树已有`.venv`执行（此机未提供uv命令）；不安装依赖、不接触硬件。

### 当前验收结果

| 检查 | 结果与限制 |
| --- | --- |
| PR profile | FAIL；doctor、工程布局、architecture、hygiene通过；全部25项native及17项integration通过 |
| Python unit | 42/43通过；唯一失败是历史报告缺少10条outputs链接，4项新打包/向量用例通过 |
| format / lint | 全仓仍为原17/601条问题；与初审最终日志计数一致；本批修改文件没有新增问题 |
| interfaces | 34条现有接口契约缺失，位于未修改模块；本批新增公共声明具备完整契约 |
| release profile | 配置本机Keil后doctor通过；仍因上述问题及工作树未提交而FAIL，未由profile触发构建 |
| 独立Keil重建 | Loader、LoaderApp、普通APP分别0 errors / 0 warnings；构建钩子均关闭，无下载 |
| 实际产物打包 | 当前LoaderApp AXF转BIN，再用实际打包CLI输出v2镜像/清单通过；开发image_version=0，不是发布包 |
| 实际镜像启动模型 | 当前打包BIN替换合成黄金镜像，真实runtime通过正常启动/ENTER/看门狗驻留/损坏拒绝/链路失效；硬件仍是桩 |
| 台架与栈峰值 | 未测试；不得宣称实机掉电、ECC、CAN可靠性或最坏栈通过 |

源码资源结果（非正式发布）：

| 目标 | map ROM/实际加载跨度 | 静态RAM含栈堆 | 容量边界 |
| --- | --- | --- | --- |
| Loader | 13,520B / 13,520B | 4,952B | Flash20,480B，余6,960B |
| LoaderApp | ROM90,540B；保守未压缩跨度91,032B | 31,616B | Flash94,208B；RAM32,736B，余1,120B |
| 普通APP | ROM88,892B；保守未压缩跨度89,316B | 31,568B | 原独立APP目标，不能直接用于Loader升级 |

LoaderApp原始BIN90,580B，补齐后image.bin90,584B，槽余3,624B。
区别来自链接段间隙与RW压缩；验收不能只看Code数字。新增健康接口没有扩大栈/堆预留。

当前证据（均在outputs，未跟踪）：

- `outputs/runs/loader-fixes-pr-20260928/summary.json`：完整PR结果。
- `outputs/runs/loader-fixes-release-configured-20260928/summary.json`：完整release失败原因。
- `outputs/reviews/loader-fixes-20260928/summary.json`：源码与当前AXF/HEX/MAP、BIN/清单哈希及离线验证索引。
- 同目录三目标构建日志、`package/`与`runtime-real-image/test.log`：当前产物及实际镜像模型日志。

不提交、清理或改写无关用户工作来强行满足release的干净来源要求；保留当前修复可审阅。
剩余发布门禁属于仓库已有问题，P1/P2源码修复已完成，硬件验收仍待明确台架步骤。
