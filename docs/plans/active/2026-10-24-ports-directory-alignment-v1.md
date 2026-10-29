# 双板ports目录对齐与接口实现表 v1

日期：2026-10-24。用户授权目录/引用调整和离线验证，禁止烧录或操作硬件。

后续CAN独立批次已将两板comm端口拆为BSP/控制器/IRQ及共享帧校验，
当前实现和验证见[CAN职责计划](../completed/2026-10-24-can-responsibility-refactor-v1.md)；
下方目录迁移仍保留为本轮原始范围，不代表CAN当前继续使用旧comm文件。

## 目标和冻结边界

两板ports统一采用board、boot、comm、motor、time五类职责目录。
旧板已按该结构组织，本轮主要搬迁AT32现有平铺文件，保持文件名、函数和行为。
不搬迁公共drivers或BSP实现，不改变APP、中间层、公共API、协议、参数布局、
控制频率、CAN 1M/5M、功率/故障保护、编译和链接选项。
只允许同步源文件/头文件路径、IncludePath、工程源清单及测试/文档引用；
相对include调整必须仍指向原来的同一个文件，声明和实现内容保持。

## 搬迁与文档

- board：board_at32f456.c、indicator_at32f456.c、param_store_at32f456.c。
- boot：boot_at32f456.c；comm：comm_at32f456.c；time：time_at32f456.c。
- motor：motor_sensing、temperature、motor_temperature及其私有头、sin/cos诊断和采样/存储协作头。
- 头文件仅按所属能力放置，不因当前实现跨BSP/ports而额外移动函数或改变所有权。
- 更新七份AT32工程的源清单和必要搜索路径，保留旧板普通APP和Loader配套工程。
- 对应表按公共头/函数族记录两板真正实现位置；PWM、采样、编码器等在drivers/BSP的实现如实列出。
  配置头、无支持的Loader能力和平台私有诊断不冒充同等已实现功能。

## 验证、基线和回退

基线为实际工作树，源码哈希、搬迁表和工程备份：`outputs/ports-alignment-20261024/`。
先构建迁移前九目标：旧板、新板Off/限时、Loader配套APP及五个独立台架；
迁移后重建相同目标，比对HEX加载字节、Code/RO/RW/ZI与工程源文件对应关系。
检查搬迁文件仅include路径不同，其他源码只允许必要引用调整，公共API和中间层哈希不变。
运行受影响原生回归、项目路径、架构/接口、文档和整仓release离线检查。
原有未提交工作保留；不为通过发布门禁擅自提交或清理用户修改。
回退时按搬迁表恢复文件和相应路径，不覆盖其他独立任务改动；历史实机结果不外推到新镜像。

## 本轮完成结果

12个AT32文件已按上述五类目录搬迁，文件名保持；旧板ports不需要搬迁。
七个AT32工程同步源路径，仅增加或替换必要的`ports/motor`私有头搜索路径；
旧板普通APP和Loader配套APP工程内容保持。现有三个本地Keil选项文件同步路径，
不修改调试/下载设置；原生测试只更新路径，没有改断言或行为。
公共16个API头的实际实现、编译期配置和私有扩展分别见
[公共接口与实现对应表](../../architecture/platform_interface_implementations_v1.md)。

| 核对项 | 本轮结果 |
| --- | --- |
| 基线/迁移后九目标完整重建 | 全部0错误、0警告；包括四个完整APP和五个独立台架目标 |
| 九目标加载镜像及资源 | 每目标HEX逐字节一致，Code/RO/RW/ZI均一致；AXF/MAP另存哈希，调试路径变化不要求整文件哈希相同 |
| 工程与路径 | 九份工程除路径外逐字节一致；九目标源文件/include解析均0错误 |
| 源码边界 | 296份基线源码逐项核对，12个文件迁移；仅7份C/H需调整相对include，逆向路径替换后均与基线逐字节一致 |
| APP/中间层/公共接口 | APP、Motor、Parameters、Communication、shared、Loader及16个公共API头逐字节一致；两板main/platform_app保持 |
| 离线测试 | 109项单元、14项集成、36组原生回归全部PASS；包含两板采样、编码器、CAN故障、PWM关相、存储及Loader兼容回归 |
| 质量/边界检查 | format、lint、project-layout、architecture、interfaces、docs、hygiene全部PASS；原有允许项未增加 |
| 正式release状态 | FAIL：工作树有既有及本轮未提交改动，clean-release-source门禁失败，故该流水线未执行keil-build，release-manifest随后失败；不能标为正式发布通过 |
| 硬件 | 未烧录、复位、连接探针、操作CAN或驱动电机 |

证据：`outputs/ports-alignment-20261024/summary.json`汇总本轮结果；
同目录`source-before.json`、`migration-map.json`、`path-edits.json`、`boundary.json`、
`all-project-layout.json`及`baseline-build/`、`after-build/`保存边界、日志与产物哈希。
整仓检查记录为`outputs/runs/ports-alignment-20261024-release/summary.json`，
其中tests和各离线门禁PASS，只有上述正式发布资格/清单两项FAIL。
未提交、清理或覆盖其他工作来绕过门禁；本轮镜像一致性不能代替后续实机验收。
