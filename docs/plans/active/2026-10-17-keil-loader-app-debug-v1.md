# Keil 仅下载 LoaderApp 的构建方案 v1

日期：2026-10-17。范围是 Loader 配套 APP 的 Keil 构建与离线产物；不包含本轮实机下载。

## 目标和原状态

Loader 已占用 Flash 起始区，配套 APP 链接在独立 APP 槽。过去 Keil 生成的原始
LoaderApp AXF 只有占位描述符，缺少镜像长度、CRC 和启动标记；直接下载后 Loader
不会把它当作可启动镜像。调试器若强制修改 PC、SP 或 VTOR，会绕过真正的启动验证。

目标是只下载 APP 槽，让复位后的 Loader 按现有镜像判据检查并跳转。CAN 升级包仍需
保持未激活状态，交由 Loader 在完整写入和校验后激活。

## 实施边界

- LoaderApp 工程的 Keil After Build 调用 `keil_loader_app.py`。脚本检查 AXF 的单一
  APP Flash 加载段、目标身份、向量表、描述符占位和长度；失败时返回非零。
- 同一次构建产生两种不同用途的字节：AXF、HEX、BIN 含已激活、已尝试、已确认
  的调试标记；`app-package/image.bin` 的三个标记保持擦除态，用于 CAN 升级。
- Keil 重复执行 After Build 而没有重新链接时，只复用目标、AXF 和全部派生产物
  哈希均匹配的现有清单。任何校验失败都会撤销旧成功清单及派生下载文件，不能
  把旧包误认为本次构建通过。
- 两种镜像共享其余镜像内容与归一化 CRC。后处理生成哈希清单，离线配套构建据此
  核对产物新鲜度和一致性。目标版本、参数 ABI 与 schema 由共同目标清单读取。
- 只改变 LoaderApp 工程，不修改普通 APP 工程、Loader 运行逻辑或参数 Flash 布局。
  Keil 下载时仍需由操作员确认分区擦除和校验设置。

## 验收与回退

离线验收：旧版黄金向量保持不变；当前目标的命令行 CAN 包可生成；合成 AXF 的
AXF/HEX/BIN 字节相同且 Loader 标记有效；CAN 包标记仍为擦除态；无效目标拒绝输出；
重复 After Build 可复用，损坏旧清单或新 AXF 后旧成功产物失效。
最终还需由集成构建核对真实 AXF、资源、Keil 后处理和 `summary.json`。实机下载及
复位跳转只在独占设备的台架流程中验证，本计划不据离线结果宣称实机通过。

回退时关闭 LoaderApp After Build 钩子并恢复原工程及打包入口。这样 Keil 原始
LoaderApp 只可供离线链接检查，不能当作通过 Loader 校验的直下镜像。
