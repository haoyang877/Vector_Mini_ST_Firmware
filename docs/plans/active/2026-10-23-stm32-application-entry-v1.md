# STM32G4 APP入口统一到application v1

日期：2026-10-23。范围仅目录、工程引用和离线验证，不授权硬件操作。

## 目标与不变边界

旧板入口从`stm32g4/cubemx/Core/Src/main.c`迁移到`stm32g4/application/main.c`，
与新板`at32f456/application/main.c`采用相同目录层级。
入口源码逐字节保留；BootHw_Init、HAL/时钟/外设初始化、预取、App_Init与App_Process顺序不变。
中断接线、20kHz/2kHz时基、状态/故障、功率及电流门限、模型参数、ABI和BSP接口不变。
旧板普通APP和Loader配套APP同时更新，Loader本体和AT32代码不改。

## 实施与生成边界

更新两份Keil工程的入口引用、CubeMX的MainLocation、当前路径映射及相关文档/测试。
搬迁的文件仍为保留USER CODE标记的CubeMX入口，沿用生成文件格式，
根格式规则仅对这一文件保留生成代码豁免；不豁免整个application目录。
架构检查将新目录明确登记为平台应用装配层，允许依赖APP、平台接口和CubeMX外设声明。
本轮不运行CubeMX生成器；后续再生成时核对入口落点、工程分组与源清单，不允许两个main同时链接。

## 基线、验收与回退

基线为`outputs/stm32-application-entry-20261023/`中的源码哈希、原入口/工程/IOC及旧板镜像。
核对搬迁前后入口内容完全一致，其他固件源码哈希不变；比较普通APP的实际HEX加载字节。
编译普通APP与Loader配套APP，检查入口唯一、工程路径与头文件可解析，运行相关离线测试及release检查。
若存在其他本地工作导致的全仓失败，单列记录，不据此报告发布通过。
回退时恢复入口原位置及本轮路径/规则变更，不覆盖其他改动。

## 定向验收结果

- 搬迁完成，原路径不存在；两份工程均只引用`stm32g4/application/main.c`，工程内容各仅更改一个FilePath。
- 普通APP的uvoptx仅同步main.c文件路径，保留其他调试选项；个人uvguix窗口缓存不作为构建输入。
- 入口逐字节相同，普通APP的HEX文件也完全相同：80248个Flash加载字节，0个地址差异。
- 普通APP和Loader配套APP显式重建均0错误0警告；Loader配套镜像的本地后处理通过。
- 相关12项目标/USB集成测试、工程路径、架构和文档检查通过；更新的两份Python测试格式通过。
- 参数管理、通信业务、应用及电机功能层、BSP接口和旧板驱动相对迁移基线未变。
- 工作区同期AT32入口和编码器端口存在独立改动，边界记录单列，未由本轮修改或回退。
- IOC已设置`MainLocation=../application`；未运行CubeMX再生成，实际落点仍需后续再生成时核验。
- 未烧录、复位或接触硬件；详细源码/工程边界、镜像哈希及日志见`outputs/stm32-application-entry-20261023/summary.json`。

## 整仓离线检查

证据：`outputs/runs/20261023T085501464027Z-16d38643/summary.json`，release结果FAIL。
工程布局、架构、文档和lint通过，集成14项及全部已登记原生测试通过。
单元99项中2项失败：同期AT32关相APP的调试前下载设置不符合测试要求，
以及既有native测试清单缺少`test_power_stage_start`。
另有3处既有格式、fake头4项契约注释，以及未提交源码/发布清单条件不满足。
这些问题未由本次入口迁移引入，也未在本轮扩大修改范围处理；不报告整仓发布验收通过。
