# 编码器标定命名整理 v1

本轮只调整标定函数、变量及其调用点的名称。使用 `EncoderCalibration_` 作为公开前缀，
用 `GetPhase`、`IsIdle` 和 `StepElecZero` 表达返回值或动作。私有名称保留角度坐标系和单位。
AT32 scatter 中显式指定的发布函数与当前接口同步改名。

基线是 `the-system-refactor` 工作区的 618 行版本。四组原生轨迹与相关文件副本保存在
`outputs/encoder-calibration-naming-20261104/baseline/`。验收要求前后轨迹逐字节相同，
阶段编号、算法、数据布局和功率控制时序一致。重建双板 APP，确认 AT32 的发布函数位于 Flash，
快速标定入口位于 SRAM，并执行仓库检查。回退时以本轮快照为准，只还原符号改名。

## 验证结果

标定实现的 3236 个 C 词法单元逐项比较，全部 194 处差异均为标识符替换。
两种板卡和阻尼环开关共四组原生测试的输出日志与基线逐字节一致。
124 个单元测试、16 个集成测试、全部原生入口、格式、lint、架构及接口检查通过。

STM32G431 与 AT32F456 APP 均独立重建通过，各为零错误、零警告。AT32 链接 map 将
`EncoderCalibration_PublishLut` 放在 Flash（`0x08000f81`），`EncoderCalibration_StepLut`
和 `EncoderCalibration_StepElecZero` 放在 SRAM（`0x20000d9d`、`0x20000c21`）。
STM32 首次构建缺少 RCC 对象文件，完整重建后链接成功。

`verify --profile release` 的功能检查均通过；工作区含未提交修改，发布源码清洁度和
发布清单两项未通过。检查记录在 `outputs/runs/encoder-calibration-naming-20261104-release/`，
前后轨迹及双板构建日志位于 `outputs/encoder-calibration-naming-20261104/`。
