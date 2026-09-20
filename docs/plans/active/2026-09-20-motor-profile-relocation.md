# 电机装配/启动参数归位 motor 层 v1.0

2026-09-20。把关节模组相关的启动/标定默认参数与阻尼环装配选择从
`platform/api` 移入 `motor` 层，`platform/api` 只保留时基与遥测采样契约。

## 1. 结构问题与目标

- 问题：`platform/api/control_config.h` 混装平台时基契约与电机装配参数（无感启动
  整定、编码器标定默认值、阻尼环变体）。这些参数除 `FOC_FREQ` / `SUPERVISOR_FREQ`
  外没有板级消费者；不同轴装配取值不同（roll/yaw/pitch 关节带阻尼环，左右轮电机
  不带），属于关节模组参数而不是平台能力，放在“平台接口”层会掩盖归属。
- 目标：按所有权拆分——时基/采样率留 `platform/api/control_config.h`；装配选择与
  启动/标定默认值归 `firmware/motor/`。宏名与取值不变，调用点零改动（仅 include 归属）。

## 2. 变更清单

- 移动：`firmware/platform/api/motor_hardware_profile.h` →
  `firmware/motor/motor_hardware_profile.h`（内容逐字节不变）。
- 新增：`firmware/motor/motor_startup_profile.h` = 原 `control_config.h` 的
  “无感启动整定”“编码器标定（Mode 13/15）默认值”“电角度零位对齐时序”三段，
  含 `MOTOR_HAS_DAMPING_RING` 变体（include 新的 profile 头）。
- `control_config.h` 只保留 FOC/速度环/位置环/级联位置环/监督 tick 时基与 RTT 采样契约。
- include 更新：`foc_sensorless_run.c`、`foc_encoder_calibration.c`（新增 startup
  profile）、`app/foc_run.c`（新增 hardware profile）、`hw_conf.h`（删除无消费的
  profile include 并更新注释）、`test_sensorless_transitions.py` 夹具（新增 startup profile）。
- `position_impedance_config.h` 的 `#include "motor_hardware_profile.h"` 经 include
  路径（Keil `../../../../motor`、原生 `firmware/motor`）解析到新位置，无需改动。

## 3. 不变行为

- 全部宏名与取值不变：旧 `control_config.h` + `motor_hardware_profile.h` 的 110 个
  `#define` 与新 `control_config.h` + `motor_startup_profile.h` +
  `motor_hardware_profile.h` 逐一比对一致（唯一新增为 `MOTOR_STARTUP_PROFILE_H` guard）。
- 无 ABI、参数、协议、控制输出、时序、状态机与故障行为改动；仅头文件归属与 include。
- 板级不引用电机/启动参数头；`platform/api` 不再 include motor 相关头，依赖方向更干净。

## 4. 基线与验收

- 基线：提交 `38927516` 的 PR 档证据
  `outputs/runs/20260919T175623949472Z-90982812/summary.json`。
- 验收：`verify --profile pr` 全绿（architecture / interfaces / docs / hygiene + 原生夹具
  `test_sensorless_transitions` 按 damping=0/1 双变体编译执行）。
- Keil/release：待提交后执行（release 档要求干净工作树）。

## 5. 回滚

- 单提交 revert：`git mv` 回 `platform/api/`，恢复 `control_config.h` 原文，还原 5 处
  include 与夹具。无持久化数据或协议痕迹。

## 6. 证据

- PR 档：`outputs/runs/20260919T182014558749Z-a04d0890/summary.json`
  （format/lint/tests/architecture/interfaces/docs/hygiene 全绿）。
