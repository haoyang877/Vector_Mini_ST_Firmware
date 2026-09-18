#ifndef FAST_LOOP_PROFILE_H
#define FAST_LOOP_PROFILE_H

/* 可选快速环诊断钩子：板级端口拥有时钟与累计值，控制代码只传递阶段标识。
 * 未启用 FAST_LOOP_STAGE_PROFILE 的构建不会产生函数调用或 RAM 占用。 */
enum
{
    FAST_PROFILE_SENSING = 1,
    FAST_PROFILE_ENCODER,
    FAST_PROFILE_COMMANDS,
    FAST_PROFILE_POSITION_WITH_CURRENT,
    FAST_PROFILE_CURRENT,
    FAST_PROFILE_POST_CONTROL,
    FAST_PROFILE_TELEMETRY,
    FAST_PROFILE_EMPTY,
    FAST_PROFILE_ENCODER_REQUEST,
    FAST_PROFILE_POSITION_ONLY,
    FAST_PROFILE_TRAJECTORY,
    FAST_PROFILE_FRICTION,
    FAST_PROFILE_SPEED_PI,
    FAST_PROFILE_PLAN_PREPARE
};

#if defined(FAST_LOOP_STAGE_PROFILE) && FAST_LOOP_STAGE_PROFILE
/**
 * @brief 标记一个快速环阶段开始，用于累计所选阶段的执行时间。
 * @param stage 阶段标识，取值为 FAST_PROFILE 系列枚举；允许不同标识嵌套。
 * @note 仅由单一快速 ISR 调用；选择或清零统计阶段时电机必须处于禁用状态。
 */
void FastLoopProfile_Begin(unsigned stage);
/**
 * @brief 结束与最近一次 Begin 对应的快速环阶段。
 * @param stage 必须与待结束的 FAST_PROFILE 系列阶段标识一致。
 * @note 仅累计当前选中的诊断阶段；不应跨中断周期保留未闭合阶段。
 */
void FastLoopProfile_End(unsigned stage);
#define FAST_PROFILE_BEGIN(stage) FastLoopProfile_Begin(stage)
#define FAST_PROFILE_END(stage) FastLoopProfile_End(stage)
#else
#define FAST_PROFILE_BEGIN(stage) ((void)0)
#define FAST_PROFILE_END(stage) ((void)0)
#endif

#endif
