#ifndef FAST_LOOP_PROFILE_H
#define FAST_LOOP_PROFILE_H

/* Optional diagnostic hooks. The board port owns the clock and accumulation;
 * control code sees only stage identifiers. Disabled builds have no calls/RAM. */
enum {
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
/* Single ISR owner. Select/reset a stage only while the motor is disabled.
 * Different stage IDs may nest; only the selected stage is accumulated. */
void FastLoopProfile_Begin(unsigned stage);
void FastLoopProfile_End(unsigned stage);
#define FAST_PROFILE_BEGIN(stage) FastLoopProfile_Begin(stage)
#define FAST_PROFILE_END(stage) FastLoopProfile_End(stage)
#else
#define FAST_PROFILE_BEGIN(stage) ((void)0)
#define FAST_PROFILE_END(stage) ((void)0)
#endif

#endif
