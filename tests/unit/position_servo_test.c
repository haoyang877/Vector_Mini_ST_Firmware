#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include "position_cascade.h"
#include "position_cascade_config.h"

static PositionCascadeConfig_TypeDef config(void)
{
    PositionCascadeConfig_TypeDef c;
    memset(&c, 0, sizeof(c));
    c.update_period_s = 0.0005f;
    c.call_divider = 1U;
    c.position_error_window = 0.001f;
    c.hold_enter_position = POSITION_SERVO_HOLD_ENTER_POSITION_RAD;
    c.hold_exit_position = POSITION_SERVO_HOLD_EXIT_POSITION_RAD;
    c.velocity_filter_hz = POSITION_SERVO_VELOCITY_FILTER_HZ;
    c.hold_velocity_filter_hz = POSITION_SERVO_HOLD_VELOCITY_FILTER_HZ;
    c.following_error_limit = POSITION_SERVO_FOLLOWING_ERROR_LIMIT_RAD;
    c.stiction_integral_rate = POSITION_SERVO_STICTION_INTEGRAL_RATE_A_PER_S;
    c.acceleration = c.deceleration = 0.785398f;
    c.maximum_speed = 0.785398f;
    c.speed_limit = 10.0f;
    c.jerk_limit = 7.85398f;
    c.position_kp = 0.05f;
    c.position_kd = 0.5f;
    c.speed_kp = 0.05f;
    c.speed_ki = 0.5f;
    c.current_limit = 6.0f;
    c.friction_feedforward_enabled = 1;
    c.passive_hold_enabled = false;
    c.friction_coulomb_positive = 1.55f;
    c.friction_coulomb_negative = 1.50f;
    c.friction_breakaway_ratio = 1.20f;
    c.friction_attack_slew_rate = 200.0f;
    c.friction_fast_release_slew_rate = 200.0f;
    c.friction_release_slew_rate = 15.0f;
    c.friction_reference_speed = 0.03f;
    c.friction_stop_speed = 0.02f;
    c.friction_move_speed = 0.05f;
    c.friction_breakaway_distance = 0.003f;
    c.friction_stuck_time = 0.05f;
    return c;
}

static PositionCascadeOutput_TypeDef step(PositionCascadeConfig_TypeDef *c,
    float position, float speed)
{
    PositionCascadeOutput_TypeDef o;
    assert(PositionCascade_Update(c, position, speed, &o));
    assert(isfinite(o.iq_reference));
    assert(fabsf(o.iq_reference) <= c->current_limit + 0.00001f);
    assert(fabsf(o.iq_reference - o.feedback_current -
        o.acceleration_feedforward_current - o.friction_feedforward_current) <
        0.00001f || fabsf(o.iq_reference) >= c->current_limit - 0.00001f);
    return o;
}

static void test_invalid_config(void)
{
    PositionCascadeConfig_TypeDef c = config();
    PositionCascadeOutput_TypeDef o;
    PositionCascade_Reset();
    c.hold_exit_position = c.hold_enter_position;
    assert(!PositionCascade_Update(&c, 0, 0, &o));
    c = config(); c.following_error_limit = -1;
    assert(!PositionCascade_Update(&c, 0, 0, &o));
    c = config(); c.velocity_filter_hz = NAN;
    assert(!PositionCascade_Update(&c, 0, 0, &o));
    c = config(); c.stiction_integral_rate = -1;
    assert(!PositionCascade_Update(&c, 0, 0, &o));
    c = config(); c.stiction_integral_rate = INFINITY;
    assert(!PositionCascade_Update(&c, 0, 0, &o));
    c = config();
    assert(!PositionCascade_Update(&c, NAN, 0, &o));
    assert(!PositionCascade_Update(&c, 0, INFINITY, &o));
    assert(!PositionCascade_Update(&c, 0, 0, NULL));
    assert(PositionCascade_Update(&c, 0, 0, &o));
    assert(PositionCascade_Update(&c, 0, 0, &o));
    c.target_position = NAN;
    assert(!PositionCascade_Update(&c, 0, 0, &o));
    c.target_position = 0.1f;
    assert(PositionCascade_Update(&c, 0, 0, &o));
    c.current_limit = NAN;
    assert(!PositionCascade_Update(&c, 0, 0, &o));
    c = config();
    assert(PositionCascade_Update(&c, 0, 0, &o));
    c.hold_exit_position = c.hold_enter_position;
    assert(!PositionCascade_Update(&c, 0, 0, &o));
}

static void test_low_speed_feedback(void)
{
    PositionCascadeConfig_TypeDef c = config();
    PositionCascadeOutput_TypeDef o;
    PositionCascadeTelemetry_TypeDef t;
    int i;
    c.target_position = 1;
    c.friction_feedforward_enabled = 0;
    PositionCascade_Reset();
    step(&c, 0, 0);
    for (i = 0; i < 400; ++i)
        o = step(&c, 0.02f * (float)i * c.update_period_s, 0.02f);
    assert(fabsf(o.speed_feedback - 0.02f) < 0.0001f);
    assert(PositionCascade_GetTelemetry(&t));
    assert(t.speed_feedback == o.speed_feedback);
}

static void test_quiet_hold_with_recorded_noise_range(void)
{
    PositionCascadeConfig_TypeDef c = config();
    PositionCascadeOutput_TypeDef o;
    int i;
    PositionCascade_Reset();
    /* 此噪声范围为 0.03～0.09°，显式配置对应精度后验证安静保持。 */
    c.position_error_window = .0016f;
    for (i = 0; i < 12000; ++i)
    {
        float p = 0.001f + 0.00045f * sinf((float)i * 0.17f);
        float v = (i & 1) ? 0.012f : -0.012f;
        o = step(&c, p, v);
        assert(fabsf(o.friction_feedforward_current) < 0.000001f);
    }
    assert(o.target_reached && o.phase == POSITION_SERVO_PHASE_HOLD);
    assert(fabsf(o.hold_current) < 0.000001f);
}

/* Fixed sensor injection measures controller noise transfer, not plant stability. */
static void test_hold_velocity_noise_transfer(void)
{
    int rate, frequency, i;
    for (rate = 1000; rate <= 2000; rate += 1000)
    for (frequency = 40; frequency <= 80; frequency += 40)
    {
        PositionCascadeConfig_TypeDef c = config();
        PositionCascadeOutput_TypeDef o = {0};
        float base_velocity = 0, ratio, alpha;
        double baseline_energy = 0, actual_energy = 0;
        c.update_period_s = 1.0f / (float)rate;
        c.position_kp = 8; c.position_kd = 2;
        c.speed_kp = .5f; c.speed_ki = 2;
        c.friction_feedforward_enabled = 0;
        ratio = 6.2831853072f * c.velocity_filter_hz * c.update_period_s;
        alpha = ratio / (1 + ratio);
        PositionCascade_Reset();
        for (i = 0; i < rate; ++i) o = step(&c, 0, 0);
        assert(o.target_reached);
        for (i = 0; i < 6 * rate; ++i)
        {
            float omega = 6.2831853072f * (float)frequency;
            float phase = omega * (float)i * c.update_period_s;
            float position = -.02f / omega * cosf(phase);
            float speed = .02f * sinf(phase);
            float baseline;
            base_velocity += alpha * (speed - base_velocity);
            baseline = c.speed_kp * c.current_limit *
                (-c.position_kp * position - (1 + c.position_kd) * base_velocity);
            o = step(&c, position, speed);
            assert(o.target_reached && o.phase == POSITION_SERVO_PHASE_HOLD);
            assert(fabsf(o.hold_current) < .000001f);
            if (i >= rate) {
                baseline_energy += (double)baseline * baseline;
                actual_energy += (double)o.iq_reference * o.iq_reference;
            }
        }
        if (POSITION_SERVO_HOLD_VELOCITY_FILTER_HZ == 10.0f)
            assert(actual_energy < .10 * baseline_energy);
        else if (POSITION_SERVO_HOLD_VELOCITY_FILTER_HZ == 0.0f)
            assert(fabs(actual_energy / baseline_energy - 1) < .0001);
        printf("HOLD injection %d Hz at %d Hz loop: baseline %.6f A RMS, actual %.6f A RMS\n",
            frequency, rate, sqrt(baseline_energy / (5 * rate)),
            sqrt(actual_energy / (5 * rate)));
    }
}

static void test_hold_filter_keeps_stiffness_and_exit_sensing(void)
{
    PositionCascadeConfig_TypeDef c = config();
    PositionCascadeOutput_TypeDef o;
    PositionCascadeTelemetry_TypeDef t;
    int i, sign;
    c.position_kp = 8; c.position_kd = 2;
    c.speed_kp = .5f; c.speed_ki = 2;
    c.friction_feedforward_enabled = 0;
    c.velocity_filter_hz = 0;
    for (sign = -1; sign <= 1; sign += 2) {
        PositionCascade_Reset();
        c.target_position = 0;
        for (i = 0; i < 200; ++i) o = step(&c, 0, 0);
        assert(o.target_reached);
        o = step(&c, sign * .001f, 0);
        assert(o.target_reached);
        assert(fabsf(o.iq_reference + sign * .024f) < .000001f);
        o = step(&c, 0, sign * .09f);
        assert(!o.target_reached && o.phase == POSITION_SERVO_PHASE_SETTLE);
        assert(o.speed_feedback == sign * .09f);
        for (i = 0; i < 400; ++i) o = step(&c, 0, 0);
        assert(o.target_reached);
        c.target_position = sign * .1f;
        o = step(&c, 0, sign * .01f);
        assert(!o.target_reached && o.phase == POSITION_SERVO_PHASE_MOVE);
        assert(o.speed_feedback == sign * .01f);
        assert(PositionCascade_GetTelemetry(&t));
        assert(t.speed_feedback == o.speed_feedback);
    }
}

static void test_live_hold_filter_switch_preserves_state(void)
{
    PositionCascadeConfig_TypeDef c = config();
    PositionCascadeOutput_TypeDef o;
    float support;
    int i, toggle;
    c.friction_feedforward_enabled = 0;
    c.target_position = .05f;
    PositionCascade_Reset();
    for (i = 0; i < 3000; ++i) o = step(&c, 0, 0);
    for (i = 0; i < 3000; ++i) o = step(&c, c.target_position, 0);
    support = o.hold_current;
    assert(o.target_reached && support > .01f);
    for (toggle = 0; toggle < 20; ++toggle) {
        c.hold_velocity_filter_hz = (float)(toggle % 3) * 5.0f;
        for (i = 0; i < 20; ++i) {
            o = step(&c, c.target_position, .005f);
            assert(o.phase == POSITION_SERVO_PHASE_HOLD && o.target_reached);
            assert(o.position_reference == c.target_position);
            assert(fabsf(o.hold_current - support) < .000001f);
        }
        assert(PositionCascade_GetConfiguration()->hold_velocity_filter_hz == c.hold_velocity_filter_hz);
    }
    c.hold_velocity_filter_hz = NAN;
    assert(!PositionCascade_Update(&c, c.target_position, 0, &o));
    c.hold_velocity_filter_hz = -1;
    assert(!PositionCascade_Update(&c, c.target_position, 0, &o));
}

static void test_hold_retains_support_and_recovers_disturbance(void)
{
    PositionCascadeConfig_TypeDef c = config();
    PositionCascadeOutput_TypeDef o;
    float support;
    int i;
    PositionCascade_Reset();
    c.target_position = 0.05f;
    c.following_error_limit = 0;
    c.friction_feedforward_enabled = 0;
    for (i = 0; i < 3000; ++i) o = step(&c, 0, 0);
    for (i = 0; i < 3000; ++i) o = step(&c, c.target_position, 0);
    assert(o.target_reached);
    support = o.hold_current;
    assert(support > 0.01f);
    for (i = 0; i < 2000; ++i)
    {
        o = step(&c, c.target_position + c.position_error_window * 1.2f, 0);
        assert(o.target_reached);
        assert(fabsf(o.hold_current - support) < 0.00001f);
    }
    c.friction_feedforward_enabled = 1;
    o = step(&c, c.target_position + 0.01f, 0);
    assert(!o.target_reached && o.phase == POSITION_SERVO_PHASE_SETTLE);
    assert(fabsf(o.hold_current - support) < 0.001f);
    for (i = 0; i < 500; ++i) o = step(&c, c.target_position + 0.01f, 0);
    assert(o.friction_feedforward_current < -0.5f);
    for (i = 0; i < 500; ++i) o = step(&c, c.target_position + c.position_error_window*.8f, 0);
    assert(o.target_reached && fabsf(o.friction_feedforward_current) < 0.00001f);
}

static void test_hold_exit_restarts_opposing_integral_transport(void)
{
    int sign, i;
    for (sign=-1; sign<=1; sign+=2) {
        PositionCascadeConfig_TypeDef c = config();
        PositionCascadeOutput_TypeDef o;
        float support, previous;
        c.target_position = sign * 0.05f;
        c.position_kp = 8; c.position_kd = 2;
        c.speed_kp = .5f; c.speed_ki = .5f;
        c.friction_feedforward_enabled = 0;
        c.stiction_integral_rate = 0;
        PositionCascade_Reset();
        for (i=0; i<3000; ++i) o=step(&c, 0, 0);
        for (i=0; i<3000; ++i) o=step(&c, c.target_position, 0);
        assert(o.target_reached);
        support=o.hold_current;
        assert(sign*support > .3f);
        c.speed_ki=0; /* Isolate transport from ordinary PI integration. */
        for (i=0; i<200; ++i) {
            o=step(&c,c.target_position-sign*.01f,0);
            assert(fabsf(o.hold_current-support)<.00001f); /* Helpful support stays. */
        }
        for (i=0; i<300; ++i) o=step(&c,c.target_position,0);
        assert(o.target_reached);
        previous=o.hold_current;
        for (i=0; i<200; ++i) {
            o=step(&c,c.target_position+sign*.01f,sign*.1f);
            assert(!o.target_reached);
            assert(fabsf(o.hold_current-previous) <=
                POSITION_SERVO_INTEGRAL_OPPOSING_MAX_SLEW_A_PER_S*c.update_period_s+.00001f);
            previous=o.hold_current;
        }
        assert(fabsf(o.hold_current) < fabsf(support)-.1f);
    }
}

static void test_recover_stalled_landing_before_trajectory_ends(void)
{
    PositionCascadeConfig_TypeDef c = config();
    PositionCascadeOutput_TypeDef o;
    PositionCascadeTelemetry_TypeDef t;
    int i;
    PositionCascade_Reset();
    c.target_position = 0.2f;
    c.velocity_filter_hz = 0;
    c.acceleration = c.deceleration = 0.02f;
    c.jerk_limit = 0.2f;
    step(&c, 0.16f, 0.4f);
    for (i = 0; i < 10; ++i) step(&c, 0.19f, 0.15f);
    for (i = 0; i < 400; ++i) o = step(&c, 0.19f, 0);
    assert(o.phase == POSITION_SERVO_PHASE_MOVE);
    assert(PositionCascade_GetTelemetry(&t));
    assert(t.settle_recovery_active);
    assert(!t.friction_landing_active);
    assert(o.friction_feedforward_current > 1.0f);
}

static void test_deceleration_keeps_friction_until_capture(void)
{
    int sign, i;
    for (sign = -1; sign <= 1; sign += 2)
    {
        PositionCascadeConfig_TypeDef c = config();
        PositionCascadeOutput_TypeDef o;
        PositionCascadeTelemetry_TypeDef t;
        float full = sign > 0 ? c.friction_coulomb_positive : c.friction_coulomb_negative;
        c.target_position = (float)sign;
        c.velocity_filter_hz = 0;
        PositionCascade_Reset();
        step(&c, 0, 0);
        /* Already inside the inertial braking envelope, still 8.6 deg away. */
        for (i = 0; i < 100; ++i) o = step(&c, sign * .85f, sign * .6f);
        assert(PositionCascade_GetTelemetry(&t) && t.friction_landing_active);
        assert(fabsf(o.friction_feedforward_current - sign * full) < .001f);
        /* Retain compensation outside capture, including the old plateau. */
        for (i = 0; i < 100; ++i) o = step(&c, sign * .995f, sign * .04f);
        assert(fabsf(o.friction_feedforward_current - sign * full) < .001f);
        /* Smoothly unload near capture, both directions. */
        for (i = 0; i < 100; ++i)
            o = step(&c, sign * (1.0f-c.position_error_window*.65f), sign * .04f);
        assert(sign * o.friction_feedforward_current > .1f);
        assert(sign * o.friction_feedforward_current < full * .75f);
        for (i = 0; i < 100; ++i) o = step(&c, sign * 1.01f, sign * .04f);
        assert(fabsf(o.friction_feedforward_current) < .001f);
    }
}

static void test_reversal_and_saturation(void)
{
    PositionCascadeConfig_TypeDef c = config();
    PositionCascadeOutput_TypeDef o;
    float previous;
    int i, crossed = 0;
    PositionCascade_Reset();
    c.target_position = 0.2f;
    for (i = 0; i < 500; ++i) o = step(&c, 0, 0);
    previous = o.friction_feedforward_current;
    assert(previous > 1);
    c.target_position = -0.2f;
    for (i = 0; i < 2000; ++i)
    {
        o = step(&c, 0, 0);
        assert(!(previous > 0 && o.friction_feedforward_current < 0));
        if (o.friction_feedforward_current < -1) crossed = 1;
        previous = o.friction_feedforward_current;
    }
    assert(crossed);
    c.current_limit = 0.3f;
    for (i = 0; i < 4000; ++i)
    {
        o = step(&c, 0, 0);
        assert(fabsf(o.hold_current) <= 0.30001f);
    }
}

static void test_braking_preserves_learned_load_until_reversal(void)
{
    PositionCascadeConfig_TypeDef c = config();
    PositionCascadeOutput_TypeDef o;
    float learned;
    int i;
    c.target_position = 1.0f;
    c.friction_feedforward_enabled = 0;
    c.velocity_filter_hz = 0;
    PositionCascade_Reset();
    for (i = 0; i < 2000; ++i) o = step(&c, 0, 0);
    learned = o.hold_current;
    assert(learned > .1f);
    /* Isolate transport from ordinary PI updates and saturation. */
    c.speed_kp = c.speed_ki = 0;
    for (i = 0; i < 100; ++i)
    {
        o = step(&c, .5f, 2.0f);
        assert(o.speed_reference < 2.0f); /* Brake, still short of target. */
        assert(fabsf(o.hold_current-learned) < .0001f);
    }
    c.target_position = -1.0f;
    for (i = 0; i < 100; ++i) o = step(&c, .5f, 2.0f);
    assert(o.hold_current < learned * .9f);
}

static void test_nearly_finished_reference_retains_approach(void)
{
    PositionCascadeConfig_TypeDef c = config();
    PositionCascadeOutput_TypeDef o;
    PositionCascadeTelemetry_TypeDef t;
    int i, observed = 0;
    c.target_position = .1f;
    c.velocity_filter_hz = 0;
    PositionCascade_Reset();
    o = step(&c, 0, 0);
    assert(PositionCascade_GetTelemetry(&t));
    for (i = 0; i < 4000; ++i)
    {
        o = step(&c, o.position_reference-.01f, t.trajectory_speed_reference);
        assert(PositionCascade_GetTelemetry(&t));
        if (o.phase == POSITION_SERVO_PHASE_MOVE &&
            fabsf(c.target_position-o.position_reference) <= c.position_error_window &&
            fabsf(t.trajectory_speed_reference) < c.friction_reference_speed)
        {
            ++observed;
            assert(o.friction_feedforward_current > .5f);
        }
    }
    assert(observed > 10);
}

static void test_trajectory_decelerates_without_second_speed_rise(void)
{
    const float distances[] = {.087266463f, .34906585f, 1.48352986f};
    int sign, k, i, profile;
    for (profile = 0; profile < 2; ++profile)
    for (sign = -1; sign <= 1; sign += 2) for (k = 0; k < 3; ++k)
    {
        PositionCascadeConfig_TypeDef c = config();
        PositionCascadeOutput_TypeDef o;
        PositionCascadeTelemetry_TypeDef t;
        float previous_speed = 0, previous_acceleration = 0, previous_position = 0;
        int braking = 0;
        if (profile != 0)
        {
            c.deceleration = POSITION_SERVO_DECELERATION_MAX_RAD_S2;
            c.jerk_limit = c.acceleration / POSITION_SERVO_JERK_RAMP_TIME_S;
        }
        c.target_position = sign * distances[k];
        c.friction_feedforward_enabled = 0;
        PositionCascade_Reset(); o = step(&c,0,0);
        assert(PositionCascade_GetTelemetry(&t));
        previous_acceleration = o.acceleration_reference;
        previous_speed = sign*t.trajectory_speed_reference;
        previous_position = sign*o.position_reference;
        for (i = 0; i < 14000; ++i)
        {
            float speed;
            o = step(&c,o.position_reference,t.trajectory_speed_reference);
            assert(PositionCascade_GetTelemetry(&t));
            speed = sign * t.trajectory_speed_reference;
            assert(sign * o.acceleration_reference >= -c.deceleration-.00001f);
            assert(sign * o.acceleration_reference <= c.acceleration+.00001f);
            assert(speed >= -.000001f && speed <= c.maximum_speed+.000001f);
            if (speed < previous_speed-.000001f) braking = 1;
            if (braking) assert(speed <= previous_speed+.000001f);
            assert(sign*o.position_reference >= previous_position-.000001f);
            assert(sign*o.position_reference <= distances[k]+.000001f);
            assert(fabsf(o.acceleration_reference-previous_acceleration) <=
                   c.jerk_limit*c.update_period_s+.00001f);
            previous_acceleration = o.acceleration_reference;
            previous_speed = speed;
            previous_position = sign*o.position_reference;
        }
        assert(braking && fabsf(o.position_reference-c.target_position)<.000001f);
    }
}

static float stalled_reference_peak(float following_limit, int *limited)
{
    PositionCascadeConfig_TypeDef c = config();
    PositionCascadeOutput_TypeDef o;
    PositionCascadeTelemetry_TypeDef t;
    float peak = 0;
    float previous_acceleration = 0;
    int i;
    c.target_position = 1;
    c.following_error_limit = following_limit;
    PositionCascade_Reset();
    for (i = 0; i < 8000; ++i)
    {
        o = step(&c, 0, 0);
        assert(fabsf(o.acceleration_reference - previous_acceleration) <=
            c.jerk_limit * c.update_period_s + 0.00001f);
        previous_acceleration = o.acceleration_reference;
        if (fabsf(o.position_reference) > peak) peak = fabsf(o.position_reference);
        assert(PositionCascade_GetTelemetry(&t));
        if (t.trajectory_limited) *limited = 1;
    }
    return peak;
}

static void test_reference_governor(void)
{
    int limited = 0, unlimited_flag = 0;
    float governed = stalled_reference_peak(0.017453293f, &limited);
    float ungoverned = stalled_reference_peak(0, &unlimited_flag);
    assert(limited && !unlimited_flag);
    assert(governed < ungoverned * 0.25f);
}

static void test_residual_load_can_accumulate_after_boost(void)
{
    PositionCascadeConfig_TypeDef c = config();
    PositionCascadeOutput_TypeDef o;
    float initial_integral;
    int i;
    c.target_position = 0.2f;
    PositionCascade_Reset();
    for (i = 0; i < 1000; ++i) o = step(&c, 0, 0);
    initial_integral = o.hold_current;
    for (i = 0; i < 4000; ++i) o = step(&c, 0, 0);
    assert(o.hold_current > initial_integral + 0.001f);
}

static void test_divider_and_small_retarget(void)
{
    PositionCascadeConfig_TypeDef c = config();
    PositionCascadeOutput_TypeDef o;
    PositionCascadeTelemetry_TypeDef t;
    int i;
    c.call_divider = 10U;
    PositionCascade_Reset();
    o=step(&c,0,0);
    assert(PositionCascade_ShouldDeferTelemetry()); /* Entry validates tuning. */
    for (i = 1; i < 999; ++i) o = step(&c, 0, 0);
    assert(!o.target_reached);
    assert(!PositionCascade_ShouldDeferTelemetry());
    o = step(&c, 0, 0);
    assert(o.target_reached);
    assert(PositionCascade_ShouldDeferTelemetry());
    c.target_position = c.position_error_window*.5f;
    for (i = 0; i < 20000; ++i)
    {
        o = step(&c, 0, 0);
        assert(o.friction_feedforward_current == 0.0f);
    }
    assert(o.target_reached);
    PositionCascade_Reset();
    assert(!PositionCascade_GetTelemetry(&t));
}

static void test_noise_within_position_window_can_hold(void)
{
    PositionCascadeConfig_TypeDef c = config();
    PositionCascadeOutput_TypeDef o;
    PositionCascadeTelemetry_TypeDef t;
    int i;
    /* 到位噪声必须在实际定位精度内，不能把 0.18° 平台当成 0.057° 达标。 */
    PositionCascade_Reset();
    for (i = 0; i < 4000; ++i) step(&c, -0.011f, 0);
    for (i = 0; i < 8000; ++i)
    {
        float p = -(0.75f + 0.15f * sinf((float)i * 0.17f)) *
            c.position_error_window;
        o = step(&c, p, (i & 1) ? 0.006f : -0.006f);
        if (i > 400)
        {
            assert(o.target_reached);
            assert(o.friction_feedforward_current == 0);
            assert(PositionCascade_GetTelemetry(&t));
            assert(!t.stiction_integrating && !t.friction_landing_active);
        }
    }
    o = step(&c, -0.0053f, 0); /* >0.3 deg: exit immediately. */
    assert(!o.target_reached);
}

static void test_stiction_growth_and_motion_exit(void)
{
    int direction, i;
    for (direction = -1; direction <= 1; direction += 2)
    {
        PositionCascadeConfig_TypeDef c = config();
        PositionCascadeOutput_TypeDef o;
        PositionCascadeTelemetry_TypeDef t;
        float before;
        c.target_position = (float)direction * 0.15f;
        c.following_error_limit = 0.017453293f; /* Even if explicitly enabled. */
        c.speed_ki = 0; /* Isolate static-error integration from normal PI. */
        c.velocity_filter_hz = 0;
        PositionCascade_Reset();
        for (i = 0; i < 4000; ++i) o = step(&c, 0, 0);
        before = o.hold_current;
        for (i = 0; i < 2000; ++i) o = step(&c, 0, 0);
        assert((float)direction * (o.hold_current - before) > 0.29f);
        assert((float)direction * (o.hold_current - before) < 0.31f);
        assert(PositionCascade_GetTelemetry(&t) && t.stiction_integrating);
        /* A continuous-motion sample cancels assistance immediately. */
        o = step(&c, 0, (float)direction * 0.03f);
        assert(PositionCascade_GetTelemetry(&t) && !t.stiction_integrating);
        for (i = 0; i < 90; ++i)
        {
            o = step(&c, 0, 0);
            assert(PositionCascade_GetTelemetry(&t) && !t.stiction_integrating);
        }
        c.current_limit = 0.5f;
        for (i = 0; i < 16000; ++i) o = step(&c, 0, 0);
        assert(fabsf(o.iq_reference) <= 0.50001f);
        assert(fabsf(o.hold_current) <= 0.50001f);
        assert(PositionCascade_GetTelemetry(&t) && !t.stiction_integrating);
        PositionCascade_Reset();
        assert(!PositionCascade_GetTelemetry(&t) && !t.stiction_integrating);
    }
}

static void test_capture_survives_feedforward_release(void)
{
    PositionCascadeConfig_TypeDef c = config();
    PositionCascadeOutput_TypeDef o;
    int i;
    PositionCascade_Reset();
    for(i=0; i<4000; ++i) o=step(&c,-0.011f,0);
    assert(fabsf(o.friction_feedforward_current) > 0.1f);
    /* 确认到位前离开精度窗口需重新捕获，HOLD 后才允许滞环漂移。 */
    o=step(&c,-c.position_error_window*.8f,0);
    assert(!o.target_reached && fabsf(o.friction_feedforward_current)>0);
    for(i=0; i<500; ++i) o=step(&c,-c.position_error_window*1.2f,0);
    assert(!o.target_reached);
    for(i=0; i<500; ++i) o=step(&c,-c.position_error_window*.8f,0);
    assert(o.target_reached && o.friction_feedforward_current==0);
    for(i=0; i<500; ++i) o=step(&c,-c.position_error_window*1.2f,0);
    assert(o.target_reached && o.friction_feedforward_current==0);
    for(i=0;i<1000;++i) o=step(&c,-c.position_error_window*1.6f,0);
    assert(!o.target_reached);
}

static void test_stiction_disabled_and_default_governor_off(void)
{
    PositionCascadeConfig_TypeDef c = config();
    PositionCascadeOutput_TypeDef o;
    PositionCascadeTelemetry_TypeDef t;
    int i;
    assert(c.following_error_limit == 0);
    c.target_position = 0.15f;
    c.speed_ki = 0;
    c.stiction_integral_rate = 0;
    PositionCascade_Reset();
    for (i = 0; i < 8000; ++i)
    {
        o = step(&c, 0, 0);
        assert(PositionCascade_GetTelemetry(&t));
        assert(!t.trajectory_limited && !t.stiction_integrating);
        assert(fabsf(o.hold_current) < 0.00001f);
    }
    assert(o.phase == POSITION_SERVO_PHASE_SETTLE);
    assert(fabsf(o.position_reference - c.target_position) < 0.00001f);
}

static void test_uncaptured_hysteresis_band_keeps_correcting(void)
{
    PositionCascadeConfig_TypeDef c = config();
    PositionCascadeOutput_TypeDef o;
    PositionCascadeTelemetry_TypeDef t;
    float initial;
    int i;
    c.speed_ki = 0;
    PositionCascade_Reset();
    /* Hardware trial: never entered HOLD, stalled between enter and exit.
     * This band must not suppress both capture and static recovery. */
    for (i=0; i<4000; ++i) o=step(&c,-0.004276057f,0); /* 0.245 deg */
    initial=o.hold_current;
    for (i=0; i<2000; ++i) o=step(&c,-0.004276057f,0);
    assert(o.hold_current > initial + .29f);
    for (i=0; i<2000; ++i) o=step(&c,-c.position_error_window * .8f,0);
    assert(o.target_reached && o.friction_feedforward_current == 0);
    initial=o.hold_current;
    for (i=0; i<2000; ++i) {
        o=step(&c,-c.position_error_window * 1.2f,0); /* 实际 HOLD 滞环内保持安静。 */
        assert(o.target_reached && o.friction_feedforward_current == 0);
        assert(PositionCascade_GetTelemetry(&t) && !t.stiction_integrating);
    }
    assert(fabsf(o.hold_current-initial)<.00001f);
}

static int replay(const char *input_path, const char *output_path)
{
    PositionCascadeConfig_TypeDef c = config();
    PositionCascadeOutput_TypeDef o;
    PositionCascadeTelemetry_TypeDef telemetry;
    FILE *input = fopen(input_path, "r");
    FILE *output = fopen(output_path, "w");
    char line[256];
    float time_s, target, position, velocity;
    assert(input && output);
    PositionCascade_Reset();
    fputs("time_s,reference_rad,speed_rad_s,iq_A,ff_A,integral_A,phase,reached,governed\n", output);
    while (fgets(line, sizeof(line), input))
    {
        assert(sscanf(line, "%f,%f,%f,%f", &time_s, &target, &position, &velocity) == 4);
        c.target_position = target;
        o = step(&c, position, velocity);
        assert(PositionCascade_GetTelemetry(&telemetry));
        assert(!(telemetry.friction_landing_active && telemetry.settle_recovery_active));
        fprintf(output, "%.4f,%.8f,%.8f,%.6f,%.6f,%.6f,%d,%d,%d\n", time_s,
            o.position_reference, o.speed_feedback, o.iq_reference,
            o.friction_feedforward_current, o.hold_current, (int)o.phase,
            (int)o.target_reached, (int)telemetry.trajectory_limited);
    }
    assert(!ferror(input));
    fclose(input); fclose(output);
    puts("PASS recorded-input replay (not a closed-loop performance prediction)");
    return 0;
}

static void test_bounded_speed_correction_headroom(void)
{
    int direction, i;
    for(direction=-1; direction<=1; direction+=2) {
        PositionCascadeConfig_TypeDef c=config();
        PositionCascadeOutput_TypeDef o;
        PositionCascadeTelemetry_TypeDef t;
        c.target_position=(float)direction;
        c.maximum_speed=.1f; c.speed_limit=.12f; c.position_kp=10;
        c.friction_feedforward_enabled=0;
        PositionCascade_Reset();
        for(i=0; i<4000; ++i) {
            o=step(&c,0,0);
            assert(fabsf(o.speed_reference)<=.120001f);
            assert(PositionCascade_GetTelemetry(&t));
            assert(fabsf(t.trajectory_speed_reference)<=.100001f);
        }
        assert((float)direction*o.speed_reference>.119f);
        c.speed_limit=1;
        o=step(&c,0,0);
        assert(fabsf(o.speed_reference-(float)direction*.15f)<.00001f);
    }
}

static void test_configuration_padding_does_not_trigger_revalidation(void)
{
    PositionCascadeConfig_TypeDef c = config();
    unsigned char *bytes = (unsigned char *)&c;
    size_t i;
    c.call_divider = 10;
    PositionCascade_Reset();
    step(&c, 0, 0);
    assert(PositionCascade_ShouldDeferTelemetry());
    for (i = offsetof(PositionCascadeConfig_TypeDef, call_divider) + sizeof(c.call_divider);
         i < offsetof(PositionCascadeConfig_TypeDef, target_position); ++i) bytes[i] = 0xa5;
    for (i = offsetof(PositionCascadeConfig_TypeDef, passive_hold_enabled) + sizeof(c.passive_hold_enabled);
         i < offsetof(PositionCascadeConfig_TypeDef, friction_coulomb_positive); ++i) bytes[i] = 0x5a;
    step(&c, 0, 0);
    assert(!PositionCascade_ShouldDeferTelemetry());
    c.position_kp += .1f;
    step(&c, 0, 0);
    assert(PositionCascade_ShouldDeferTelemetry());
}

static void test_retarget_preserves_reference_state(void)
{
    PositionCascadeConfig_TypeDef c = config();
    PositionCascadeOutput_TypeDef o, previous;
    PositionCascadeTelemetry_TypeDef t;
    float old_speed;
    int i;
    c.target_position = 1.0f;
    c.friction_feedforward_enabled = 0;
    PositionCascade_Reset();
    o=step(&c,0,0);
    for(i=0;i<600;++i) {
        assert(PositionCascade_GetTelemetry(&t));
        o=step(&c,o.position_reference,t.trajectory_speed_reference);
    }
    assert(PositionCascade_GetTelemetry(&t));
    previous=o; old_speed=t.trajectory_speed_reference;
    assert(old_speed>.01f);
    c.target_position=-.2f;
    o=step(&c,previous.position_reference,old_speed);
    assert(PositionCascade_GetTelemetry(&t));
    assert(fabsf(o.acceleration_reference-previous.acceleration_reference) <= c.jerk_limit*c.update_period_s+.00001f);
    assert(fabsf(t.trajectory_speed_reference-old_speed) <= c.acceleration*c.update_period_s+.00001f);
    assert(fabsf(o.position_reference-previous.position_reference) <= c.maximum_speed*c.update_period_s+.00001f);
    for(i=0;i<16000;++i) {
        previous=o; old_speed=t.trajectory_speed_reference;
        o=step(&c,o.position_reference,old_speed);
        assert(PositionCascade_GetTelemetry(&t));
        assert(fabsf(o.acceleration_reference-previous.acceleration_reference) <= c.jerk_limit*c.update_period_s+.00001f);
    }
    assert(fabsf(o.position_reference-c.target_position)<.000001f);
    /* A stream of targets must not keep restarting the stationary planner. */
    PositionCascade_Reset(); c.target_position=.1f;
    o=step(&c,0,0);
    for(i=0;i<100;++i) {
        c.target_position+=.001f;
        o=step(&c,o.position_reference,0);
    }
    assert(o.position_reference>0.0001f);
}

static void test_capture_release_is_soft_until_target_crossing(void)
{
    int sign, i;
    for (sign = -1; sign <= 1; sign += 2)
    {
        PositionCascadeConfig_TypeDef c = config();
        PositionCascadeOutput_TypeDef o;
        float before;
        c.target_position = sign * .1f;
        c.velocity_filter_hz = 0;
        PositionCascade_Reset();
        for (i = 0; i < 6000; ++i) o = step(&c, 0, 0);
        before = sign * o.friction_feedforward_current;
        assert(before > 1.0f);
        o = step(&c, sign * (.1f-c.position_error_window*.8f), 0);
        assert(fabsf(before - sign * o.friction_feedforward_current -
            POSITION_SERVO_FRICTION_CAPTURE_RELEASE_SLEW_A_PER_S *
            c.update_period_s) < .00001f);
        before = sign * o.friction_feedforward_current;
        o = step(&c, sign * (.1f+c.position_error_window*.8f), 0);
        assert(fabsf(before - sign * o.friction_feedforward_current -
            c.friction_fast_release_slew_rate * c.update_period_s) < .00001f);
        for (i = 0; i < 2000; ++i) o = step(&c, sign * (.1f+c.position_error_window*.8f), 0);
        assert(o.target_reached && o.friction_feedforward_current == 0);

        /* Actual position may enter first, before the reference can latch
         * capture. The approach taper must obey the same soft release. */
        c.target_position = sign * 1.0f;
        PositionCascade_Reset();
        for (i = 0; i < 100; ++i) o = step(&c, 0, sign * .1f);
        assert(fabsf(o.position_reference-c.target_position) > .01f);
        before = sign * o.friction_feedforward_current;
        assert(before > 1.0f);
        o = step(&c, sign * (1.0f-c.position_error_window*.8f), sign * .1f);
        assert(fabsf(before - sign * o.friction_feedforward_current -
            POSITION_SERVO_FRICTION_CAPTURE_RELEASE_SLEW_A_PER_S *
            c.update_period_s) < .00001f);
        before = sign * o.friction_feedforward_current;
        o = step(&c, sign * (1.0f+c.position_error_window*.8f), sign * .1f);
        assert(fabsf(before - sign * o.friction_feedforward_current -
            c.friction_fast_release_slew_rate * c.update_period_s) < .00001f);
    }
}

/* Synthetic ideal follower: verifies numerical/configuration envelopes only.
 * These are test inputs, not identified motors or predictions of stability. */
static void test_operating_envelopes(void)
{
    const float currents[] = {1.0f, 3.0f, 6.0f};
    const float friction_ratios[] = {0.0f, 0.2f, 0.6f};
    const float angles_deg[] = {0.1f, 5.0f, 20.0f, 85.0f};
    unsigned ci, fi, ai;
    int sign, tick;
    for (ci = 0; ci < 3; ++ci)
    for (fi = 0; fi < 3; ++fi)
    for (ai = 0; ai < 4; ++ai)
    for (sign = -1; sign <= 1; sign += 2)
    {
        PositionCascadeConfig_TypeDef c = config();
        PositionCascadeOutput_TypeDef o;
        PositionCascadeTelemetry_TypeDef t;
        float q = 0, v = 0;
        c.current_limit = currents[ci];
        c.friction_feedforward_enabled = fi != 0;
        c.friction_coulomb_positive = currents[ci] * friction_ratios[fi];
        c.friction_coulomb_negative = c.friction_coulomb_positive * 0.8f;
        c.friction_viscous_positive = c.friction_viscous_negative = 0.05f * fi;
        c.acceleration = (ci + 1) * 0.261799388f;
        c.deceleration = fminf(c.acceleration, POSITION_SERVO_DECELERATION_MAX_RAD_S2);
        c.maximum_speed = (ci + 1) * 0.261799388f;
        c.speed_limit = c.maximum_speed * 2;
        c.jerk_limit = fmaxf(c.acceleration, c.deceleration) / POSITION_SERVO_JERK_RAMP_TIME_S;
        c.position_kp = 8; c.position_kd = 2;
        c.speed_kp = .5f; c.speed_ki = 1;
        c.target_position = sign * angles_deg[ai] * 0.01745329252f;
        PositionCascade_Reset();
        for (tick = 0; tick < 40000; ++tick)
        {
            o = step(&c, q, v);
            assert(PositionCascade_GetTelemetry(&t));
            assert(fabsf(t.trajectory_speed_reference) <= c.maximum_speed + .0001f);
            assert(fabsf(t.acceleration_reference) <= c.acceleration + .0001f);
            assert(sign * t.acceleration_reference >= -c.deceleration - .0001f);
            assert(sign * (o.position_reference - q) >= -.000001f);
            assert(sign * (c.target_position - o.position_reference) >= -.000001f);
            q = o.position_reference;
            v = t.trajectory_speed_reference;
            if (o.target_reached && fabsf(q - c.target_position) < .00001f) break;
        }
        assert(tick < 40000);
        assert(o.phase == POSITION_SERVO_PHASE_HOLD);
        assert(fabsf(v) < .00001f);
    }
}

/* Fixed sensor injection verifies one recurrence through phase changes;
 * it is not a mechanical stability simulation. */
static void test_single_filter_across_phases(void)
{
    PositionCascadeConfig_TypeDef c = config();
    PositionCascadeOutput_TypeDef o;
    float filtered = 0, ratio, alpha;
    bool saw_move = false, saw_hold = false;
    int i;
    c.velocity_filter_hz = 10.0f;
    c.hold_velocity_filter_hz = 0.0f;
    c.friction_feedforward_enabled = 0;
    ratio = 6.2831853072f * c.velocity_filter_hz * c.update_period_s;
    alpha = ratio / (1.0f + ratio);
    PositionCascade_Reset();
    for (i = 0; i < 8000; ++i) {
        float speed = .01f * sinf((float)i * .12f);
        if (i == 1000) c.target_position = 1.0f;
        filtered += alpha * (speed - filtered);
        o = step(&c, i < 2000 ? 0.0f : 1.0f, speed);
        assert(fabsf(o.speed_feedback - filtered) < .000001f);
        saw_move = saw_move || o.phase == POSITION_SERVO_PHASE_MOVE;
        saw_hold = saw_hold || o.phase == POSITION_SERVO_PHASE_HOLD;
    }
    assert(saw_move && saw_hold && o.target_reached);
}

static void test_live_filter_reconfiguration(void)
{
    PositionCascadeConfig_TypeDef c = config();
    PositionCascadeOutput_TypeDef o;
    float before, expected, ratio;
    PositionCascade_Reset();
    o = step(&c, 0, 0);
    o = step(&c, 0, .1f);
    before = o.speed_feedback;
    c.velocity_filter_hz = 10.0f;
    c.update_period_s = .001f;
    ratio = 6.2831853072f * 10.0f * .001f;
    expected = before + ratio / (1.0f + ratio) * (.1f - before);
    o = step(&c, 0, .1f);
    assert(fabsf(o.speed_feedback - expected) < .000001f);
    c.velocity_filter_hz = 0;
    o = step(&c, 0, -.1f);
    assert(o.speed_feedback == -.1f);
    c.velocity_filter_hz = 40;
    before = o.speed_feedback;
    ratio = 6.2831853072f * 40.0f * c.update_period_s;
    expected = before + ratio / (1.0f + ratio) * (.1f - before);
    o = step(&c, 0, .1f);
    assert(fabsf(o.speed_feedback - expected) < .000001f);
}

static void test_live_hold_period_reconfiguration(void)
{
    PositionCascadeConfig_TypeDef c = config();
    PositionCascadeOutput_TypeDef o;
    int i;
    c.velocity_filter_hz = 0;
    PositionCascade_Reset();
    o = step(&c, 0, .1f); /* Initialize without starting hold confirmation. */
    assert(!o.target_reached);
    c.update_period_s = .001f;
    for (i = 0; i < 49; ++i) {
        o = step(&c, 0, 0);
        assert(!o.target_reached);
    }
    o = step(&c, 0, 0);
    assert(o.target_reached); /* Still 50 ms after changing from 2 kHz to 1 kHz. */
}

static void test_capture_precision_and_single_static_integral(void)
{
    PositionCascadeConfig_TypeDef c = config();
    PositionCascadeOutput_TypeDef o;
    PositionCascadeTelemetry_TypeDef t;
    float before;
    int direction, i;
    for (direction=-1; direction<=1; direction+=2)
    {
        PositionCascade_Reset();
        for (i=0;i<6000;++i) o=step(&c,direction*.00418879f,0); /* 0.24° 不能提前到位。 */
        assert(!o.target_reached && fabsf(o.friction_feedforward_current) > 1.0f);
        for (i=0;i<1000;++i)
        {
            o=step(&c,direction*.00418879f,direction*.025f);
            assert(!o.target_reached && fabsf(o.friction_feedforward_current)>1.0f);
        }
        o=step(&c,direction*c.position_error_window*.8f,0);
        assert(!o.target_reached && fabsf(o.friction_feedforward_current)>1.0f);
        for (i=0;i<1000;++i) o=step(&c,direction*c.position_error_window*1.3f,0);
        assert(!o.target_reached && o.friction_feedforward_current==0);
        for (i=0;i<2000;++i) o=step(&c,direction*c.position_error_window*.8f,0);
        assert(o.target_reached && o.friction_feedforward_current == 0);
        o=step(&c,direction*c.hold_exit_position*1.1f,0);
        assert(!o.target_reached); /* 大偏差使用原始反馈立即退出，不等待资格滤波。 */
    }
    c.friction_feedforward_enabled=0;
    c.position_kp=8; c.speed_ki=1; c.velocity_filter_hz=0;
    PositionCascade_Reset();
    for (i=0;i<4000;++i) o=step(&c,-.004f,0);
    before=o.hold_current;
    for (i=0;i<2000;++i)
    {
        o=step(&c,-.004f,0);
        assert(PositionCascade_GetTelemetry(&t) && t.stiction_integrating);
    }
    assert(fabsf(o.hold_current-before-c.stiction_integral_rate) < .002f);
    o=step(&c,-.004f,.03f);
    assert(PositionCascade_GetTelemetry(&t) && !t.stiction_integrating);
    /* 量化噪声瞬时越过捕获精度，但均值达标：只影响资格滤波，不反复释放 HOLD。 */
    c=config();c.velocity_filter_hz=5;
    PositionCascade_Reset();
    for(i=0;i<10000;++i)
    {
        o=step(&c,(i&1)? .0011f : .0003f,(i&1)? .012f : -.012f);
        if(i>1000){assert(o.target_reached && o.friction_feedforward_current==0);}
    }
    o=step(&c,c.hold_exit_position*1.1f,0);
    assert(!o.target_reached);
}

static void test_settle_near_target_no_friction_restart(void)
{
    int sign,i;
    for(sign=-1;sign<=1;sign+=2)
    {
        PositionCascadeConfig_TypeDef c=config();
        PositionCascadeOutput_TypeDef o;
        float before;
        c.target_position=sign*.1f;c.velocity_filter_hz=0;c.speed_ki=0;
        PositionCascade_Reset();
        for(i=0;i<6000;++i){o=step(&c,0,0);}
        assert(o.phase==POSITION_SERVO_PHASE_SETTLE && fabsf(o.friction_feedforward_current)>1);
        for(i=0;i<500;++i){o=step(&c,c.target_position-sign*.0022f,0);}
        assert(!o.target_reached && o.friction_feedforward_current==0);
        before=o.hold_current;
        for(i=0;i<2000;++i)
        {
            float error=(i&1)? .0028f : .0035f;
            o=step(&c,c.target_position-sign*error,0);
            assert(!o.target_reached && o.friction_feedforward_current==0);
        }
        assert(fabsf(sign*(o.hold_current-before)-c.stiction_integral_rate)<.002f);
        for(i=0;i<500;++i){o=step(&c,c.target_position-sign*c.hold_exit_position*1.2f,0);}
        assert(fabsf(o.friction_feedforward_current)>1); /* 大扰动仍保留模型恢复。 */
        for(i=0;i<500;++i){o=step(&c,c.target_position-sign*.002f,0);}
        assert(o.friction_feedforward_current==0);
        c.target_position+=sign*.0015f;
        for(i=0;i<30;++i){o=step(&c,c.target_position-sign*.0035f,0);}
        assert(o.phase==POSITION_SERVO_PHASE_MOVE && fabsf(o.friction_feedforward_current)>0);
    }
}

/* 固定传感器输入验证命令与状态，不代表实物阻尼保持能力。 */
static void test_damped_passive_hold(void)
{
    int sign, rate, i;
    const float rad_per_deg = 0.01745329252f;
    for (sign = -1; sign <= 1; sign += 2)
    for (rate = 1000; rate <= 2000; rate += 1000)
    {
        PositionCascadeConfig_TypeDef c = config();
        PositionCascadeOutput_TypeDef o;
        PositionCascadeTelemetry_TypeDef t;
        float previous, frozen;
        int entered = 0;
        c.passive_hold_enabled = true;
        c.position_error_window = POSITION_SERVO_DAMPED_ERROR_WINDOW_RAD;
        c.current_limit = 8;
        c.position_kp = 8;
        c.speed_kp = .5f;
        c.speed_ki = 1;
        c.velocity_filter_hz = 0;
        c.update_period_s = 1.0f / (float)rate;
        c.target_position = sign * .1f;
        PositionCascade_Reset();
        for (i = 0; i < 3 * rate; ++i) { o = step(&c, 0, 0); }
        assert(fabsf(o.hold_current) > .5f);
        /* 用更严格精度验证停靠与精确到位标志独立。 */
        c.position_error_window = POSITION_SERVO_PASSIVE_ENTER_POSITION_RAD * .25f;
        o = step(&c, c.target_position - sign * POSITION_SERVO_PASSIVE_ENTER_POSITION_RAD * .75f, 0);
        frozen = o.hold_current;
        previous = o.iq_reference;
        for (i = 0; i < 4 * rate; ++i)
        {
            o = step(&c, c.target_position - sign * POSITION_SERVO_PASSIVE_ENTER_POSITION_RAD * .75f, 0);
            assert(PositionCascade_GetTelemetry(&t));
            if (t.passive_hold)
            {
                entered = 1;
                assert(o.hold_current == 0 && o.friction_feedforward_current == 0);
                assert(fabsf(o.iq_reference - previous) <=
                    POSITION_SERVO_PASSIVE_RELEASE_A_PER_S * c.update_period_s + .00001f);
            }
            else { assert(fabsf(o.hold_current) <= fabsf(frozen) + .00001f); }
            assert(!o.target_reached);
            previous = o.iq_reference;
        }
        assert(entered && o.iq_reference == 0 && o.phase == POSITION_SERVO_PHASE_SETTLE);
        c.position_error_window = POSITION_SERVO_DAMPED_ERROR_WINDOW_RAD;
        /* 精确窗口内的±0.025°量化与PLL速度噪声，不得产生位置保持电流。 */
        c.velocity_filter_hz = 5;
        for (i = 0; i < rate; ++i)
        {
            float error_deg = .04f + ((i & 1) ? .025f : -.025f);
            o = step(&c, c.target_position - sign * error_deg * rad_per_deg,
                (i & 1) ? .012f : -.012f);
            assert(o.iq_reference == 0 && o.hold_current == 0);
            assert(PositionCascade_GetTelemetry(&t) && t.passive_hold);
        }
        assert(o.target_reached && o.phase == POSITION_SERVO_PHASE_HOLD);
        for (i = 0; i < rate; ++i)
        {
            o = step(&c, c.target_position - sign * .13f * rad_per_deg, 0);
            assert(o.iq_reference == 0 && PositionCascade_GetTelemetry(&t) && t.passive_hold);
        }
        assert(!o.target_reached); /* 停靠滞环不是精确到位窗口。 */
        c.velocity_filter_hz = 0;
        for (i = 0; i <= rate / 20; ++i)
        { o = step(&c, c.target_position - sign * 1.1f * rad_per_deg, 0); }
        assert(PositionCascade_GetTelemetry(&t) && !t.passive_hold);
        /* 回位先准备有界轨迹，不要求唤醒第一拍产生电流阶跃。 */
        for (i = 0; i < rate / 5; ++i) { o = step(&c, c.target_position - sign * 1.1f * rad_per_deg, 0); }
        assert(sign * o.iq_reference > 0);
        for (i = 0; i < rate; ++i)
        { o = step(&c, c.target_position - sign * POSITION_SERVO_PASSIVE_ENTER_POSITION_RAD * .5f, 0); }
        assert(PositionCascade_GetTelemetry(&t) && t.passive_hold);
        c.passive_hold_enabled = false;
        o = step(&c, c.target_position - sign * .09f * rad_per_deg, 0);
        assert(PositionCascade_GetTelemetry(&t) && !t.passive_hold);
        assert(sign * o.iq_reference > 0);
        c.passive_hold_enabled = true;
        for (i = 0; i < rate; ++i)
        { o = step(&c, c.target_position - sign * POSITION_SERVO_PASSIVE_ENTER_POSITION_RAD * .5f, 0); }
        c.target_position += sign * .1f;
        for (i = 0; i < 100; ++i) { o = step(&c, c.target_position - sign * .1f, 0); }
        assert(PositionCascade_GetTelemetry(&t) && !t.passive_hold);
        assert(o.phase == POSITION_SERVO_PHASE_MOVE && sign * o.iq_reference > 0);
    }
}

static void test_passive_hold_limit_and_raw_exit(void)
{
    PositionCascadeConfig_TypeDef c = config();
    PositionCascadeOutput_TypeDef o;
    PositionCascadeTelemetry_TypeDef t;
    int i;
    c.passive_hold_enabled = true;
    c.position_error_window = POSITION_SERVO_DAMPED_ERROR_WINDOW_RAD;
    c.target_position = .1f;
    c.velocity_filter_hz = 0;
    PositionCascade_Reset();
    for (i = 0; i < 6000; ++i) { o = step(&c, 0, 0); }
    for (i = 0; i < 110; ++i) { o = step(&c, .0998f, 0); }
    assert(PositionCascade_GetTelemetry(&t) && t.passive_hold && fabsf(o.iq_reference) > .1f);
    c.current_limit = .1f;
    o = step(&c, .0998f, 0);
    assert(fabsf(o.iq_reference) <= .1f && o.hold_current == 0);
    for (i = 0; i < 100; ++i) { o = step(&c, .0998f, 0); }
    assert(o.iq_reference == 0);
    c.velocity_filter_hz = 5;
    for(i=0;i<300;++i) { o=step(&c,.08f,0); } /* 原始和资格偏差均超过1度确认后有限纠偏。 */
    assert(PositionCascade_GetTelemetry(&t) && !t.passive_hold);
    c.velocity_filter_hz = 0;
    c.friction_feedforward_enabled = false; /* 阻尼档1未开前馈也必须被动停靠。 */
    for (i = 0; i < 4000; ++i) { o = step(&c, .0998f, 0); } /* 先完成回位轨迹再确认停靠。 */
    assert(PositionCascade_GetTelemetry(&t) && t.passive_hold && o.iq_reference == 0);
    o = step(&c, .0998f, POSITION_SERVO_HOLD_EXIT_SPEED_RAD_S * 1.1f);
    assert(PositionCascade_GetTelemetry(&t) && t.passive_hold && o.iq_reference == 0);
    assert(!PositionCascade_Update(&c, NAN, 0, &o));
    PositionCascade_Reset();
    assert(!PositionCascade_GetTelemetry(&t) && !t.passive_hold);
}

static void test_damped_braking_feedforward_latch(void)
{
    int sign, i;
    for (sign = -1; sign <= 1; sign += 2)
    {
        PositionCascadeConfig_TypeDef c = config();
        PositionCascadeOutput_TypeDef o;
        c.passive_hold_enabled = true;
        c.target_position = (float)sign;
        c.velocity_filter_hz = 0;
        PositionCascade_Reset();
        (void)step(&c, 0, 0);
        /* 距目标8.6°时即使速度纠偏反向，仍须补偿摩擦，避免提前停住。 */
        for (i = 0; i < 100; ++i) { o = step(&c, sign * .85f, sign * .6f); }
        assert(sign * o.speed_reference <= 0 && sign * o.friction_feedforward_current > .5f);
        /* 捕获区内才允许锁存撤力，速度噪声不能重新加力。 */
        for (i = 0; i < 200; ++i) { o = step(&c, sign * .9995f, sign * .6f); }
        assert(sign * o.speed_reference <= 0 && o.friction_feedforward_current == 0);
        for (i = 0; i < 100; ++i)
        {
            o = step(&c, sign * .9995f, sign * ((i & 1) ? .04f : .015f));
            assert(o.friction_feedforward_current == 0);
        }
        c.target_position = sign * 2.0f;
        for (i = 0; i < 200; ++i) { o = step(&c, sign * .85f, 0); }
        assert(sign * o.friction_feedforward_current > .5f);
    }
}

/* 固定输入验证新停靠区外仍保留接近力矩，单拍噪声不能提前卸流。 */
static void test_damped_approach_retains_friction_before_docking(void)
{
    int sign, i;
    for (sign = -1; sign <= 1; sign += 2)
    {
        PositionCascadeConfig_TypeDef c = config();
        PositionCascadeOutput_TypeDef o;
        PositionCascadeTelemetry_TypeDef t;
        c.passive_hold_enabled = true;
        c.position_error_window = POSITION_SERVO_DAMPED_ERROR_WINDOW_RAD;
        c.velocity_filter_hz = 5;
        c.target_position = sign * .1f;
        PositionCascade_Reset();
        for (i = 0; i < 6000; ++i) { o = step(&c, 0, 0); }
        for (i = 0; i < 1000; ++i)
        {
            float error = (i & 1) ? .010f : .011f;
            o = step(&c, c.target_position - sign * error, 0);
            assert(PositionCascade_GetTelemetry(&t) && !t.passive_hold);
            assert(sign * o.friction_feedforward_current > 1.0f);
        }
        /* 单个原始噪声样本进入停靠带，资格尚未确认，不能锁死模型前馈。 */
        o = step(&c, c.target_position - sign * .0042f, 0);
        for (i = 0; i < 200; ++i)
        {
            o = step(&c, c.target_position - sign * .011f, 0);
            assert(sign * o.friction_feedforward_current > 1.0f);
        }
        /* 0.60°处尚未进入停靠窗口，不能提前撤去接近力矩。 */
        for (i = 0; i < 1000; ++i)
        {
            o = step(&c, c.target_position - sign * .60f * .01745329252f, 0);
            assert(PositionCascade_GetTelemetry(&t) && !t.passive_hold);
            assert(o.phase != POSITION_SERVO_PHASE_HOLD);
            assert(sign * o.friction_feedforward_current > 1.0f);
        }
    }
}

/* 06：卸流尚未结束越过软边界，不能马上重新加力；不改变硬退出。 */
static void test_passive_release_completes_before_soft_wake(void)
{
    const float radians = .01745329252f;
    int sign, rate, i;
    for (sign=-1; sign<=1; sign+=2)
    for (rate=1000; rate<=2000; rate+=1000)
    {
        PositionCascadeConfig_TypeDef c=config();
        PositionCascadeOutput_TypeDef o;
        PositionCascadeTelemetry_TypeDef t;
        float before;
        c.passive_hold_enabled=true;
        c.position_error_window=POSITION_SERVO_DAMPED_ERROR_WINDOW_RAD;
        c.velocity_filter_hz=0;
        c.update_period_s=1.0f/rate;
        c.target_position=sign*.1f;
        PositionCascade_Reset();
        for(i=0;i<3*rate;i++) { o=step(&c,0,0); }
        for(i=0;i<rate/20;i++) { o=step(&c,c.target_position-sign*POSITION_SERVO_PASSIVE_ENTER_POSITION_RAD*.75f,0); }
        assert(PositionCascade_GetTelemetry(&t) && t.passive_hold);
        assert(fabsf(o.iq_reference)>.1f);
        /* 1.1°偏差持续期间，先完整卸至零，不能叠加新PI/前馈。 */
        for(i=0;i<4*rate && o.iq_reference!=0;i++)
        {
            before=o.iq_reference;
            o=step(&c,c.target_position-sign*1.1f*radians,0);
            assert(PositionCascade_GetTelemetry(&t) && t.passive_hold);
            assert(fabsf(o.iq_reference)<=fabsf(before));
            assert(fabsf(o.iq_reference-before)<=POSITION_SERVO_PASSIVE_RELEASE_A_PER_S*c.update_period_s+.00001f);
            assert(o.hold_current==0 && o.friction_feedforward_current==0);
        }
        assert(o.iq_reference==0);
        /* 越界不足50 ms不唤醒，回到边界内清空计数。 */
        for(i=0;i<rate/25;i++) { o=step(&c,c.target_position-sign*1.1f*radians,0); }
        assert(PositionCascade_GetTelemetry(&t) && t.passive_hold);
        o=step(&c,c.target_position-sign*.46f*radians,0);
        for(i=0;i<rate/25;i++) { o=step(&c,c.target_position-sign*1.1f*radians,0); }
        assert(PositionCascade_GetTelemetry(&t) && t.passive_hold);
        for(i=0;i<rate/50+1;i++) { o=step(&c,c.target_position-sign*1.1f*radians,0); }
        assert(PositionCascade_GetTelemetry(&t) && !t.passive_hold);
        assert(!o.target_reached);
    }
}

/* 确认期前馈不得先卸掉，避免由自身撤力打断50 ms资格窗口。 */
static void test_passive_qualification_preserves_approach_current(void)
{
    int sign, i;
    for(sign=-1;sign<=1;sign+=2)
    {
        PositionCascadeConfig_TypeDef c=config();
        PositionCascadeOutput_TypeDef o;
        PositionCascadeTelemetry_TypeDef t;
        float ff;
        c.passive_hold_enabled=true;
        c.position_error_window=POSITION_SERVO_DAMPED_ERROR_WINDOW_RAD;
        c.velocity_filter_hz=0;
        c.target_position=sign*.1f;
        PositionCascade_Reset();
        for(i=0;i<6000;i++) { o=step(&c,0,0); }
        ff=o.friction_feedforward_current;
        assert(sign*ff>.5f);
        for(i=0;i<80;i++)
        {
            o=step(&c,c.target_position-sign*POSITION_SERVO_PASSIVE_ENTER_POSITION_RAD*.75f,0);
            assert(PositionCascade_GetTelemetry(&t) && !t.passive_hold);
            assert(fabsf(o.friction_feedforward_current-ff)<.00001f);
        }
    }
}

/* 近区同时禁止速度Ki、静摩擦累积与饱和回算；历史Ui只能平滑减小。 */
static void test_damped_near_target_no_integral(void)
{
    int sign, rate, i;
    for(sign=-1;sign<=1;sign+=2)
    for(rate=1000;rate<=2000;rate+=1000)
    {
        PositionCascadeConfig_TypeDef c=config();
        PositionCascadeOutput_TypeDef o;
        PositionCascadeTelemetry_TypeDef t;
        float before;
        c.passive_hold_enabled=true;
        c.position_error_window=POSITION_SERVO_DAMPED_ERROR_WINDOW_RAD;
        c.target_position=sign*.1f;
        c.velocity_filter_hz=0;
        c.update_period_s=1.0f/rate;
        PositionCascade_Reset();
        for(i=0;i<3*rate;i++) { o=step(&c,0,0); }
        assert(fabsf(o.hold_current)>.5f);
        /* 近区仍在移动时冻结已有负载补偿，不新增也不提前撤力。 */
        before=o.hold_current;
        for(i=0;i<rate;i++)
        {
            float remaining_deg=.495f-.14f*i/rate;
            o=step(&c,c.target_position-sign*remaining_deg*.01745329252f,sign*.01f);
            assert(PositionCascade_GetTelemetry(&t) && !t.stiction_integrating);
            assert(fabsf(o.hold_current-before)<.00001f);
        }
        /* 持续停滞仍须卸除，不能冻结成大保持电流。 */
        c.friction_feedforward_enabled=false;
        for(i=0;i<4*rate;i++)
        {
            before=o.hold_current;
            float noise_deg=(i&1)?.025f:-.025f;
            o=step(&c,c.target_position-sign*(.4f+noise_deg)*.01745329252f,0);
            assert(PositionCascade_GetTelemetry(&t) && !t.stiction_integrating);
            assert(fabsf(o.hold_current)<=fabsf(before)+.00001f);
            if (!t.passive_hold)
            {
                assert(fabsf(o.hold_current-before)<=POSITION_SERVO_PASSIVE_RELEASE_A_PER_S*c.update_period_s+.00001f);
            }
            else { assert(o.hold_current == 0); }
        }
        assert(o.hold_current==0 && !o.target_reached);
        c.friction_feedforward_enabled=true;
        /* 0.5°附近的测量噪声不得恢复积分。 */
        for(i=0;i<rate;i++)
        {
            float error=((i&1)?.525f:.475f)*.01745329252f;
            o=step(&c,c.target_position-sign*error,0);
            assert(o.hold_current==0);
        }
        /* 比例项饱和也不能通过反算生成隐藏积分。 */
        c.position_kd=10; c.speed_kp=10;
        for(i=0;i<10;i++)
        {
            o=step(&c,c.target_position-sign*.4f*.01745329252f,sign*2);
            assert(o.hold_current==0 && fabsf(o.iq_reference)<=c.current_limit);
        }
        c.position_kd=.5f; c.speed_kp=.05f;
        for(i=0;i<rate;i++) { o=step(&c,c.target_position-sign*.8f*.01745329252f,0); }
        assert(o.hold_current == 0 && fabsf(o.iq_reference) <= 1);
    }
}

/* 惯量项只跟随轨迹加速度，双向加减速符号正确，限流及停靠归零不变。 */
static void test_inertia_feedforward(void)
{
    int sign, trial, i;
    const float gains[] = {0.0f, 1.034f, 100.0f};
    for (sign = -1; sign <= 1; sign += 2)
    for (trial = 0; trial < 3; ++trial)
    {
        PositionCascadeConfig_TypeDef c = config();
        PositionCascadeOutput_TypeDef o;
        int accelerating = 0, braking = 0, saturated = 0;
        c.friction_feedforward_enabled = false;
        c.stiction_integral_rate = 0;
        c.speed_kp = c.speed_ki = 0;
        c.current_limit = .3f;
        c.acceleration_feedforward_gain = gains[trial];
        PositionCascade_Reset();
        o = step(&c, 0, 0);
        c.target_position = sign * .2f;
        for (i = 0; i < 16000; ++i)
        {
            float expected;
            o = step(&c, 0, 0);
            expected = fmaxf(-c.current_limit, fminf(c.current_limit,
                gains[trial] * o.acceleration_reference));
            assert(fabsf(o.acceleration_feedforward_current - expected) < .00001f);
            if (sign * expected > .001f) { accelerating = 1; }
            if (sign * expected < -.001f) { braking = 1; }
            if (fabsf(expected) == c.current_limit) { saturated = 1; }
        }
        assert(o.acceleration_reference == 0 && o.acceleration_feedforward_current == 0);
        assert(trial == 0 ? (!accelerating && !braking) : (accelerating && braking));
        if (trial == 2) { assert(saturated); }
        c.passive_hold_enabled = true;
        for (i = 0; i < 2000; ++i)
        {
            o = step(&c, c.target_position, 0);
        }
        assert(o.acceleration_feedforward_current == 0 && o.iq_reference == 0);
        c.acceleration_feedforward_gain = -1;
        assert(!PositionCascade_Update(&c, c.target_position, 0, &o));
    }
}

/* 07的0.21°残差与卸流回弹：禁用旧0.26°唤醒，窗口内不得重建Ui或前馈。 */
static void test_record07_passive_recoil(void)
{
    const float radians = .01745329252f;
    int sign, rate, i;
    for (sign = -1; sign <= 1; sign += 2)
    {
        for (rate = 1000; rate <= 2000; rate += 1000)
        {
            PositionCascadeConfig_TypeDef c = config();
            PositionCascadeOutput_TypeDef o;
            PositionCascadeTelemetry_TypeDef t;
            c.passive_hold_enabled = true;
            c.update_period_s = 1.0f / rate;
            c.position_error_window = POSITION_SERVO_DAMPED_ERROR_WINDOW_RAD;
            c.velocity_filter_hz = 5;
            c.current_limit = 6;
            c.position_kp = 1; /* 先以未饱和速度误差建立历史Ui，再进入停靠。 */
            c.position_kd = 2;
            c.speed_kp = .5f;
            c.speed_ki = 1;
            c.friction_coulomb_positive = 2.946f;
            c.friction_coulomb_negative = 3.259f;
            c.target_position = sign * .1f;
            PositionCascade_Reset();
            for (i = 0; i < 3 * rate; ++i)
            {
                o = step(&c, 0, 0);
            }
            assert(fabsf(o.hold_current) > .5f);
            c.position_kp = 8;
            /* 6 A按2 A/s卸流最多3 s，另留滤波及资格确认时间。 */
            for (i = 0; i < 4 * rate; ++i)
            {
                float error_deg = .21f + ((i & 1) ? .025f : -.025f);
                o = step(&c, c.target_position - sign * error_deg * radians, 0);
            }
            assert(PositionCascade_GetTelemetry(&t) && t.passive_hold);
            assert(o.iq_reference == 0 && o.hold_current == 0);
            assert(o.friction_feedforward_current == 0 && !o.target_reached);
            /* 0.4°回弹叠加测量噪声仍在用户容差内，不应周期助推。 */
            for (i = 0; i < 5 * rate; ++i)
            {
                float error_deg = .4f + ((i & 1) ? .025f : -.025f);
                o = step(&c, c.target_position - sign * error_deg * radians,
                    (i & 1) ? .012f : -.012f);
                assert(PositionCascade_GetTelemetry(&t) && t.passive_hold);
                assert(o.iq_reference == 0 && o.hold_current == 0);
                assert(o.friction_feedforward_current == 0);
            }
            /* 越界须连续确认，只允许有限纠偏，不能重建完整摩擦前馈。 */
            for (i = 0; i < rate / 5; ++i) { o = step(&c, c.target_position - sign * 1.1f * radians, 0); }
            assert(PositionCascade_GetTelemetry(&t) && !t.passive_hold);
            assert(sign * o.iq_reference > 0);
        }
    }
}

/* 静止残差不能带入下一次反向启动；前馈建立受限，移动中重定向不跳参考。 */
static void test_damped_restart_continuity(void)
{
    int sign, rate, i;
    for (sign = -1; sign <= 1; sign += 2)
    {
        for (rate = 1000; rate <= 2000; rate += 1000)
        {
            PositionCascadeConfig_TypeDef c = config();
            PositionCascadeOutput_TypeDef o, before;
            PositionCascadeTelemetry_TypeDef t;
            float q;
            c.passive_hold_enabled = true;
            c.update_period_s = 1.0f / rate;
            c.position_error_window = POSITION_SERVO_DAMPED_ERROR_WINDOW_RAD;
            c.position_kp = 8;
            c.position_kd = 2;
            c.velocity_filter_hz = 5;
            c.friction_attack_slew_rate = POSITION_SERVO_FRICTION_ATTACK_SLEW_A_PER_S;
            c.friction_coulomb_positive = 2.946f;
            c.friction_coulomb_negative = 3.259f;
            c.friction_breakaway_ratio = 1;
            c.target_position = sign * .1f;
            PositionCascade_Reset();
            q = c.target_position - sign * .2f * .01745329252f;
            for (i = 0; i < 5 * rate; ++i) { o = step(&c, q, 0); }
            assert(PositionCascade_GetTelemetry(&t) && t.passive_hold);
            assert(o.iq_reference == 0);
            c.target_position = -sign * .1f;
            o = step(&c, q, 0);
            assert(fabsf(o.position_reference - q) < .000001f);
            for (i = 0; i < rate / 10; ++i)
            {
                o = step(&c, q, 0);
                assert(fabsf(o.friction_feedforward_current) <= POSITION_SERVO_FRICTION_ATTACK_SLEW_A_PER_S * .101f);
                assert(sign * (o.position_reference - q) <= .000001f);
            }
            for (i = 0; i < rate / 5; ++i) { o = step(&c, q, 0); }
            before = o;
            assert(PositionCascade_GetTelemetry(&t));
            assert(fabsf(t.trajectory_speed_reference) > .01f);
            c.target_position = sign * .2f;
            o = step(&c, q, 0);
            assert(fabsf(o.position_reference - before.position_reference) <= c.maximum_speed / rate + .000001f);
            assert(fabsf(o.acceleration_reference - before.acceleration_reference) <= c.jerk_limit / rate + .000001f);
        }
    }
}

/* 扰动后复用轨迹/PI/前馈，静止与低速外推都恢复，稳态再回到零电流停靠。 */
static void test_half_degree_zero_current_and_bounded_recovery(void)
{
    const float rad = 0.01745329252f;
    int rate, sign, i;
    for (rate = 1000; rate <= 2000; rate += 1000)
    for (sign = -1; sign <= 1; sign += 2)
    {
        PositionCascadeConfig_TypeDef c = config();
        PositionCascadeOutput_TypeDef o;
        PositionCascadeTelemetry_TypeDef t;
        c.passive_hold_enabled = true;
        c.position_error_window = POSITION_SERVO_DAMPED_ERROR_WINDOW_RAD;
        c.current_limit = 7;
        c.position_kp = 8; c.position_kd = 2;
        c.speed_kp = .5f; c.speed_ki = 1;
        c.update_period_s = 1.0f / rate;
        c.velocity_filter_hz = 0;
        c.target_position = 0;
        c.maximum_speed = .2f; c.acceleration = c.deceleration = .2f; c.jerk_limit = 1;
        c.friction_breakaway_ratio = 1;
        c.friction_coulomb_positive = c.friction_coulomb_negative = 3;
        c.friction_attack_slew_rate = 2;
        c.friction_fast_release_slew_rate = c.friction_release_slew_rate = 15;
        PositionCascade_Reset();
        for (i = 0; i < rate; ++i) { o = step(&c, 0, 0); }
        for (i = 0; i < rate; ++i)
        {
            o = step(&c, -sign * ((i & 1) ? 1.0f : .75f) * rad, 0);
            assert(o.iq_reference == 0 && o.hold_current == 0 && o.friction_feedforward_current == 0);
        }
        for (i = 0; i < rate / 25; ++i) { o = step(&c, -sign * 2 * rad, 0); }
        assert(o.iq_reference == 0);
        o = step(&c, -sign * .9f * rad, 0);
        for (i = 0; i < rate / 25; ++i) { o = step(&c, -sign * 2 * rad, 0); }
        assert(o.iq_reference == 0);
        /* 越界确认后参考必须从实际位置起步，不能直接跳到旧目标。 */
        for (i = 0; i < rate / 50 + 1; ++i) { o = step(&c, -sign * 2 * rad, 0); }
        assert(PositionCascade_GetTelemetry(&t) && !t.passive_hold);
        assert(fabsf(o.position_reference + sign * 2 * rad) < .0001f);
        assert(o.phase == POSITION_SERVO_PHASE_MOVE);
        for (i = 0; i < rate / 2; ++i)
        {
            float before = o.friction_feedforward_current;
            o = step(&c, -sign * 2 * rad, 0);
            assert(fabsf(o.friction_feedforward_current - before) <= 2.0f / rate + .00001f);
            assert(fabsf(o.iq_reference) <= 7 && fabsf(o.acceleration_reference) <= .20001f);
            assert(PositionCascade_GetTelemetry(&t));
            assert(fabsf(t.trajectory_speed_reference) <= .20001f);
        }
        assert(sign * o.friction_feedforward_current > .8f && sign * o.iq_reference > .8f);
        c.current_limit = .1f;
        o = step(&c, -sign * 2 * rad, 0);
        assert(fabsf(o.iq_reference) <= .1f);
        c.current_limit = 7;
        /* 实际返回后，参考减速完成并确认静止，再平滑卸总电流。 */
        for (i = 0; i < 5 * rate; ++i) { o = step(&c, -sign * .4f * rad, 0); }
        assert(PositionCascade_GetTelemetry(&t) && t.passive_hold);
        assert(o.iq_reference == 0 && o.hold_current == 0 && o.friction_feedforward_current == 0);
        for (i = 0; i < rate; ++i) { o = step(&c, -sign * .75f * rad, 0); assert(o.iq_reference == 0); }
        /* 外推仍在移动时在允许速度内保留实测速度，随后仍须用原速度约束。 */
        for (i = 0; i <= rate / 20; ++i) { o = step(&c, -sign * 2 * rad, -sign * .05f); }
        assert(PositionCascade_GetTelemetry(&t) && !t.passive_hold);
        assert(sign * t.trajectory_speed_reference < -.04f);
        c.target_position = sign * .2f;
        for (i = 0; i < rate; ++i) { o = step(&c, 0, 0); }
        assert(sign * o.friction_feedforward_current > 0);
    }
}


/* 双向、双频率验证共享上升/下降，下降大于30时也不能被旧捕获上限截断。 */
static void test_uniform_friction_slew(void)
{
    int sign, rate, i;
    for (sign = -1; sign <= 1; sign += 2)
    for (rate = 1000; rate <= 2000; rate += 1000)
    {
        PositionCascadeConfig_TypeDef c = config();
        PositionCascadeOutput_TypeDef o;
        float before;
        c.update_period_s = 1.0f / rate;
        c.target_position = sign * .1f;
        c.velocity_filter_hz = 0;
        c.friction_coulomb_positive = c.friction_coulomb_negative = 3;
        c.friction_breakaway_ratio = 1;
        c.friction_attack_slew_rate = 2;
        c.friction_fast_release_slew_rate = c.friction_release_slew_rate = 60;
        PositionCascade_Reset();
        o = step(&c, 0, 0);
        for (i = 0; i < rate; ++i)
        {
            before = o.friction_feedforward_current;
            o = step(&c, 0, 0);
            assert(fabsf(o.friction_feedforward_current - before) <= 2.0f / rate + .00001f);
        }
        assert(sign * o.friction_feedforward_current > 1.9f);
        before = o.friction_feedforward_current;
        o = step(&c, c.target_position - sign * .0001f, 0);
        assert(fabsf(o.friction_feedforward_current - before) <= 60.0f / rate + .00001f);
        /* 接近捕获窗口时第一拍已经按60而不是旧30 A/s卸流。 */
        assert(fabsf(o.friction_feedforward_current - before) >= 59.0f / rate);
        c.friction_attack_slew_rate = 0;
        assert(!PositionCascade_Update(&c,0,0,&o));
        c.friction_attack_slew_rate = 1001;
        assert(!PositionCascade_Update(&c,0,0,&o));
    }
}

int main(int argc, char **argv)
{
    if (argc == 3) return replay(argv[1], argv[2]);
    test_uniform_friction_slew(); puts("PASS configurable friction slew: signed rise/release, no hidden capture limit, both signs/rates, invalid bound");
    test_half_degree_zero_current_and_bounded_recovery(); puts("PASS passive disturbance recovery: 1 degree qualification, actual q/v seeded trajectory, bounded model FF and PI, recapture zero current, both signs/rates");
    test_damped_restart_continuity(); puts("PASS damped restart: seed stopped residual, bounded friction onset, retain moving q/v/a, both signs/rates");
    test_record07_passive_recoil(); puts("PASS record07: residual and recoil unload Ui/Iq/FF, noise stays quiet, qualified 1 degree bounded recovery, both signs/rates");
    test_inertia_feedforward(); puts("PASS inertia feedforward: signed acceleration, braking, saturation, disabled and passive zero");
    test_damped_near_target_no_integral(); puts("PASS damped near target: no new Ui, smooth decay, hysteresis, no saturation backfill, both signs and rates");
    test_passive_release_completes_before_soft_wake(); puts("PASS passive release finishes before 50 ms soft wake, both signs and rates");
    test_passive_qualification_preserves_approach_current(); puts("PASS approach current retained until passive qualification completes");
    test_damped_passive_hold(); puts("PASS damped passive hold: 0.08 precise, 0.5 passive degrees, zero Ui/FF, 2 A/s release, noise, wake and retarget at 1/2 kHz, both signs");
    test_passive_hold_limit_and_raw_exit(); puts("PASS passive release current limit, qualified bounded recovery, invalid input and reset");
    test_damped_braking_feedforward_latch(); puts("PASS damped braking stops forward assist, noise cannot restart it, new target restores assist");
    test_damped_approach_retains_friction_before_docking(); puts("PASS damped approach retains model outside 0.5 degree docking qualification, including single position noise sample");
    test_single_filter_across_phases(); puts("PASS single 10 Hz feedback across HOLD, retarget, MOVE and capture");
    test_live_filter_reconfiguration(); puts("PASS live filter frequency/period changes and disable/re-enable");
    test_live_hold_period_reconfiguration(); puts("PASS hold confirmation time after live update-period change");
    test_operating_envelopes(); puts("PASS 72 synthetic operating envelopes: current, asymmetric friction, range, both signs");
    test_invalid_config(); puts("PASS invalid inputs/configuration");
    test_low_speed_feedback(); puts("PASS continuous low-speed feedback");
    test_quiet_hold_with_recorded_noise_range(); puts("PASS quiet hold within recorded noise range");
    test_hold_velocity_noise_transfer(); puts("PASS HOLD noise transfer at both update rates");
    test_live_hold_filter_switch_preserves_state(); puts("PASS live HOLD A/B preserves target, support and state");
    test_hold_filter_keeps_stiffness_and_exit_sensing(); puts("PASS HOLD stiffness, unfiltered exit sensing and retarget");
    test_hold_retains_support_and_recovers_disturbance(); puts("PASS retained support and disturbance recovery");
    test_hold_exit_restarts_opposing_integral_transport(); puts("PASS HOLD loss restarts bounded opposing-integral transport; helpful support retained, both signs");
    test_recover_stalled_landing_before_trajectory_ends(); puts("PASS recovery before trajectory completion");
    test_deceleration_keeps_friction_until_capture(); puts("PASS deceleration friction retained until capture in both directions");
    test_braking_preserves_learned_load_until_reversal(); puts("PASS braking retains learned load while reversal unloads it");
    test_nearly_finished_reference_retains_approach(); puts("PASS no feedforward gap before reference completion");
    test_trajectory_decelerates_without_second_speed_rise(); puts("PASS 5/20/85 degree trajectories: monotonic braking, no overshoot, jerk bounds, both signs");
    test_reversal_and_saturation(); puts("PASS reversal and total current saturation");
    test_reference_governor(); puts("PASS soft governor and acceleration continuity");
    test_residual_load_can_accumulate_after_boost(); puts("PASS residual load integration after boost");
    test_divider_and_small_retarget(); puts("PASS 20kHz/2kHz divider and quiet small retarget");
    test_noise_within_position_window_can_hold(); puts("PASS precise capture: quiet HOLD within actual position window");
    test_capture_precision_and_single_static_integral(); puts("PASS precise capture, near-target friction recovery and single bounded static integral");
    test_settle_near_target_no_friction_restart(); puts("PASS near SETTLE model release latch, bounded residual integral, disturbance and new target recovery");
    test_stiction_growth_and_motion_exit(); puts("PASS static recovery: both signs, bounded ramp, motion exit, saturation, reset");
    test_stiction_disabled_and_default_governor_off(); puts("PASS default governor off and optional static recovery disabled");
    test_uncaptured_hysteresis_band_keeps_correcting(); puts("PASS capture hysteresis: correction before entry, quiet after entry");
    test_capture_survives_feedforward_release(); puts("PASS capture retained while feedforward unloads, exit still enforced");
    test_bounded_speed_correction_headroom(); puts("PASS speed correction headroom preserves trajectory and motor limits in both directions");
    test_capture_release_is_soft_until_target_crossing(); puts("PASS soft capture release and fast target-crossing release in both directions");
    test_configuration_padding_does_not_trigger_revalidation(); puts("PASS config padding ignored, actual tuning changes revalidated");
    test_retarget_preserves_reference_state(); puts("PASS mid-motion retarget preserves q/v/a and converges");
    return 0;
}
