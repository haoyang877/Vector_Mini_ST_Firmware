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
    /* Recorded final errors: approximately 0.03-0.09 deg, no macro movement. */
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
        o = step(&c, c.target_position + 0.0028f, 0);
        assert(o.target_reached);
        assert(fabsf(o.hold_current - support) < 0.00001f);
    }
    c.friction_feedforward_enabled = 1;
    o = step(&c, c.target_position + 0.01f, 0);
    assert(!o.target_reached && o.phase == POSITION_SERVO_PHASE_SETTLE);
    assert(fabsf(o.hold_current - support) < 0.001f);
    for (i = 0; i < 500; ++i) o = step(&c, c.target_position + 0.01f, 0);
    assert(o.friction_feedforward_current < -0.5f);
    for (i = 0; i < 500; ++i) o = step(&c, c.target_position + 0.001f, 0);
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
        for (i = 0; i < 100; ++i) o = step(&c, sign * .998f, sign * .04f);
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
    c.target_position = 0.002f;
    for (i = 0; i < 20000; ++i)
    {
        o = step(&c, 0, 0);
        assert(o.friction_feedforward_current == 0.0f);
    }
    assert(o.target_reached);
    PositionCascade_Reset();
    assert(!PositionCascade_GetTelemetry(&t));
}

static void test_pid4_noise_plateau_can_hold(void)
{
    PositionCascadeConfig_TypeDef c = config();
    PositionCascadeOutput_TypeDef o;
    PositionCascadeTelemetry_TypeDef t;
    int i;
    /* Capture _4 stopped around 0.18 deg, with tails reaching 0.203 deg.
     * Start outside the window to exercise landing/FF withdrawal as well. */
    PositionCascade_Reset();
    for (i = 0; i < 4000; ++i) step(&c, -0.011f, 0);
    for (i = 0; i < 8000; ++i)
    {
        float p = -(0.18f + 0.025f * sinf((float)i * 0.17f)) *
            0.017453293f;
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
    /* Enter once while torque is still unloading. Subsequent encoder samples
     * stay in the hysteresis band: the capture must not be forgotten. */
    o=step(&c,-0.17f*0.017453293f,0);
    assert(!o.target_reached && fabsf(o.friction_feedforward_current)>0);
    for(i=0; i<500; ++i) o=step(&c,-0.20f*0.017453293f,0);
    assert(o.target_reached && o.friction_feedforward_current==0);
    o=step(&c,-0.27f*0.017453293f,0);
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
    for (i=0; i<2000; ++i) o=step(&c,-0.003141593f,0); /* 0.18 deg */
    assert(o.target_reached && o.friction_feedforward_current == 0);
    initial=o.hold_current;
    for (i=0; i<2000; ++i) {
        o=step(&c,-0.004276057f,0); /* Same band after capture: remain quiet. */
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
    for (i = offsetof(PositionCascadeConfig_TypeDef, friction_feedforward_enabled) + sizeof(c.friction_feedforward_enabled);
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
        o = step(&c, sign * .098f, 0);
        assert(fabsf(before - sign * o.friction_feedforward_current -
            POSITION_SERVO_FRICTION_CAPTURE_RELEASE_SLEW_A_PER_S *
            c.update_period_s) < .00001f);
        before = sign * o.friction_feedforward_current;
        o = step(&c, sign * .101f, 0); /* Cross target within capture hysteresis. */
        assert(fabsf(before - sign * o.friction_feedforward_current -
            c.friction_fast_release_slew_rate * c.update_period_s) < .00001f);
        for (i = 0; i < 2000; ++i) o = step(&c, sign * .101f, 0);
        assert(o.target_reached && o.friction_feedforward_current == 0);

        /* Actual position may enter first, before the reference can latch
         * capture. The approach taper must obey the same soft release. */
        c.target_position = sign * 1.0f;
        PositionCascade_Reset();
        for (i = 0; i < 100; ++i) o = step(&c, 0, sign * .1f);
        assert(fabsf(o.position_reference-c.target_position) > .01f);
        before = sign * o.friction_feedforward_current;
        assert(before > 1.0f);
        o = step(&c, sign * .998f, sign * .1f);
        assert(fabsf(before - sign * o.friction_feedforward_current -
            POSITION_SERVO_FRICTION_CAPTURE_RELEASE_SLEW_A_PER_S *
            c.update_period_s) < .00001f);
        before = sign * o.friction_feedforward_current;
        o = step(&c, sign * 1.001f, sign * .1f);
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

int main(int argc, char **argv)
{
    if (argc == 3) return replay(argv[1], argv[2]);
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
    test_pid4_noise_plateau_can_hold(); puts("PASS capture-4 plateau: quiet HOLD and immediate outside-tolerance exit");
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
