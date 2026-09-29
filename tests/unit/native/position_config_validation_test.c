/* 冻结 cb21b25d 的配置判定作为独立参考；仅主机测试编译，覆盖所有 float 位模式类别。 */
#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "position_cascade.h"
#include "position_cascade_config.h"
#include "validation_actual.inc"
static bool reference_valid(const PositionCascadeConfig *config)
{
    return config != NULL && isfinite(config->update_period_s) && config->update_period_s > 0.0f &&
           config->call_divider > 0U && isfinite(config->target_position) &&
           isfinite(config->position_error_window) && config->position_error_window > 0.0f &&
           isfinite(config->hold_enter_position) &&
           config->hold_enter_position >= config->position_error_window &&
           isfinite(config->hold_exit_position) &&
           config->hold_exit_position > config->hold_enter_position &&
           isfinite(config->velocity_filter_hz) && config->velocity_filter_hz >= 0.0f &&
           config->velocity_filter_hz * config->update_period_s <= 0.5f &&
           isfinite(config->hold_velocity_filter_hz) && config->hold_velocity_filter_hz >= 0.0f &&
           config->hold_velocity_filter_hz * config->update_period_s <= 0.5f &&
           isfinite(config->following_error_limit) && config->following_error_limit >= 0.0f &&
           isfinite(config->stiction_integral_rate) && config->stiction_integral_rate >= 0.0f &&
           isfinite(config->stiction_integral_rate * config->update_period_s) &&
           isfinite(config->acceleration) && config->acceleration > 0.0f &&
           isfinite(config->deceleration) && config->deceleration > 0.0f &&
           isfinite(config->maximum_speed) && config->maximum_speed > 0.0f &&
           isfinite(config->speed_limit_rad_s) && config->speed_limit_rad_s > 0.0f &&
           config->maximum_speed <= config->speed_limit_rad_s && isfinite(config->jerk_limit) &&
           config->jerk_limit > 0.0f && isfinite(config->position_kp) &&
           config->position_kp >= 0.0f && config->position_kp <= CASCADE_POSITION_KP_MAX_PER_S &&
           isfinite(config->position_kd) && config->position_kd >= 0.0f &&
           config->position_kd <= CASCADE_POSITION_KD_MAX && isfinite(config->speed_kp) &&
           config->speed_kp >= 0.0f && isfinite(config->speed_ki) && config->speed_ki >= 0.0f &&
           isfinite(config->acceleration_feedforward_gain) &&
           config->acceleration_feedforward_gain >= 0.0f && isfinite(config->current_limit_a) &&
           config->current_limit_a > 0.0f && isfinite(config->friction_coulomb_positive) &&
           config->friction_coulomb_positive >= 0.0f &&
           isfinite(config->friction_coulomb_negative) &&
           config->friction_coulomb_negative >= 0.0f &&
           isfinite(config->friction_viscous_positive) &&
           config->friction_viscous_positive >= 0.0f &&
           isfinite(config->friction_viscous_negative) &&
           config->friction_viscous_negative >= 0.0f &&
           isfinite(config->friction_breakaway_ratio) && config->friction_breakaway_ratio >= 1.0f &&
           isfinite(config->friction_attack_slew_rate) &&
           config->friction_attack_slew_rate > 0.0f &&
           isfinite(config->friction_fast_release_slew_rate) &&
           config->friction_fast_release_slew_rate > 0.0f &&
           isfinite(config->friction_release_slew_rate) &&
           config->friction_release_slew_rate > 0.0f &&
           isfinite(config->friction_reference_speed) && config->friction_reference_speed > 0.0f &&
           isfinite(config->friction_stop_speed) && config->friction_stop_speed >= 0.0f &&
           isfinite(config->friction_move_speed) &&
           config->friction_move_speed > config->friction_stop_speed &&
           isfinite(config->friction_breakaway_distance) &&
           config->friction_breakaway_distance > 0.0f && isfinite(config->friction_stuck_time) &&
           config->friction_stuck_time >= 0.0f;
}

static uint32_t rng_state = 0x274db991U;
static unsigned compared;
static uint32_t next_bits(void)
{
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 17;
    rng_state ^= rng_state << 5;
    return rng_state;
}
static void compare(const PositionCascadeConfig *config)
{
    bool expected = reference_valid(config);
    bool actual = position_cascade_config_is_valid(config);
    ++compared;
    if (actual != expected)
    {
        fprintf(stderr, "Validation mismatch case %u: %d / %d\n", compared, actual, expected);
        exit(1);
    }
}
static void set_bits(PositionCascadeConfig *config, size_t field, uint32_t bits)
{
    memcpy((unsigned char *)config + test_offsets[field], &bits, sizeof(bits));
}
int main(void)
{
    static const uint32_t special[] = {0,
                                       0x80000000U,
                                       1,
                                       0x80000001U,
                                       0x007fffffU,
                                       0x00800000U,
                                       0x7f7fffffU,
                                       0xff7fffffU,
                                       0x7f800000U,
                                       0xff800000U,
                                       0x7fc00000U,
                                       0xffc00000U,
                                       0x7f800001U,
                                       0xff800001U,
                                       0x3f800000U,
                                       0xbf800000U};
    static const float boundaries[] = {
        0, 0.5f, 1, 10, 50, 1000, 0.001f, 0.003141593f, 0.004537856f, 0.02f, 0.05f, 0.785398f};
    PositionCascadeConfig base = valid_config();
    PositionCascadeConfig c;
    size_t field, index;
    unsigned trial;
    compare(NULL);
    compare(&base);
    if (!reference_valid(&base))
    {
        return 1;
    }
    for (field = 0; field < sizeof(test_offsets) / sizeof(test_offsets[0]); ++field)
    {
        for (index = 0; index < sizeof(special) / sizeof(special[0]); ++index)
        {
            c = base;
            set_bits(&c, field, special[index]);
            compare(&c);
        }
        for (index = 0; index < sizeof(boundaries) / sizeof(boundaries[0]); ++index)
        {
            float value = boundaries[index];
            uint32_t bits;
            memcpy(&bits, &value, sizeof(bits));
            for (int delta = -1; delta <= 1; ++delta)
            {
                c = base;
                set_bits(&c, field, bits + (uint32_t)delta);
                compare(&c);
            }
        }
        for (trial = 0; trial < 65536; ++trial)
        {
            c = base;
            set_bits(&c, field, next_bits());
            compare(&c);
        }
    }
    /* 多字段同时污染可暴露短路顺序变化后的漏检；整数分频与禁用摩擦同样保留校验。 */
    for (trial = 0; trial < 300000; ++trial)
    {
        c = base;
        for (index = 0; index < 4; ++index)
        {
            field = next_bits() % (sizeof(test_offsets) / sizeof(test_offsets[0]));
            set_bits(&c, field, next_bits());
        }
        c.call_divider = (uint16_t)next_bits();
        c.friction_feedforward_enabled = (next_bits() & 1U) != 0;
        compare(&c);
    }
    c = base;
    c.call_divider = 0;
    compare(&c);
    c = base;
    c.update_period_s = FLT_MAX;
    c.velocity_filter_hz = c.hold_velocity_filter_hz = 0;
    c.stiction_integral_rate = 2;
    compare(&c);
    c.stiction_integral_rate = 0;
    compare(&c);
    printf("PASS %u configuration decisions match frozen reference\n", compared);
    return 0;
}
