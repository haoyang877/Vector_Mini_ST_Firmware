#include "mechanical_load_profiles.h"

#define TEST_CHECK(condition) do { if (!(condition)) return __LINE__; } while (0)

int MechanicalLoadProfile_RunHostTests(void)
{
	const MechanicalLoadProfile *no_damper =
		MechanicalLoadProfile_GetById(MECHANICAL_LOAD_PROFILE_NO_DAMPER);
	const MechanicalLoadProfile *damping_ring =
		MechanicalLoadProfile_GetById(MECHANICAL_LOAD_PROFILE_DAMPING_RING_1P5NM);

	TEST_CHECK(no_damper != 0);
	TEST_CHECK(damping_ring != 0);
	TEST_CHECK(!no_damper->damping_ring_present);
	TEST_CHECK(!no_damper->position_friction_feedforward_enabled);
	TEST_CHECK(no_damper->default_speed_limit_rps == 372.0f / 60.0f);
	TEST_CHECK(no_damper->default_position_max_speed_rps == 0.125f);
	TEST_CHECK(no_damper->friction_positive_current_a == 0.0f);
	TEST_CHECK(damping_ring->damping_ring_present);
	TEST_CHECK(damping_ring->position_friction_feedforward_enabled);
	TEST_CHECK(damping_ring->default_speed_limit_rps == 0.50f);
	TEST_CHECK(damping_ring->default_position_max_speed_rps == 0.50f);
	TEST_CHECK(damping_ring->encoder_calibration_startup.minimum_current_limit_a ==
		5.50f);
	TEST_CHECK(damping_ring->encoder_calibration_startup.startup_iq_a == 4.50f);
	TEST_CHECK(damping_ring->encoder_electrical_zero_min_align_current_a == 4.50f);
	TEST_CHECK(damping_ring->friction_positive_current_a == 1.55f);
	TEST_CHECK(no_damper->friction_identification_speed_point_count == 4U);
	TEST_CHECK(damping_ring->friction_identification_speed_point_count == 4U);
	TEST_CHECK(damping_ring->friction_identification_speed_points_rad_s[0] > 0.62f);
	TEST_CHECK(damping_ring->friction_identification_speed_points_rad_s[3] > 3.14f);
	TEST_CHECK(damping_ring->friction_identification_speed_points_rad_s[3] < 3.15f);
	TEST_CHECK(damping_ring->friction_identification_current_ratio_max == 0.90f);
	TEST_CHECK(no_damper->cogging_identification_turns >= 1U);
	TEST_CHECK(no_damper->cogging_identification_stage_timeout_s == 30.0f);
	TEST_CHECK(no_damper->cogging_identification_speed_tolerance_ratio == 0.10f);
	TEST_CHECK(damping_ring->cogging_identification_min_samples_per_bin >= 1U);
	TEST_CHECK(damping_ring->cogging_identification_max_current_a <= 1.0f);
	TEST_CHECK(damping_ring->cogging_identification_stage_timeout_s == 45.0f);
	TEST_CHECK(damping_ring->cogging_identification_speed_tolerance_ratio == 0.25f);
	TEST_CHECK(MechanicalLoadProfile_GetById(2U) == 0);
	return 0;
}
