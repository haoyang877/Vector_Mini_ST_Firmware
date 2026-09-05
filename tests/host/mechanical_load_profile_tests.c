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
	TEST_CHECK(no_damper->friction_positive_current_a == 0.0f);
	TEST_CHECK(damping_ring->damping_ring_present);
	TEST_CHECK(damping_ring->position_friction_feedforward_enabled);
	TEST_CHECK(damping_ring->encoder_calibration_startup.minimum_current_limit_a ==
		5.50f);
	TEST_CHECK(damping_ring->encoder_calibration_startup.startup_iq_a == 4.50f);
	TEST_CHECK(damping_ring->encoder_electrical_zero_min_align_current_a == 4.50f);
	TEST_CHECK(damping_ring->friction_positive_current_a == 1.55f);
	TEST_CHECK(MechanicalLoadProfile_GetById(2U) == 0);
	return 0;
}
