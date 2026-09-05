#include "friction_identification.h"

#define TEST_CHECK(condition) do { if (!(condition)) return __LINE__; } while (0)

static float TestAbs(float value) { return value >= 0.0f ? value : -value; }

int FrictionIdentification_RunHostTests(void)
{
	FrictionIdentificationContext context;
	FrictionIdentificationConfig config = {0};
	FrictionIdentificationInput input = {0};
	const FrictionIdentificationResult *result;
	float position = 0.0f;
	uint32_t point;

	config.update_period_s = 0.01f;
	config.speed_points_rad_s[0] = 1.0f;
	config.speed_points_rad_s[1] = 2.0f;
	config.speed_points_rad_s[2] = 3.0f;
	config.speed_points_rad_s[3] = 4.0f;
	config.speed_point_count = 4U;
	config.stable_time_s = 0.02f;
	config.track_timeout_s = 0.10f;
	config.sample_timeout_s = 0.10f;
	config.stop_hold_time_s = 0.02f;
	config.stop_timeout_s = 0.10f;
	config.speed_tolerance_ratio = 0.05f;
	config.minimum_speed_tolerance_rad_s = 0.01f;
	config.stop_speed_rad_s = 0.01f;
	config.sample_turns = 0.001f;
	config.minimum_sample_time_s = 0.01f;
	config.saturation_time_s = 0.05f;
	config.rmse_floor_a = 0.001f;
	config.rmse_ratio_max = 0.01f;

	TEST_CHECK(FrictionIdentification_Init(&context, &config));
	TEST_CHECK(FrictionIdentification_Start(&context));
	for (point = 0U; point < 8U; point++)
	{
		float target = FrictionIdentification_GetTargetSpeed(&context);
		float magnitude = TestAbs(target);
		input.measured_speed_rad_s = target;
		input.ramped_speed_reference_rad_s = target;
		input.iq_a = target > 0.0f ? 0.12f + 0.03f * magnitude :
			-(0.15f + 0.02f * magnitude);
		input.mechanical_position_rad = position;
		FrictionIdentification_Update(&context, &input);
		FrictionIdentification_Update(&context, &input);
		TEST_CHECK(FrictionIdentification_GetState(&context) ==
			FRICTION_IDENT_SAMPLING);
		position += target > 0.0f ? 0.01f : -0.01f;
		input.mechanical_position_rad = position;
		FrictionIdentification_Update(&context, &input);
		TEST_CHECK(FrictionIdentification_GetState(&context) ==
			FRICTION_IDENT_STOPPING);
		input.measured_speed_rad_s = 0.0f;
		input.ramped_speed_reference_rad_s = 0.0f;
		input.iq_a = 0.0f;
		FrictionIdentification_Update(&context, &input);
		FrictionIdentification_Update(&context, &input);
	}
	TEST_CHECK(FrictionIdentification_GetState(&context) == FRICTION_IDENT_COMPLETE);
	result = FrictionIdentification_GetResult(&context);
	TEST_CHECK(result != 0 && result->valid);
	TEST_CHECK(TestAbs(result->coulomb_pos_a - 0.12f) < 0.0001f);
	TEST_CHECK(TestAbs(result->coulomb_neg_a - 0.15f) < 0.0001f);
	TEST_CHECK(TestAbs(result->viscous_pos_a_per_rad_s - 0.03f) < 0.0001f);
	TEST_CHECK(TestAbs(result->viscous_neg_a_per_rad_s - 0.02f) < 0.0001f);
	return 0;
}
