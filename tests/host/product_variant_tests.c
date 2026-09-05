#include "product_variant.h"

#define TEST_CHECK(condition) do { if (!(condition)) return __LINE__; } while (0)

int ProductVariant_RunHostTests(void)
{
	ProductVariant variant;
	BoardProfile board;
	MotorProfile motor;
	const BoardProfile *active_board;
	const MotorProfile *active_motor;

	TEST_CHECK(ProductVariant_GetActive(&variant));
	active_board = variant.board;
	active_motor = variant.motor;
	board = *active_board;
	motor = *active_motor;
	variant.board = &board;
	variant.motor = &motor;
	TEST_CHECK(ProductVariant_Validate(&variant));
	board.current_command_limit_a = board.current_sense_reliable_limit_a + 1.0f;
	TEST_CHECK(!ProductVariant_Validate(&variant));
	board = *active_board;
	board.temperature_protection_enabled = true;
	board.temperature_sensor = TEMPERATURE_SENSOR_NONE;
	TEST_CHECK(!ProductVariant_Validate(&variant));
	board = *active_board;
	motor.position_max_speed_rps = motor.position_speed_limit_rps + 1.0f;
	TEST_CHECK(!ProductVariant_Validate(&variant));
	return 0;
}
