#include <stdint.h>

#include "angle_serial_stm32g431.h"

#define TEST_CHECK(condition_) \
	do { if (!(condition_)) return __LINE__; } while (0)

static AngleSerialStm32g431ResourceConfig ValidConfig(void *spi_handle,
	void *select_gpio, void *data_gpio)
{
	AngleSerialStm32g431ResourceConfig resources;

	resources.spi_handle = spi_handle;
	resources.select_gpio_port = select_gpio;
	resources.data_gpio_port = data_gpio;
	resources.transfer_spin_limit = 340U;
	resources.select_pin_mask = UINT16_C(1) << 12U;
	resources.data_pin_index = 15U;
	resources.data_alternate_function = 5U;
	resources.maximum_step_count = 2U;
	resources.select_active_low = true;
	return resources;
}

static int Test_ValidResourceConfigurations(void)
{
	uint32_t spi_storage;
	uint32_t gpio_b_storage;
	uint32_t gpio_c_storage;
	AngleSerialStm32g431ResourceConfig resources =
		ValidConfig(&spi_storage, &gpio_b_storage, &gpio_b_storage);

	TEST_CHECK(AngleSerialStm32g431_ResourceConfigIsValid(&resources));
	resources.select_active_low = false;
	TEST_CHECK(AngleSerialStm32g431_ResourceConfigIsValid(&resources));
	resources.select_gpio_port = &gpio_c_storage;
	resources.select_pin_mask = UINT16_C(1) << 11U;
	TEST_CHECK(AngleSerialStm32g431_ResourceConfigIsValid(&resources));
	return 0;
}

static int Test_RequiredResourcesAreRejectedWhenMissing(void)
{
	uint32_t spi_storage;
	uint32_t gpio_storage;
	AngleSerialStm32g431ResourceConfig resources =
		ValidConfig(&spi_storage, &gpio_storage, &gpio_storage);

	TEST_CHECK(!AngleSerialStm32g431_ResourceConfigIsValid(0));
	resources.spi_handle = 0;
	TEST_CHECK(!AngleSerialStm32g431_ResourceConfigIsValid(&resources));
	resources = ValidConfig(&spi_storage, &gpio_storage, &gpio_storage);
	resources.select_gpio_port = 0;
	TEST_CHECK(!AngleSerialStm32g431_ResourceConfigIsValid(&resources));
	resources = ValidConfig(&spi_storage, &gpio_storage, &gpio_storage);
	resources.data_gpio_port = 0;
	TEST_CHECK(!AngleSerialStm32g431_ResourceConfigIsValid(&resources));
	return 0;
}

static int Test_PinAndAlternateFunctionRanges(void)
{
	uint32_t spi_storage;
	uint32_t gpio_storage;
	AngleSerialStm32g431ResourceConfig resources =
		ValidConfig(&spi_storage, &gpio_storage, &gpio_storage);

	resources.select_pin_mask = 0U;
	TEST_CHECK(!AngleSerialStm32g431_ResourceConfigIsValid(&resources));
	resources.select_pin_mask = (UINT16_C(1) << 11U) |
		(UINT16_C(1) << 12U);
	TEST_CHECK(!AngleSerialStm32g431_ResourceConfigIsValid(&resources));
	resources = ValidConfig(&spi_storage, &gpio_storage, &gpio_storage);
	resources.data_pin_index = ANGLE_SERIAL_STM32G431_GPIO_PIN_COUNT;
	TEST_CHECK(!AngleSerialStm32g431_ResourceConfigIsValid(&resources));
	resources = ValidConfig(&spi_storage, &gpio_storage, &gpio_storage);
	resources.data_alternate_function = ANGLE_SERIAL_STM32G431_GPIO_AF_COUNT;
	TEST_CHECK(!AngleSerialStm32g431_ResourceConfigIsValid(&resources));
	resources = ValidConfig(&spi_storage, &gpio_storage, &gpio_storage);
	resources.select_pin_mask = UINT16_C(1) << resources.data_pin_index;
	TEST_CHECK(!AngleSerialStm32g431_ResourceConfigIsValid(&resources));
	return 0;
}

static int Test_ExecutionBudgetMustBeFiniteAndBounded(void)
{
	uint32_t spi_storage;
	uint32_t gpio_storage;
	AngleSerialStm32g431ResourceConfig resources =
		ValidConfig(&spi_storage, &gpio_storage, &gpio_storage);

	resources.transfer_spin_limit = 0U;
	TEST_CHECK(!AngleSerialStm32g431_ResourceConfigIsValid(&resources));
	resources = ValidConfig(&spi_storage, &gpio_storage, &gpio_storage);
	resources.maximum_step_count = 0U;
	TEST_CHECK(!AngleSerialStm32g431_ResourceConfigIsValid(&resources));
	resources.maximum_step_count =
		(uint8_t)(ANGLE_SERIAL_STM32G431_MAX_STEP_COUNT + 1U);
	TEST_CHECK(!AngleSerialStm32g431_ResourceConfigIsValid(&resources));
	return 0;
}

int AngleSerialStm32g431Config_RunHostTests(void)
{
	int result;

	result = Test_ValidResourceConfigurations();
	if (result != 0)
		return result;
	result = Test_RequiredResourcesAreRejectedWhenMissing();
	if (result != 0)
		return result;
	result = Test_PinAndAlternateFunctionRanges();
	if (result != 0)
		return result;
	return Test_ExecutionBudgetMustBeFiniteAndBounded();
}
