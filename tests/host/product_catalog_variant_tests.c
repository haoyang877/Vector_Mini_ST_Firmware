#include "product_catalog.h"

#define TEST_CHECK(condition) do { if (!(condition)) return __LINE__; } while (0)

int ProductCatalogVariant_RunHostTests(void)
{
	const ProductCatalogEntry *current = ProductCatalog_GetCurrent();
	const ProductCatalogEntry *no_damper = ProductCatalog_GetByVariant(
		PRODUCT_CATALOG_VARIANT_NO_DAMPER);
	const ProductCatalogEntry *damped = ProductCatalog_GetByVariant(
		PRODUCT_CATALOG_VARIANT_DAMPED);

	TEST_CHECK(no_damper != 0 && no_damper->config != 0);
	TEST_CHECK(damped != 0 && damped->config != 0);
	TEST_CHECK(current != 0 && current->config != 0);
#if PRODUCT_CATALOG_ACTIVE_VARIANT == PRODUCT_CATALOG_VARIANT_NO_DAMPER
	TEST_CHECK(current == no_damper);
	TEST_CHECK(current->config->identity.variant_id ==
		PRODUCT_CATALOG_VARIANT_NO_DAMPER);
	TEST_CHECK(current->config->identity.configuration_fingerprint ==
		PRODUCT_CATALOG_FINGERPRINT_NO_DAMPER);
#else
	TEST_CHECK(current == damped);
	TEST_CHECK(current->config->identity.variant_id ==
		PRODUCT_CATALOG_VARIANT_DAMPED);
	TEST_CHECK(current->config->identity.configuration_fingerprint ==
		PRODUCT_CATALOG_FINGERPRINT_DAMPED);
#endif
	TEST_CHECK(no_damper->config->identity.variant_id !=
		damped->config->identity.variant_id);
	TEST_CHECK(no_damper->config->load->design_id !=
		damped->config->load->design_id);
	TEST_CHECK(no_damper->config->identity.configuration_fingerprint !=
		damped->config->identity.configuration_fingerprint);
	TEST_CHECK(!no_damper->persistence.allow_erased_fingerprint_migration);
	TEST_CHECK(damped->persistence.allow_erased_fingerprint_migration);
	TEST_CHECK(damped->config->identity.product_id == 0x564D5354UL);
	TEST_CHECK(damped->config->identity.variant_id == 0x00010001UL);
	TEST_CHECK(damped->config->identity.configuration_fingerprint ==
		0x9C501111UL);
	TEST_CHECK(damped->manifest.parameter_schema_version == 10U);
	TEST_CHECK(damped->manifest.hardware_profile_id == 1U);
	TEST_CHECK(damped->manifest.motor_profile_id == 1U);
	TEST_CHECK(damped->manifest.encoder_profile_id == 1U);
	TEST_CHECK(damped->manifest.mechanical_load_profile_id == 1U);

	TEST_CHECK(!no_damper->config->control.position_friction.enabled);
	TEST_CHECK(no_damper->config->control.default_speed_limit_rad_s > 38.9f);
	TEST_CHECK(no_damper->config->control.default_position_max_speed_rad_s >
		0.78f);
	TEST_CHECK(damped->config->control.position_friction.enabled);
	TEST_CHECK(damped->config->control.default_speed_limit_rad_s > 3.14f);
	TEST_CHECK(damped->config->control.default_speed_limit_rad_s < 3.15f);
	TEST_CHECK(damped->config->commissioning_tuning.angle.startup.
		minimum_current_limit_a == 5.50f);
	TEST_CHECK(damped->config->commissioning_tuning.angle.startup.startup_iq_a ==
		4.50f);
	TEST_CHECK(damped->config->commissioning_tuning.angle.
		electrical_zero_min_align_current_a == 4.50f);
	TEST_CHECK(damped->config->control.position_friction.
		friction_positive_current_a == 1.55f);
	TEST_CHECK(damped->config->commissioning_tuning.friction.speed_point_count ==
		PRODUCT_FRICTION_IDENTIFICATION_SPEED_POINT_COUNT);
	TEST_CHECK(damped->config->commissioning_tuning.friction.
		speed_points_rad_s[3] > 3.14f);
	TEST_CHECK(damped->config->commissioning_tuning.friction.current_ratio_max ==
		0.90f);
	TEST_CHECK(no_damper->config->commissioning_tuning.cogging.turns >= 1U);
	TEST_CHECK(no_damper->config->commissioning_tuning.cogging.stage_timeout_s ==
		30.0f);
	TEST_CHECK(no_damper->config->commissioning_tuning.cogging.
		speed_tolerance_ratio == 0.10f);
	TEST_CHECK(damped->config->commissioning_tuning.cogging.stage_timeout_s ==
		45.0f);
	TEST_CHECK(damped->config->commissioning_tuning.cogging.
		speed_tolerance_ratio == 0.25f);
	TEST_CHECK(ProductCatalog_GetByVariant(2U) == 0);
	return 0;
}
