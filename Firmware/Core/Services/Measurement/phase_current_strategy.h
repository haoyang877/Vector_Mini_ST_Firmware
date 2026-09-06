#ifndef FIRMWARE_CORE_SERVICES_MEASUREMENT_PHASE_CURRENT_STRATEGY_H
#define FIRMWARE_CORE_SERVICES_MEASUREMENT_PHASE_CURRENT_STRATEGY_H

#include <stdbool.h>
#include <stdint.h>

#define PHASE_CURRENT_PHASE_COUNT             3U
#define PHASE_CURRENT_DC_LINK_SECTOR_COUNT    6U
#define PHASE_CURRENT_DC_LINK_WINDOW_COUNT    2U

typedef enum
{
	PHASE_CURRENT_TOPOLOGY_INVALID = 0,
	PHASE_CURRENT_TOPOLOGY_INLINE_3_SHUNT,
	PHASE_CURRENT_TOPOLOGY_LOW_SIDE_3_SHUNT,
	PHASE_CURRENT_TOPOLOGY_LOW_SIDE_2_SHUNT,
	PHASE_CURRENT_TOPOLOGY_DC_LINK_1_SHUNT
} PhaseCurrentTopology;

typedef enum
{
	PHASE_CURRENT_PHASE_A = 0,
	PHASE_CURRENT_PHASE_B,
	PHASE_CURRENT_PHASE_C
} PhaseCurrentPhase;

typedef enum
{
	PHASE_CURRENT_VALID_NONE = 0U,
	PHASE_CURRENT_VALID_A = 1U << PHASE_CURRENT_PHASE_A,
	PHASE_CURRENT_VALID_B = 1U << PHASE_CURRENT_PHASE_B,
	PHASE_CURRENT_VALID_C = 1U << PHASE_CURRENT_PHASE_C,
	PHASE_CURRENT_VALID_ALL = PHASE_CURRENT_VALID_A |
		PHASE_CURRENT_VALID_B | PHASE_CURRENT_VALID_C
} PhaseCurrentValidMask;

typedef enum
{
	PHASE_CURRENT_QUALITY_INVALID = 0,
	PHASE_CURRENT_QUALITY_DIRECT,
	PHASE_CURRENT_QUALITY_KCL_RECONSTRUCTED,
	PHASE_CURRENT_QUALITY_DC_LINK_RECONSTRUCTED,
	PHASE_CURRENT_QUALITY_SATURATED,
	PHASE_CURRENT_QUALITY_STALE
} PhaseCurrentQuality;

typedef enum
{
	PHASE_CURRENT_STATUS_OK = 0,
	PHASE_CURRENT_STATUS_NULL_ARGUMENT,
	PHASE_CURRENT_STATUS_NOT_CONFIGURED,
	PHASE_CURRENT_STATUS_UNSUPPORTED_TOPOLOGY,
	PHASE_CURRENT_STATUS_INVALID_SAMPLE_COUNT,
	PHASE_CURRENT_STATUS_INVALID_PHASE_MASK,
	PHASE_CURRENT_STATUS_NONFINITE_SAMPLE,
	PHASE_CURRENT_STATUS_SATURATED,
	PHASE_CURRENT_STATUS_INVALID_SECTOR,
	PHASE_CURRENT_STATUS_INVALID_WINDOW,
	PHASE_CURRENT_STATUS_INVALID_WINDOW_MAPPING,
	PHASE_CURRENT_STATUS_DUPLICATE_SEQUENCE,
	PHASE_CURRENT_STATUS_OUT_OF_ORDER_SEQUENCE
} PhaseCurrentStatus;

typedef enum
{
	PHASE_CURRENT_DC_LINK_WINDOW_INVALID = 0,
	PHASE_CURRENT_DC_LINK_WINDOW_FIRST_ACTIVE_VECTOR,
	PHASE_CURRENT_DC_LINK_WINDOW_SECOND_ACTIVE_VECTOR
} PhaseCurrentDcLinkWindowId;

/*
 * A DC-link sample is a signed observation of one phase.  The sign is part of
 * the PWM-vector mapping, not a property of the current-sense amplifier.
 */
typedef enum
{
	PHASE_CURRENT_DC_LINK_MAP_INVALID = 0,
	PHASE_CURRENT_DC_LINK_MAP_POSITIVE_A,
	PHASE_CURRENT_DC_LINK_MAP_NEGATIVE_A,
	PHASE_CURRENT_DC_LINK_MAP_POSITIVE_B,
	PHASE_CURRENT_DC_LINK_MAP_NEGATIVE_B,
	PHASE_CURRENT_DC_LINK_MAP_POSITIVE_C,
	PHASE_CURRENT_DC_LINK_MAP_NEGATIVE_C
} PhaseCurrentDcLinkSampleMapping;

typedef struct
{
	PhaseCurrentDcLinkSampleMapping first_window;
	PhaseCurrentDcLinkSampleMapping second_window;
} PhaseCurrentDcLinkSectorMapping;

typedef struct
{
	PhaseCurrentTopology topology;
	/* Indexed by sector - 1. Ignored by all topologies except 1-shunt. */
	PhaseCurrentDcLinkSectorMapping
		dc_link_sector_map[PHASE_CURRENT_DC_LINK_SECTOR_COUNT];
} PhaseCurrentStrategyConfig;

/*
 * Direct phase observations are calibrated to amperes by the MCU-independent
 * measurement service before reaching this strategy. The BSP deliberately
 * publishes raw acquisition-order observations only. valid_phase_mask
 * identifies the observed phases: three-shunt strategies require all three;
 * the two-shunt strategy requires exactly two.
 */
typedef struct
{
	float phase_current_a[PHASE_CURRENT_PHASE_COUNT];
	uint8_t valid_phase_mask;
	uint8_t saturated_phase_mask;
} PhaseCurrentDirectFrame;

typedef struct
{
	float current_a;
	PhaseCurrentDcLinkWindowId id;
	bool valid;
	bool saturated;
} PhaseCurrentDcLinkWindow;

typedef struct
{
	uint8_t sector;
	PhaseCurrentDcLinkWindow windows[PHASE_CURRENT_DC_LINK_WINDOW_COUNT];
} PhaseCurrentDcLinkFrame;

typedef struct
{
	uint32_t sequence;
	union
	{
		PhaseCurrentDirectFrame direct;
		PhaseCurrentDcLinkFrame dc_link;
	} data;
} PhaseCurrentAcquisition;

typedef struct
{
	float ia_a;
	float ib_a;
	float ic_a;
	uint8_t valid_phase_mask;
	PhaseCurrentQuality quality;
	uint32_t sequence;
} PhaseCurrentSample;

typedef struct
{
	PhaseCurrentStrategyConfig config;
	uint32_t last_sequence;
	bool has_last_sequence;
	bool is_configured;
} PhaseCurrentStrategy;

/*
 * Fills a complete, MCU-independent strategy configuration.  The default
 * 1-shunt mapping assumes conventional SVPWM active-vector ordering; a BSP
 * may replace the six mapping entries before Configure() when it uses another
 * ordering convention.
 */
PhaseCurrentStatus PhaseCurrentStrategy_MakeDefaultConfig(
	PhaseCurrentTopology topology, PhaseCurrentStrategyConfig *config);

void PhaseCurrentStrategy_Reset(PhaseCurrentStrategy *strategy);

PhaseCurrentStatus PhaseCurrentStrategy_Configure(
	PhaseCurrentStrategy *strategy, const PhaseCurrentStrategyConfig *config);

PhaseCurrentStatus PhaseCurrentStrategy_GetDcLinkMapping(
	const PhaseCurrentStrategy *strategy, uint8_t sector,
	PhaseCurrentDcLinkWindowId window,
	PhaseCurrentDcLinkSampleMapping *mapping);

/*
 * sequence uses modulo-2^32 ordering: equal is duplicate, a forward distance
 * below 2^31 is newer, and wrap from UINT32_MAX to zero is valid.  An observed
 * sequence is consumed before payload validation so a bad frame cannot later
 * be replayed as fresh data.
 */
PhaseCurrentStatus PhaseCurrentStrategy_Process(
	PhaseCurrentStrategy *strategy, const PhaseCurrentAcquisition *acquisition,
	PhaseCurrentSample *sample);

#endif
