#ifndef FIRMWARE_BSP_API_BSP_MOTOR_DRIVE_H
#define FIRMWARE_BSP_API_BSP_MOTOR_DRIVE_H

#include "bsp_system.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BSP_MOTOR_PHASE_COUNT 3U
#define BSP_MOTOR_MAX_CURRENT_SENSOR_COUNT 3U
#define BSP_MOTOR_MAX_SAMPLING_POINT_COUNT 3U

typedef enum
{
	BSP_CURRENT_SENSE_NONE = 0,
	BSP_CURRENT_SENSE_PHASE_INLINE_THREE_SENSOR,
	BSP_CURRENT_SENSE_LOW_SIDE_THREE_SHUNT,
	BSP_CURRENT_SENSE_LOW_SIDE_TWO_SHUNT,
	BSP_CURRENT_SENSE_DC_LINK_SINGLE_SHUNT,
	BSP_CURRENT_SENSE_TOPOLOGY_COUNT
} BspCurrentSenseTopology;

typedef uint32_t BspCurrentSenseTopologySet;

#define BSP_CURRENT_SENSE_TOPOLOGY_BIT(topology_) \
	(UINT32_C(1) << (uint32_t)(topology_))

typedef uint8_t BspMotorPhaseSet;

enum
{
	BSP_MOTOR_PHASE_A = UINT8_C(1) << 0,
	BSP_MOTOR_PHASE_B = UINT8_C(1) << 1,
	BSP_MOTOR_PHASE_C = UINT8_C(1) << 2,
	BSP_MOTOR_PHASE_ALL = BSP_MOTOR_PHASE_A | BSP_MOTOR_PHASE_B |
		BSP_MOTOR_PHASE_C
};

typedef uint32_t BspMotorDriveFaultSet;

enum
{
	BSP_MOTOR_DRIVE_FAULT_OVERCURRENT = UINT32_C(1) << 0,
	BSP_MOTOR_DRIVE_FAULT_POWER_STAGE = UINT32_C(1) << 1,
	BSP_MOTOR_DRIVE_FAULT_SAMPLING = UINT32_C(1) << 2,
	BSP_MOTOR_DRIVE_FAULT_TIMING = UINT32_C(1) << 3
};

typedef struct
{
	BspEndpointId endpoint_id;
	BspEndpointAvailability availability;
	BspCurrentSenseTopologySet supported_current_sense_topologies;
	uint8_t current_sensor_capacity;
	BspEndpointId
		current_sensor_endpoints[BSP_MOTOR_MAX_CURRENT_SENSOR_COUNT];
	bool supports_synchronized_sampling;
	bool supports_hardware_shutdown;
} BspMotorDriveEndpointCapabilities;

typedef struct
{
	BspCurrentSenseTopology current_sense_topology;
	uint32_t pwm_frequency_hz;
} BspMotorDriveConfiguration;

/*
 * Positions span one complete PWM period: 0 is the period start and UINT16_MAX
 * is the final timer position. The adapter translates this normalized value to
 * its timer domain without exposing timer counts to the caller.
 */
typedef enum
{
	BSP_PWM_COUNTER_SLOPE_UNSPECIFIED = 0,
	BSP_PWM_COUNTER_SLOPE_UP,
	BSP_PWM_COUNTER_SLOPE_DOWN
} BspPwmCounterSlope;

typedef enum
{
	BSP_CURRENT_SAMPLING_WINDOW_UNSPECIFIED = 0,
	BSP_CURRENT_SAMPLING_WINDOW_ZERO_VECTOR,
	BSP_CURRENT_SAMPLING_WINDOW_ACTIVE_VECTOR_FIRST,
	BSP_CURRENT_SAMPLING_WINDOW_ACTIVE_VECTOR_SECOND
} BspCurrentSamplingWindow;

typedef struct
{
	uint16_t trigger_position_u16;
	BspPwmCounterSlope counter_slope;
	BspCurrentSamplingWindow window;
	uint8_t current_sensor_mask;
	BspMotorPhaseSet valid_phase_currents;
} BspCurrentSamplingPoint;

typedef struct
{
	/* 0 means not sectorized; values 1..6 identify the modulation sector. */
	uint8_t modulation_sector;
	uint8_t sampling_point_count;
	BspMotorPhaseSet cycle_valid_phase_currents;
	uint32_t sequence;
	BspCurrentSamplingPoint points[BSP_MOTOR_MAX_SAMPLING_POINT_COUNT];
} BspCurrentSamplingPlan;

typedef struct
{
	int32_t current_sensor_raw[BSP_MOTOR_MAX_CURRENT_SENSOR_COUNT];
	float current_sensor_a[BSP_MOTOR_MAX_CURRENT_SENSOR_COUNT];
	uint8_t valid_current_sensors;
} BspCurrentSamplingResult;

/*
 * Samples remain in physical sensor/window form. Phase-current reconstruction
 * belongs to the selected measurement strategy above the BSP.
 */
typedef struct
{
	float bus_voltage_v;
	uint32_t applied_plan_sequence;
	uint8_t valid_sampling_points;
	BspCurrentSamplingResult points[BSP_MOTOR_MAX_SAMPLING_POINT_COUNT];
} BspMotorDriveSample;

typedef struct
{
	/* Each duty is normalized to [0, 1]. */
	float phase_duty[BSP_MOTOR_PHASE_COUNT];
} BspMotorDriveCommand;

typedef struct
{
	BspMotorDriveCommand pwm;
	BspCurrentSamplingPlan sampling;
} BspMotorDriveCycleCommand;

/*
 * read_sample(), commit_cycle(), disable_immediate(), and read_faults() are
 * hard-real-time calls: bounded, non-blocking, allocation-free, and silent.
 * commit_cycle() applies PWM values and the complete sampling plan atomically
 * at one PWM update boundary. initialize_safe(), arm(), and disarm() execute
 * outside the motor ISR.
 */
typedef struct
{
	void *context;
	const BspMotorDriveEndpointCapabilities *capabilities;
	BspResult (*initialize_safe)(void *context,
		const BspMotorDriveConfiguration *configuration);
	BspResult (*arm)(void *context);
	BspResult (*disarm)(void *context);
	BspResult (*read_sample)(void *context, BspMotorDriveSample *sample);
	BspResult (*commit_cycle)(void *context,
		const BspMotorDriveCycleCommand *command);
	void (*disable_immediate)(void *context);
	BspMotorDriveFaultSet (*read_faults)(void *context);
} BspMotorDrivePort;

#ifdef __cplusplus
}
#endif

#endif
