#ifndef PORTS_MOTOR_COMMAND_PORT_H
#define PORTS_MOTOR_COMMAND_PORT_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
	MOTOR_PORT_MODE_NONE = 0,
	MOTOR_PORT_MODE_CURRENT,
	MOTOR_PORT_MODE_SPEED,
	MOTOR_PORT_MODE_SENSORLESS_SPEED,
	MOTOR_PORT_MODE_POSITION,
	MOTOR_PORT_MODE_POSITION_IMPEDANCE,
	MOTOR_PORT_MODE_VOLTAGE_OPEN_LOOP,
	MOTOR_PORT_MODE_VQ
} MotorPortMode;

typedef enum
{
	MOTOR_PORT_SERVICE_CURRENT_OFFSET_CALIBRATION = 0,
	MOTOR_PORT_SERVICE_ENCODER_LINEARIZATION,
	MOTOR_PORT_SERVICE_ELECTRICAL_ZERO_CALIBRATION,
	MOTOR_PORT_SERVICE_OBSERVER_CALIBRATION,
	MOTOR_PORT_SERVICE_PHASE_RESISTANCE_IDENTIFICATION,
	MOTOR_PORT_SERVICE_FRICTION_IDENTIFICATION,
	MOTOR_PORT_SERVICE_SET_MECHANICAL_ZERO,
	MOTOR_PORT_SERVICE_PARAMETER_SAVE,
	MOTOR_PORT_SERVICE_RESTORE_DEFAULTS,
	MOTOR_PORT_SERVICE_ENCODER_DIRECTION_CALIBRATION,
	MOTOR_PORT_SERVICE_COGGING_IDENTIFICATION,
	MOTOR_PORT_SERVICE_FULL_COMMISSIONING
} MotorPortService;

typedef enum
{
	MOTOR_PORT_REQUEST_ACCEPTED = 0,
	MOTOR_PORT_REQUEST_OUT_OF_RANGE,
	MOTOR_PORT_REQUEST_INVALID_STATE
} MotorPortRequestResult;

typedef struct
{
	void *context;
	MotorPortMode (*get_mode)(void *context);
	bool (*request_mode)(void *context, MotorPortMode mode);
	bool (*request_service)(void *context, MotorPortService service);
	bool (*request_standby)(void *context);
	bool (*request_clear_faults)(void *context);
	float (*get_current_limit_a)(void *context);
	float (*get_speed_limit_rad_s)(void *context);
	void (*set_current_reference_a)(void *context, float current_a);
	void (*set_speed_reference_rad_s)(void *context, float speed_rad_s);
	void (*set_position_reference_rad)(void *context, float position_rad);
} MotorCommandPort;

#endif
