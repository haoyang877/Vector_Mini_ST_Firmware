#ifndef FIRMWARE_BSP_API_BSP_SYSTEM_H
#define FIRMWARE_BSP_API_BSP_SYSTEM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef uint16_t BspEndpointId;

#define BSP_ENDPOINT_ID_NONE ((BspEndpointId)0U)

typedef enum
{
	BSP_RESULT_OK = 0,
	BSP_RESULT_BUSY,
	BSP_RESULT_NOT_READY,
	BSP_RESULT_INVALID_ARGUMENT,
	BSP_RESULT_NOT_SUPPORTED,
	BSP_RESULT_IO_ERROR,
	BSP_RESULT_SAFETY_FAULT
} BspResult;

typedef enum
{
	BSP_ENDPOINT_UNAVAILABLE = 0,
	BSP_ENDPOINT_AVAILABLE,
	BSP_ENDPOINT_PROVISIONED_UNPOPULATED
} BspEndpointAvailability;

typedef uint32_t BspSystemFeatureSet;

enum
{
	BSP_SYSTEM_FEATURE_MONOTONIC_CLOCK = UINT32_C(1) << 0,
	BSP_SYSTEM_FEATURE_EXECUTION_TIMER = UINT32_C(1) << 1,
	BSP_SYSTEM_FEATURE_CRITICAL_SECTION = UINT32_C(1) << 2,
	BSP_SYSTEM_FEATURE_UNIQUE_ID = UINT32_C(1) << 3,
	BSP_SYSTEM_FEATURE_NONVOLATILE_STORAGE = UINT32_C(1) << 4,
	BSP_SYSTEM_FEATURE_SOFTWARE_RESET = UINT32_C(1) << 5,
	BSP_SYSTEM_FEATURE_RESET_REASON = UINT32_C(1) << 6,
	BSP_SYSTEM_FEATURE_DIAGNOSTIC_SINK = UINT32_C(1) << 7,
	BSP_SYSTEM_FEATURE_STATUS_INDICATOR = UINT32_C(1) << 8
};

typedef struct
{
	BspSystemFeatureSet features;
	uint32_t nonvolatile_capacity_bytes;
	uint32_t nonvolatile_erase_size_bytes;
	uint32_t nonvolatile_program_alignment_bytes;
} BspSystemCapabilities;

/*
 * Time values intentionally wrap at 32 bits. Callers must use unsigned
 * subtraction. read_ms() must be bounded and safe from real-time context.
 */
typedef struct
{
	void *context;
	uint32_t (*read_ms)(void *context);
} BspMonotonicClockPort;

/* A free-running execution timer is used for deadline and WCET measurements. */
typedef struct
{
	void *context;
	uint32_t frequency_hz;
	uint32_t (*read_cycles)(void *context);
} BspExecutionTimerPort;

typedef uintptr_t BspCriticalSectionToken;

/* enter()/exit() must be bounded, nesting-safe, and callable from an ISR. */
typedef struct
{
	void *context;
	BspCriticalSectionToken (*enter)(void *context);
	void (*exit)(void *context, BspCriticalSectionToken token);
} BspCriticalSectionPort;

typedef struct
{
	void *context;
	/* Returns an opaque, stable byte sequence. Its bytes have no numeric
	 * endianness and must not be reinterpreted above the BSP. */
	BspResult (*read)(void *context, uint8_t *buffer, size_t capacity,
		size_t *length);
} BspUniqueIdPort;

typedef uint32_t BspResetReasonFlagSet;

enum
{
	BSP_RESET_REASON_NONE = 0U,
	BSP_RESET_REASON_POWER_OR_BROWN_OUT = UINT32_C(1) << 0,
	BSP_RESET_REASON_EXTERNAL_PIN = UINT32_C(1) << 1,
	BSP_RESET_REASON_SOFTWARE = UINT32_C(1) << 2,
	BSP_RESET_REASON_INDEPENDENT_WATCHDOG = UINT32_C(1) << 3,
	BSP_RESET_REASON_WINDOW_WATCHDOG = UINT32_C(1) << 4,
	BSP_RESET_REASON_LOW_POWER = UINT32_C(1) << 5,
	BSP_RESET_REASON_OPTION_BYTES = UINT32_C(1) << 6
};

typedef struct
{
	void *context;
	BspResetReasonFlagSet (*read_and_clear)(void *context);
} BspResetReasonPort;

typedef struct
{
	void *context;
	BspResult (*write)(void *context, const void *data, size_t length);
} BspDiagnosticSinkPort;

typedef struct
{
	uint32_t capacity_bytes;
	uint32_t erase_size_bytes;
	/* Program offsets must be aligned. Length may be shorter than a program
	 * granule; an implementation may fill the unused tail with erased bits, so
	 * callers must own the complete rounded-up granule. */
	uint32_t program_alignment_bytes;
} BspNonvolatileStorageGeometry;

/*
 * Storage erase/program operations may block and are forbidden in motor-loop
 * and ISR context. The implementation owns all address translation.
 */
typedef struct
{
	void *context;
	BspNonvolatileStorageGeometry geometry;
	BspResult (*read)(void *context, uint32_t offset, void *destination,
		size_t length);
	BspResult (*erase)(void *context, uint32_t offset, size_t length);
	BspResult (*program)(void *context, uint32_t offset, const void *source,
		size_t length);
} BspNonvolatileStoragePort;

typedef struct
{
	void *context;
	BspResult (*request)(void *context);
} BspResetPort;

/* Optional services are represented by NULL pointers, never hidden globals. */
typedef struct
{
	const BspMonotonicClockPort *clock;
	const BspExecutionTimerPort *execution_timer;
	const BspCriticalSectionPort *critical_section;
	const BspUniqueIdPort *unique_id;
	const BspResetReasonPort *reset_reason;
	const BspNonvolatileStoragePort *nonvolatile_storage;
	const BspDiagnosticSinkPort *diagnostic_sink;
	const BspResetPort *reset;
} BspSystemPorts;

#ifdef __cplusplus
}
#endif

#endif
