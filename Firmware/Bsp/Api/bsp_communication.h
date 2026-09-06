#ifndef FIRMWARE_BSP_API_BSP_COMMUNICATION_H
#define FIRMWARE_BSP_API_BSP_COMMUNICATION_H

#include "bsp_system.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BSP_CAN_FRAME_DATA_CAPACITY 64U

typedef enum
{
	BSP_COMMUNICATION_UNSPECIFIED = 0,
	BSP_COMMUNICATION_CAN,
	BSP_COMMUNICATION_BYTE_STREAM
} BspCommunicationKind;

typedef uint32_t BspCommunicationFeatureSet;

enum
{
	BSP_COMMUNICATION_FEATURE_CAN_CLASSIC = UINT32_C(1) << 0,
	BSP_COMMUNICATION_FEATURE_CAN_FD = UINT32_C(1) << 1,
	BSP_COMMUNICATION_FEATURE_CAN_BRS = UINT32_C(1) << 2,
	BSP_COMMUNICATION_FEATURE_EXTENDED_ID = UINT32_C(1) << 3,
	BSP_COMMUNICATION_FEATURE_BYTE_STREAM = UINT32_C(1) << 4,
	BSP_COMMUNICATION_FEATURE_FULL_DUPLEX = UINT32_C(1) << 5
};

typedef struct
{
	BspEndpointId endpoint_id;
	BspEndpointAvailability availability;
	BspCommunicationKind kind;
	BspCommunicationFeatureSet features;
	uint16_t maximum_payload_bytes;
	uint32_t maximum_nominal_bit_rate;
	uint32_t maximum_data_bit_rate;
} BspCommunicationEndpointCapabilities;

typedef uint8_t BspCanFrameFlagSet;

enum
{
	BSP_CAN_FRAME_EXTENDED_ID = UINT8_C(1) << 0,
	BSP_CAN_FRAME_REMOTE = UINT8_C(1) << 1,
	BSP_CAN_FRAME_FD = UINT8_C(1) << 2,
	BSP_CAN_FRAME_BRS = UINT8_C(1) << 3,
	BSP_CAN_FRAME_ERROR_STATE_INDICATOR = UINT8_C(1) << 4
};

typedef struct
{
	uint32_t identifier;
	uint32_t timestamp_us;
	BspCanFrameFlagSet flags;
	uint8_t length;
	uint8_t data[BSP_CAN_FRAME_DATA_CAPACITY];
} BspCanFrame;

typedef struct
{
	uint32_t nominal_bit_rate;
	uint32_t data_bit_rate;
	uint16_t nominal_sample_point_per_mille;
	uint16_t data_sample_point_per_mille;
	bool enable_fd;
	bool enable_brs;
	bool listen_only;
} BspCanConfiguration;

typedef uint32_t BspCommunicationFaultSet;

enum
{
	BSP_COMMUNICATION_FAULT_RX_OVERFLOW = UINT32_C(1) << 0,
	BSP_COMMUNICATION_FAULT_TX_OVERFLOW = UINT32_C(1) << 1,
	BSP_COMMUNICATION_FAULT_BUS_OFF = UINT32_C(1) << 2,
	BSP_COMMUNICATION_FAULT_IO = UINT32_C(1) << 3
};

/*
 * try_receive()/try_transmit() must never wait. Interrupt or DMA adapters own
 * their queues and only exchange complete frames through this interface.
 */
typedef struct
{
	void *context;
	const BspCommunicationEndpointCapabilities *capabilities;
	BspResult (*configure)(void *context,
		const BspCanConfiguration *configuration);
	BspResult (*start)(void *context);
	BspResult (*stop)(void *context);
	BspResult (*try_receive)(void *context, BspCanFrame *frame);
	BspResult (*try_transmit)(void *context, const BspCanFrame *frame);
	BspCommunicationFaultSet (*read_faults)(void *context);
} BspCanPort;

/*
 * Byte-stream try calls return BSP_RESULT_NOT_READY when no progress is
 * possible. accepted/read_count report partial progress without blocking.
 */
typedef struct
{
	void *context;
	const BspCommunicationEndpointCapabilities *capabilities;
	BspResult (*start)(void *context);
	BspResult (*stop)(void *context);
	BspResult (*try_read)(void *context, uint8_t *buffer, size_t capacity,
		size_t *read_count);
	BspResult (*try_write)(void *context, const uint8_t *data, size_t length,
		size_t *accepted_count);
	BspResult (*cancel_write)(void *context);
	BspCommunicationFaultSet (*read_faults)(void *context);
} BspByteStreamPort;

#ifdef __cplusplus
}
#endif

#endif
