#include "can_fdcan1_transport.h"

#include <stddef.h>
#include <string.h>

#include "fdcan.h"

#if defined(__CC_ARM)
#pragma O3
#pragma Ospace
#endif

#define CAN_RX_QUEUE_COUNT 3U
#define CAN_RX_DRAIN_LIMIT 3U
#define CAN_FILTER_COUNT 1U
#define CAN_MAX_DATA_LENGTH 8U
#define CAN_STANDARD_ID_MAX 0x7FFUL
#define CAN_SAMPLE_POINT_PER_MILLE 765U

typedef struct
{
	uint32_t identifier;
	BspCanFrameFlagSet flags;
	uint8_t length;
	uint8_t data[CAN_MAX_DATA_LENGTH];
} CanQueuedFrame;

typedef struct
{
	volatile BspCommunicationFaultSet interrupt_faults;
	BspCommunicationFaultSet foreground_faults;
	CanQueuedFrame receive_queue[CAN_RX_QUEUE_COUNT];
	volatile uint32_t receive_read_sequence;
	volatile uint32_t receive_write_sequence;
	uint8_t receive_read_index;
	uint8_t receive_write_index;
	bool configured;
	volatile bool started;
} CanTransportContext;

static CanTransportContext CanState;

static uint32_t CanTransport_Prescaler(uint32_t bit_rate)
{
	uint32_t factor;

	if (bit_rate == 100000UL)
		return 100U;
	if (bit_rate == 200000UL)
		return 50U;
	if ((bit_rate % 125000UL) != 0U)
		return 0U;
	factor = bit_rate / 125000UL;
	if (factor == 40U)
		return 2U;
	if (factor >= 32U || ((UINT32_C(0x00110116) >> factor) & 1U) == 0U)
		return 0U;
	return 80U / factor;
}

static BspResult CanTransport_Configure(void *context,
	const BspCanConfiguration *configuration)
{
	uint32_t nominal_prescaler;
	uint32_t data_prescaler;
	uint32_t filter_word;
	const BspCanAcceptanceFilter *source = NULL;

	if (context != &CanState || configuration == NULL)
		return BSP_RESULT_INVALID_ARGUMENT;
	if (CanState.started)
		return BSP_RESULT_BUSY;
	if ((uint32_t)configuration->unmatched_standard_policy >
			(uint32_t)BSP_CAN_UNMATCHED_ACCEPT ||
		configuration->unmatched_extended_policy != BSP_CAN_UNMATCHED_REJECT ||
		(uint32_t)configuration->remote_standard_policy >
			(uint32_t)BSP_CAN_REMOTE_FILTER ||
		(uint32_t)configuration->remote_extended_policy >
			(uint32_t)BSP_CAN_REMOTE_FILTER ||
		(configuration->nominal_sample_point_per_mille != 0U &&
		 configuration->nominal_sample_point_per_mille !=
			CAN_SAMPLE_POINT_PER_MILLE) ||
		(configuration->data_sample_point_per_mille != 0U &&
		 configuration->data_sample_point_per_mille !=
			CAN_SAMPLE_POINT_PER_MILLE) ||
		configuration->acceptance_filter_count > CAN_FILTER_COUNT ||
		(configuration->acceptance_filter_count != 0U &&
		 configuration->acceptance_filters == NULL) ||
		(configuration->enable_brs && !configuration->enable_fd) ||
		(configuration->enable_fd && !configuration->enable_brs &&
		 configuration->data_bit_rate != configuration->nominal_bit_rate))
	{
		return BSP_RESULT_INVALID_ARGUMENT;
	}

	nominal_prescaler = CanTransport_Prescaler(
		configuration->nominal_bit_rate);
	data_prescaler = configuration->enable_fd ?
		CanTransport_Prescaler(configuration->data_bit_rate) :
		nominal_prescaler;
	if (nominal_prescaler == 0U ||
		configuration->nominal_bit_rate > 1000000UL ||
		data_prescaler == 0U ||
		(configuration->enable_fd &&
		 configuration->data_bit_rate > 5000000UL) ||
		(!configuration->enable_fd && configuration->data_bit_rate != 0U))
	{
		return BSP_RESULT_INVALID_ARGUMENT;
	}

	if (configuration->acceptance_filter_count != 0U)
	{
		source = configuration->acceptance_filters;
		if (source->filter_index >= CAN_FILTER_COUNT ||
			source->identifier_kind != BSP_CAN_IDENTIFIER_STANDARD ||
			(uint32_t)source->filter_kind > (uint32_t)BSP_CAN_FILTER_MASK ||
			source->identifier_a > CAN_STANDARD_ID_MAX ||
			source->identifier_b > CAN_STANDARD_ID_MAX ||
			(source->filter_kind == BSP_CAN_FILTER_RANGE &&
			 source->identifier_a > source->identifier_b))
		{
			return BSP_RESULT_INVALID_ARGUMENT;
		}
	}

	hfdcan1.Init.StdFiltersNbr = CAN_FILTER_COUNT;
	hfdcan1.Init.ExtFiltersNbr = 0U;
	hfdcan1.Init.FrameFormat = configuration->enable_fd ?
		(configuration->enable_brs ?
		 FDCAN_FRAME_FD_BRS : FDCAN_FRAME_FD_NO_BRS) :
		FDCAN_FRAME_CLASSIC;
	hfdcan1.Init.Mode = configuration->listen_only ?
		FDCAN_MODE_BUS_MONITORING : FDCAN_MODE_NORMAL;
	hfdcan1.Init.NominalPrescaler = nominal_prescaler;
	hfdcan1.Init.DataPrescaler = data_prescaler;
	if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
		goto io_error;

	if (source == NULL)
	{
		filter_word = FDCAN_FILTER_DISABLE << 27U;
	}
	else
	{
		filter_word =
			((source->filter_kind == BSP_CAN_FILTER_RANGE ?
			  FDCAN_FILTER_RANGE : FDCAN_FILTER_MASK) << 30U) |
			(FDCAN_FILTER_TO_RXFIFO0 << 27U) |
			(source->identifier_a << 16U) |
			source->identifier_b;
	}
	*(uint32_t *)hfdcan1.msgRam.StandardFilterSA = filter_word;
	MODIFY_REG(hfdcan1.Instance->RXGFC,
		FDCAN_RXGFC_ANFS | FDCAN_RXGFC_ANFE |
		FDCAN_RXGFC_RRFS | FDCAN_RXGFC_RRFE,
		((configuration->unmatched_standard_policy ==
			BSP_CAN_UNMATCHED_ACCEPT ? FDCAN_ACCEPT_IN_RX_FIFO0 :
			FDCAN_REJECT) << FDCAN_RXGFC_ANFS_Pos) |
		(FDCAN_REJECT << FDCAN_RXGFC_ANFE_Pos) |
		((configuration->remote_standard_policy == BSP_CAN_REMOTE_FILTER ?
			FDCAN_FILTER_REMOTE : FDCAN_REJECT_REMOTE) <<
			FDCAN_RXGFC_RRFS_Pos) |
		((configuration->remote_extended_policy == BSP_CAN_REMOTE_FILTER ?
			FDCAN_FILTER_REMOTE : FDCAN_REJECT_REMOTE) <<
			FDCAN_RXGFC_RRFE_Pos));

	CanState.receive_read_sequence = 0U;
	CanState.receive_write_sequence = 0U;
	CanState.receive_read_index = 0U;
	CanState.receive_write_index = 0U;
	CanState.configured = true;
	return BSP_RESULT_OK;

io_error:
	CanState.foreground_faults |= BSP_COMMUNICATION_FAULT_IO;
	CanState.configured = false;
	return BSP_RESULT_IO_ERROR;
}

static BspResult CanTransport_Start(void *context)
{
	if (context != &CanState)
		return BSP_RESULT_INVALID_ARGUMENT;
	if (CanState.started)
		return BSP_RESULT_OK;
	if (!CanState.configured)
		return BSP_RESULT_NOT_READY;
	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
		goto io_error;
	CanState.started = true;
	CLEAR_BIT(hfdcan1.Instance->ILS, FDCAN_IT_GROUP_RX_FIFO0);
	SET_BIT(hfdcan1.Instance->IE, FDCAN_IT_LIST_RX_FIFO0);
	SET_BIT(hfdcan1.Instance->ILE, FDCAN_INTERRUPT_LINE0);
	return BSP_RESULT_OK;
io_error:
	CanState.foreground_faults |= BSP_COMMUNICATION_FAULT_IO;
	return BSP_RESULT_IO_ERROR;
}

static BspResult CanTransport_Stop(void *context)
{
	if (context != &CanState)
		return BSP_RESULT_INVALID_ARGUMENT;
	if (!CanState.started)
		return BSP_RESULT_OK;
	CLEAR_BIT(hfdcan1.Instance->IE, FDCAN_IT_LIST_RX_FIFO0);
	CanState.started = false;
	if (HAL_FDCAN_Stop(&hfdcan1) != HAL_OK)
	{
		CanState.started = true;
		SET_BIT(hfdcan1.Instance->IE, FDCAN_IT_LIST_RX_FIFO0);
		CanState.foreground_faults |= BSP_COMMUNICATION_FAULT_IO;
		return BSP_RESULT_IO_ERROR;
	}
	return BSP_RESULT_OK;
}

void CanFdcan1Transport_OnReceiveInterrupt(uint32_t interrupt_flags)
{
	uint8_t drained;

	if (!CanState.started)
		return;
	/* FIFO FULL is a high-water indication, not proof of data loss.  The ISR
	 * can still drain every hardware entry into the equally-sized software
	 * queue.  Only the hardware MESSAGE_LOST condition or an actually full
	 * software queue is reported as an overflow. */
	if ((interrupt_flags & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != 0U)
	{
		CanState.interrupt_faults |= BSP_COMMUNICATION_FAULT_RX_OVERFLOW;
	}
	for (drained = 0U; drained < CAN_RX_DRAIN_LIMIT; ++drained)
	{
		FDCAN_RxHeaderTypeDef header;
		uint8_t data[BSP_CAN_FD_MAX_DATA_LENGTH];
		CanQueuedFrame *queued;

		if ((hfdcan1.Instance->RXF0S & FDCAN_RXF0S_F0FL) == 0U)
			break;
		if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0,
			&header, data) != HAL_OK)
		{
			CanState.interrupt_faults |= BSP_COMMUNICATION_FAULT_IO;
			break;
		}
		if (header.IdType != FDCAN_STANDARD_ID ||
			header.DataLength > FDCAN_DLC_BYTES_8)
		{
			continue;
		}

		if ((CanState.receive_write_sequence -
			CanState.receive_read_sequence) >= CAN_RX_QUEUE_COUNT)
		{
			CanState.interrupt_faults |= BSP_COMMUNICATION_FAULT_RX_OVERFLOW;
			continue;
		}
		queued = &CanState.receive_queue[CanState.receive_write_index];
		queued->identifier = header.Identifier;
		queued->flags = 0U;
		if (header.RxFrameType == FDCAN_REMOTE_FRAME)
			queued->flags |= BSP_CAN_FRAME_REMOTE;
		if (header.FDFormat == FDCAN_FD_CAN)
			queued->flags |= BSP_CAN_FRAME_FD;
		if (header.BitRateSwitch == FDCAN_BRS_ON)
			queued->flags |= BSP_CAN_FRAME_BRS;
		if (header.ErrorStateIndicator == FDCAN_ESI_PASSIVE)
			queued->flags |= BSP_CAN_FRAME_ERROR_STATE_INDICATOR;
		queued->length = (uint8_t)header.DataLength;
		if (queued->length != 0U &&
			header.RxFrameType == FDCAN_DATA_FRAME)
		{
			(void)memcpy(queued->data, data, queued->length);
		}
		CanState.receive_write_index++;
		if (CanState.receive_write_index == CAN_RX_QUEUE_COUNT)
			CanState.receive_write_index = 0U;
		CanState.receive_write_sequence++;
	}
}

static BspResult CanTransport_TryReceive(void *context, BspCanFrame *frame)
{
	const CanQueuedFrame *queued;

	if (context != &CanState || frame == NULL)
		return BSP_RESULT_INVALID_ARGUMENT;
	if (!CanState.started ||
		CanState.receive_read_sequence == CanState.receive_write_sequence)
	{
		return BSP_RESULT_NOT_READY;
	}
	queued = &CanState.receive_queue[CanState.receive_read_index];
	frame->identifier = queued->identifier;
	frame->timestamp_us = 0U;
	frame->flags = queued->flags;
	frame->length = queued->length;
	if (queued->length != 0U &&
		(queued->flags & BSP_CAN_FRAME_REMOTE) == 0U)
	{
		(void)memcpy(frame->data, queued->data, queued->length);
	}
	CanState.receive_read_index++;
	if (CanState.receive_read_index == CAN_RX_QUEUE_COUNT)
		CanState.receive_read_index = 0U;
	CanState.receive_read_sequence++;
	return BSP_RESULT_OK;
}

static BspResult CanTransport_TryTransmit(void *context,
	const BspCanFrame *frame)
{
	FDCAN_TxHeaderTypeDef header;
	HAL_StatusTypeDef status;
	bool fd;
	bool brs;

	if (context != &CanState || frame == NULL)
		return BSP_RESULT_INVALID_ARGUMENT;
	if (!CanState.started)
		return BSP_RESULT_NOT_READY;
	if (hfdcan1.Init.Mode == FDCAN_MODE_BUS_MONITORING)
		return BSP_RESULT_NOT_SUPPORTED;
	fd = (frame->flags & BSP_CAN_FRAME_FD) != 0U;
	brs = (frame->flags & BSP_CAN_FRAME_BRS) != 0U;
	if ((frame->flags & BSP_CAN_FRAME_EXTENDED_ID) != 0U ||
		(frame->flags & UINT8_C(0xE0)) != 0U ||
		frame->identifier > CAN_STANDARD_ID_MAX ||
		frame->length > CAN_MAX_DATA_LENGTH ||
		(fd && hfdcan1.Init.FrameFormat == FDCAN_FRAME_CLASSIC) ||
		(brs && (!fd || hfdcan1.Init.FrameFormat != FDCAN_FRAME_FD_BRS)) ||
		(fd && (frame->flags & BSP_CAN_FRAME_REMOTE) != 0U))
	{
		return BSP_RESULT_INVALID_ARGUMENT;
	}

	header.Identifier = frame->identifier;
	header.IdType = FDCAN_STANDARD_ID;
	header.TxFrameType = (frame->flags & BSP_CAN_FRAME_REMOTE) != 0U ?
		FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
	header.DataLength = frame->length;
	header.ErrorStateIndicator =
		(frame->flags & BSP_CAN_FRAME_ERROR_STATE_INDICATOR) != 0U ?
		FDCAN_ESI_PASSIVE : FDCAN_ESI_ACTIVE;
	header.BitRateSwitch = brs ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
	header.FDFormat = fd ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN;
	header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	header.MessageMarker = 0U;
	status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header,
		(uint8_t *)frame->data);
	if (status == HAL_OK)
		return BSP_RESULT_OK;
	if ((hfdcan1.ErrorCode & HAL_FDCAN_ERROR_FIFO_FULL) != 0U)
	{
		hfdcan1.ErrorCode &= ~HAL_FDCAN_ERROR_FIFO_FULL;
		/* Hardware FIFO pressure is recoverable backpressure.  The caller
		 * retains the complete frame and retries it, so no overflow occurred. */
		return BSP_RESULT_BUSY;
	}
	CanState.foreground_faults |= BSP_COMMUNICATION_FAULT_IO;
	return BSP_RESULT_IO_ERROR;
}

static BspCommunicationFaultSet CanTransport_ReadFaults(void *context)
{
	if (context != &CanState)
		return BSP_COMMUNICATION_FAULT_IO;
	if ((hfdcan1.Instance->PSR & FDCAN_PSR_BO) != 0U)
		CanState.foreground_faults |= BSP_COMMUNICATION_FAULT_BUS_OFF;
	return CanState.foreground_faults | CanState.interrupt_faults;
}

bool CanFdcan1Transport_CreatePort(
	const BspCommunicationEndpointCapabilities *capabilities,
	BspCanPort *port)
{
	const BspCommunicationFeatureSet features =
		BSP_COMMUNICATION_FEATURE_CAN_CLASSIC |
		BSP_COMMUNICATION_FEATURE_CAN_FD |
		BSP_COMMUNICATION_FEATURE_CAN_BRS;

	if (port == NULL || capabilities == NULL ||
		capabilities->endpoint_id == BSP_ENDPOINT_ID_NONE ||
		capabilities->availability != BSP_ENDPOINT_AVAILABLE ||
		capabilities->kind != BSP_COMMUNICATION_CAN ||
		capabilities->features != features ||
		capabilities->maximum_payload_bytes != CAN_MAX_DATA_LENGTH ||
		capabilities->maximum_nominal_bit_rate != 1000000UL ||
		capabilities->maximum_data_bit_rate != 5000000UL)
	{
		return false;
	}
	(void)memset(&CanState, 0, sizeof(CanState));
	port->context = &CanState;
	port->capabilities = capabilities;
	port->configure = CanTransport_Configure;
	port->start = CanTransport_Start;
	port->stop = CanTransport_Stop;
	port->try_receive = CanTransport_TryReceive;
	port->try_transmit = CanTransport_TryTransmit;
	port->read_faults = CanTransport_ReadFaults;
	return true;
}
