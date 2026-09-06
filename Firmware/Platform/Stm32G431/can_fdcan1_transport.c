#include "can_fdcan1_transport.h"

#include <stddef.h>
#include <string.h>

#include "fdcan.h"

#if defined(__CC_ARM)
#pragma O3
#pragma Ospace
#endif

#define CAN_FILTER_COUNT 1U
#define CAN_MAX_DATA_LENGTH 8U
#define CAN_STANDARD_ID_MAX 0x7FFUL
#define CAN_SAMPLE_POINT_PER_MILLE 765U

typedef struct
{
	volatile BspCommunicationFaultSet interrupt_faults;
	volatile BspCommunicationFaultSet foreground_faults;
	bool configured;
	volatile bool started;
} CanTransportContext;

static CanTransportContext CanState;

static uint32_t CanTransport_Prescaler(uint32_t bit_rate)
{
	if (bit_rate == 0U || bit_rate > 5000000UL ||
		(10000000UL % bit_rate) != 0U)
		return 0U;
	return 10000000UL / bit_rate;
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
	/* This endpoint deliberately exposes one fail-closed standard-ID range.
	 * Reject unsupported policy requests instead of silently widening ingress. */
	if (configuration->listen_only ||
		configuration->unmatched_standard_policy != BSP_CAN_UNMATCHED_REJECT ||
		configuration->unmatched_extended_policy != BSP_CAN_UNMATCHED_REJECT ||
		configuration->remote_standard_policy != BSP_CAN_REMOTE_REJECT ||
		configuration->remote_extended_policy != BSP_CAN_REMOTE_REJECT ||
		(configuration->nominal_sample_point_per_mille != 0U &&
		 configuration->nominal_sample_point_per_mille !=
			CAN_SAMPLE_POINT_PER_MILLE) ||
		(configuration->data_sample_point_per_mille != 0U &&
		 configuration->data_sample_point_per_mille !=
			CAN_SAMPLE_POINT_PER_MILLE) ||
		configuration->acceptance_filter_count != CAN_FILTER_COUNT ||
		configuration->acceptance_filters == NULL ||
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

	hfdcan1.Init.StdFiltersNbr = CAN_FILTER_COUNT;
	hfdcan1.Init.ExtFiltersNbr = 0U;
	hfdcan1.Init.FrameFormat = configuration->enable_fd ?
		(configuration->enable_brs ?
		 FDCAN_FRAME_FD_BRS : FDCAN_FRAME_FD_NO_BRS) :
		FDCAN_FRAME_CLASSIC;
	hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
	hfdcan1.Init.NominalPrescaler = nominal_prescaler;
	hfdcan1.Init.DataPrescaler = data_prescaler;
	if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
		goto io_error;

	filter_word = ((source->filter_kind == BSP_CAN_FILTER_RANGE ?
		FDCAN_FILTER_RANGE : FDCAN_FILTER_MASK) << 30U) |
		(FDCAN_FILTER_TO_RXFIFO0 << 27U) |
		(source->identifier_a << 16U) | source->identifier_b;
	*(uint32_t *)hfdcan1.msgRam.StandardFilterSA = filter_word;
	MODIFY_REG(hfdcan1.Instance->RXGFC,
		FDCAN_RXGFC_ANFS | FDCAN_RXGFC_ANFE |
		FDCAN_RXGFC_RRFS | FDCAN_RXGFC_RRFE,
		(FDCAN_REJECT << FDCAN_RXGFC_ANFS_Pos) |
		(FDCAN_REJECT << FDCAN_RXGFC_ANFE_Pos) |
		(FDCAN_REJECT_REMOTE << FDCAN_RXGFC_RRFS_Pos) |
		(FDCAN_REJECT_REMOTE << FDCAN_RXGFC_RRFE_Pos));

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
	/* FIFO FULL is only a high-water indication. MESSAGE_LOST proves loss. */
	if (CanState.started &&
		(interrupt_flags & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != 0U)
		CanState.interrupt_faults |= BSP_COMMUNICATION_FAULT_RX_OVERFLOW;
}

static BspResult CanTransport_TryReceive(void *context, BspCanFrame *frame)
{
	FDCAN_RxHeaderTypeDef header;

	if (context != &CanState || frame == NULL)
		return BSP_RESULT_INVALID_ARGUMENT;
	if (!CanState.started)
		return BSP_RESULT_NOT_READY;
	if ((hfdcan1.Instance->RXF0S & FDCAN_RXF0S_F0FL) == 0U)
		return BSP_RESULT_NOT_READY;
	if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0,
		&header, frame->data) != HAL_OK)
	{
		CanState.interrupt_faults |= BSP_COMMUNICATION_FAULT_IO;
		return BSP_RESULT_IO_ERROR;
	}
	frame->identifier = header.Identifier;
	frame->timestamp_us = 0U;
	frame->flags = 0U;
	if (header.IdType == FDCAN_EXTENDED_ID)
		frame->flags |= BSP_CAN_FRAME_EXTENDED_ID;
	if (header.RxFrameType == FDCAN_REMOTE_FRAME)
		frame->flags |= BSP_CAN_FRAME_REMOTE;
	if (header.FDFormat == FDCAN_FD_CAN)
		frame->flags |= BSP_CAN_FRAME_FD;
	if (header.BitRateSwitch == FDCAN_BRS_ON)
		frame->flags |= BSP_CAN_FRAME_BRS;
	if (header.ErrorStateIndicator == FDCAN_ESI_PASSIVE)
		frame->flags |= BSP_CAN_FRAME_ERROR_STATE_INDICATOR;
	frame->length = (uint8_t)header.DataLength;
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
	fd = (frame->flags & BSP_CAN_FRAME_FD) != 0U;
	brs = (frame->flags & BSP_CAN_FRAME_BRS) != 0U;
	if ((frame->flags & (BSP_CAN_FRAME_EXTENDED_ID |
		BSP_CAN_FRAME_REMOTE | UINT8_C(0xE0))) != 0U ||
		frame->identifier > CAN_STANDARD_ID_MAX ||
		frame->length > CAN_MAX_DATA_LENGTH ||
		(fd && hfdcan1.Init.FrameFormat == FDCAN_FRAME_CLASSIC) ||
		(brs && (!fd || hfdcan1.Init.FrameFormat != FDCAN_FRAME_FD_BRS)))
	{
		return BSP_RESULT_INVALID_ARGUMENT;
	}

	header.Identifier = frame->identifier;
	header.IdType = FDCAN_STANDARD_ID;
	header.TxFrameType = FDCAN_DATA_FRAME;
	header.DataLength = frame->length;
	header.ErrorStateIndicator =
		(frame->flags & BSP_CAN_FRAME_ERROR_STATE_INDICATOR) != 0U ?
		FDCAN_ESI_PASSIVE : FDCAN_ESI_ACTIVE;
	header.BitRateSwitch = brs ?
		FDCAN_BRS_ON : FDCAN_BRS_OFF;
	header.FDFormat = fd ?
		FDCAN_FD_CAN : FDCAN_CLASSIC_CAN;
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
	BspCommunicationFaultSet faults;

	if (context != &CanState)
		return BSP_COMMUNICATION_FAULT_IO;
	faults = CanState.foreground_faults | CanState.interrupt_faults;
	if ((hfdcan1.Instance->PSR & FDCAN_PSR_BO) != 0U)
		faults |= BSP_COMMUNICATION_FAULT_BUS_OFF;
	/* The interface latches this snapshot. Keep read_faults() side-effect free
	 * so TIM7 cannot race a foreground fault-set read/modify/write. */
	return faults;
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
